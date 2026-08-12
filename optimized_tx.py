#!/usr/bin/env python3
import logging
import can
import time, subprocess, struct, json, zlib
import serial
import pandas as pd
from datetime import datetime
from pathlib import Path
import RPi.GPIO as GPIO

# ========= 설정 =========
BASE_DIR = Path("/home/solo/Desktop/WSC")
BASE_DIR.mkdir(parents=True, exist_ok=True)

LED_PIN = 11
CAN_CHANNEL = 'can0'
CAN_BITRATE = 500000

TELEM_PORT = "/dev/ttyUSB0"
TELEM_BAUD = 57600

UART_PORT = "/dev/ttyAMA0"     # 원시 CAN 중계용
UART_BAUD = 115200

MAGIC = b'\xA5\x5A'

# ========= 로깅 =========
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# ========= 부팅 스크립트(옵션) =========
try:
    subprocess.run("bash /home/solo/mppton.sh", shell=True, check=False)
except Exception as e:
    logging.warning(f"mppton.sh 실행 경고: {e}")

# ========= GPIO =========
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

# ========= 유틸 =========
def get_csv_file_path():
    today_str = datetime.now().strftime('%Y-%m-%d')
    return BASE_DIR / f"{today_str}_CAN log.csv"

def is_can0_available():
    try:
        output = subprocess.check_output(['ip', 'link', 'show', CAN_CHANNEL]).decode('utf-8')
        return 'state UP' in output
    except Exception:
        return False

def wait_for_can_interface():
    while not is_can0_available():
        logging.info(f"{CAN_CHANNEL} 인터페이스 대기 중...")
        time.sleep(3)

def wait_for_handshake(ser):
    logging.info("RX 핸드셰이크 대기 중...")
    ser.timeout = 0.2
    while True:
        try:
            ser.write(b'TX_READY\n')
        except Exception:
            time.sleep(0.2)
            continue
        # 상대가 선제적으로 RX_READY/RESYNC를 줄 수도 있음
        try:
            line = ser.readline().strip()
        except Exception:
            line = b''
        if not line:
            time.sleep(0.2)
            continue
        if line.upper().startswith(b'RX_READY') or line.upper().startswith(b'RESYNC'):
            logging.info("RX 신호 수신 → TX_ACK 전송")
            try:
                ser.write(b'TX_ACK\n')
            except Exception:
                pass
            return True

# ========= CAN 파싱 =========
def parse_float_from_bytes(data, start_index):
    return struct.unpack('<f', data[start_index:start_index + 4])[0]

def translate_fault_status(fault_code):
    fault_dict = {
        0x00: "No Fault", 0x01: "Over Voltage", 0x02: "Under Voltage",
        0x04: "Over Temperature", 0x08: "Overcurrent Discharge",
        0x10: "Overtemperature Discharge", 0x20: "Undertemperature Charge",
        0x40: "Short Circuit Discharge", 0x80: "Cell Imbalance Protection"
    }
    return fault_dict.get(fault_code, "Unknown Fault")

def parse_bms_message(msg, bms_dict):
    bms_dict['Voltage'] = ((msg.data[0] << 8) | msg.data[1]) * 0.01
    # 오프셋은 BMS 스펙에 맞춰 확인 필요. 기존 구현 유지.
    bms_dict['Current'] = ((((msg.data[2] << 8) | msg.data[3])) - 32767) * 0.01
    bms_dict['SoC'] = msg.data[4]
    bms_dict['Pack Fault Status'] = translate_fault_status(msg.data[5])
    bms_dict['batt_temp_1'] = msg.data[6] - 55
    bms_dict['batt_temp_2'] = msg.data[7] - 55

def parse_mppt_message(msg, mppt_dict):
    mppt_index = (msg.arbitration_id - 0x600) >> 4
    if mppt_index not in mppt_dict:
        mppt_dict[mppt_index] = {}
    mdata = mppt_dict[mppt_index]
    mdata['MPPT_ID'] = 600 + (mppt_index * 10)  # 관례적 매핑
    message_type = msg.arbitration_id & 0x0F
    try:
        if message_type == 0x00:
            mdata['Input Voltage'] = round(parse_float_from_bytes(msg.data, 0), 2)
            mdata['Input Current'] = round(parse_float_from_bytes(msg.data, 4), 5)
        elif message_type == 0x01:
            mdata['Output Voltage'] = round(parse_float_from_bytes(msg.data, 0), 2)
            mdata['Output Current'] = round(parse_float_from_bytes(msg.data, 4), 5)
        elif message_type == 0x02:
            mdata['FetTemp'] = round(parse_float_from_bytes(msg.data, 0), 2)
            mdata['ControllerTemp'] = round(parse_float_from_bytes(msg.data, 4), 2)
        elif message_type == 0x03:
            mdata['Aux 12V'] = round(parse_float_from_bytes(msg.data, 0), 2)
            mdata['Aux 3V'] = round(parse_float_from_bytes(msg.data, 4), 2)
        elif message_type == 0x04:
            mdata['Max Output Voltage Setting'] = round(parse_float_from_bytes(msg.data, 0))
            mdata['Max Input Current Setting'] = round(parse_float_from_bytes(msg.data, 4))
        elif message_type == 0x05:
            mdata['CAN Rx Error Counter'] = msg.data[0]
            mdata['CAN Tx Error Counter'] = msg.data[1]
            mdata['CAN Tx Overflow'] = msg.data[2]
            mdata['Error Flags'] = msg.data[3]
            mdata['Limit Flags'] = msg.data[4]
            mdata['Mode'] = msg.data[5]
        elif message_type == 0x06:
            mdata['Output Voltage (at fuse)'] = parse_float_from_bytes(msg.data, 0)
            mdata['Power Connector Temp'] = parse_float_from_bytes(msg.data, 4)
        elif message_type == 0x08:
            mdata['Mode'] = msg.data[0]
    except struct.error as e:
        logging.error(f"MPPT 데이터 unpack 오류 (ID={msg.arbitration_id}): {e}")

def parse_velocity_message(msg, data_dict):
    data_dict['Velocity'] = parse_float_from_bytes(msg.data, 0)  # 단위는 송신원 기준

def flatten_data(second_data):
    row = {}
    row['Timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    bms = second_data.get('BMS', {})
    row['BMS_Voltage'] = bms.get('Voltage')
    row['BMS_Current'] = bms.get('Current')
    row['BMS_SoC'] = bms.get('SoC')
    row['BMS_PackFaultStatus'] = bms.get('Pack Fault Status')
    row['BMS_batt_temp_1'] = bms.get('batt_temp_1')
    row['BMS_batt_temp_2'] = bms.get('batt_temp_2')

    for i in range(4):
        mppt = second_data.get('MPPT', {}).get(i, {})
        prefix = f'MPPT{i+1}_'
        row[prefix + 'ID'] = mppt.get('MPPT_ID')
        row[prefix + 'InputVoltage'] = mppt.get('Input Voltage')
        row[prefix + 'InputCurrent'] = mppt.get('Input Current')
        row[prefix + 'OutputVoltage'] = mppt.get('Output Voltage')
        row[prefix + 'OutputCurrent'] = mppt.get('Output Current')
        row[prefix + 'FetTemp'] = mppt.get('FetTemp')
        row[prefix + 'ControllerTemp'] = mppt.get('ControllerTemp')
        row[prefix + 'MaxOutputVoltageSetting'] = mppt.get('Max Output Voltage Setting')
        row[prefix + 'MaxInputCurrentSetting'] = mppt.get('Max Input Current Setting')
        row[prefix + 'CANRxErrorCounter'] = mppt.get('CAN Rx Error Counter')
        row[prefix + 'CANTxErrorCounter'] = mppt.get('CAN Tx Error Counter')
        row[prefix + 'CANTxOverflow'] = mppt.get('CAN Tx Overflow')
        row[prefix + 'ErrorFlags'] = mppt.get('Error Flags')
        row[prefix + 'LimitFlags'] = mppt.get('Limit Flags')
        row[prefix + 'Mode'] = mppt.get('Mode')

    row['Velocity'] = second_data.get('Velocity')
    return row

def log_to_csv(data_dict, file_path):
    try:
        header = not file_path.exists()
        df = pd.DataFrame([data_dict])
        df.to_csv(file_path, mode='a', index=False, header=header)
        logging.info(f"CSV 기록됨 (Timestamp: {data_dict.get('Timestamp')}) -> {file_path.name}")
    except Exception as e:
        logging.error(f"CSV 기록 오류: {e}")

# ========= 텔레메트리 전송/제어 =========
seq = 0
resync_event = None  # threading.Event()는 main에서 생성

def send_telemetry_data(ser, data_dict):
    global seq
    seq = (seq + 1) & 0xFFFF
    data_dict = dict(data_dict)  # 원본 보존
    data_dict['seq'] = seq
    data_dict['tx_unix_ms'] = int(time.time() * 1000)

    payload = json.dumps(data_dict, separators=(',', ':')).encode('utf-8')
    length  = len(payload).to_bytes(4, 'big')
    crc32   = (zlib.crc32(payload) & 0xffffffff).to_bytes(4, 'big')
    packet  = MAGIC + length + payload + crc32
    ser.write(packet)

def telemetry_control_listener(ser):
    """RX로부터 RESYNC/RX_READY를 수신하면 즉시 TX_ACK, 키프레임 트리거."""
    global seq
    ser.timeout = 0.05
    buf = bytearray()
    last_beacon = 0.0
    while True:
        # 주기 비콘(디버깅용, 필요 없으면 주석)
        now = time.time()
        if now - last_beacon > 2.0:
            try:
                ser.write(f"BEACON {seq}\n".encode('ascii'))
            except Exception:
                pass
            last_beacon = now

        try:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                if chunk:
                    buf.extend(chunk)
                    # 너무 커지면 뒷부분만 유지(바이너리 혼입 대비)
                    if len(buf) > 4096:
                        del buf[:3072]
                    while b'\n' in buf:
                        i = buf.find(b'\n')
                        line = bytes(buf[:i]).strip()
                        del buf[:i+1]
                        if not line:
                            continue
                        try:
                            cmd = line.decode('ascii', errors='ignore').upper()
                        except Exception:
                            continue
                        if cmd.startswith("RX_READY") or cmd.startswith("RESYNC"):
                            try:
                                ser.write(b"TX_ACK\n")
                            except Exception:
                                pass
                            if resync_event:
                                resync_event.set()
        except Exception:
            time.sleep(0.05)
        time.sleep(0.01)

def main():
    wait_for_can_interface()

    bus = None
    uart_ser = None
    telemetry_ser = None

    # CSV 헤더 생성
    csv_path = get_csv_file_path()
    if not csv_path.exists():
        pd.DataFrame(columns=list(flatten_data({'BMS': {}, 'MPPT': {}, 'Velocity': None}).keys())).to_csv(csv_path, index=False)

    try:
        try:
            bus = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
            logging.info("CAN bus 초기화 성공.")
        except Exception as e:
            logging.error(f"CAN bus 초기화 실패: {e}")
            return

        try:
            uart_ser = serial.Serial(UART_PORT, UART_BAUD)
            logging.info(f"UART 포트 ({UART_PORT}) 초기화 성공.")
        except Exception as e:
            logging.error(f"UART 포트 초기화 실패: {e}")
            uart_ser = None

        try:
            telemetry_ser = serial.Serial(
                port=TELEM_PORT,
                baudrate=TELEM_BAUD,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            if wait_for_handshake(telemetry_ser):
                logging.info("텔레메트리 연결 성공.")
        except Exception as e:
            logging.error(f"텔레메트리 포트 초기화 실패: {e}")
            telemetry_ser = None

        # 컨트롤 리스너 시작
        import threading
        global resync_event, seq
        resync_event = threading.Event()
        if telemetry_ser:
            threading.Thread(target=telemetry_control_listener, args=(telemetry_ser,), daemon=True).start()

        logging.info("데이터 수집 시작...")
        while True:
            csv_path = get_csv_file_path()  # 날짜 바뀌면 자동 갱신
            cycle_start = time.time()
            second_data = {'BMS': {}, 'MPPT': {}, 'Velocity': None}

            # 약 0.5초 수집
            while time.time() - cycle_start < 0.5:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue
                try:
                    if msg.arbitration_id == 0x100:
                        if uart_ser:  # 원시 중계
                            data_length = len(msg.data)
                            packet = struct.pack(f'>HB{data_length}s', 0x100, data_length, msg.data)
                            uart_ser.write(packet)
                        parse_bms_message(msg, second_data['BMS'])

                    elif 0x600 <= msg.arbitration_id <= 0x638:
                        if uart_ser:
                            data_length = len(msg.data)
                            packet = struct.pack(f'>HB{data_length}s', msg.arbitration_id, data_length, msg.data)
                            uart_ser.write(packet)
                        parse_mppt_message(msg, second_data['MPPT'])

                    elif msg.arbitration_id == 0x200:
                        if uart_ser:
                            data_length = len(msg.data)
                            packet = struct.pack(f'>HB{data_length}s', 0x200, data_length, msg.data)
                            uart_ser.write(packet)
                        parse_velocity_message(msg, second_data)
                except Exception as e:
                    logging.error(f"프레임 처리 오류: {e}")

            row_data = flatten_data(second_data)
            log_to_csv(row_data, csv_path)

            # Fault LED
            if row_data.get('BMS_PackFaultStatus') == 'No Fault':
                GPIO.output(LED_PIN, GPIO.LOW)
            else:
                GPIO.output(LED_PIN, GPIO.HIGH)

            # 텔레메트리 송신
            if telemetry_ser:
                # RESYNC 요청 즉시 키프레임
                if resync_event and resync_event.is_set():
                    logging.info("RESYNC 요청 → 시퀀스 리셋 및 키프레임 전송")
                    seq = 0
                    resync_event.clear()
                    try:
                        send_telemetry_data(telemetry_ser, row_data)
                    except Exception as e:
                        logging.error(f"키프레임 전송 실패: {e}")
                try:
                    send_telemetry_data(telemetry_ser, row_data)
                except Exception as e:
                    logging.error(f"텔레메트리 전송 실패: {e}")

    except KeyboardInterrupt:
        logging.info("데이터 수집 중단됨 (사용자 인터럽트).")
    finally:
        try:
            if bus: bus.shutdown()
        except Exception:
            pass
        try:
            if telemetry_ser: telemetry_ser.close()
        except Exception:
            pass
        try:
            if uart_ser: uart_ser.close()
        except Exception:
            pass
        try:
            GPIO.output(LED_PIN, GPIO.LOW)
            GPIO.cleanup()
        except Exception:
            pass

if __name__ == '__main__':
    main()