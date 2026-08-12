import sys
import glob
import serial
import serial.tools.list_ports

def get_available_ports():
    """
    운영체제별로 사용 가능한 시리얼 포트 목록을 반환
    """
    if sys.platform.startswith('win'):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    elif sys.platform.startswith('linux'):
        return glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        return glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

def select_port(mode):
    """
    단일 포트 선택 함수
    mode: 'TX' 또는 'RX'
    """
    available_ports = get_available_ports()
    
    if not available_ports:
        raise serial.SerialException("No serial ports found")
    
    print(f"\nSelecting {mode} port...")
    print("\nAvailable ports:")
    for i, port in enumerate(available_ports):
        try:
            with serial.Serial(port, 57600, timeout=1) as ser:
                info = f"{port} (Available)"
        except serial.SerialException:
            info = f"{port} (In use)"
        print(f"{i}: {info}")
    
    while True:
        try:
            port_idx = int(input(f"\nSelect {mode} port number: "))
            
            if 0 <= port_idx < len(available_ports):
                selected_port = available_ports[port_idx]
                
                # Test port availability
                try:
                    with serial.Serial(selected_port, 57600, timeout=1) as ser:
                        print(f"\nSuccessfully connected to {selected_port}")
                        return selected_port
                except serial.SerialException:
                    print("Selected port is in use. Please choose a different port.")
                    continue
            else:
                print("Invalid selection. Please choose a valid port number.")
        except ValueError:
            print("Please enter a valid number.")