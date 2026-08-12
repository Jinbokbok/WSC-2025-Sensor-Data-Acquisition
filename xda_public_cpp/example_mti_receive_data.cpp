//  Copyright (c) 2003-2025 Movella Technologies
//  (동일한 라이선스 헤더 생략)

//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi receive data (MTB + CSV append).
//--------------------------------------------------------------------------------
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xstypes/xsutctime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>
#include <string>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cmath>
#include <csignal>
#include <atomic>


Journaller* gJournal = 0;

using namespace std;
//namespace fs = std::filesystem;


class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}
	virtual ~CallbackHandler() throw() {}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
	}

private:
	mutable xsens::Mutex m_mutex;
	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};

// 작은 헬퍼: 현재 로컬시간 → "YYYY-MM-DD" / "YYYY-MM-DD_HH-MM-SS" 문자열
static std::string currentDate()
{
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm lt = *std::localtime(&t);
	std::ostringstream ss;
	ss << std::put_time(&lt, "%Y-%m-%d");
	return ss.str();
}
static std::string currentDateTime()
{
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm lt = *std::localtime(&t);
	std::ostringstream ss;
	// 파일명에 콜론 대신 하이픈 사용
	ss << std::put_time(&lt, "%Y-%m-%d_%H-%M-%S");
	return ss.str();
}

static std::atomic_bool g_running(true);

static void handleSigint(int) {
    g_running = false;
}


//--------------------------------------------------------------------------------
int main(void)
{
	std::signal(SIGINT, handleSigint);  // Ctrl+C 누르면 g_running=false
	cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString()
	     << " @ port: " << mtPort.portName().toStdString()
	     << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString()
	     << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;

	// Enable log metadata
	device->readEmtsAndDeviceConfiguration();

	// ==== 원하는 출력 설정 (예: 400 Hz) ====
	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
	configArray.push_back(XsOutputConfiguration(XDI_UtcTime, 0));

	if (device->deviceId().isImu())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 400));
	}
	else if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
	}
	else if (device->deviceId().isGnss())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 400));
		//configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 400));
		// IMU 보정값도 보고 싶으면:
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 400));
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	// ==== 파일명 결정 ====
	const std::string dateStr = currentDate();                // "YYYY-MM-DD"
	const std::string dateTimeStr = currentDateTime();        // "YYYY-MM-DD_HH-MM-SS"

	// MTB: 실행마다 새 파일 → "YYYY-MM-DD_HH-MM-SS log.mtb"
	const std::string mtbFileName = dateTimeStr + " log.mtb";

	// CSV: 날짜별 누적 → "YYYY-MM-DD imu log.csv"
	const std::string csvFileName = dateStr + " imu log.csv";

	// ==== CSV 열기(append), 없으면 헤더 쓰기 ====
	bool csvExists = static_cast<bool>(std::ifstream(csvFileName));
	std::ofstream csv(csvFileName, std::ios::app);
	if (!csv.is_open())
		return handleError("Failed to open CSV file for appending.");

	if (!csvExists)
	{
		csv
		  << "UTC,HOST,SampleTimeFine,"
		  << "AccX,AccY,AccZ,"
		  << "GyrX,GyrY,GyrZ,"
		  << "MagX,MagY,MagZ,"
		  << "q0,q1,q2,q3,"
		  << "Roll,Pitch,Yaw,"
		  << "Lat,Lon,"
		  << "E,N,U"
		  << "\n";
		csv.flush();
	}

	// ==== MTB 로그 파일 생성 (매 실행마다 새로) ====
	cout << "Creating a log file..." << endl;
	if (device->createLogFile(mtbFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << mtbFileName << endl;

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");

	cout << "\nMain loop. Recording data for 10 seconds." << endl;
	cout << string(79, '-') << endl;

	int64_t startTime = XsTime::timeStampNow();
	cout << std::fixed << std::setprecision(2);

	while (g_running)
	{
		if (!callback.packetAvailable())
		{
			XsTime::msleep(0);
			continue;
		}

		XsDataPacket packet = callback.getNextPacket();

		// 화면 출력 준비
		std::ostringstream line;
		line << std::fixed << std::setprecision(2);

		// 공통: HOST(PC) 시간 문자열
		auto now = std::chrono::system_clock::now();
		std::time_t now_c = std::chrono::system_clock::to_time_t(now);
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		std::tm local_tm = *std::localtime(&now_c);
		std::ostringstream hostSS;
		hostSS << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
		       << "." << std::setw(3) << std::setfill('0') << ms.count();
		std::string hostStr = hostSS.str();

		// CSV 각 필드 기본값
		std::string utcStr = "";
		long long sampleTimeFine = 0;
		double accX=NAN, accY=NAN, accZ=NAN;
		double gyrX=NAN, gyrY=NAN, gyrZ=NAN;
		double magX=NAN, magY=NAN, magZ=NAN;
		double q0=NAN, q1=NAN, q2=NAN, q3=NAN;
		double roll=NAN, pitch=NAN, yaw=NAN;
		double lat=NAN, lon=NAN;
		double vE=NAN, vN=NAN, vU=NAN;

		// SampleTimeFine
		if (packet.containsSampleTimeFine())
		{
			sampleTimeFine = static_cast<long long>(packet.sampleTimeFine());
			line << "\rSampleTimeFine:" << sampleTimeFine;
		}

		// UTC
		if (packet.containsUtcTime())
		{
			XsUtcTime utc = packet.utcTime();
			std::ostringstream utcSS;
			// UTC → time_t 변환
			std::tm utc_tm = {};
			utc_tm.tm_year = utc.m_year - 1900;
			utc_tm.tm_mon  = utc.m_month - 1;
			utc_tm.tm_mday = utc.m_day;
			utc_tm.tm_hour = utc.m_hour;
			utc_tm.tm_min  = utc.m_minute;
			utc_tm.tm_sec  = utc.m_second;
			time_t utc_time_t = timegm(&utc_tm);  // UTC 기준으로 변환

			// +9시간
			utc_time_t += 9 * 3600;

			// 다시 tm으로 변환 (KST)
			std::tm kst_tm = *std::gmtime(&utc_time_t);

			// 문자열 생성
			utcSS << std::setfill('0')
				<< (kst_tm.tm_year + 1900) << "-" << std::setw(2) << (kst_tm.tm_mon + 1)
				<< "-" << std::setw(2) << kst_tm.tm_mday
				<< " " << std::setw(2) << kst_tm.tm_hour
				<< ":" << std::setw(2) << kst_tm.tm_min
				<< ":" << std::setw(2) << kst_tm.tm_sec
				<< "." << std::setw(3) << (utc.m_nano / 1000000);
			utcStr = utcSS.str();

			line << " |UTC:" << utcStr;
		}

		// HOST
		line << " |HOST:" << hostStr;

		// Calibrated data
		if (packet.containsCalibratedData())
		{
			XsVector acc = packet.calibratedAcceleration();
			accX = acc[0]; accY = acc[1]; accZ = acc[2];
			line << " |Acc X:" << accX << ", Y:" << accY << ", Z:" << accZ;

			XsVector gyr = packet.calibratedGyroscopeData();
			gyrX = gyr[0]; gyrY = gyr[1]; gyrZ = gyr[2];
			line << " |Gyr X:" << gyrX << ", Y:" << gyrY << ", Z:" << gyrZ;

			XsVector mag = packet.calibratedMagneticField();
			magX = mag[0]; magY = mag[1]; magZ = mag[2];
			line << " |Mag X:" << magX << ", Y:" << magY << ", Z:" << magZ;
		}

		// Orientation
		if (packet.containsOrientation())
		{
			XsQuaternion quaternion = packet.orientationQuaternion();
			q0 = quaternion.w(); q1 = quaternion.x(); q2 = quaternion.y(); q3 = quaternion.z();
			line << " |q0:" << q0 << ", q1:" << q1 << ", q2:" << q2 << ", q3:" << q3;

			XsEuler euler = packet.orientationEuler();
			roll = euler.roll(); pitch = euler.pitch(); yaw = euler.yaw();
			line << " |Roll:" << roll << ", Pitch:" << pitch << ", Yaw:" << yaw;
		}

		// Lat/Lon
		if (packet.containsLatitudeLongitude())
		{
			XsVector latLon = packet.latitudeLongitude();
			lat = latLon[0]; lon = latLon[1];
			line << " |Lat:" << lat << ", Lon:" << lon;
		}
		// if (packet.containsAltitude())
 			// 	cout << " |Alt:" << packet.altitude();


		// Velocity ENU
		if (packet.containsVelocity())
		{
			XsVector vel = packet.velocity(XDI_CoordSysEnu);
			vE = vel[0]; vN = vel[1]; vU = vel[2];
			line << " |E:" << vE << ", N:" << vN << ", U:" << vU;
		}

		// 화면 출력
		cout << line.str() << std::flush;

		// === CSV 한 줄 추가 (모든 패킷마다) ===
		csv  << (utcStr.empty() ? "" : utcStr) << ","
		     << hostStr << ","
		     << sampleTimeFine << ","
		     << accX << "," << accY << "," << accZ << ","
		     << gyrX << "," << gyrY << "," << gyrZ << ","
		     << magX << "," << magY << "," << magZ << ","
		     << q0 << "," << q1 << "," << q2 << "," << q3 << ","
		     << roll << "," << pitch << "," << yaw << ","
		     << lat << "," << lon << ","
		     << vE << "," << vN << "," << vU
		     << "\n";
		// 너무 자주 flush하면 성능 저하—여기서는 안전하게 가끔씩만 해도 됨
	}
	cout << "\n" << string(79, '-') << "\n" << endl;

	cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");

	// CSV 닫기
	// (ofstream 소멸 시 자동 닫힘이지만 명시적으로도 괜찮음)
	// csv.close();

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;
	//cout << "Press [ENTER] to continue." << endl;
	//cin.get();

	return 0;
}

