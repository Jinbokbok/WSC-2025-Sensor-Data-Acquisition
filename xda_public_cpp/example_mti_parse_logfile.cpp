//  Copyright (c) 2003-2025 Movella Technologies B.V.
//  ... (동일한 라이선스 헤더 생략)

//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi parse log file.
//--------------------------------------------------------------------------------
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xstypes/xsthread.h>
#include <xstypes/xsutctime.h>  // ← 추가!


#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <atomic>
#include <chrono>
#include <ctime>

Journaller* gJournal = 0;

using namespace std;

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler() : m_progress(0) {}
	virtual ~CallbackHandler() throw() {}
	int progress() const { return m_progress; }

protected:
	void onProgressUpdated(XsDevice* dev, int current, int total, const XsString* identifier) override
	{
		(void)dev; (void)total; (void)identifier;
		m_progress = current;
	}
private:
	volatile std::atomic_int m_progress;
};

//--------------------------------------------------------------------------------
int main(void)
{
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

	cout << "Opening log file..." << endl;
	string logFileName = "logfile.mtb";
	if (!control->openLogFile(logFileName))
		return handleError("Failed to open log file. Aborting.");
	else
		cout << "Opened log file: " << logFileName.c_str() << endl;

	// Get devices from file
	vector<XsDeviceId> deviceIdArray = control->mainDeviceIds();
	XsDeviceId mtDevice;
	for (auto const &deviceId : deviceIdArray)
	{
		if (deviceId.isMti() || deviceId.isMtig())
		{
			mtDevice = deviceId;
			break;
		}
	}
	if (!mtDevice.isValid())
		return handleError("No MTi device found. Aborting.");

	XsDevice* device = control->device(mtDevice);
	assert(device != nullptr);
	cout << "Device with ID: " << device->deviceId().toString() << " found in file." << endl;

	// Retain buffered data to access packets by index
	device->setOptions(XSO_RetainBufferedData, XSO_None);

	// Load file asynchronously; watch progress via callback
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	cout << "Loading the file..." << endl;
	device->loadLogFile();
	while (callback.progress() != 100)
		xsYield();
	cout << "File is fully loaded" << endl;

	cout << "Exporting the data..." << endl;
	string exportFileName = "exportfile.txt";
	ofstream outfile(exportFileName);
	outfile << fixed << setprecision(2);

	// Get total number of samples
	XsSize packetCount = device->getDataPacketCount();
	for (XsSize i = 0; i < packetCount; i++)
	{
		// Retrieve packet
		XsDataPacket packet = device->getDataPacketByIndex(i);

		// >>> ADDED: Time fields first (if present in log)
		if (packet.containsSampleTimeFine())
			outfile << "SampleTimeFine:" << packet.sampleTimeFine();

		if (packet.containsUtcTime())
		{
			XsUtcTime utc = packet.utcTime();
			outfile << " |UTC:" << utc.m_year << "-" << setw(2) << setfill('0') << (int)utc.m_month
			        << "-" << setw(2) << (int)utc.m_day
			        << " " << setw(2) << (int)utc.m_hour
			        << ":" << setw(2) << (int)utc.m_minute
			        << ":" << setw(2) << (int)utc.m_second
			        << "." << setw(3) << (utc.m_nano / 1000000);
			outfile << setfill(' ');

			    // === 호스트(PC) 시간 추가 ===
			auto now = std::chrono::system_clock::now();
			std::time_t now_c = std::chrono::system_clock::to_time_t(now);
			auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
			std::tm local_tm = *std::localtime(&now_c);
			outfile << " |HOST:" << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
					<< "." << std::setfill('0') << std::setw(3) << ms.count();
			outfile << setfill(' ');
		}

		if (packet.containsCalibratedData())
		{
			XsVector acc = packet.calibratedAcceleration();
			outfile << " |Acc X:" << acc[0]
			        << ", Y:" << acc[1]
			        << ", Z:" << acc[2];

			XsVector gyr = packet.calibratedGyroscopeData();
			outfile << " |Gyr X:" << gyr[0]
			        << ", Y:" << gyr[1]
			        << ", Z:" << gyr[2];

			XsVector mag = packet.calibratedMagneticField();
			outfile << " |Mag X:" << mag[0]
			        << ", Y:" << mag[1]
			        << ", Z:" << mag[2];
		}

		if (packet.containsOrientation())
		{
			XsQuaternion quaternion = packet.orientationQuaternion();
			outfile << " |q0:" << quaternion.w()
			        << ", q1:" << quaternion.x()
			        << ", q2:" << quaternion.y()
			        << ", q3:" << quaternion.z();

			XsEuler euler = packet.orientationEuler();
			outfile << " |Roll:" << euler.roll()
			        << ", Pitch:" << euler.pitch()
			        << ", Yaw:" << euler.yaw();
		}

		if (packet.containsLatitudeLongitude())
		{
			XsVector latLon = packet.latitudeLongitude();
			outfile << " |Lat:" << latLon[0]
			        << ", Lon:" << latLon[1];
		}

		if (packet.containsAltitude())
			outfile << " |Alt:" << packet.altitude();

		if (packet.containsVelocity())
		{
			XsVector vel = packet.velocity(XDI_CoordSysEnu);
			outfile << " |E:" << vel[0]
			        << ", N:" << vel[1]
			        << ", U:" << vel[2];
		}

		outfile << '\n';
	}
	outfile.close();
	cout << "File is exported to: " << exportFileName << endl;

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;
	cout << "Press [ENTER] to continue." << endl;
	cin.get();

	return 0;
}
