
/*===========================================================
 * \file 
 *		ibeo_scala_node.h
 * \author 
 * 		Lindong Guo (guolindong@gmail.com)
 * \date 
 *		04/2017
 * \description
 * 		ROS driver node for scala libar, using ibeoSDK
 *----------------------------------------------------------*/

#include <ibeosdk/scala.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <iostream>
#include <cstdlib>

using namespace ibeosdk;
using namespace std;

IbeoSDK ibeoSDK;

const ibeosdk::Version::MajorVersion majorVersion(5);
const ibeosdk::Version::MinorVersion minorVersion(0);
const ibeosdk::Version::Revision revision(4);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "IbeoSdkScalaLiveDemo";

ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);

TimeConversion tc;

class AllScalaListener : public ibeosdk::DataListener<FrameEndSeparator>,
                         public ibeosdk::DataListener<ScanLux>,
                         public ibeosdk::DataListener<Scan2208>,
                         public ibeosdk::DataListener<ObjectListLux>,
                         public ibeosdk::DataListener<ObjectListScala>,
                         public ibeosdk::DataListener<ObjectListScala2271>,
                         public ibeosdk::DataListener<VehicleStateBasicLux>,
                         public ibeosdk::DataListener<DeviceStatus>,
                         public ibeosdk::DataListener<DeviceStatus6303>,
                         public ibeosdk::DataListener<LogMessageError>,
                         public ibeosdk::DataListener<LogMessageDebug>,
                         public ibeosdk::DataListener<LogMessageNote>,
                         public ibeosdk::DataListener<LogMessageWarning> {
public:
	virtual ~AllScalaListener() {}

public:
	//========================================
	void onData(const FrameEndSeparator* const fes)
	{
		logInfo << std::setw(5) << fes->getSerializedSize() << " Bytes  "
				<< "Frame received: # " << fes->getFrameId() << std::endl;
	}

	//========================================
	void onData(const ScanLux* const scan)
	{
		// logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
		// 		<< "Scan 2202 received: # " << scan->getScanNumber()
		// 	<<"  time: " << tc.toString(scan->getStartTimestamp().toPtime(), 3)
		// 	<< std::endl;
	}

	//========================================
	void onData(const Scan2208* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "Scan 2208 received: # " << scan->getScanNumber()
			<<"  time: " << tc.toString(scan->getSubScans().at(0).getStartScanTimestamp().toPtime(), 3)
			<< std::endl;
	}

	//========================================
	void onData(const ObjectListLux* const objs)
	{
		// logInfo << std::setw(5) << objs->getSerializedSize() << " Bytes  "
				// << "ObjectList 2221 received. #Obj:" << objs->getNumberOfObjects()
				// << std::endl;
	}

	//========================================
	void onData(const ObjectListScala* const objs)
	{
		logInfo << std::setw(5) << objs->getSerializedSize() << " Bytes  "
				<< "ObjectList 2270 received. Scan: " << objs->getScanNumber()
				<< "  #Obj:" << objs->getNumberOfObjects()
				<< std::endl;

	}

	//========================================

	void onData(const ObjectListScala2271* const objs)
	{
		logInfo << std::setw(5) << objs->getSerializedSize() << " Bytes  "
				<< "ObjectList 2271 received. Scan: " << objs->getScanNumber()
				<< "  ObjLstId: " << int(objs->getObjectListId())
				<< "  #Obj:" << objs->getNumberOfObjects()
				<< std::endl;

	}

	//========================================

	void onData(const VehicleStateBasicLux* const vsb)
	{
		// logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  "
				// << "VSB for Scan #: " << vsb->getScanNumber() << "  "
				// << tc.toString(vsb->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const DeviceStatus* const deviceStatus)
	{
		// logInfo << std::setw(5) << deviceStatus->getSerializedSize() << " Bytes  "
				// << "DevStat received" << std::endl;
	}

	//========================================
	void onData(const DeviceStatus6303* const deviceStatus)
	{
		// logInfo << std::setw(5) << deviceStatus->getSerializedSize() << " Bytes  "
				// << "DevStat 0x6303 received" << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageError* const logMsg)
	{
		// logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				// << "LogMessage (Error) received: time: " << logMsg->getHeaderNtpTime().toPtime() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageWarning* const logMsg)
	{
		// logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				// << "LogMessage (Warning) received: time: " << logMsg->getHeaderNtpTime().toPtime() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageNote* const logMsg)
	{
		// logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				// << "LogMessage (Note) received: time: " << logMsg->getHeaderNtpTime().toPtime() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageDebug* const logMsg)
	{
		// logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				// << "LogMessage (Debug) received: time: " << logMsg->getHeaderNtpTime().toPtime() << ": " << logMsg->getMessage() << std::endl;
	}
}; // AllScalaListener


int checkArguments(const int argc, const char** argv, bool& hasLogFile)
{
	const int minNbOfNeededArguments = 2;
	const int maxNbOfNeededArguments = 3;

	bool wrongNbOfArguments = false;
	if (argc < minNbOfNeededArguments) {
		std::cerr << "Missing argument" << std::endl;
		wrongNbOfArguments = true;
	}
	else if (argc > maxNbOfNeededArguments) {
		std::cerr << "Too many argument" << std::endl;
		wrongNbOfArguments = true;
	}

	if (wrongNbOfArguments) {
		std::cerr << argv[0] << " " << " IP [LOGFILE]" << std::endl;
		std::cerr << "\tIP is the ip address of the Scala sensor, e.g. 192.168.0.1." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}