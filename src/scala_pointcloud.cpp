/*===========================================================
 * \file 
 *		ibeo_scala_node.cpp
 * \author 
 * 		Lindong Guo (guolindong@gmail.com)
 * \date 
 *		04/2017
 * \description
 * 		ROS driver node for scala libar, using ibeoSDK
 *----------------------------------------------------------*/
#include "ros/ros.h"

#include "ibeo_scala_node.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "scala_pointcloud");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string device_ip;
	int port;
	pnh.param<std::string>("device_ip",device_ip,"192.168.1.52");
	pnh.param<int>("port", port, 12004);

	std::cerr << argv[0] << " Version " << appVersion.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	// bool hasLogFile;
	// const int checkResult = checkArguments(argc, argv, hasLogFile);
	// if (checkResult != 0)
	// 	exit(checkResult);
	// int currArg = 1;

	//std::string ip = "192.168.1.52";

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

	// if (hasLogFile) {
	// 	ibeosdk::LogFile::setLogFileBaseName(argv[currArg++]);
	// }
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);

	logFileManager.start();

	// if (hasLogFile) {
	// 	logInfo << argv[0] << " Version " << appVersion.toString()
	// 	        << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
	// }


	AllScalaListener allScalaListener;

	//const uint16_t port = getPort(ip, 12004);
	IbeoScala scala(device_ip, port);
	scala.setLogFileManager(&logFileManager);

	scala.registerListener(&allScalaListener);

	scala.getConnected();

	ros::Rate loop_rate(25);

	while(ros::ok())
	{
		if (!scala.isConnected()){
			ROS_INFO("!scala.isConnected\n");
			return -1;
		}
// #		ifdef _WIN32
// 			::Sleep(1);
// #		else // _WIN32
// 			sleep(1);
// #		endif // _WIN32

		ros::spinOnce();
		loop_rate.sleep();
	}
	// ros::waitForShutdown();
	return 0;

}