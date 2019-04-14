#include "main.h"
#include <iostream>
#include <cmath>

#include <stdio.h>
#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

// MAESTRO
#include "Maestro-lib/include/RPMSerialInterface.h"

// MYNT Eye
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/api/api.h"
#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"

using namespace H;
using namespace std;
using namespace MYNTEYE_NAMESPACE;

int main(int argc, char *argv[]) {
	main_class mc;
	mc.do_nothing();
	mc.test_maestro();
	mc.test_mynteye(argc, argv);
	return 0;
}

void main_class::do_nothing()
{
	cout << "Do Nothing" << endl;
}

void main_class::test_mynteye(int argc, char *argv[]) 
{
	auto &&api = API::Create(argc, argv);

	api->EnableStreamData(Stream::DEPTH);
	
	api->Start(Source::VIDEO_STREAMING);

	cv::namedWindow("frame");
	cv::namedWindow("depth");

	while (true) {
	  api->WaitForStreams();

	  auto &&left_data = api->GetStreamData(Stream::LEFT);
	  auto &&right_data = api->GetStreamData(Stream::RIGHT);

	  cv::Mat img;
	  cv::hconcat(left_data.frame, right_data.frame, img);
	  cv::imshow("frame", img);

	  auto &&depth_data = api->GetStreamData(Stream::DEPTH);
	  if (!depth_data.frame.empty()) {
	    cv::imshow("depth", depth_data.frame);  // CV_16UC1
	  }

	  char key = static_cast<char>(cv::waitKey(1));
	  if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
	    break;
	  }
	}

	api->Stop(Source::VIDEO_STREAMING);

}

void main_class::test_maestro()
{
	//unsigned char deviceNumber = 12;
	//unsigned char channelNumber = 2;

	std::string portName = "/dev/ttyACM0";

	unsigned int baudRate = 9600;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);

	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ) {
		printf("Failed to create serial interface. %s\n", errorMessage.c_str());
	} else {
		printf("Created serial interface! \n");
	}
}
