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

#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"

using namespace H;
using namespace std;
using namespace MYNTEYE_NAMESPACE;

int main() {
	main_class mc;
	mc.do_nothing();
	mc.test_maestro();
	mc.test_mynteye();
	return 0;
}

void main_class::do_nothing()
{
	cout << "Do Nothing" << endl;
}

void main_class::test_mynteye() 
{
	auto &&device = device::select();
	if (!device) printf("Failed to select device");

	bool ok;
	auto &&request = device::select_request(device, &ok);
	if (!ok) printf("Failed to request from device/n");
	device->ConfigStreamRequest(request);

	

}

void main_class::test_maestro()
{
	unsigned char deviceNumber = 12;
	unsigned char channelNumber = 2;

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
