#include "main.h"
#include <iostream>
#include <cmath>

#include <stdio.h>
#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

// For Joystick
#include <fcntl.h>
#include <sys/ioctl.h>

// MAESTRO
#include "include/Maestro-lib/include/RPMSerialInterface.h"

// MYNT Eye
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "util/cv_painter.h"

#include "mynteye/api/api.h"
#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"

// CV PAINTER
#include <iomanip>
#include <memory>
#include <utility>

// Joystick
#include <linux/joystick.h>

// XBox Controller Port
#define JOY_DEV "/dev/input/js1"

#define FONT_FACE cv::FONT_HERSHEY_PLAIN
#define FONT_SCALE 1
#define FONT_COLOR cv::Scalar(255, 255, 255)
#define THICKNESS 1

using namespace H;
using namespace std;
using namespace MYNTEYE_NAMESPACE;

// CVPAINTER SECTION START
std::shared_ptr<std::ios> NewFormat(int width, int prec, char fillch = ' ') {
  auto fmt = std::make_shared<std::ios>(nullptr);
  fmt->setf(std::ios::fixed);
  if (width > 0)
    fmt->width(std::move(width));
  if (prec > 0)
    fmt->precision(std::move(prec));
  fmt->fill(std::move(fillch));
  return fmt;
}

std::ostringstream &Clear(std::ostringstream &os) {
  os.str("");
  os.clear();
  return os;
}

std::ostream &operator<<(
    std::ostream &os, const std::shared_ptr<std::ios> &fmt) {
  if (fmt)
    os.copyfmt(*fmt);
  return os;
}

CVPainter::CVPainter(std::int32_t frame_rate)
    : frame_rate_(std::move(frame_rate)) {
  VLOG(2) << __func__;
}

CVPainter::~CVPainter() {
  VLOG(2) << __func__;
}

cv::Rect CVPainter::DrawSize(const cv::Mat &img, const gravity_t &gravity) {
  std::ostringstream ss;
  ss << img.cols << "x" << img.rows;
  return DrawText(img, ss.str(), gravity, 5);
}

cv::Rect CVPainter::DrawImgData(
    const cv::Mat &img, const mynteye::ImgData &data,
    const gravity_t &gravity) {
  int sign = 1;
  if (gravity == BOTTOM_LEFT || gravity == BOTTOM_RIGHT)
    sign = -1;

  static auto fmt_time = NewFormat(0, 2);

  std::ostringstream ss;
  ss << "frame_id: " << data.frame_id;
  ss << ", stamp: " << fmt_time << (0.001f * data.timestamp);  // ms
  ss << ", expo: ";
  if (frame_rate_ == 0) {
    ss << data.exposure_time;
  } else {
    ss << fmt_time << mynteye::utils::get_real_exposure_time(
                          frame_rate_, data.exposure_time);
  }
  cv::Rect rect_i = DrawText(img, ss.str(), gravity, 5);

  Clear(ss) << "size: " << img.cols << "x" << img.rows;
  cv::Rect rect_s =
      DrawText(img, ss.str(), gravity, 5, 0, sign * (5 + rect_i.height));

  // rect_i.width is the max one
  if (sign > 0) {
    return cv::Rect(
        rect_i.tl(),
        cv::Point(rect_i.x + rect_i.width, rect_s.y + rect_s.height));
  } else {
    return cv::Rect(rect_s.tl(), rect_i.br());
  }
}

cv::Rect CVPainter::DrawImuData(
    const cv::Mat &img, const mynteye::ImuData &data,
    const gravity_t &gravity) {
  static std::ostringstream ss;
  static auto fmt_imu = NewFormat(8, 4);
  static auto fmt_temp = NewFormat(6, 4);

  int sign = 1;
  if (gravity == BOTTOM_LEFT || gravity == BOTTOM_RIGHT)
    sign = -1;

  Clear(ss) << "stamp: " << data.timestamp
            << ", temp: " << fmt_temp << data.temperature;
  cv::Rect rect_i = DrawText(img, ss.str(), gravity, 5);

  Clear(ss) << "accel(x,y,z): " << fmt_imu << data.accel[0] << "," << fmt_imu
            << data.accel[1] << "," << fmt_imu << data.accel[2];
  cv::Rect rect_a =
      DrawText(img, ss.str(), gravity, 5, 0, sign * (5 + rect_i.height));

  Clear(ss) << "gyro(x,y,z): " << fmt_imu << data.gyro[0] << "," << fmt_imu
            << data.gyro[1] << "," << fmt_imu << data.gyro[2];
  cv::Rect rect_g = DrawText(
      img, ss.str(), gravity, 5, 0,
      sign * (10 + rect_i.height + rect_a.height));

  // rect_i.width is the max one
  if (sign > 0) {
    return cv::Rect(
        rect_i.tl(),
        cv::Point(rect_i.x + rect_i.width, rect_g.y + rect_g.height));
  } else {
    return cv::Rect(rect_g.tl(), rect_i.br());
  }
}

cv::Rect CVPainter::DrawText(
    const cv::Mat &img, const std::string &text, const gravity_t &gravity,
    const int &margin, const int &offset_x, const int &offset_y) {
  int w = img.cols, h = img.rows;

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(text, FONT_FACE, FONT_SCALE, THICKNESS, &baseline);

  int x, y;
  switch (gravity) {
    case TOP_LEFT:
      x = margin;
      y = margin + textSize.height;
      break;
    case TOP_RIGHT:
      x = w - margin - textSize.width;
      y = margin + textSize.height;
      break;
    case BOTTOM_LEFT:
      x = margin;
      y = h - margin;
      break;
    case BOTTOM_RIGHT:
      x = w - margin - textSize.width;
      y = h - margin;
      break;
    default:  // TOP_LEFT
      x = margin;
      y = margin + textSize.height;
      break;
  }
  x += offset_x;
  y += offset_y;

  cv::Point org(x, y);
#ifdef WITH_OPENCV2
  cv::putText(
      const_cast<cv::Mat &>(img), text, org, FONT_FACE, FONT_SCALE, FONT_COLOR,
      THICKNESS);
#else
  cv::putText(img, text, org, FONT_FACE, FONT_SCALE, FONT_COLOR, THICKNESS);
#endif
  return cv::Rect(org, textSize);
}

//CVPAINTER END

// OUR STUFF

int main(int argc, char *argv[]) {
	main_class mc;
	//mc.do_nothing();
	//mc.test_maestro();
	//mc.test_mynteye(argc, argv);

	mc.manual_control();
	return 0;
}

void main_class::manual_control() {
	std::string portName;

	portName = "/dev/ttyACM0";

	unsigned char numChannels = 6; 
	
	std::cout << "Opening Pololu Maestro on serial interface \"" << portName << "\"..." << std::endl;
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface(portName, 9600, &errorMessage );
	if ( !serialInterface )
		std::cerr << "Error: " << errorMessage << std::endl;

	
	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];
	struct js_event js;

	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		printf( "Couldn't open joystick\n" );
		return;
	}

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	button = (char *) calloc( num_of_buttons, sizeof( char ) );

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
		, name_of_joystick
		, num_of_axis
		, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */

	while( 1 ) 	/* infinite loop */
	{

			/* read the joystick state */
		read(joy_fd, &js, sizeof(struct js_event));
		
			/* see what to do with the event */
		switch (js.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_AXIS:
				axis   [ js.number ] = js.value;
				this->tank_drive(axis,serialInterface);
				break;
			case JS_EVENT_BUTTON:
				button [ js.number ] = js.value;
				break;
		}

			/* print the results 
		printf( "X: %6d  Y: %6d  ", axis[0], axis[1] );
		
		if( num_of_axis > 2 )
			printf("Z: %6d  ", axis[2] );
			
		if( num_of_axis > 3 )
			printf("R: %6d  ", axis[3] );
			
		for( x=0 ; x<num_of_buttons ; ++x )
			printf("B%d: %d  ", x, button[x] );

		printf("  \r");
		fflush(stdout);*/
	}

}

int xbox2maestro(int raw) {
	return int((double(-raw/2.0 + 32767)/65534)*4032 + 3968);
}

void main_class::tank_drive(int *axis, RPM::SerialInterface* serialInterface) {
	int servoL = xbox2maestro(axis[1]); // left stick y	
	int servoR = xbox2maestro(axis[3]); // right stick y
	
	serialInterface->setTargetCP( 0, servoL );
	serialInterface->setTargetCP( 1, servoR );
	printf("L:%d,\tR:%d\n",servoL,servoR);
}

void main_class::do_nothing()
{
	cout << "Do Nothing" << endl;
}

void main_class::test_mynteye(int argc, char *argv[]) 
{
	auto &&api = API::Create(argc, argv);

	api->SetOptionValue(Option::IR_CONTROL, 80);
	api->SetDisparityComputingMethodType(DisparityComputingMethod::BM);
	api->EnableStreamData(Stream::DISPARITY_NORMALIZED);
	api->EnableStreamData(Stream::POINTS);
	api->EnableStreamData(Stream::DEPTH);
	
	api->Start(Source::VIDEO_STREAMING);

	cv::namedWindow("frame");
	cv::namedWindow("depth");

	CVPainter painter;

	while (true) {
		api->WaitForStreams();

		auto &&left_data = api->GetStreamData(Stream::LEFT);
		auto &&right_data = api->GetStreamData(Stream::RIGHT);

		cv::Mat img;
		cv::hconcat(left_data.frame, right_data.frame, img);

		painter.DrawImgData(img, *left_data.img);

		cv::imshow("frame", img);
		// LOG(INFO) << "left id: " << left_data.frame_id
		//     << ", right id: " << right_data.frame_id;

		auto &&disp_data = api->GetStreamData(Stream::DISPARITY_NORMALIZED);
		auto &&depth_data = api->GetStreamData(Stream::DEPTH);
		if (!disp_data.frame.empty() && !depth_data.frame.empty()) {

			// Show disparity instead of depth, but show depth values in region.
			auto &&depth_frame = disp_data.frame;
			printf("here");
			cv::applyColorMap(depth_frame, depth_frame, cv::COLORMAP_JET);
			cv::imshow("depth", depth_frame);
			auto &&points_data = api->GetStreamData(Stream::POINTS);
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
