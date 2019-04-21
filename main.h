#pragma once

#include <string>
#include <opencv2/core/core.hpp>
#include "mynteye/types.h"
// MAESTRO
#include "include/Maestro-lib/include/RPMSerialInterface.h"
namespace H {
	
	class CVPainter {
	 public:
	  typedef enum Gravity {
	    TOP_LEFT,
	    TOP_RIGHT,
	    BOTTOM_LEFT,
	    BOTTOM_RIGHT
	  } gravity_t;

	  explicit CVPainter(std::int32_t frame_rate = 0);
	  ~CVPainter();

	  cv::Rect DrawSize(const cv::Mat &img, const gravity_t &gravity = TOP_LEFT);
	  cv::Rect DrawImgData(
	      const cv::Mat &img, const mynteye::ImgData &data,
	      const gravity_t &gravity = TOP_LEFT);
	  cv::Rect DrawImuData(
	      const cv::Mat &img, const mynteye::ImuData &data,
	      const gravity_t &gravity = TOP_RIGHT);

	  cv::Rect DrawText(
	      const cv::Mat &img, const std::string &text,
	      const gravity_t &gravity = TOP_LEFT, const int &margin = 5,
	      const int &offset_x = 0, const int &offset_y = 0);

	 private:
	  std::int32_t frame_rate_;
	};

	class main_class {
	public:
		void do_nothing();
		void test_maestro();
		void test_mynteye(int argc, char *argv[]);
		void manual_control();
		void tank_drive(int *axis, RPM::SerialInterface* serialInterface);
	};
}
