#pragma once

#include <opencv2/core.hpp>
#include "settings.h"

namespace calib {



	class CameraCalibrator {
	private:

		std::vector<std::vector<cv::Point2f> > imagePoints;
		cv::Mat cameraMatrix, distCoeffs;
		cv::Size imageSize;

		const char ESC_KEY = 27;
		Settings s;
	public:
		int extractPoints(std::vector<cv::Mat> &images, std::vector< cv::Mat >& masks,int cbRows,int cbCols, std::vector< std::vector<cv::Point2f> > &extractedPoints);
		int doCalibration(std::vector<cv::Mat> &images, std::vector< cv::Mat > &masks);

	};

}