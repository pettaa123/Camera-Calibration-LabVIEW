#pragma once

#include <opencv2/core.hpp>
#include "settings.h"

namespace calib {

	enum FIND_METHOD {
		FIND_CB_CORNERS,
		FIND_CB_CORNERS_SB
	};

	enum BOARD_TYPE {
		CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
	};

	class CamCalibrator {
	public:
		static CamCalibrator& getInstance() {
			static CamCalibrator instance; // Guaranteed to be initialized only once
			return instance;
		}

		int32_t setBoardSettings(int boardType, int rows, int cols, float squareSize) {
			s.type = (BOARD_TYPE)boardType;
			s.boardSize = cv::Size(rows, cols);
			s.squareSize = squareSize;

			return EXIT_SUCCESS;
		}

		int extractPoints(cv::Mat& image, cv::Mat& mask, \
			std::vector<cv::Point2f>& extractedPoints, std::vector<cv::Mat>& imagesExtracted, int adaptiveThreshBlockSize = 51, FIND_METHOD = FIND_CB_CORNERS_SB);
		/*int extractPointsMulti(std::vector<cv::Mat>& images, std::vector< cv::Mat >& masks, \
			int cbRows, int cbCols, std::vector< std::vector<cv::Point2f> >& extractedPoints, std::vector<cv::Mat>& imagesExtracted);*/
		int runCalibration(std::vector<std::vector<cv::Point2f>> imagePoints, int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,\
			std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs, double& totalAvgErr);

	private:

		cv::Mat cameraMatrix, distCoeffs;
		cv::Size imageSize;

		const char ESC_KEY = 27;
		Settings s;

		double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints, const std::vector<std::vector<cv::Point2f> >& imagePoints,
			const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, std::vector<float>& perViewErrors);

		CamCalibrator() {} // Private constructor to prevent instantiation
		~CamCalibrator() {} // Private destructor to prevent deletion
		CamCalibrator(const CamCalibrator&) = delete; // Delete copy constructor
		CamCalibrator& operator=(const CamCalibrator&) = delete; // Delete assignment operator
	};

}