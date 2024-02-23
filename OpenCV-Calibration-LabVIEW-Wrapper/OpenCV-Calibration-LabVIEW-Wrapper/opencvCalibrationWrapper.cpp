#include "opencvCalibrationWrapper.h"
#include "cameraCalibrator.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
/*
int32_t calibrateCamera(const Arr3D_U16Hdl images, const Arr3D_U8Hdl masks) {
	return EXIT_SUCCESS;
}
*/

template <typename T>
cv::Mat createMat(T* data, int rows, int cols, int chs = 1) {
	// Create Mat from buffer 
	cv::Mat mat(rows, cols, CV_MAKETYPE(cv::DataType<T>::type, chs));
	memcpy(mat.data, data, rows * cols * chs * sizeof(T));
	return mat;
}

int32_t extractCorners(const Arr3D_U16Hdl images, const Arr_ClusterPointXYHdl maskPolygons, const ClusterPointXYHdl extractedPoints) {


	int nbrMasks = (*maskPolygons)->dimSizes[1];

	int imgCount = (*images)->dimSizes[0];
	int rows = (*images)->dimSizes[1];
	int cols = (*images)->dimSizes[2];
	int stride = rows * cols;

	//dimension check
	if (nbrMasks != imgCount || rows == 0 || cols == 0)
		return EXIT_FAILURE;

	std::vector<cv::Mat> mats;
	std::vector<cv::Mat> masks;
	std::vector<cv::Mat> roieds;
	for (int i = 0; i < imgCount; i++) {
		mats.push_back(createMat(&(**images).elt[1 + stride * i], rows, cols));
		cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);
		std::vector<cv::Point2i> pts;
		int32_t nbrPts = (**maskPolygons).dimSizes[1];
		for (size_t j = 0; j < nbrPts; j++)
		{
			int32_t x, y;
			x = (**maskPolygons).Cluster[i * nbrPts + j].x;
			y = (**maskPolygons).Cluster[i * nbrPts + j].y;
			pts.push_back(cv::Point2i(x, y));
		}
		cv::fillConvexPoly(mask, pts, cv::Scalar(1));
		masks.push_back(mask);
		cv::Mat roied(rows, cols, CV_8UC1, cv::Scalar(127));
		cv::normalize(mats[i], roied, 255, 0, cv::NORM_MINMAX, CV_8UC1, mask);
		roieds.push_back(roied);
		cv::imshow(std::to_string(i), roied);
		cv::waitKey(1);
	}

	calib::CameraCalibrator calibrator;
	std::vector < std::vector<cv::Point2f>> extractedPointsVec;

	if (calibrator.extractPoints(roieds, extractedPointsVec))
		return EXIT_FAILURE;



	return EXIT_SUCCESS;
}