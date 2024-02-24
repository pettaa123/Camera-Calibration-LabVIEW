#include "opencvCalibrationWrapper.h"
#include "cameraCalibrator.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

template <typename T>
cv::Mat createMat(T* data, int rows, int cols, int chs = 1) {
	// Create Mat from buffer 
	cv::Mat mat(rows, cols, CV_MAKETYPE(cv::DataType<T>::type, chs));
	memcpy(mat.data, data, rows * cols * chs * sizeof(T));
	return mat;
}

int32_t extractCorners(const Arr3D_U16Hdl images, const Arr_ClusterPointXYHdl maskPolygons,\
	const uint32_t cbRows, const uint32_t cbCols, Arr_ClusterPointXYfHdl extractedPoints, Arr3D_U32Hdl imagesExtracted) {

	int nbrMasks = (*maskPolygons)->dimSizes[0];

	int imgCount = (*images)->dimSizes[0];
	int rows = (*images)->dimSizes[1];
	int cols = (*images)->dimSizes[2];
	int stride = rows * cols;

	//dimension check
	if (nbrMasks != imgCount || rows == 0 || cols == 0)
		return EXIT_FAILURE;

	std::vector<cv::Mat> mats;
	std::vector<cv::Mat> masks;

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
		cv::fillConvexPoly(mask, pts, cv::Scalar(255));
		masks.push_back(mask);
	}

	calib::CameraCalibrator calibrator;
	std::vector < std::vector<cv::Point2f>> extractedPointsVec;

	const int nbrCbPoints = cbRows * cbCols;

	std::vector<cv::Mat> imagesExtractedVec;

	if (calibrator.extractPoints(mats, masks, cbRows, cbCols, extractedPointsVec, imagesExtractedVec))
		return EXIT_FAILURE;

	if (extractedPointsVec.size() != imgCount)
		return EXIT_FAILURE;

	for (auto vec : extractedPointsVec) {
		if (vec.size() != nbrCbPoints)
			return EXIT_FAILURE;
	}

	int extractedPointsSize = (extractedPointsVec.size() * sizeof(cv::Point2f) * nbrCbPoints);
	MgErr err = NumericArrayResize(uW, 1L, (UHandle*)&extractedPoints, extractedPointsSize);
	if (err)
		return err;

	(**extractedPoints).dimSizes[0] = extractedPointsVec.size();
	(**extractedPoints).dimSizes[1] = nbrCbPoints;

	//does this produces same output as for loop?
	//MoveBlock(extractedPointsVec.data(), (*extractedPoints)->Cluster, extractedPointsSize);

	for (int m = 0; m < extractedPointsVec.size(); m++) {
		int nbrExtractedPts = extractedPointsVec[m].size();
		for (size_t n = 0; n < nbrExtractedPts; n++)
		{
			(**extractedPoints).Cluster[m * nbrCbPoints + n].x = extractedPointsVec[m][n].x;
			(**extractedPoints).Cluster[m * nbrCbPoints + n].y = extractedPointsVec[m][n].y;
		}
	}



	err = NumericArrayResize(uW, 1L, (UHandle*)&imagesExtracted, sizeof(uint32_t) * imgCount * 2 * rows * cols);
	if (err)
		return err;

	for (int p = 0; p < imagesExtractedVec.size(); p++) {
		MoveBlock(imagesExtractedVec[p].data, &(*imagesExtracted)->elt[p* rows * cols], imgCount * rows * cols * sizeof(uint32_t));
	}

	(*imagesExtracted)->dimSizes[0] = imgCount * 2;
	(*imagesExtracted)->dimSizes[1] = rows;
	(*imagesExtracted)->dimSizes[2] = cols;

	return EXIT_SUCCESS;
}