#include "opencvCalibrationWrapper.h"
#include "cameraCalibrator.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

//#define EXIT_SUCCESS 0
//#define EXIT_FAILURE 1

template <typename T>
cv::Mat createMat(T* data, int rows, int cols, int chs = 1) {
	// Create Mat from buffer 
	cv::Mat mat(rows, cols, CV_MAKETYPE(cv::DataType<T>::type, chs));
	memcpy(mat.data, data, rows * cols * chs * sizeof(T));
	return mat;
}

int32_t extractCorners(const Arr2D_U16Hdl image, const Arr_ClusterPointXYHdl maskPolygon, \
	Arr_ClusterPointXYfHdl extractedPoints, Arr3D_U32Hdl imagesExtracted, int adaptiveThreshBlockSize, int findMethod) {

	try {
		int nbrPolygonPoints = (*maskPolygon)->dimSize;

		int rows = (*image)->dimSizes[0];
		int cols = (*image)->dimSizes[1];

		//dimension check
		if (rows == 0 || cols == 0)
			return EXIT_FAILURE;

		std::vector<cv::Mat> masks;
		cv::Mat mat = createMat(&(**image).elt[0], rows, cols);
		cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);
		std::vector<cv::Point2i> pts;

		if (nbrPolygonPoints >= 3) {
			for (size_t j = 0; j < nbrPolygonPoints; j++)
			{
				int32_t x, y;
				x = (**maskPolygon).Cluster[j].x;
				y = (**maskPolygon).Cluster[j].y;
				pts.push_back(cv::Point2i(x, y));
			}
			cv::fillConvexPoly(mask, pts, cv::Scalar(255));
		}
		else { //if no polygon is given use complete image
			mask = cv::Mat(rows, cols, CV_8U, cv::Scalar(255));
		}

		calib::CamCalibrator& calibrator = calib::CamCalibrator::getInstance();
		std::vector<cv::Point2f> extractedPointsVec;

		std::vector<cv::Mat> imagesExtractedVec;

		calibrator.extractPoints(mat, mask, extractedPointsVec, imagesExtractedVec, adaptiveThreshBlockSize, (calib::FIND_METHOD)findMethod);

		int elemSize = sizeof((*imagesExtracted)->elt);
		MgErr  err = NumericArrayResize(uW, 1L, (UHandle*)&imagesExtracted, elemSize * imagesExtractedVec.size() * rows * cols);
		if (err)
			return err;
		for (int p = 0; p < imagesExtractedVec.size(); p++) {
			MoveBlock(imagesExtractedVec[p].data, &(*imagesExtracted)->elt[p * rows * cols], rows * cols * elemSize);
		}

		(*imagesExtracted)->dimSizes[0] = 2;
		(*imagesExtracted)->dimSizes[1] = rows;
		(*imagesExtracted)->dimSizes[2] = cols;

		int extractedPointsSize = (extractedPointsVec.size() * sizeof((*extractedPoints)->Cluster));
		err = NumericArrayResize(uW, 1L, (UHandle*)&extractedPoints, extractedPointsSize);
		if (err)
			return err;

		(**extractedPoints).dimSize = extractedPointsVec.size();

		for (int m = 0; m < extractedPointsVec.size(); m++) {
			(**extractedPoints).Cluster[m].x = extractedPointsVec[m].x;
			(**extractedPoints).Cluster[m].y = extractedPointsVec[m].y;
		}
	}
	catch (const std::exception& e) {
		//log
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int32_t setBoardSettings(const uint32_t boardType, const uint32_t cbRows, const uint32_t cbCols, const float squareSize) {
	try {
		calib::CamCalibrator& c = calib::CamCalibrator::getInstance();
		c.setBoardSettings(boardType, cbRows, cbCols, squareSize);
	}
	catch (const std::exception& e) {
		//log
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int32_t calibrateCamera(const Arr2D_ClusterPointXYfHdl extractedPoints, int flags, Arr2D_DBLHdl intrinsics, Arr_DBLHdl distCoeffs, Arr_SGLHdl reprojErrs,double &totalAvgErr) {
	try {

		int nbrExtractedPtsSets = (*extractedPoints)->dimSizes[0];
		int nbrExtractedPts = (*extractedPoints)->dimSizes[1];

		//check dims
		if (nbrExtractedPtsSets < 3)
			return 5009;

		if ((**intrinsics).dimSizes[0] != 3 || (**intrinsics).dimSizes[1] != 3)
			return 5010;

		if ((**distCoeffs).dimSize != 8)
			return 5011;

		std::vector<std::vector<cv::Point2f>> extractedPtsVec(nbrExtractedPtsSets);
		std::vector<cv::Point2f> extractedPts(nbrExtractedPts);
		for (int i = 0; i < nbrExtractedPtsSets; i++) {
			for (size_t j = 0; j < nbrExtractedPts; j++)
			{
				float x = (**extractedPoints).Cluster[i * nbrExtractedPts + j].x;
				float y = (**extractedPoints).Cluster[i * nbrExtractedPts + j].y;
				extractedPts.at(j) = cv::Point2f(x, y);
			}
			extractedPtsVec.at(i) = extractedPts;
		}

		cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distCoeffsMat = cv::Mat::zeros(8, 1, CV_64F);
		std::vector<cv::Mat> rvecs;
		std::vector<cv::Mat> tvecs;
		std::vector<float> reprojErrsVec;

		calib::CamCalibrator& calibrator = calib::CamCalibrator::getInstance();
		if (calibrator.runCalibration(extractedPtsVec, flags, cameraMatrix, distCoeffsMat, rvecs, tvecs, reprojErrsVec, totalAvgErr))
			return EXIT_FAILURE;

		//copy camera matrix to labview array
		MoveBlock(cameraMatrix.data, &(**intrinsics).elt, 9 * sizeof(double));
		MoveBlock(distCoeffsMat.data, &(**distCoeffs).elt, 8 * sizeof(double));

		MgErr err = NumericArrayResize(uW, 1L, (UHandle*)&reprojErrs, sizeof(float) * nbrExtractedPtsSets);
		if (err)
			return err;

		(*reprojErrs)->dimSize = nbrExtractedPtsSets;

		MoveBlock(reprojErrsVec.data(), &(*reprojErrs)->elt, nbrExtractedPtsSets * sizeof(float));

	}
	catch (const std::exception& e) {
		//log
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int32_t undistort(const Arr2D_U16Hdl inOutput, Arr2D_DBLHdl cameraMatrix, Arr_DBLHdl distCoeffs) {

	int rows = (*inOutput)->dimSizes[0];
	int cols = (*inOutput)->dimSizes[1];

	//dimension check
	if (rows == 0 || cols == 0)
		return EXIT_FAILURE;

	cv::Mat input = createMat(&(**inOutput).elt[0], rows, cols);

	cv::Mat output;
	cv::undistort(input, output, cameraMatrix, distCoeffs);
}

/*int32_t extractCornersMulti(const Arr3D_U16Hdl images, const Arr2D_ClusterPointXYHdl maskPolygons, \
	const uint32_t cbRows, const uint32_t cbCols, Arr2D_ClusterPointXYfHdl extractedPoints, Arr3D_U32Hdl imagesExtracted) {

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

	if (calibrator.extractPointsMulti(mats, masks, cbRows, cbCols, extractedPointsVec, imagesExtractedVec))
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
}*/