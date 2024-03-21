#include "cameraCalibrator.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

namespace calib {

	bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<std::vector<cv::Point2f> > imagePoints);

	int CamCalibrator::runCalibration(std::vector<std::vector<cv::Point2f>> imagePoints, int flags, cv::Mat& cameraMatrix,\
		cv::Mat& distCoeffs, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs, double& totalAvgErr) {

		if (flags & CV_CALIB_FIX_ASPECT_RATIO)
			cameraMatrix.at<double>(0, 0) = s.aspectRatio;
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
		std::vector<std::vector<cv::Point3f> > objectPoints(1);

		for (int i = 0; i < s.boardSize.height; i++)
			for (int j = 0; j < s.boardSize.width; j++)
				objectPoints[0].push_back(cv::Point3f(float(j * s.squareSize), float(i * s.squareSize), 0));

		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		double rms = calibrateCamera(objectPoints, imagePoints, s.imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs, flags | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
		///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
			rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return EXIT_SUCCESS;

	}

	double CamCalibrator::computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors)
	{
		std::vector<cv::Point2f> imagePoints2;
		int i, totalPoints = 0;
		double totalErr = 0, err;
		perViewErrors.resize(objectPoints.size());

		for (i = 0; i < (int)objectPoints.size(); i++)
		{
			projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
				cameraMatrix, distCoeffs, imagePoints2);
			err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
			int n = (int)objectPoints[i].size();
			perViewErrors[i] = (float)std::sqrt(err * err / n);
			totalErr += err * err;
			totalPoints += n;
		}

		return std::sqrt(totalErr / totalPoints);
	}

	int CamCalibrator::extractPoints(cv::Mat& image, cv::Mat& mask, \
		std::vector<cv::Point2f>& extractedPoints, std::vector<cv::Mat>& imagesExtracted, int adaptiveThreshBlockSize, FIND_METHOD findMethod) {

		s.imageSize = image.size();

		std::vector<cv::Point2f> pointBuf;

		cv::Mat normalized;
		cv::normalize(image, normalized, 255, 0, cv::NORM_MINMAX, CV_8UC1, mask);
		cv::Mat invertedMask = 255 - mask;
		normalized = normalized + invertedMask;
		cv::Mat thresholded;
		cv::adaptiveThreshold(normalized, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, adaptiveThreshBlockSize, 2);
		cv::Mat thresholdBGRA;
		cv::cvtColor(thresholded, thresholdBGRA, cv::COLOR_GRAY2BGRA);
		imagesExtracted.push_back(thresholdBGRA);
		//cv::imshow(std::to_string(i), thresholded);
		//cv::waitKey(1);

		cv::Mat normalizedF;
		cv::normalize(image, normalizedF, 1.0, 0, cv::NORM_MINMAX, CV_32FC1, mask);

		bool found;

		switch (findMethod) {
		case 0:
			found = cv::findChessboardCorners(thresholded, s.boardSize, pointBuf, cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
			if (!found)
				return EXIT_FAILURE;
			cv::cornerSubPix(normalizedF, pointBuf, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 11, 0.1));
			break;

		case 1:
			found = cv::findChessboardCornersSB(thresholded, s.boardSize, pointBuf, cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
			break;

		case 2:
			//cornerSubPix
			break;

		default:
			found = cv::findChessboardCornersSB(thresholded, s.boardSize, pointBuf, cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
		}

		if (!found) {
			return EXIT_FAILURE;
		}

		cv::Mat imageBGRA;
		cv::cvtColor(normalized, imageBGRA, cv::COLOR_GRAY2BGRA);
		cv::drawChessboardCorners(imageBGRA, s.boardSize, cv::Mat(pointBuf), found);
		imagesExtracted.push_back(imageBGRA);
		//cv::imshow(std::to_string(i)+"e", imageBGRA);
		//cv::waitKey(1);


		if (pointBuf.size() != s.boardSize.height * s.boardSize.width)
			return EXIT_FAILURE;

		for (size_t k = 0; k < pointBuf.size(); k++)
		{
			extractedPoints.push_back(pointBuf[k]);
		}

		return EXIT_SUCCESS;
	}

	/*int CameraCalibrator::extractPointsMulti(std::vector< cv::Mat >& images, std::vector< cv::Mat >& masks, \
		int cbRows, int cbCols, std::vector< std::vector<cv::Point2f> >& extractedPoints, std::vector<cv::Mat>& imagesExtracted) {

		Settings s;
		s.boardSize = cv::Size(cbRows, cbCols);
		std::vector<cv::Point2f> pointBuf;
		bool found;
		for (size_t i = 0; i < images.size(); i++)
		{
			cv::Mat normalized;
			cv::normalize(images[i], normalized, 255, 0, cv::NORM_MINMAX, CV_8UC1, masks[i]);
			cv::Mat invertedMask = 255 - masks[i];
			normalized = normalized + invertedMask;
			cv::Mat thresholded;
			cv::adaptiveThreshold(normalized, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, 2);
			cv::Mat thresholdBGRA;
			cv::cvtColor(thresholded, thresholdBGRA, cv::COLOR_GRAY2BGRA);
			imagesExtracted.push_back(thresholdBGRA);
			//cv::imshow(std::to_string(i), thresholded);
			//cv::waitKey(1);

			found = cv::findChessboardCorners(thresholded, s.boardSize, pointBuf, cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
			if (!found) {
				found = cv::findChessboardCornersSB(thresholded, s.boardSize, pointBuf, cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
			}
			if (!found)
				return EXIT_FAILURE;
			cv::Mat normalizedF;
			cv::normalize(images[i], normalizedF, 1.0, 0, cv::NORM_MINMAX, CV_32FC1, masks[i]);
			cv::cornerSubPix(normalizedF, pointBuf, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 11, 0.1));

			if (found) {
				cv::Mat imageBGRA;
				cv::cvtColor(normalized, imageBGRA, cv::COLOR_GRAY2BGRA);
				cv::drawChessboardCorners(imageBGRA, s.boardSize, cv::Mat(pointBuf), found);
				imagesExtracted.push_back(imageBGRA);
				//cv::imshow(std::to_string(i)+"e", imageBGRA);
				//cv::waitKey(1);
			}
			extractedPoints.push_back(std::vector<cv::Point2f>());

			if (pointBuf.size() != cbRows * cbCols)
				return EXIT_FAILURE;
			for (size_t k = 0; k < pointBuf.size(); k++)
			{
				extractedPoints[i].push_back(pointBuf[k]);
			}
		}
		return EXIT_SUCCESS;
	}*/



	static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors)
	{
		std::vector<cv::Point2f> imagePoints2;
		int i, totalPoints = 0;
		double totalErr = 0, err;
		perViewErrors.resize(objectPoints.size());

		for (i = 0; i < (int)objectPoints.size(); ++i)
		{
			projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs, imagePoints2);
			err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);

			int n = (int)objectPoints[i].size();
			perViewErrors[i] = (float)std::sqrt(err * err / n);
			totalErr += err * err;
			totalPoints += n;
		}

		return std::sqrt(totalErr / totalPoints);
	}

	static bool runCalibration(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
		std::vector<float>& reprojErrs, double& totalAvgErr)
	{

		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		if (s.flag & cv::CALIB_FIX_ASPECT_RATIO)
			cameraMatrix.at<double>(0, 0) = 1.0;

		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

		std::vector<std::vector<cv::Point3f> > objectPoints(1);


		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		//Find intrinsic and extrinsic camera parameters
		double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs, s.flag | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);

		std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
			rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return ok;
	}

	// Print camera parameters to the output file
	static void saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
		double totalAvgErr)
	{
		cv::FileStorage fs(s.outputFileName, cv::FileStorage::WRITE);

		struct tm newtime;
		time_t now = time(0);
		localtime_s(&newtime, &now);
		char buf[1024];
		strftime(buf, sizeof(buf) - 1, "%c", &newtime);

		fs << "calibration_Time" << buf;

		if (!rvecs.empty() || !reprojErrs.empty())
			fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
		fs << "image_Width" << imageSize.width;
		fs << "image_Height" << imageSize.height;
		fs << "board_Width" << s.boardSize.width;
		fs << "board_Height" << s.boardSize.height;
		fs << "square_Size" << s.squareSize;

		if (s.flag & cv::CALIB_FIX_ASPECT_RATIO)
			fs << "FixAspectRatio" << s.aspectRatio;

		if (s.flag)
		{
			sprintf_s(buf, "flags: %s%s%s%s",
				s.flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
				s.flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
				s.flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
				s.flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
			//cvWriteComment( *fs, buf, 0 );
		}

		fs << "flagValue" << s.flag;

		fs << "Camera_Matrix" << cameraMatrix;
		fs << "Distortion_Coefficients" << distCoeffs;

		fs << "Avg_Reprojection_Error" << totalAvgErr;
		if (!reprojErrs.empty())
			fs << "Per_View_Reprojection_Errors" << cv::Mat(reprojErrs);

		if (!rvecs.empty() && !tvecs.empty())
		{
			CV_Assert(rvecs[0].type() == tvecs[0].type());
			cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
			for (int i = 0; i < (int)rvecs.size(); i++)
			{
				cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
				cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				r = rvecs[i].t();
				t = tvecs[i].t();
			}
			//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
			fs << "Extrinsic_Parameters" << bigmat;
		}

		if (!imagePoints.empty())
		{
			cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
			for (int i = 0; i < (int)imagePoints.size(); i++)
			{
				cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
				cv::Mat imgpti(imagePoints[i]);
				imgpti.copyTo(r);
			}
			fs << "Image_points" << imagePtMat;
		}
	}


	bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f> > imagePoints)
	{
		std::vector<cv::Mat> rvecs, tvecs;
		std::vector<float> reprojErrs;
		double totalAvgErr = 0;

		bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
			reprojErrs, totalAvgErr);
		std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
			<< ". avg re projection error = " << totalAvgErr;

		if (ok)
			saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs,
				imagePoints, totalAvgErr);
		return ok;
	}

}