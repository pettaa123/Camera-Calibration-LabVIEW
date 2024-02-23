#pragma once

#include <opencv2/core.hpp>

namespace calib {

    struct Settings
    {
        cv::Size boardSize;            // The size of the board -> Number of items by width and height
        float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
        int nrFrames;              // The number of frames to use from the input for calibration
        float aspectRatio;         // The aspect ratio              // In case of a video input
        bool bwritePoints;         //  Write detected feature points
        bool bwriteExtrinsics;     // Write extrinsic parameters
        bool calibZeroTangentDist; // Assume zero tangential distortion
        bool calibFixPrincipalPoint;// Fix the principal point at the center
        bool flipVertical;          // Flip the captured images around the horizontal axis
        std::string outputFileName;      // The name of the file where to write
        bool showUndistorsed;       // Show undistorted images after calibration

        std::vector<std::string> imageList;
        int atImageList;
        int flag;
    };

}