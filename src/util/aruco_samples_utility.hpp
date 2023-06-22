/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <ctime>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

namespace
{
    inline static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
            return false;
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        return true;
    }

    inline static bool saveCameraParams(const std::string& filename, cv::Size imageSize, float aspectRatio, int flags, const cv::Mat& cameraMatrix,
                                        const cv::Mat& distCoeffs, double totalAvgErr)
    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened())
            return false;

        time_t tt;
        time(&tt);
        struct tm* t2 = localtime(&tt);
        char buf[1024];
        strftime(buf, sizeof(buf) - 1, "%c", t2);

        fs << "calibration_time" << buf;
        fs << "image_width" << imageSize.width;
        fs << "image_height" << imageSize.height;

        if (flags & cv::CALIB_FIX_ASPECT_RATIO)
            fs << "aspectRatio" << aspectRatio;

        if (flags != 0) {
            sprintf(buf, "flags: %s%s%s%s", flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                    flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "", flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                    flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        }
        fs << "flags" << flags;
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs << "avg_reprojection_error" << totalAvgErr;
        return true;
    }

}  // namespace
