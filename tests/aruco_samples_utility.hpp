#include <ctime>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

namespace	 // This namespace is anonymous and only accessible within this file. (hence no name)
{
	/******************************************************************************
	 * @brief Reads camera params given info about the camera and its precalculated
	 * 		camera matrix and distortion coefficients. Stores it in given mats.
	 *
	 * @param filename - String containing path to calibration file.
	 * @param camMatrix - Mat for storing cam matrix.
	 * @param distCoeffs - Mat for storing distortion coefficients.
	 * @return true - Config properly read and stored.
	 * @return false - Config was unable to be read.
	 *
	 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
	 * @date 2023-0620
	 ******************************************************************************/
	inline static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs)
	{
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if (!fs.isOpened()) return false;
		fs["camera_matrix"] >> camMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
		return true;
	}

	/******************************************************************************
	 * @brief Given a bunch of camera info, this function will write it to a
	 * 		file at the given path.
	 *
	 * @param filename - Path to write file to.
	 * @param imageSize - The resolution of the camera.
	 * @param aspectRatio - Aspect ration of the camera.
	 * @param flags - Special flags camera was intialized with.
	 * @param cameraMatrix - Calculated camera matrix.
	 * @param distCoeffs - Calculated distortion coefficients.
	 * @param totalAvgErr - Accuracy of calculated values.
	 * @return true - Config was written succesfully.
	 * @return false - Config was unable to be written.
	 *
	 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
	 * @date 2023-0620
	 ******************************************************************************/
	inline static bool saveCameraParams(const std::string &filename,
										cv::Size imageSize,
										float aspectRatio,
										int flags,
										const cv::Mat &cameraMatrix,
										const cv::Mat &distCoeffs,
										double totalAvgErr)
	{
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
		if (!fs.isOpened()) return false;

		time_t tt;
		time(&tt);
		struct tm *t2 = localtime(&tt);
		char buf[1024];
		strftime(buf, sizeof(buf) - 1, "%c", t2);

		fs << "calibration_time" << buf;
		fs << "image_width" << imageSize.width;
		fs << "image_height" << imageSize.height;

		if (flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

		if (flags != 0)
		{
			sprintf(buf,
					"flags: %s%s%s%s",
					flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
					flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
					flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
					flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		}
		fs << "flags" << flags;
		fs << "camera_matrix" << cameraMatrix;
		fs << "distortion_coefficients" << distCoeffs;
		fs << "avg_reprojection_error" << totalAvgErr;
		return true;
	}

}	 // namespace
