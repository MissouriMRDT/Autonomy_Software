/******************************************************************************
 * @brief Test file for OpenCV functionality.
 *
 * @file opencv_test.hpp
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <opencv2/opencv.hpp>

#ifndef OPENCV_TEST
#	define OPENCV_TEST

/******************************************************************************
 * @brief Test multiple OpenCV functions and their output.
 *
 *
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 ******************************************************************************/
void TEST_OpenCV()
{
	// Test contrib libraries.
	cv::Mat markerImage;
	//// MEMORY LEAK ISSUES. PROBABLY HAS TO DO WITH OPENCV VERSIONS AND HOW WE COMPILED IT.

	// cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	// cv::aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
	// cv::imwrite("marker23.png", markerImage);
}

#endif	  // OPENCV_TEST
