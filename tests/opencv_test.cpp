/*
   opencv_test.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:            6/18/2023
   Author:		    Eli Byrd and Clayton Cowen
   Description:	    Test for the OpenCV Module
*/

#include "opencv_test.h"

void TEST_OpenCV() {
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
    cv::imwrite("marker23.png", markerImage);
}