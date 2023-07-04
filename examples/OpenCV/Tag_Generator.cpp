/*
   Tag_Generator.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:            6/18/2023
   Author:          Eli Byrd and Clayton Cowen
   Description:     Generates an Aruco Tag of your choice and saves to the location of your executable
*/

#include "Tag_Generator.h"

void GenerateOpenCVArucoMarker(cv::aruco::PredefinedDictionaryType eDictionary, unsigned short sMarker)
{
    switch (eDictionary)
    {
        case cv::aruco::DICT_4X4_50:
        case cv::aruco::DICT_5X5_50:
        case cv::aruco::DICT_6X6_50:
        case cv::aruco::DICT_7X7_50:
            if (sMarker > 49)
            {
                return;
            }
            break;
        case cv::aruco::DICT_4X4_100:
        case cv::aruco::DICT_5X5_100:
        case cv::aruco::DICT_6X6_100:
        case cv::aruco::DICT_7X7_100:
            if (sMarker > 99)
            {
                return;
            }
            break;
        case cv::aruco::DICT_4X4_250:
        case cv::aruco::DICT_5X5_250:
        case cv::aruco::DICT_6X6_250:
        case cv::aruco::DICT_7X7_250:
            if (sMarker > 249)
            {
                return;
            }
            break;
        case cv::aruco::DICT_4X4_1000:
        case cv::aruco::DICT_5X5_1000:
        case cv::aruco::DICT_6X6_1000:
        case cv::aruco::DICT_7X7_1000:
            if (sMarker > 999)
            {
                return;
            }
            break;
        default: break;
    }

    std::string szMarkerFilename = "marker";
    szMarkerFilename += std::to_string(sMarker);
    szMarkerFilename += ".png";

    cv::Mat cvMarkerImage;
    cv::aruco::Dictionary cvDictionary = cv::aruco::getPredefinedDictionary(eDictionary);
    // cv::aruco::generateImageMarker(cvDictionary, sMarker, 200, cvMarkerImage, 1);
    // cv::imwrite(szMarkerFilename.c_str(), cvMarkerImage);
}
