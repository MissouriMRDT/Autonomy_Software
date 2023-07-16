/******************************************************************************
 * @brief Defines the OpenCV ArUco Generator
 *
 * @file TagGenerator.hpp
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0709
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <opencv2/opencv.hpp>
#include <string>

/******************************************************************************
 * @brief Generate an ArUco Tag
 *
 * @param eDictionary - The OpenCV Dictonary to pull from
 * @param sMarker - The id of the marker to generate
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0709
 ******************************************************************************/
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
    cv::aruco::generateImageMarker(cvDictionary, sMarker, 200, cvMarkerImage, 1);
    cv::imwrite(szMarkerFilename.c_str(), cvMarkerImage);
}
