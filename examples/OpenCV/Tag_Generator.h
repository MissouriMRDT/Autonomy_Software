/*
   Tag_Generator.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:            6/18/2023
   Author:          Eli Byrd and Clayton Cowen
   Description:     Generates an Aruco Tag of your choice and saves to the location of your executable
*/

#include <opencv2/opencv.hpp>
#include <string>

#ifndef TAG_GENERATOR_H
#define TAG_GENERATOR_H

/**
 *
 * @param eDictionary
 * @param sMarker
 */
void GenerateOpenCVArucoMarker(cv::aruco::PredefinedDictionaryType eDictionary, unsigned short sMarker);

#endif
