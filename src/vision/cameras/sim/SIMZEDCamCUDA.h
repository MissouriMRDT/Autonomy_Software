#ifndef SIMZEDCAMCUDA_H
#define SIMZEDCAMCUDA_H

#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/mat.hpp>

void EstimateDepthMeasureCUDA(const cv::Mat& DepthImage, cv::Mat& DepthMeasure, float fMaxDepth);
void CalculatePointCloudCUDA(const cv::Mat& depthMeasure, cv::Mat& pointCloud, float fx, float fy, float cx, float cy);

#endif
