#include "../../../AutonomyLogging.h"
#include "SIMZEDCamCUDA.h"

__global__ void EstimateDepthMeasureKernel(cv::cuda::PtrStepSz<uchar1> cvDepthImage, cv::cuda::PtrStepSz<float> cvDepthMeasure, float fMaxDepth, float fFarClipDepth)
{
    int nX = blockIdx.x * blockDim.x + threadIdx.x;
    int nY = blockIdx.y * blockDim.y + threadIdx.y;
    if (nX < cvDepthImage.cols && nY < cvDepthImage.rows)
    {
        // For this, we are just using the depth image to estimate the depth measure. We will treat 255 as 0 cm and 0 as fMaxDepth.
        // Get the depth value from the depth image.
        uchar1 ucDepthValue = cvDepthImage(nY, nX);

        // Calculate the depth in cm. No-return pixels become 0, which consumers treat as invalid.
        float fDepth           = (1.0f - (ucDepthValue.x / 255.0f)) * fMaxDepth;
        cvDepthMeasure(nY, nX) = fDepth >= fFarClipDepth ? 0.0f : fDepth / 100.0f;    // Convert cm to m.
    }
}

__global__ void CalculatePointCloudKernel(cv::cuda::PtrStepSz<float> depthMeasure,
                                          cv::cuda::PtrStepSz<float4> cvPointCloud,
                                          float fFovX,
                                          float fFovY,
                                          float fCenterX,
                                          float fCenterY)
{
    int nX = blockIdx.x * blockDim.x + threadIdx.x;
    int nY = blockIdx.y * blockDim.y + threadIdx.y;
    if (nX < depthMeasure.cols && nY < depthMeasure.rows)
    {
        float fDepth = depthMeasure(nY, nX);
        if (fDepth > 0)
        {
            float fZ = fDepth;    // Convert back to depth
            float fX = (nX - fCenterX) * fDepth / fFovX;
            // Image rows grow downward, but Y is up to match ZED_COORD_SYSTEM (LEFT_HANDED_Y_UP) and the CPU path.
            float fY             = (fCenterY - nY) * fDepth / fFovY;
            cvPointCloud(nY, nX) = float4(fX, fY, fZ, 255.0f);
        }
        else
        {
            cvPointCloud(nY, nX) = float4(0, 0, 0, 0);    // or some other value indicating invalid point
        }
    }
}

void EstimateDepthMeasureCUDA(const cv::Mat& cvDepthImage, cv::Mat& cvDepthMeasure, float fMaxDepth, float fFarClipDepth)
{
    cv::cuda::GpuMat cvGpuDepthImage, cvGpuDepthMeasure;
    cvGpuDepthImage.upload(cvDepthImage);
    cvGpuDepthMeasure.create(cvDepthImage.size(), CV_32FC1);

    dim3 blockSize(16, 16);
    dim3 gridSize((cvGpuDepthImage.cols + blockSize.x - 1) / blockSize.x, (cvGpuDepthImage.rows + blockSize.y - 1) / blockSize.y);
    EstimateDepthMeasureKernel<<<gridSize, blockSize>>>(cvGpuDepthImage, cvGpuDepthMeasure, fMaxDepth, fFarClipDepth);
    cudaDeviceSynchronize();

    cvGpuDepthMeasure.download(cvDepthMeasure);

    cudaError_t cvErr = cudaGetLastError();
    if (cvErr != cudaSuccess)
    {
        LOG_WARNING(logging::g_qSharedLogger, "CUDA error in EstimateDepthMeasureCUDA: {}", cudaGetErrorString(cvErr));
    }
}

void CalculatePointCloudCUDA(const cv::Mat& cvDepthMeasure, cv::Mat& cvPointCloud, float fFovX, float fFovY, float fCenterX, float fCenterY)
{
    cv::cuda::GpuMat cvGpuDepthMeasure, cvGpuPointCloud;
    cvGpuDepthMeasure.upload(cvDepthMeasure);
    cvGpuPointCloud.create(cvDepthMeasure.size(), CV_32FC4);

    dim3 blockSize(16, 16);
    dim3 gridSize((cvGpuDepthMeasure.cols + blockSize.x - 1) / blockSize.x, (cvGpuDepthMeasure.rows + blockSize.y - 1) / blockSize.y);
    CalculatePointCloudKernel<<<gridSize, blockSize>>>(cvGpuDepthMeasure, cvGpuPointCloud, fFovX, fFovY, fCenterX, fCenterY);
    cudaDeviceSynchronize();

    cvGpuPointCloud.download(cvPointCloud);

    cudaError_t cvErr = cudaGetLastError();
    if (cvErr != cudaSuccess)
    {
        LOG_WARNING(logging::g_qSharedLogger, "CUDA error in CalculatePointCloudCUDA: {}", cudaGetErrorString(cvErr));
    }
}
