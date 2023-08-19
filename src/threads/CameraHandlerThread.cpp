/******************************************************************************
 * @brief Implements the CameraHandlerThread class.
 *
 * @file CameraHandlerThread.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "CameraHandlerThread.h"

/******************************************************************************
 * @brief Construct a new Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandlerThread::CameraHandlerThread()
{
    // Do nothing.
}

/******************************************************************************
 * @brief Destroy the Camera Handler Thread:: Camera Handler Thread object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
CameraHandlerThread::~CameraHandlerThread()
{
    // Do nothing.
}

/******************************************************************************
 * @brief This code is ran in a seperate thread and mostly handles camera classes
 *      and threadpool management.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
void CameraHandlerThread::ThreadedContinuousCode()
{
    //
}

/******************************************************************************
 * @brief This code is highly parallelizable and grabs a frame from
 *
 * @return cv::Mat -
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
void CameraHandlerThread::PooledLinearCode()
{
    //
}
