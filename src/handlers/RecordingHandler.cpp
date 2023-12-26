/******************************************************************************
 * @brief Implements the RecordingHandler class.
 *
 * @file RecordingHandler.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "RecordingHandler.h"
#include "../AutonomyConstants.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new Recording Handler:: Recording Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
RecordingHandler::RecordingHandler()
{
    // Initialize member variables.
    m_nRecordingFPS = constants::GLOBAL_RECORDING_FPS;
}

/******************************************************************************
 * @brief Destroy the Recording Handler:: Recording Handler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
RecordingHandler::~RecordingHandler()
{
    // Nothing to destroy.
}

/******************************************************************************
 * @brief This code will run continuously in a separate thread. New frames from
 *      the cameras that have recording enabled are grabbed and the images are
 *      written to the filesystem.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
void RecordingHandler::ThreadedContinuousCode() {}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It currently does nothing and is not
 *      needed in the current implementation of the RecordingHandler.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-12-26
 ******************************************************************************/
void RecordingHandler::PooledLinearCode() {}
