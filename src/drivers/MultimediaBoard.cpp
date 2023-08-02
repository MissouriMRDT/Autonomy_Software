/******************************************************************************
 * @brief Implements the MultimediaBoard class.
 *
 * @file MultimediaBoard.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "MultimediaBoard.h"

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Construct a new Multimedia Board:: Multimedia Board object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
MultimediaBoard::MultimediaBoard() {}

/******************************************************************************
 * @brief Destroy the Multimedia Board:: Multimedia Board object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
MultimediaBoard::~MultimediaBoard() {}

/******************************************************************************
 * @brief Sends a predetermined color pattern to board.
 *
 * @param eState - The lighting state. Enum defined in header file for
 * 					MultimediaBoard.h
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
void MultimediaBoard::SendLightingState(MultimediaBoardLightingState eState) {}

/******************************************************************************
 * @brief Send a custom RGB value to the board.
 *
 * @param rgbVal - RGB struct containing color information. Struct defined in
 * 					MultimediaBoard.h
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 ******************************************************************************/
void MultimediaBoard::SendRGB(RGB rgbVal) {}
