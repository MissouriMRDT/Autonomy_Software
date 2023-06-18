/******************************************************************************
 * @brief Implements the interface for sending commands to the drive board on
 * 		the Rover.
 *
 * @file DriveBoard.cpp
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0618
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/
#include "DriveBoard.h"

#include "../Autonomy_Globals.h"
#include "../util/NumberOperations.h"

/******************************************************************************
 * @brief Construct a new Drive Board:: Drive Board object.
 *
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0618
 ******************************************************************************/
DriveBoard::DriveBoard()
{
	m_iTargetSpeedLeft	= 0;
	m_iTargetSpeedRight = 0;
}

/******************************************************************************
 * @brief Destroy the Drive Board:: Drive Board object.
 *
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0618
 ******************************************************************************/
DriveBoard::~DriveBoard() {}

/******************************************************************************
 * @brief This method determines drive powers to make the Rover drive towards a
 * 		given heading at a given speed
 *
 * @param fSpeed - The speed to drive at (-1 to 1)
 * @param fAngle - The angle to drive towards.
 * @return std::vector<int> - 1D vector with two values. (left power, right power)
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0618
 ******************************************************************************/
std::vector<int> DriveBoard::CalculateMove(float fSpeed, float fAngle)
{
	double dSpeedLeft  = fSpeed;
	double dSpeedRight = fSpeed;

	if (fAngle > 0) { dSpeedRight = dSpeedRight * (1 - (fAngle / 180.0)); }
	else if (fAngle < 0) { dSpeedLeft = dSpeedLeft * (1 + (fAngle / 180.0)); }

	m_iTargetSpeedLeft	= int(numops::Clamp<double>(dSpeedLeft, constants::MIN_DRIVE_POWER, constants::MAX_DRIVE_POWER));
	m_iTargetSpeedRight = int(numops::Clamp<double>(dSpeedRight, constants::MIN_DRIVE_POWER, constants::MAX_DRIVE_POWER));

	PLOG_DEBUG_(AL_ConsoleLogger) << "Driving at (" << m_iTargetSpeedLeft << ", " << m_iTargetSpeedRight << ")";

	return {m_iTargetSpeedLeft, m_iTargetSpeedRight};
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param nLeftTarget - Left drive speed (-1 to 1)
 * @param nRightTarget - Right drive speed (-1 to 1)
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0618
 ******************************************************************************/
void DriveBoard::SendDrive(int nLeftTarget, int nRightTarget) {}

/******************************************************************************
 * @brief Stop the drivetrain of the Rover.
 *
 *
 * @author Byrdman32 (eli@byrdneststudios.com)
 * @date 2023-0618
 ******************************************************************************/
void DriveBoard::SendStop() {}
