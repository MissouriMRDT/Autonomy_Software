/******************************************************************************
 * @brief Defines the NavigationBoard class.
 *
 * @file NavigationBoard.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-20
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef NAVIGATIONBOARD_H
#define NAVIGATIONBOARD_H

#include <chrono>
#include <ctime>

struct NavBoardPacket_IMU
{
        double dPitch;
        double dRoll;
        double dHeading;

        NavBoardPacket_IMU()
        {
            dPitch   = 0.0;
            dRoll    = 0.0;
            dHeading = 0.0;
        }

        NavBoardPacket_IMU(double dPitch, double dRoll, double dHeading)
        {
            this->dPitch   = dPitch;
            this->dRoll    = dRoll;
            this->dHeading = dHeading;
        }
};

struct NavBoardPacket_GPS
{
        double dLatitude;
        double dLongitude;

        NavBoardPacket_GPS()
        {
            dLatitude  = 0.0;
            dLongitude = 0.0;
        }

        NavBoardPacket_GPS(double dLatitude, double dLongitude)
        {
            this->dLatitude  = dLatitude;
            this->dLongitude = dLongitude;
        }
};

enum NavigationBoardPacketDoubleComponents
{
    NBPC_PITCH,
    NBPC_ROLL,
    NBPC_HEADING
};

enum NavigationBoardPacketCoordinateComponents
{
    NBPCC_LOCATION
};

class NavigationBoard
{
    private:
        int m_iDistanceToGround;
        int m_iLidarQuality;

        double m_dPitch;
        double m_dRoll;
        double m_dHeading;

        time_t m_tLastTime;

        NavBoardPacket_GPS m_sLocation;

        // TODO: RoveComm Node

        // TODO: RoveComm Callback -> IMU Data
        // TODO: RoveComm Callback -> GPS Data

    public:
        NavigationBoard();
        ~NavigationBoard();

        void ProcessIMUData(NavBoardPacket_IMU packet);
        void ProcessGPSData(NavBoardPacket_GPS packet);

        double GetDData(NavigationBoardPacketDoubleComponents eKey) const;
        NavBoardPacket_GPS GetSData(NavigationBoardPacketCoordinateComponents eKey) const;
};

#endif    // NAVIGATIONBOARD_H
