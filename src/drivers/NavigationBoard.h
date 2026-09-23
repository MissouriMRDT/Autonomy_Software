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

#include "../util/GeospatialOperations.hpp"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <chrono>
#include <atomic>
#include <shared_mutex>

/// \endcond

/******************************************************************************
 * @brief This class handles communication with the navigation board on the rover
 *      by sending RoveComm packets over the network.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
class NavigationBoard
{
    public:
        /////////////////////////////////////////
        // Declare public enums and structs that are specific to and used withing this class.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        NavigationBoard();
        ~NavigationBoard();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        geoops::GPSCoordinate GetGPSData();
        geoops::UTMCoordinate GetUTMData();
        double GetHeading();
        double GetHeadingAccuracy();
        double GetVelocity();
        double GetAngularVelocity();
        std::chrono::system_clock::duration GetGPSLastUpdateTime();
        std::chrono::system_clock::duration GetCompassLastUpdateTime();
        bool IsOutOfDate();

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        // LOCKING RULES FOR THIS CLASS.
        //
        // Every member below names the ONE mutex that guards it, and nothing may be read or
        // written without holding that mutex. Two rules follow from how this class is used,
        // and both were being broken before:
        //
        //   1. Never take the same std::shared_mutex twice on one thread. It is not
        //      recursive, so a second acquisition is undefined behaviour - it happens to
        //      work under glibc's reader-preferring rwlock and deadlocks under a
        //      writer-preferring one. The public accessors used to take a lock and then call
        //      another public accessor that took the same lock again. Anything that needs
        //      data-age arithmetic from inside a critical section calls the *Locked() helper
        //      below, which assumes the lock is already held.
        //
        //   2. A member is guarded by ITS OWN mutex, not by whichever one happens to be held.
        //      m_tmLastGPSUpdateTime belongs to m_muLocationMutex even when the code reading
        //      it is in the middle of updating velocity.
        //
        // Lock ordering, where two are genuinely needed at once: Location -> Heading. Nothing
        // else nests, and no writer holds two at once except ProcessAccuracyData().

        geoops::GPSCoordinate m_stLocation;                                 // Guarded by m_muLocationMutex.
        double m_dHeading;                                                  // Guarded by m_muHeadingMutex.
        double m_dHeadingAccuracy;                                          // Guarded by m_muHeadingMutex.
        double m_dVelocity;                                                 // Guarded by m_muVelocityMutex.
        double m_dAngularVelocity;                                          // Guarded by m_muAngularVelocityMutex.
        std::shared_mutex m_muLocationMutex;                                // Guards m_stLocation and m_tmLastGPSUpdateTime.
        std::shared_mutex m_muHeadingMutex;                                 // Guards m_dHeading, m_dHeadingAccuracy and m_tmLastCompassUpdateTime.
        std::shared_mutex m_muVelocityMutex;                                // Guards m_dVelocity.
        std::shared_mutex m_muAngularVelocityMutex;                         // Guards m_dAngularVelocity.
        std::chrono::system_clock::time_point m_tmLastGPSUpdateTime;        // Guarded by m_muLocationMutex. Timestamp of the last GPS update.
        std::chrono::system_clock::time_point m_tmLastCompassUpdateTime;    // Guarded by m_muHeadingMutex. Timestamp of the last compass update.
        // Written from accessors holding three different mutexes, so no single mutex could
        // ever have guarded it. Atomic instead: it is one independent boolean, and every
        // writer is simply publishing "the data I just looked at was stale".
        std::atomic<bool> m_bNavBoardOutOfDate{false};

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        std::chrono::system_clock::duration GetGPSLastUpdateTimeLocked() const;
        std::chrono::system_clock::duration GetCompassLastUpdateTimeLocked() const;
        void ProcessGPSData(const rovecomm::RoveCommPacket<double>& stPacket);
        void ProcessAccuracyData(const rovecomm::RoveCommPacket<float>& stPacket);
        void ProcessCompassData(const rovecomm::RoveCommPacket<float>& stPacket);
};

#endif    // NAVIGATIONBOARD_H
