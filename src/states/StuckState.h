/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine.
 *
 * @file StuckState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STUCKSTATE_H
#define STUCKSTATE_H

#include "../algorithms/DifferentialDrive.hpp"
#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"

#include <chrono>

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The StuckState class implements the Stuck state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    class StuckState : public State
    {
        private:
            bool m_bInitialized;

            unsigned int m_unAttempts;                                   // Current attempt we are on for a given position.
            geoops::GPSCoordinate m_stOriginalPosition;                  // Original position where rover was reported stuck.
            double m_dOriginalHeading;                                   // Original heading the rover was at when reported stuck.
            bool m_bIsCurrentlyAligning;                                 // Is the rover currently trying to align with a target heading.

            std::chrono::system_clock::time_point m_tmLastStuckCheck;    // Time since the rover was last checked for being stuck.
            unsigned int m_unStuckChecksOnAttempt;                       //

            bool SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition);

        protected:
            void Start() override;
            void Exit() override;

        public:
            StuckState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // STUCKSTATE_H
