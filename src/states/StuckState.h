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

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"

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
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class StuckState : public State
    {
        private:
            time_t m_tStuckCheckTime;
            bool m_bInitialized;

            unsigned int m_unAttempts;                     // Current attempt we are on for a given position.
            geoops::GPSCoordinate m_stOriginalPosition;    // Original position where rover was reported stuck.
            double m_dOriginalHeading;                     // Original heading the rover was at when reported stuck.

            double m_dHeadingTolerance;                    // How close the current heading must be to the target heading to be considered aligned.
            double m_dInplaceRotationMotorPower;           // Power on left and right motors when rotating the rotor.
            double m_dSamePositionThreshold;               // Distance threshold determining if we are still in the same position.

            bool m_bIsCurrentlyAligning;

            bool SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition);

        protected:
            void Start() override;
            void Exit() override;

        public:
            StuckState();
            States Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // STUCKSTATE_H
