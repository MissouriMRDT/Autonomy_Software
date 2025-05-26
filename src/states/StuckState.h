/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine.
 *
 * @file StuckState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STUCK_STATE_H
#define STUCK_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"

/// \cond
#include <chrono>

/// \endcond

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
            /////////////////////////////////////////
            // Declare private enums and structs that are specific to and used withing this class.
            /////////////////////////////////////////

            enum class AttemptType
            {
                eReverseCurrentHeading,
                eReverseLeft,
                eReverseRight,
                eGiveUp
            };

            enum class StuckLeg
            {
                eUnsticking,
                eGoingRight,
                eGoingLeft,
                eReturning
            };

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            bool m_bInitialized;
            StuckLeg m_eStuckLeg;                                        // Current stage of attempting to drive around an obstacle.
            geoops::GPSCoordinate m_stOriginalPosition;                  // Original position where rover was first reported stuck.
            double m_dOriginalHeading;                                   // Original heading the rover was at when first reported stuck.
            geoops::GPSCoordinate m_stObstaclePosition;                  // Position where rover was reported stuck for this StuckState run.
            double m_dObstacleHeading;                                   // Heading the rover was at when reported stuck for this StuckState run.
            geoops::GPSCoordinate m_stHomePosition;                      // Position the rover ends up after getting unstuck for the first time.
            statemachine::States m_eTriggeringState;                     // State that originally triggered StuckState.
            bool m_bReachedGoal;                                         // Whether to return to m_eTriggeringState.
            std::vector<geoops::Waypoint> m_vRightPath, m_vLeftPath;     // Routes for the right and left legs.
            std::chrono::system_clock::time_point m_tmStuckStartTime;    // The timestamp storing when the rover started this StuckState run.

            // Unsticking state variables.
            AttemptType m_eAttemptType;                                  // Current attempt we are on for a given position.
            bool m_bIsCurrentlyAligning;                                 // Is the rover currently trying to align with a target heading.
            std::chrono::system_clock::time_point m_tmAlignStartTime;    // The timestamp storing when the rover starting realigning.

            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////
            bool SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition);
            void TryUnsticking();
            void GeneratePaths();

        protected:
            /////////////////////////////////////////
            // Declare protected class methods.
            /////////////////////////////////////////
            void Start() override;
            void Exit() override;

        public:
            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////
            StuckState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;

            static bool IsStuckWaypoint(int nID);
            static bool IsStuckWaypointGoal(int nID);
    };
}    // namespace statemachine

#endif    // STUCKSTATE_H
