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

            enum AttemptType
            {
                eReverseCurrentHeading,
                eReverseLeft,
                eReverseRight,
                eGiveUp
            };

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            bool m_bInitialized;
            States m_eTriggeringState;                                   // The state that the rover got stuck in before triggering a stuck event.
            AttemptType m_eAttemptType;                                  // Current attempt we are on for a given position.
            geoops::GPSCoordinate m_stOriginalPosition;                  // Original position where rover was reported stuck.
            double m_dOriginalHeading;                                   // Original heading the rover was at when reported stuck.
            bool m_bIsCurrentlyAligning;                                 // Is the rover currently trying to align with a target heading.
            std::chrono::system_clock::time_point m_tmStuckStartTime;    // The timestamp storing when the rover starting stuck state.
            std::chrono::system_clock::time_point m_tmAlignStartTime;    // The timestamp storing when the rover starting realigning.

            /////////////////////////////////////////
            // Declare private class methods.
            /////////////////////////////////////////
            bool SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition);

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
    };
}    // namespace statemachine

#endif    // STUCKSTATE_H
