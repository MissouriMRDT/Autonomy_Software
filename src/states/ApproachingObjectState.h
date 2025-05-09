/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.h
 * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef APPROACHING_OBJECT_STATE_H
#define APPROACHING_OBJECT_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/logging/PathTracer2D.hpp"
#include "../util/states/StuckDetection.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The ApproachingObjectState class implements the Approaching Object
     *        state for the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    class ApproachingObjectState : public State
    {
        private:
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;
            States m_eTriggeringState;
            bool m_bInitialized;
            geoops::Waypoint m_stGoalWaypoint;
            std::unique_ptr<logging::graphing::PathTracer> m_pRoverPathPlot;
            int m_nNumDetectionAttempts;

        protected:
            void Start() override;
            void Exit() override;

        public:
            ApproachingObjectState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHINGOBJECTSTATE_H
