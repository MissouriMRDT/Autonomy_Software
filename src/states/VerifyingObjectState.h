/******************************************************************************
 * @brief Verifying Object State Implementation for Autonomy State Machine.
 *
 * @file VerifyingObjectState.h
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYING_OBJECT_STATE_H
#define VERIFYING_OBJECT_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../vision/objects/ObjectDetector.h"
#include "../vision/objects/ObjectDetectionHandler.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The VerifyingObjectState class implements the Verifying Object state for
     *        the Autonomy State Machine.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    class VerifyingObjectState : public State
    {
        private:
            bool m_bInitialized;
            geoops::Waypoint m_stGoalWaypoint;
            ObjectDetectionHandler::ObjectDetectors m_eWinningDetector;
            std::vector<std::shared_ptr<ObjectDetector>> m_vObjectDetectors;
            std::chrono::system_clock::time_point m_tmObjectVerificationStartTime;
            std::chrono::system_clock::time_point m_tmObjectLastSeenTime;

        protected:
            void Start() override;
            void Exit() override;

        public:
            VerifyingObjectState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // VERIFYINGOBJECTSTATE_H
