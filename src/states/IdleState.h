/******************************************************************************
 * @brief Idle State Implementation for Autonomy State Machine.
 *
 * @file IdleState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/logging/PathTracer.hpp"
#include "../vision/aruco/TagDetector.h"

/// \cond
#include <tuple>

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
     * @brief The IdleState class implements the Idle state for the Autonomy State
     *        Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class IdleState : public State
    {
        private:
            geoops::RoverPose m_stStartRoverPose;
            bool m_bInitialized;
            std::vector<std::shared_ptr<TagDetector>> m_vTagDetectors;

        protected:
            void Start() override;
            void Exit() override;

        public:
            IdleState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // IDLESTATE_H
