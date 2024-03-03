/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef APPROACHINGOBJECTSTATE_H
#define APPROACHINGOBJECTSTATE_H

#include "../interfaces/State.hpp"

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
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ApproachingObjectState : public State
    {
        private:
            int m_nNumDetectionAttempts;
            bool m_bInitialized;

        protected:
            void Start() override;
            void Exit() override;

        public:
            ApproachingObjectState();
            States Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // APPROACHINGOBJECTSTATE_H
