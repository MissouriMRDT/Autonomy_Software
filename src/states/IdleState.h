/******************************************************************************
 * @brief Idle State Implementation for Autonomy State Machine.
 *
 * @file IdleState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef IDLESTATE_H
#define IDLESTATE_H

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
     * @brief The IdleState class implements the Idle state for the Autonomy State
     *        Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class IdleState : public State
    {
        private:
            time_t m_tIdleTime;
            bool m_bRealigned;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            int m_nMaxDataPoints;
            bool m_bInitialized;

        protected:
            void Start() override;
            void Exit() override;

        public:
            IdleState();
            States Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // IDLESTATE_H