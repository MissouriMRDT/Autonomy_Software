/******************************************************************************
 * @brief Verifying Object State Implementation for Autonomy State Machine.
 *
 * @file VerifyingObjectState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYING_OBJECT_STATE_H
#define VERIFYING_OBJECT_STATE_H

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
     * @brief The VerifyingObjectState class implements the Verifying Object state for
     *        the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class VerifyingObjectState : public State
    {
        private:
            std::vector<int> m_vObjectIDs;
            int m_nMaxObjectIDs;
            bool m_bInitialized;

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
