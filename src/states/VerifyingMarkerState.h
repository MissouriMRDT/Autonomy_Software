/******************************************************************************
 * @brief Verifying Marker State Implementation for Autonomy State Machine.
 *
 * @file VerifyingMarkerState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYINGMARKERSTATE_H
#define VERIFYINGMARKERSTATE_H

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
     * @brief The VerifyingMarkerState class implements the Verifying Marker state for
     *        the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class VerifyingMarkerState : public State
    {
        private:
            std::vector<int> m_vMarkerIDs;
            int m_nMaxMarkerIDs;
            bool m_bInitialized;

        protected:
            void Start() override;
            void Exit() override;

        public:
            VerifyingMarkerState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // VERIFYINGMARKERSTATE_H
