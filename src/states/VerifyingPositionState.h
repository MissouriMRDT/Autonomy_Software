/******************************************************************************
 * @brief Verifying Position State Implementation for Autonomy State Machine.
 *
 * @file VerifyingPositionState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-24
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef VERIFYING_POSITION_STATE_H
#define VERIFYING_POSITION_STATE_H

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
     * @brief The VerifyingPositionState class implements the Verifying Position state for
     *        the Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-05-24
     ******************************************************************************/
    class VerifyingPositionState : public State
    {
        private:
            std::vector<geoops::GPSCoordinate> m_vCheckPoints;
            int m_nMaxDataPoints;
            bool m_bInitialized;

        protected:
            void Start() override;
            void Exit() override;

        public:
            VerifyingPositionState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // VERIFYING_POSITION_STATE_H
