/******************************************************************************
 * @brief Reversing State Implementation for Autonomy State Machine.
 *
 * @file ReversingState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef REVERSINGSTATE_H
#define REVERSINGSTATE_H

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
     * @brief The ReversingState class implements the Reversing state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ReversingState : public State
    {
        private:
            // TODO: Add fetch of current position from GPS
            geoops::GPSCoordinate stStartPosition = globals::g_pNavigationBoard->GetGPSData();
            double dCurrentHeading;
            bool m_bInitialized;
            const double dCurDistance       = 0;    // TODO: set constants?
            const double dDistanceThreshold = 5;

        protected:
            void Start() override;
            void Exit() override;

        public:
            ReversingState();
            States Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // REVERSINGSTATE_H
