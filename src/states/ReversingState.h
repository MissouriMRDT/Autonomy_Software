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
     * @brief The ReversingState class implements the Reversing state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ReversingState : public State
    {
        private:
            // LEAD: @ryanw Don't initialize values in a header file. This won't even compile.
            geoops::GPSCoordinate stStartPosition;
            double dCurrentHeading;
            bool m_bInitialized;
            // LEAD: Don't initialize values in a header file.
            // FIXME: Move these to constants. Use the same formatting as the other constants sections. There is already a state machine section.
            const double dCurDistance       = 0;    // TODO: set constants?
            const double dDistanceThreshold = 5;

        protected:
            void Start() override;
            void Exit() override;

        public:
            ReversingState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // REVERSINGSTATE_H
