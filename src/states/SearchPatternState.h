/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef SEARCHPATTERNSTATE_H
#define SEARCHPATTERNSTATE_H

#include "../algorithms/DifferentialDrive.hpp"
#include "../algorithms/SearchPattern.hpp"
#include "../interfaces/State.hpp"
#include "../vision/aruco/TagDetector.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The SearchPatternState class implements the Search Pattern state for the
     *        Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class SearchPatternState : public State
    {
        private:
            int m_nMaxDataPoints;
            std::vector<double> m_vRoverXPosition;
            std::vector<double> m_vRoverYPosition;
            time_t m_tStuckCheckTime;
            double m_dStuckCheckLastPosition[2];
            bool m_bInitialized;

            std::vector<TagDetector*> m_vTagDetectors;    // Vector of tag detectors to use for detection in order of highest to lowest priority.

            std::vector<WaypointHandler::Waypoint> m_vSearchPath;
            int m_nSearchPathIdx;

        protected:
            void Start() override;
            void Exit() override;

        public:
            SearchPatternState();
            States Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // SEARCHPATTERNSTATE_H
