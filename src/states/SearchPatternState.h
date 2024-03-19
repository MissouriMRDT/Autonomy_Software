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

#include <utility>

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
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
            bool m_bInitialized;
            size_t m_nMaxDataPoints;
            std::vector<std::pair<double, double>> m_vRoverPosition;
            std::chrono::system_clock::time_point m_tmLastStuckCheck;
            unsigned int m_unStuckChecksOnAttempt;
            std::vector<TagDetector*> m_vTagDetectors;
            int m_nSearchPathIdx;
            std::vector<geoops::Waypoint> m_vSearchPath;

        protected:
            void Start() override;
            void Exit() override;

        public:
            SearchPatternState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // SEARCHPATTERNSTATE_H
