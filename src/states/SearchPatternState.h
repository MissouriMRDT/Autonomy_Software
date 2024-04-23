/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef SEARCH_PATTERN_STATE_H
#define SEARCH_PATTERN_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"
#include "../util/states/StuckDetection.hpp"
#include "../vision/aruco/TagDetector.h"

/// \cond

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
     * @brief The SearchPatternState class implements the Search Pattern state for the
     *        Autonomy State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class SearchPatternState : public State
    {
        private:
            /////////////////////////////////////////
            // Declare private enums and structs that are specific to and used withing this class.
            /////////////////////////////////////////

            // Enum for storing which search pattern type we are using.
            enum SearchPatternType
            {
                eSpiral,
                eZigZag,
                END
            };

            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////
            bool m_bInitialized;
            geoops::GPSCoordinate m_stSearchPatternCenter;
            std::vector<TagDetector*> m_vTagDetectors;
            std::vector<geoops::Waypoint> m_vSearchPath;
            int m_nSearchPathIdx;
            SearchPatternType m_eCurrentSearchPatternType;
            std::vector<std::pair<double, double>> m_vRoverPosition;
            size_t m_nMaxDataPoints;
            statemachine::TimeIntervalBasedStuckDetector m_StuckDetector;

        protected:
            /////////////////////////////////////////
            // Declare protected class methods.
            /////////////////////////////////////////
            void Start() override;
            void Exit() override;

        public:
            /////////////////////////////////////////
            // Declare public class methods.
            /////////////////////////////////////////
            SearchPatternState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // SEARCHPATTERNSTATE_H
