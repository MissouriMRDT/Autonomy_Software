/******************************************************************************
 * @brief Abstract State Implementation for Autonomy State Machine.
 *
 * @file State.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-15
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STATE_HPP
#define STATE_HPP

#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    enum class States
    {
        Idle,
        Navigating,
        SearchPattern,
        ApproachingMarker,
        ApproachingObject,
        VerifyingMarker,
        VerifyingObject,
        Avoidance,
        Reversing,
        Stuck,

        NUM_STATES
    };

    inline std::string StateToString(States eState)
    {
        switch (eState)
        {
            case States::Idle: return "Idle";
            case States::Navigating: return "Navigating";
            case States::SearchPattern: return "Search Pattern";
            case States::ApproachingMarker: return "Approaching Marker";
            case States::ApproachingObject: return "Approaching Object";
            case States::VerifyingMarker: return "Verifying Marker";
            case States::VerifyingObject: return "Verifying Object";
            case States::Avoidance: return "Avoidance";
            case States::Reversing: return "Reversing";
            case States::Stuck: return "Stuck";
            default: return "Unknown";
        }
    }

    enum class Event
    {
        Start,
        ReachedGpsCoordinate,
        ReachedMarker,
        ReachedObject,
        MarkerSeen,
        ObjectSeen,
        MarkerUnseen,
        ObjectUnseen,
        VerifyingComplete,
        VerifyingFailed,
        Abort,
        Restart,
        ObstacleAvoidance,
        EndObstacleAvoidance,
        NoWaypoint,
        NewWaypoint,
        Reverse,
        ReverseComplete,
        SearchFailed,
        Stuck,

        NUM_EVENTS
    };

    class State
    {
        private:
            std::string m_szStateName;

        protected:
            /******************************************************************************
             * @brief This method is called when the state is first started. It is used to
             *        initialize the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual void Start() { return; }

            /******************************************************************************
             * @brief This method is called when the state is exited. It is used to clean up
             *        the state.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual void Exit() { return; }

        public:
            /******************************************************************************
             * @brief Construct a new State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            State() { Start(); }

            State(const std::string szStateName = "UNKNOWN")
            {
                // Set State Name
                m_szStateName = szStateName;

                // Start the State
                Start();
            }

            /******************************************************************************
             * @brief Destroy the State object.
             *
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual ~State() = default;

            /******************************************************************************
             * @brief Trigger an event in the state machine. Returns the next state.
             *
             * @param eEvent - The event to trigger.
             * @return std::shared_ptr<State> - The next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual States TriggerEvent(Event eEvent) = 0;

            /******************************************************************************
             * @brief Run the state machine. Returns the next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual States Run() = 0;

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual States GetState() const = 0;

            /******************************************************************************
             * @brief Accessor for the State private member. Returns the state as a string.
             *
             * @return std::string - The current state as a string.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual std::string ToString() const { return m_szStateName; }

            /******************************************************************************
             * @brief Checks to see if the current state is equal to the passed state.
             *
             * @param other - The state to compare to.
             * @return true - The states are equal.
             * @return false - The states are not equal.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual bool operator==(const State& other) const { return ToString() == other.ToString(); }

            /******************************************************************************
             * @brief Checks to see if the current state is not equal to the passed state.
             *
             * @param other - The state to compare to.
             * @return true - The states are not equal.
             * @return false - The states are equal.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual bool operator!=(const State& other) const { return !operator==(other); }
    };
}    // namespace statemachine

#endif    // STATE_HPP
