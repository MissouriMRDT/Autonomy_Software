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
    /******************************************************************************
     * @brief The states that the state machine can be in.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-18
     ******************************************************************************/
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

    /******************************************************************************
     * @brief The events that can be triggered in the state machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-18
     ******************************************************************************/
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

    /******************************************************************************
     * @brief Converts a state object to a string.
     *
     * @param eState -
     * @return std::string -
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-18
     ******************************************************************************/
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

    /******************************************************************************
     * @brief The abstract state class. All states inherit from this class.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-18
     ******************************************************************************/
    class State
    {
        private:
            States m_eState;
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
             * @param eState - The state to start in.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-18
             ******************************************************************************/
            State(States eState)
            {
                // Set State
                m_eState = eState;

                // Set State Name
                m_szStateName = StateToString(m_eState);

                // Start the State
                Start();
            }

            /******************************************************************************
             * @brief Destroy the State object.
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
            States GetState() const { return m_eState; }

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
