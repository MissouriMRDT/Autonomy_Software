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

#include "../AutonomyConstants.h"
#include "../AutonomyLogging.h"

namespace statemachine
{
    class State
    {
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
            virtual constants::States TriggerEvent(constants::Event eEvent) = 0;

            /******************************************************************************
             * @brief Run the state machine. Returns the next state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual constants::States Run() = 0;

            /******************************************************************************
             * @brief Accessor for the State private member.
             *
             * @return constants::States - The current state.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual constants::States GetState() const = 0;

            /******************************************************************************
             * @brief Accessor for the State private member. Returns the state as a string.
             *
             * @return std::string - The current state as a string.
             *
             * @author Eli Byrd (edbgkk@mst.edu)
             * @date 2024-01-17
             ******************************************************************************/
            virtual std::string ToString() const { return typeid(*this).name(); }

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
