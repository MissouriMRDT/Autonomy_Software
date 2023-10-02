/******************************************************************************
 * @brief
 *
 * @file AutonomyState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-10-02
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef AUTONOMYSTATE_HPP
#define AUTONOMYSTATE_HPP

#include "../AutonomyLogging.h"
#include <map>

enum class States
{
    Abort,
    Idle,
    Navigation,
    SearchPattern,
    Avoidance,
    Stuck,
    Reverse,
    Approach,
    Verification
};

std::map<States, States> StateTransitions = {
    // Abort
    {States::Abort, States::Idle},
    {States::Abort, States::Abort},

    // Idle
    {States::Idle, States::Abort},
    {States::Idle, States::Navigation},
    {States::Idle, States::Reverse},

    // Navigation
    {States::Navigation, States::Abort},
    {States::Navigation, States::Navigation},
    {States::Navigation, States::Idle},
    {States::Navigation, States::SearchPattern},
    {States::Navigation, States::Avoidance},
    {States::Navigation, States::Approach},
    {States::Navigation, States::Stuck},
    {States::Navigation, States::Reverse},

    // Search Pattern
    {States::SearchPattern, States::Abort},
    {States::SearchPattern, States::Navigation},
    {States::SearchPattern, States::Avoidance},
    {States::SearchPattern, States::Stuck},

    // End States
};

class AutonomyState
{
    public:
        virtual void Start()                             = 0;
        virtual void Exit()                              = 0;
        virtual std::unique_ptr<AutonomyState> OnEvent() = 0;
        virtual void Run()                               = 0;
};

#endif    // AUTONOMYSTATE_HPP
