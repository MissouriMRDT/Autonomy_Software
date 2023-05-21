/*
   CClamp.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Clamp Function
*/

#include <algorithm>

#ifndef CCLAMP_H
#define CCLAMP_H

template <typename T>
T Clamp(T tValue, T tMin, T tMax);

#include "CClamp.hpp"

#endif // CCLAMP_H
