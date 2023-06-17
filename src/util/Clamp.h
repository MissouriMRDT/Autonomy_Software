/*
   Clamp.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Clamp Function
*/

#include <algorithm>

#ifndef CLAMP_H
#	define CLAMP_H

template<typename T>
T Clamp(T tValue, T tMin, T tMax);

#endif	  // CCLAMP_H
