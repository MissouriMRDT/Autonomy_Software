/*
   CClamp.cpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Clamp Function
*/
#include "CClamp.h"

// Use namespaces.
using namespace std;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename T> T Clamp(T tValue, T tMin, T tMax) { return max(min(tMax, tValue), tMin); }
