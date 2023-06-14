/*
   CClamp.hpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/20/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      Clamp Function
*/

template<typename T> T Clamp(T tValue, T tMin, T tMax) { return std::max(std::min(tMax, tValue), tMin); }