/*
   NumberOperations.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:			 5/20/2023
   Author:		   Eli Byrd and Clayton Cowen
   Description:	  NUMBEROPERATIONS Function
*/

#include <algorithm>

#ifndef NUMBEROPERATIONS_H
#	define NUMBEROPERATIONS_H

namespace numops
{
	template<typename T>
	T Clamp(T tValue, T tMin, T tMax)
	{
		return std::max(std::min(tMax, tValue), tMin);
	}
}	 // namespace numops

#endif	  // CNUMBEROPERATIONS_H
