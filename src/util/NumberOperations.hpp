/******************************************************************************
 * @brief Defines and implements functions related to operations on numbers within
 * 		the numops namespace.
 *
 * @file NumberOperations.hpp
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <algorithm>

#ifndef NUMBEROPERATIONS_H
#	define NUMBEROPERATIONS_H

namespace numops
{
	/******************************************************************************
	 * @brief Clamps a given value from going above or below a given threshold.
	 *
	 * @tparam T - Template argument for given value type.
	 * @param tValue - The value to clamp.
	 * @param tMin - Minimum value quantity.
	 * @param tMax - Maximum value quantity.
	 * @return T - The clamped value.
	 *
	 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
	 * @date 2023-0620
	 ******************************************************************************/
	template<typename T>
	T Clamp(T tValue, T tMin, T tMax)
	{
		return std::max(std::min(tMax, tValue), tMin);
	}
}	 // namespace numops

#endif	  // CNUMBEROPERATIONS_H
