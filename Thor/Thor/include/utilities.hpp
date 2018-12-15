#pragma once
#ifndef THOR_UTILITIES_HPP
#define THOR_UTILITIES_HPP

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#define COUNT_OF_ARRAY(x) (sizeof(x)/sizeof(*x))

namespace Thor
{
	namespace Util
	{

		/* Returns the first key associated with 'value' in the given 'map' */
		template<class M, typename K, typename V>
		bool findKeyFromVal(K& key, M map, V value)
		{
			bool result = false;

			for (auto pair = map.begin(); pair <= map.end(); pair++)
			{
				if (pair->second == value)
				{
					result = true;
					key = pair->first;
					break;
				}
			}
			return result;
		}

	}
}

#endif
