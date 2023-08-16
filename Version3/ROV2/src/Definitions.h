/*
 * Definitions.h
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#ifndef RVO_DEFINITIONS_H_
#define RVO_DEFINITIONS_H_

/**
 * \file       Definitions.h
 * \brief      Contains functions and constants used in multiple classes.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "Vector2.h"

/**
 * \brief       A sufficiently small positive number.
 */
const float RVO_EPSILON = 0.00001f;

namespace RVO {
	class Agent;
	class Obstacle;
	class RVOSimulator;

	/**
	 * \brief      Computes the squared distance from a line segment with the
	 *             specified endpoints to a specified point.
	 * \param      a               The first endpoint of the line segment.
	 * \param      b               The second endpoint of the line segment.
	 * \param      c               The point to which the squared distance is to
	 *                             be calculated.
	 * \return     The squared distance from the line segment to the point.
	 */
	/*cdp计算从具有指定端点的线段到指定点的距离的平方*/
	inline float distSqPointLineSegment(const Vector2 &a, const Vector2 &b,
										const Vector2 &c)
	{
		const float r = ((c - a) * (b - a)) / absSq(b - a);

		if (r < 0.0f) {
			return absSq(c - a);
		}
		else if (r > 1.0f) {
			return absSq(c - b);
		}
		else {
			return absSq(c - (a + r * (b - a)));
		}
	}

	/**
	 * \brief      Computes the signed distance from a line connecting the
	 *             specified points to a specified point.
	 * \param      a               The first point on the line.
	 * \param      b               The second point on the line.
	 * \param      c               The point to which the signed distance is to
	 *                             be calculated.
	 * \return     Positive when the point c lies to the left of the line ab.
	 */
	/*cdp计算从c连接ab直线的有符号距离*/
	inline float leftOf(const Vector2 &a, const Vector2 &b, const Vector2 &c)
	{
		return det(a - c, b - a);
	}

	/**
	 * \brief      Computes the square of a float.
	 * \param      a               The float to be squared.
	 * \return     The square of the float.
	 */
	/*cdp计算浮点数的平方*/
	inline float sqr(float a)
	{
		return a * a;
	}
}

#endif /* RVO_DEFINITIONS_H_ */
