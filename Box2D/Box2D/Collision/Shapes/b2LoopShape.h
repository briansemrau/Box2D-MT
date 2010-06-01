/*
* Copyright (c) 2006-2010 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_LOOP_SHAPE_H
#define B2_LOOP_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>

class b2EdgeShape;

/// A loop shape is a free form sequence of line segments that form a circular list.
/// The loop may cross upon itself, but this is not recommended for smooth collision.
/// The loop has double sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
class b2LoopShape : public b2Shape
{
public:
	b2LoopShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const;

	/// Get a child edge.
	void GetChildEdge(b2EdgeShape* edge, int32 index) const;

	/// This always return false.
	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const b2Transform& transform, int32 childIndex) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const;

	/// Chains have zero mass.
	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float32 density) const;

	b2Vec2* m_vertices;
	int32 m_count;
};

inline b2LoopShape::b2LoopShape()
{
	m_type = e_loop;
	m_radius = b2_polygonRadius;
	m_vertices = 0;
	m_count = 0;
}

#endif