/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
import { b2Assert } from '../common/b2Settings';
import { b2_epsilon, b2_epsilon_sq, b2_maxFloat, b2_maxManifoldPoints, b2MakeArray, b2MakeNumberArray, } from '../common/b2Settings';
import { b2Abs, b2Max, b2Min, b2Rot, b2Transform, b2Vec2 } from '../common/b2Math';
import { b2Distance, b2DistanceInput, b2DistanceOutput, b2SimplexCache } from './b2Distance';
/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
export class b2ContactFeature {
    constructor() {
        this._key = 0;
        this._key_invalid = 0;
        this._indexA = 0;
        this._indexB = 0;
        this._typeA = 0;
        this._typeB = 0;
    }
    get key() {
        if (this._key_invalid) {
            this._key_invalid = 0;
            this._key = this._indexA | (this._indexB << 8) | (this._typeA << 16) | (this._typeB << 24);
        }
        return this._key;
    }
    set key(value) {
        this._key = value;
        this._key_invalid = 0;
        this._indexA = this._key & 0xff;
        this._indexB = (this._key >>> 8) & 0xff;
        this._typeA = (this._key >>> 16) & 0xff;
        this._typeB = (this._key >>> 24) & 0xff;
    }
    get indexA() {
        return this._indexA;
    }
    set indexA(value) {
        this._indexA = value;
        this._key_invalid = 1;
    }
    get indexB() {
        return this._indexB;
    }
    set indexB(value) {
        this._indexB = value;
        this._key_invalid = 1;
    }
    get typeA() {
        return this._typeA;
    }
    set typeA(value) {
        this._typeA = value;
        this._key_invalid = 1;
    }
    get typeB() {
        return this._typeB;
    }
    set typeB(value) {
        this._typeB = value;
        this._key_invalid = 1;
    }
}
/// Contact ids to facilitate warm starting.
export class b2ContactID {
    constructor() {
        this.cf = new b2ContactFeature();
    }
    Copy(o) {
        this.key = o.key;
        return this;
    }
    Clone() {
        return new b2ContactID().Copy(this);
    }
    get key() {
        return this.cf.key;
    }
    set key(value) {
        this.cf.key = value;
    }
}
/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
export class b2ManifoldPoint {
    constructor() {
        this.localPoint = new b2Vec2(); ///< usage depends on manifold type
        this.normalImpulse = NaN; ///< the non-penetration impulse
        this.tangentImpulse = NaN; ///< the friction impulse
        this.id = new b2ContactID(); ///< uniquely identifies a contact point between two shapes
        this.normalImpulse = 0.0;
        this.tangentImpulse = 0.0;
    }
    static MakeArray(length) {
        return b2MakeArray(length, (i) => new b2ManifoldPoint());
    }
    Reset() {
        this.localPoint.SetZero();
        this.normalImpulse = 0.0;
        this.tangentImpulse = 0.0;
        this.id.key = 0;
    }
    Copy(o) {
        this.localPoint.Copy(o.localPoint);
        this.normalImpulse = o.normalImpulse;
        this.tangentImpulse = o.tangentImpulse;
        this.id.Copy(o.id);
        return this;
    }
}
/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
export class b2Manifold {
    constructor() {
        this.points = b2ManifoldPoint.MakeArray(b2_maxManifoldPoints);
        this.localNormal = new b2Vec2();
        this.localPoint = new b2Vec2();
        this.type = -1 /* e_unknown */;
        this.pointCount = 0;
    }
    Reset() {
        for (let i = 0; i < b2_maxManifoldPoints; ++i) {
            !!B2_DEBUG && b2Assert(this.points[i] instanceof b2ManifoldPoint);
            this.points[i].Reset();
        }
        this.localNormal.SetZero();
        this.localPoint.SetZero();
        this.type = -1 /* e_unknown */;
        this.pointCount = 0;
    }
    Copy(o) {
        this.pointCount = o.pointCount;
        for (let i = 0; i < b2_maxManifoldPoints; ++i) {
            !!B2_DEBUG && b2Assert(this.points[i] instanceof b2ManifoldPoint);
            this.points[i].Copy(o.points[i]);
        }
        this.localNormal.Copy(o.localNormal);
        this.localPoint.Copy(o.localPoint);
        this.type = o.type;
        return this;
    }
    Clone() {
        return new b2Manifold().Copy(this);
    }
}
export class b2WorldManifold {
    constructor() {
        this.normal = new b2Vec2();
        this.points = b2Vec2.MakeArray(b2_maxManifoldPoints);
        this.separations = b2MakeNumberArray(b2_maxManifoldPoints);
    }
    Initialize(manifold, xfA, radiusA, xfB, radiusB) {
        if (manifold.pointCount === 0) {
            return;
        }
        if (manifold.type === 0 /* e_circles */) {
            this.normal.Set(1, 0);
            const pointA = b2Transform.MulXV(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_pointA);
            const pointB = b2Transform.MulXV(xfB, manifold.points[0].localPoint, b2WorldManifold.Initialize_s_pointB);
            if (b2Vec2.DistanceSquaredVV(pointA, pointB) > b2_epsilon_sq) {
                b2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
            }
            const cA = b2Vec2.AddVMulSV(pointA, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
            const cB = b2Vec2.SubVMulSV(pointB, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
            b2Vec2.MidVV(cA, cB, this.points[0]);
            this.separations[0] = b2Vec2.DotVV(b2Vec2.SubVV(cB, cA, b2Vec2.s_t0), this.normal); // b2Dot(cB - cA, normal);
        }
        else if (manifold.type === 1 /* e_faceA */) {
            b2Rot.MulRV(xfA.q, manifold.localNormal, this.normal);
            const planePoint = b2Transform.MulXV(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_planePoint);
            for (let i = 0; i < manifold.pointCount; ++i) {
                const clipPoint = b2Transform.MulXV(xfB, manifold.points[i].localPoint, b2WorldManifold.Initialize_s_clipPoint);
                const s = radiusA - b2Vec2.DotVV(b2Vec2.SubVV(clipPoint, planePoint, b2Vec2.s_t0), this.normal);
                const cA = b2Vec2.AddVMulSV(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cA);
                const cB = b2Vec2.SubVMulSV(clipPoint, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                b2Vec2.MidVV(cA, cB, this.points[i]);
                this.separations[i] = b2Vec2.DotVV(b2Vec2.SubVV(cB, cA, b2Vec2.s_t0), this.normal); // b2Dot(cB - cA, normal);
            }
        }
        else if (manifold.type === 2 /* e_faceB */) {
            b2Rot.MulRV(xfB.q, manifold.localNormal, this.normal);
            const planePoint = b2Transform.MulXV(xfB, manifold.localPoint, b2WorldManifold.Initialize_s_planePoint);
            for (let i = 0; i < manifold.pointCount; ++i) {
                const clipPoint = b2Transform.MulXV(xfA, manifold.points[i].localPoint, b2WorldManifold.Initialize_s_clipPoint);
                const s = radiusB - b2Vec2.DotVV(b2Vec2.SubVV(clipPoint, planePoint, b2Vec2.s_t0), this.normal);
                const cB = b2Vec2.AddVMulSV(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cB);
                const cA = b2Vec2.SubVMulSV(clipPoint, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                b2Vec2.MidVV(cA, cB, this.points[i]);
                this.separations[i] = b2Vec2.DotVV(b2Vec2.SubVV(cA, cB, b2Vec2.s_t0), this.normal); // b2Dot(cA - cB, normal);
            }
            // Ensure normal points from A to B.
            this.normal.SelfNeg();
        }
    }
}
b2WorldManifold.Initialize_s_pointA = new b2Vec2();
b2WorldManifold.Initialize_s_pointB = new b2Vec2();
b2WorldManifold.Initialize_s_cA = new b2Vec2();
b2WorldManifold.Initialize_s_cB = new b2Vec2();
b2WorldManifold.Initialize_s_planePoint = new b2Vec2();
b2WorldManifold.Initialize_s_clipPoint = new b2Vec2();
/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
export function b2GetPointStates(state1, state2, manifold1, manifold2) {
    // Detect persists and removes.
    let i;
    for (i = 0; i < manifold1.pointCount; ++i) {
        const id = manifold1.points[i].id;
        const key = id.key;
        state1[i] = 3 /* b2_removeState */;
        for (let j = 0, jct = manifold2.pointCount; j < jct; ++j) {
            if (manifold2.points[j].id.key === key) {
                state1[i] = 2 /* b2_persistState */;
                break;
            }
        }
    }
    for (; i < b2_maxManifoldPoints; ++i) {
        state1[i] = 0 /* b2_nullState */;
    }
    // Detect persists and adds.
    for (i = 0; i < manifold2.pointCount; ++i) {
        const id = manifold2.points[i].id;
        const key = id.key;
        state2[i] = 1 /* b2_addState */;
        for (let j = 0, jct = manifold1.pointCount; j < jct; ++j) {
            if (manifold1.points[j].id.key === key) {
                state2[i] = 2 /* b2_persistState */;
                break;
            }
        }
    }
    for (; i < b2_maxManifoldPoints; ++i) {
        state2[i] = 0 /* b2_nullState */;
    }
}
/// Used for computing contact manifolds.
export class b2ClipVertex {
    constructor() {
        this.v = new b2Vec2();
        this.id = new b2ContactID();
    }
    static MakeArray(length) {
        return b2MakeArray(length, (i) => new b2ClipVertex());
    }
    Copy(other) {
        this.v.Copy(other.v);
        this.id.Copy(other.id);
        return this;
    }
}
/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
export class b2RayCastInput {
    constructor() {
        this.p1 = new b2Vec2();
        this.p2 = new b2Vec2();
        this.maxFraction = NaN;
        this.maxFraction = 1.0;
    }
    Copy(o) {
        this.p1.Copy(o.p1);
        this.p2.Copy(o.p2);
        this.maxFraction = o.maxFraction;
        return this;
    }
}
/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
export class b2RayCastOutput {
    constructor() {
        this.normal = new b2Vec2();
        this.fraction = NaN;
        this.fraction = 0.0;
    }
    Copy(o) {
        this.normal.Copy(o.normal);
        this.fraction = o.fraction;
        return this;
    }
}
/// An axis aligned bounding box.
export class b2AABB {
    constructor() {
        this.lowerBound = new b2Vec2(); ///< the lower vertex
        this.upperBound = new b2Vec2(); ///< the upper vertex
        this.m_cache_center = new b2Vec2(); // access using GetCenter()
        this.m_cache_extent = new b2Vec2(); // access using GetExtents()
    }
    Copy(o) {
        this.lowerBound.Copy(o.lowerBound);
        this.upperBound.Copy(o.upperBound);
        return this;
    }
    /// Verify that the bounds are sorted.
    IsValid() {
        if (!this.lowerBound.IsValid()) {
            return false;
        }
        if (!this.upperBound.IsValid()) {
            return false;
        }
        if (this.upperBound.x < this.lowerBound.x) {
            return false;
        }
        if (this.upperBound.y < this.lowerBound.y) {
            return false;
        }
        return true;
    }
    /// Get the center of the AABB.
    GetCenter() {
        return b2Vec2.MidVV(this.lowerBound, this.upperBound, this.m_cache_center);
    }
    /// Get the extents of the AABB (half-widths).
    GetExtents() {
        return b2Vec2.ExtVV(this.lowerBound, this.upperBound, this.m_cache_extent);
    }
    /// Get the perimeter length
    GetPerimeter() {
        const wx = this.upperBound.x - this.lowerBound.x;
        const wy = this.upperBound.y - this.lowerBound.y;
        return 2 * (wx + wy);
    }
    /// Combine an AABB into this one.
    Combine1(aabb) {
        this.lowerBound.x = b2Min(this.lowerBound.x, aabb.lowerBound.x);
        this.lowerBound.y = b2Min(this.lowerBound.y, aabb.lowerBound.y);
        this.upperBound.x = b2Max(this.upperBound.x, aabb.upperBound.x);
        this.upperBound.y = b2Max(this.upperBound.y, aabb.upperBound.y);
        return this;
    }
    /// Combine two AABBs into this one.
    Combine2(aabb1, aabb2) {
        this.lowerBound.x = b2Min(aabb1.lowerBound.x, aabb2.lowerBound.x);
        this.lowerBound.y = b2Min(aabb1.lowerBound.y, aabb2.lowerBound.y);
        this.upperBound.x = b2Max(aabb1.upperBound.x, aabb2.upperBound.x);
        this.upperBound.y = b2Max(aabb1.upperBound.y, aabb2.upperBound.y);
        return this;
    }
    static Combine(aabb1, aabb2, out) {
        out.Combine2(aabb1, aabb2);
        return out;
    }
    /// Does this aabb contain the provided AABB.
    Contains(aabb) {
        if (this.lowerBound.x <= aabb.lowerBound.x) {
            return false;
        }
        if (this.lowerBound.y <= aabb.lowerBound.y) {
            return false;
        }
        if (aabb.upperBound.x <= this.upperBound.x) {
            return false;
        }
        if (aabb.upperBound.y <= this.upperBound.y) {
            return false;
        }
        return true;
    }
    // From Real-time Collision Detection, p179.
    RayCast(output, input) {
        let tmin = -b2_maxFloat;
        let tmax = b2_maxFloat;
        const p_x = input.p1.x;
        const p_y = input.p1.y;
        const d_x = input.p2.x - input.p1.x;
        const d_y = input.p2.y - input.p1.y;
        const absD_x = b2Abs(d_x);
        const absD_y = b2Abs(d_y);
        const normal = output.normal;
        if (absD_x < b2_epsilon) {
            // Parallel.
            if (p_x < this.lowerBound.x || this.upperBound.x < p_x) {
                return false;
            }
        }
        else {
            const inv_d = 1 / d_x;
            let t1 = (this.lowerBound.x - p_x) * inv_d;
            let t2 = (this.upperBound.x - p_x) * inv_d;
            // Sign of the normal vector.
            let s = -1;
            if (t1 > t2) {
                const t3 = t1;
                t1 = t2;
                t2 = t3;
                s = 1;
            }
            // Push the min up
            if (t1 > tmin) {
                normal.x = s;
                normal.y = 0;
                tmin = t1;
            }
            // Pull the max down
            tmax = b2Min(tmax, t2);
            if (tmin > tmax) {
                return false;
            }
        }
        if (absD_y < b2_epsilon) {
            // Parallel.
            if (p_y < this.lowerBound.y || this.upperBound.y < p_y) {
                return false;
            }
        }
        else {
            const inv_d = 1 / d_y;
            let t1 = (this.lowerBound.y - p_y) * inv_d;
            let t2 = (this.upperBound.y - p_y) * inv_d;
            // Sign of the normal vector.
            let s = -1;
            if (t1 > t2) {
                const t3 = t1;
                t1 = t2;
                t2 = t3;
                s = 1;
            }
            // Push the min up
            if (t1 > tmin) {
                normal.x = 0;
                normal.y = s;
                tmin = t1;
            }
            // Pull the max down
            tmax = b2Min(tmax, t2);
            if (tmin > tmax) {
                return false;
            }
        }
        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0 || input.maxFraction < tmin) {
            return false;
        }
        // Intersection.
        output.fraction = tmin;
        return true;
    }
    TestContain(point) {
        if (point.x < this.lowerBound.x || this.upperBound.x < point.x) {
            return false;
        }
        if (point.y < this.lowerBound.y || this.upperBound.y < point.y) {
            return false;
        }
        return true;
    }
    TestOverlap(other) {
        if (this.upperBound.x < other.lowerBound.x) {
            return false;
        }
        if (this.upperBound.y < other.lowerBound.y) {
            return false;
        }
        if (other.upperBound.x < this.lowerBound.x) {
            return false;
        }
        if (other.upperBound.y < this.lowerBound.y) {
            return false;
        }
        return true;
    }
}
export function b2TestOverlapAABB(a, b) {
    if (a.upperBound.x < b.lowerBound.x) {
        return false;
    }
    if (a.upperBound.y < b.lowerBound.y) {
        return false;
    }
    if (b.upperBound.x < a.lowerBound.x) {
        return false;
    }
    if (b.upperBound.y < a.lowerBound.y) {
        return false;
    }
    return true;
}
/// Clipping for contact manifolds.
export function b2ClipSegmentToLine(vOut, vIn, normal, offset, vertexIndexA) {
    // Start with no output points
    let numOut = 0;
    const vIn0 = vIn[0];
    const vIn1 = vIn[1];
    // Calculate the distance of end points to the line
    const distance0 = b2Vec2.DotVV(normal, vIn0.v) - offset;
    const distance1 = b2Vec2.DotVV(normal, vIn1.v) - offset;
    // If the points are behind the plane
    if (distance0 <= 0) {
        vOut[numOut++].Copy(vIn0);
    }
    if (distance1 <= 0) {
        vOut[numOut++].Copy(vIn1);
    }
    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0) {
        // Find intersection point of edge and plane
        const interp = distance0 / (distance0 - distance1);
        const v = vOut[numOut].v;
        v.x = vIn0.v.x + interp * (vIn1.v.x - vIn0.v.x);
        v.y = vIn0.v.y + interp * (vIn1.v.y - vIn0.v.y);
        // VertexA is hitting edgeB.
        const id = vOut[numOut].id;
        id.cf.indexA = vertexIndexA;
        id.cf.indexB = vIn0.id.cf.indexB;
        id.cf.typeA = 0 /* e_vertex */;
        id.cf.typeB = 1 /* e_face */;
        ++numOut;
    }
    return numOut;
}
/// Determine if two generic shapes overlap.
const b2TestOverlapShape_s_input = new b2DistanceInput();
const b2TestOverlapShape_s_simplexCache = new b2SimplexCache();
const b2TestOverlapShape_s_output = new b2DistanceOutput();
export function b2TestOverlapShape(shapeA, indexA, shapeB, indexB, xfA, xfB) {
    const input = b2TestOverlapShape_s_input.Reset();
    input.proxyA.SetShape(shapeA, indexA);
    input.proxyB.SetShape(shapeB, indexB);
    input.transformA.Copy(xfA);
    input.transformB.Copy(xfB);
    input.useRadii = true;
    const simplexCache = b2TestOverlapShape_s_simplexCache.Reset();
    simplexCache.count = 0;
    const output = b2TestOverlapShape_s_output.Reset();
    b2Distance(output, simplexCache, input);
    return output.distance < 10 * b2_epsilon;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaXNpb24uanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9zcmMvY29sbGlzaW9uL2IyQ29sbGlzaW9uLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLFFBQVEsRUFBRSxNQUFNLHNCQUFzQixDQUFDO0FBQ2hELE9BQU8sRUFDTCxVQUFVLEVBQ1YsYUFBYSxFQUNiLFdBQVcsRUFDWCxvQkFBb0IsRUFDcEIsV0FBVyxFQUNYLGlCQUFpQixHQUNsQixNQUFNLHNCQUFzQixDQUFDO0FBQzlCLE9BQU8sRUFBRSxLQUFLLEVBQUUsS0FBSyxFQUFFLEtBQUssRUFBRSxLQUFLLEVBQUUsV0FBVyxFQUFFLE1BQU0sRUFBTSxNQUFNLGtCQUFrQixDQUFDO0FBRXZGLE9BQU8sRUFBRSxVQUFVLEVBQUUsZUFBZSxFQUFFLGdCQUFnQixFQUFFLGNBQWMsRUFBRSxNQUFNLGNBQWMsQ0FBQztBQVc3Rix5REFBeUQ7QUFDekQsaUNBQWlDO0FBQ2pDLE1BQU0sT0FBTyxnQkFBZ0I7SUFBN0I7UUFDVSxTQUFJLEdBQUcsQ0FBQyxDQUFDO1FBQ1QsaUJBQVksR0FBRyxDQUFDLENBQUM7UUFDakIsWUFBTyxHQUFHLENBQUMsQ0FBQztRQUNaLFlBQU8sR0FBRyxDQUFDLENBQUM7UUFDWixXQUFNLEdBQXlCLENBQUMsQ0FBQztRQUNqQyxXQUFNLEdBQXlCLENBQUMsQ0FBQztJQXNEM0MsQ0FBQztJQXBEQyxJQUFJLEdBQUc7UUFDTCxJQUFJLElBQUksQ0FBQyxZQUFZLEVBQUU7WUFDckIsSUFBSSxDQUFDLFlBQVksR0FBRyxDQUFDLENBQUM7WUFDdEIsSUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsSUFBSSxDQUFDLE9BQU8sSUFBSSxDQUFDLENBQUMsR0FBRyxDQUFDLElBQUksQ0FBQyxNQUFNLElBQUksRUFBRSxDQUFDLEdBQUcsQ0FBQyxJQUFJ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QUFNLGlDQUFpQyxHQUFtQixJQUFJLGNBQWMsRUFBRSxDQUFDO0FBQy9FLE1BQU0sMkJBQTJCLEdBQXFCLElBQUksZ0JBQWdCLEVBQUUsQ0FBQztBQUU3RSxNQUFNLFVBQVUsa0JBQWtCLENBQ2hDLE1BQWUsRUFDZixNQUFjLEVBQ2QsTUFBZSxFQUNmLE1BQWMsRUFDZCxHQUFnQixFQUNoQixHQUFnQjtJQUVoQixNQUFNLEtBQUssR0FBb0IsMEJBQTBCLENBQUMsS0FBSyxFQUFFLENBQUM7SUFDbEUsS0FBSyxDQUFDLE1BQU0sQ0FBQyxRQUFRLENBQUMsTUFBTSxFQUFFLE1BQU0sQ0FBQyxDQUFDO0lBQ3RDLEtBQUssQ0FBQyxNQUFNLENBQUMsUUFBUSxDQUFDLE1BQU0sRUFBRSxNQUFNLENBQUMsQ0FBQztJQUN0QyxLQUFLLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxHQUFHLENBQUMsQ0FBQztJQUMzQixLQUFLLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxHQUFHLENBQUMsQ0FBQztJQUMzQixLQUFLLENBQUMsUUFBUSxHQUFHLElBQUksQ0FBQztJQUV0QixNQUFNLFlBQVksR0FBbUIsaUNBQWlDLENBQUMsS0FBSyxFQUFFLENBQUM7SUFDL0UsWUFBWSxDQUFDLEtBQUssR0FBRyxDQUFDLENBQUM7SUFFdkIsTUFBTSxNQUFNLEdBQXFCLDJCQUEyQixDQUFDLEtBQUssRUFBRSxDQUFDO0lBRXJFLFVBQVUsQ0FBQyxNQUFNLEVBQUUsWUFBWSxFQUFFLEtBQUssQ0FBQyxDQUFDO0lBRXhDLE9BQU8sTUFBTSxDQUFDLFFBQVEsR0FBRyxFQUFFLEdBQUcsVUFBVSxDQUFDO0FBQzNDLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMDYtMjAwOSBFcmluIENhdHRvIGh0dHA6Ly93d3cuYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7IGIyQXNzZXJ0IH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQge1xyXG4gIGIyX2Vwc2lsb24sXHJcbiAgYjJfZXBzaWxvbl9zcSxcclxuICBiMl9tYXhGbG9hdCxcclxuICBiMl9tYXhNYW5pZm9sZFBvaW50cyxcclxuICBiMk1ha2VBcnJheSxcclxuICBiMk1ha2VOdW1iZXJBcnJheSxcclxufSBmcm9tICcuLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7IGIyQWJzLCBiMk1heCwgYjJNaW4sIGIyUm90LCBiMlRyYW5zZm9ybSwgYjJWZWMyLCBYWSB9IGZyb20gJy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMlNoYXBlIH0gZnJvbSAnLi9zaGFwZXMvYjJTaGFwZSc7XHJcbmltcG9ydCB7IGIyRGlzdGFuY2UsIGIyRGlzdGFuY2VJbnB1dCwgYjJEaXN0YW5jZU91dHB1dCwgYjJTaW1wbGV4Q2FjaGUgfSBmcm9tICcuL2IyRGlzdGFuY2UnO1xyXG5cclxuLy8vIEBmaWxlXHJcbi8vLyBTdHJ1Y3R1cmVzIGFuZCBmdW5jdGlvbnMgdXNlZCBmb3IgY29tcHV0aW5nIGNvbnRhY3QgcG9pbnRzLCBkaXN0YW5jZVxyXG4vLy8gcXVlcmllcywgYW5kIFRPSSBxdWVyaWVzLlxyXG5cclxuZXhwb3J0IGNvbnN0IGVudW0gYjJDb250YWN0RmVhdHVyZVR5cGUge1xyXG4gIGVfdmVydGV4ID0gMCxcclxuICBlX2ZhY2UgPSAxLFxyXG59XHJcblxyXG4vLy8gVGhlIGZlYXR1cmVzIHRoYXQgaW50ZXJzZWN0IHRvIGZvcm0gdGhlIGNvbnRhY3QgcG9pbnRcclxuLy8vIFRoaXMgbXVzdCBiZSA0IGJ5dGVzIG9yIGxlc3MuXHJcbmV4cG9ydCBjbGFzcyBiMkNvbnRhY3RGZWF0dXJlIHtcclxuICBwcml2YXRlIF9rZXkgPSAwO1xyXG4gIHByaXZhdGUgX2tleV9pbnZhbGlkID0gMDtcclxuICBwcml2YXRlIF9pbmRleEEgPSAwO1xyXG4gIHByaXZhdGUgX2luZGV4QiA9IDA7XHJcbiAgcHJpdmF0ZSBfdHlwZUE6IGIyQ29udGFjdEZlYXR1cmVUeXBlID0gMDtcclxuICBwcml2YXRlIF90eXBlQjogYjJDb250YWN0RmVhdHVyZVR5cGUgPSAwO1xyXG5cclxuICBnZXQga2V5KCk6IG51bWJlciB7XHJcbiAgICBpZiAodGhpcy5fa2V5X2ludmFsaWQpIHtcclxuICAgICAgdGhpcy5fa2V5X2ludmFsaWQgPSAwO1xyXG4gICAgICB0aGlzLl9rZXkgPSB0aGlzLl9pbmRleEEgfCAodGhpcy5faW5kZXhCIDw8IDgpIHwgKHRoaXMuX3R5cGVBIDw8IDE2KSB8ICh0aGlzLl90eXBlQiA8PCAyNCk7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gdGhpcy5fa2V5O1xyXG4gIH1cclxuXHJcbiAgc2V0IGtleSh2YWx1ZTogbnVtYmVyKSB7XHJcbiAgICB0aGlzLl9rZXkgPSB2YWx1ZTtcclxuICAgIHRoaXMuX2tleV9pbnZhbGlkID0gMDtcclxuICAgIHRoaXMuX2luZGV4QSA9IHRoaXMuX2tleSAmIDB4ZmY7XHJcbiAgICB0aGlzLl9pbmRleEIgPSAodGhpcy5fa2V5ID4+PiA4KSAmIDB4ZmY7XHJcbiAgICB0aGlzLl90eXBlQSA9ICh0aGlzLl9rZXkgPj4+IDE2KSAmIDB4ZmY7XHJcbiAgICB0aGlzLl90eXBlQiA9ICh0aGlzLl9rZXkgPj4+IDI0KSAmIDB4ZmY7XHJcbiAgfVxyXG5cclxuICBnZXQgaW5kZXhBKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5faW5kZXhBO1xyXG4gIH1cclxuXHJcbiAgc2V0IGluZGV4QSh2YWx1ZTogbnVtYmVyKSB7XHJcbiAgICB0aGlzLl9pbmRleEEgPSB2YWx1ZTtcclxuICAgIHRoaXMuX2tleV9pbnZhbGlkID0gMTtcclxuICB9XHJcblxyXG4gIGdldCBpbmRleEIoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLl9pbmRleEI7XHJcbiAgfVxyXG5cclxuICBzZXQgaW5kZXhCKHZhbHVlOiBudW1iZXIpIHtcclxuICAgIHRoaXMuX2luZGV4QiA9IHZhbHVlO1xyXG4gICAgdGhpcy5fa2V5X2ludmFsaWQgPSAxO1xyXG4gIH1cclxuXHJcbiAgZ2V0IHR5cGVBKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5fdHlwZUE7XHJcbiAgfVxyXG5cclxuICBzZXQgdHlwZUEodmFsdWU6IG51bWJlcikge1xyXG4gICAgdGhpcy5fdHlwZUEgPSB2YWx1ZTtcclxuICAgIHRoaXMuX2tleV9pbnZhbGlkID0gMTtcclxuICB9XHJcblxyXG4gIGdldCB0eXBlQigpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMuX3R5cGVCO1xyXG4gIH1cclxuXHJcbiAgc2V0IHR5cGVCKHZhbHVlOiBudW1iZXIpIHtcclxuICAgIHRoaXMuX3R5cGVCID0gdmFsdWU7XHJcbiAgICB0aGlzLl9rZXlfaW52YWxpZCA9IDE7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gQ29udGFjdCBpZHMgdG8gZmFjaWxpdGF0ZSB3YXJtIHN0YXJ0aW5nLlxyXG5leHBvcnQgY2xhc3MgYjJDb250YWN0SUQge1xyXG4gIHJlYWRvbmx5IGNmOiBiMkNvbnRhY3RGZWF0dXJlID0gbmV3IGIyQ29udGFjdEZlYXR1cmUoKTtcclxuXHJcbiAgQ29weShvOiBiMkNvbnRhY3RJRCk6IGIyQ29udGFjdElEIHtcclxuICAgIHRoaXMua2V5ID0gby5rZXk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIENsb25lKCk6IGIyQ29udGFjdElEIHtcclxuICAgIHJldHVybiBuZXcgYjJDb250YWN0SUQoKS5Db3B5KHRoaXMpO1xyXG4gIH1cclxuXHJcbiAgZ2V0IGtleSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMuY2Yua2V5O1xyXG4gIH1cclxuXHJcbiAgc2V0IGtleSh2YWx1ZTogbnVtYmVyKSB7XHJcbiAgICB0aGlzLmNmLmtleSA9IHZhbHVlO1xyXG4gIH1cclxufVxyXG5cclxuLy8vIEEgbWFuaWZvbGQgcG9pbnQgaXMgYSBjb250YWN0IHBvaW50IGJlbG9uZ2luZyB0byBhIGNvbnRhY3RcclxuLy8vIG1hbmlmb2xkLiBJdCBob2xkcyBkZXRhaWxzIHJlbGF0ZWQgdG8gdGhlIGdlb21ldHJ5IGFuZCBkeW5hbWljc1xyXG4vLy8gb2YgdGhlIGNvbnRhY3QgcG9pbnRzLlxyXG4vLy8gVGhlIGxvY2FsIHBvaW50IHVzYWdlIGRlcGVuZHMgb24gdGhlIG1hbmlmb2xkIHR5cGU6XHJcbi8vLyAtZV9jaXJjbGVzOiB0aGUgbG9jYWwgY2VudGVyIG9mIGNpcmNsZUJcclxuLy8vIC1lX2ZhY2VBOiB0aGUgbG9jYWwgY2VudGVyIG9mIGNpcmxjZUIgb3IgdGhlIGNsaXAgcG9pbnQgb2YgcG9seWdvbkJcclxuLy8vIC1lX2ZhY2VCOiB0aGUgY2xpcCBwb2ludCBvZiBwb2x5Z29uQVxyXG4vLy8gVGhpcyBzdHJ1Y3R1cmUgaXMgc3RvcmVkIGFjcm9zcyB0aW1lIHN0ZXBzLCBzbyB3ZSBrZWVwIGl0IHNtYWxsLlxyXG4vLy8gTm90ZTogdGhlIGltcHVsc2VzIGFyZSB1c2VkIGZvciBpbnRlcm5hbCBjYWNoaW5nIGFuZCBtYXkgbm90XHJcbi8vLyBwcm92aWRlIHJlbGlhYmxlIGNvbnRhY3QgZm9yY2VzLCBlc3BlY2lhbGx5IGZvciBoaWdoIHNwZWVkIGNvbGxpc2lvbnMuXHJcbmV4cG9ydCBjbGFzcyBiMk1hbmlmb2xkUG9pbnQge1xyXG4gIHJlYWRvbmx5IGxvY2FsUG9pbnQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTsgLy8vPCB1c2FnZSBkZXBlbmRzIG9uIG1hbmlmb2xkIHR5cGVcclxuICBub3JtYWxJbXB1bHNlID0gTmFOOyAvLy88IHRoZSBub24tcGVuZXRyYXRpb24gaW1wdWxzZVxyXG4gIHRhbmdlbnRJbXB1bHNlID0gTmFOOyAvLy88IHRoZSBmcmljdGlvbiBpbXB1bHNlXHJcbiAgcmVhZG9ubHkgaWQ6IGIyQ29udGFjdElEID0gbmV3IGIyQ29udGFjdElEKCk7IC8vLzwgdW5pcXVlbHkgaWRlbnRpZmllcyBhIGNvbnRhY3QgcG9pbnQgYmV0d2VlbiB0d28gc2hhcGVzXHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5ub3JtYWxJbXB1bHNlID0gMC4wO1xyXG4gICAgdGhpcy50YW5nZW50SW1wdWxzZSA9IDAuMDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNYWtlQXJyYXkobGVuZ3RoOiBudW1iZXIpOiBiMk1hbmlmb2xkUG9pbnRbXSB7XHJcbiAgICByZXR1cm4gYjJNYWtlQXJyYXkobGVuZ3RoLCAoaTogbnVtYmVyKTogYjJNYW5pZm9sZFBvaW50ID0+IG5ldyBiMk1hbmlmb2xkUG9pbnQoKSk7XHJcbiAgfVxyXG5cclxuICBSZXNldCgpOiB2b2lkIHtcclxuICAgIHRoaXMubG9jYWxQb2ludC5TZXRaZXJvKCk7XHJcbiAgICB0aGlzLm5vcm1hbEltcHVsc2UgPSAwLjA7XHJcbiAgICB0aGlzLnRhbmdlbnRJbXB1bHNlID0gMC4wO1xyXG4gICAgdGhpcy5pZC5rZXkgPSAwO1xyXG4gIH1cclxuXHJcbiAgQ29weShvOiBiMk1hbmlmb2xkUG9pbnQpOiBiMk1hbmlmb2xkUG9pbnQge1xyXG4gICAgdGhpcy5sb2NhbFBvaW50LkNvcHkoby5sb2NhbFBvaW50KTtcclxuICAgIHRoaXMubm9ybWFsSW1wdWxzZSA9IG8ubm9ybWFsSW1wdWxzZTtcclxuICAgIHRoaXMudGFuZ2VudEltcHVsc2UgPSBvLnRhbmdlbnRJbXB1bHNlO1xyXG4gICAgdGhpcy5pZC5Db3B5KG8uaWQpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY29uc3QgZW51bSBiMk1hbmlmb2xkVHlwZSB7XHJcbiAgZV91bmtub3duID0gLTEsXHJcbiAgZV9jaXJjbGVzID0gMCxcclxuICBlX2ZhY2VBID0gMSxcclxuICBlX2ZhY2VCID0gMixcclxufVxyXG5cclxuLy8vIEEgbWFuaWZvbGQgZm9yIHR3byB0b3VjaGluZyBjb252ZXggc2hhcGVzLlxyXG4vLy8gQm94MkQgc3VwcG9ydHMgbXVsdGlwbGUgdHlwZXMgb2YgY29udGFjdDpcclxuLy8vIC0gY2xpcCBwb2ludCB2ZXJzdXMgcGxhbmUgd2l0aCByYWRpdXNcclxuLy8vIC0gcG9pbnQgdmVyc3VzIHBvaW50IHdpdGggcmFkaXVzIChjaXJjbGVzKVxyXG4vLy8gVGhlIGxvY2FsIHBvaW50IHVzYWdlIGRlcGVuZHMgb24gdGhlIG1hbmlmb2xkIHR5cGU6XHJcbi8vLyAtZV9jaXJjbGVzOiB0aGUgbG9jYWwgY2VudGVyIG9mIGNpcmNsZUFcclxuLy8vIC1lX2ZhY2VBOiB0aGUgY2VudGVyIG9mIGZhY2VBXHJcbi8vLyAtZV9mYWNlQjogdGhlIGNlbnRlciBvZiBmYWNlQlxyXG4vLy8gU2ltaWxhcmx5IHRoZSBsb2NhbCBub3JtYWwgdXNhZ2U6XHJcbi8vLyAtZV9jaXJjbGVzOiBub3QgdXNlZFxyXG4vLy8gLWVfZmFjZUE6IHRoZSBub3JtYWwgb24gcG9seWdvbkFcclxuLy8vIC1lX2ZhY2VCOiB0aGUgbm9ybWFsIG9uIHBvbHlnb25CXHJcbi8vLyBXZSBzdG9yZSBjb250YWN0cyBpbiB0aGlzIHdheSBzbyB0aGF0IHBvc2l0aW9uIGNvcnJlY3Rpb24gY2FuXHJcbi8vLyBhY2NvdW50IGZvciBtb3ZlbWVudCwgd2hpY2ggaXMgY3JpdGljYWwgZm9yIGNvbnRpbnVvdXMgcGh5c2ljcy5cclxuLy8vIEFsbCBjb250YWN0IHNjZW5hcmlvcyBtdXN0IGJlIGV4cHJlc3NlZCBpbiBvbmUgb2YgdGhlc2UgdHlwZXMuXHJcbi8vLyBUaGlzIHN0cnVjdHVyZSBpcyBzdG9yZWQgYWNyb3NzIHRpbWUgc3RlcHMsIHNvIHdlIGtlZXAgaXQgc21hbGwuXHJcbmV4cG9ydCBjbGFzcyBiMk1hbmlmb2xkIHtcclxuICByZWFkb25seSBwb2ludHM6IGIyTWFuaWZvbGRQb2ludFtdID0gYjJNYW5pZm9sZFBvaW50Lk1ha2VBcnJheShiMl9tYXhNYW5pZm9sZFBvaW50cyk7XHJcbiAgcmVhZG9ubHkgbG9jYWxOb3JtYWw6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBsb2NhbFBvaW50OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgdHlwZTogYjJNYW5pZm9sZFR5cGUgPSBiMk1hbmlmb2xkVHlwZS5lX3Vua25vd247XHJcbiAgcG9pbnRDb3VudCA9IDA7XHJcblxyXG4gIFJlc2V0KCk6IHZvaWQge1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBiMl9tYXhNYW5pZm9sZFBvaW50czsgKytpKSB7XHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5wb2ludHNbaV0gaW5zdGFuY2VvZiBiMk1hbmlmb2xkUG9pbnQpO1xyXG4gICAgICB0aGlzLnBvaW50c1tpXS5SZXNldCgpO1xyXG4gICAgfVxyXG4gICAgdGhpcy5sb2NhbE5vcm1hbC5TZXRaZXJvKCk7XHJcbiAgICB0aGlzLmxvY2FsUG9pbnQuU2V0WmVybygpO1xyXG4gICAgdGhpcy50eXBlID0gYjJNYW5pZm9sZFR5cGUuZV91bmtub3duO1xyXG4gICAgdGhpcy5wb2ludENvdW50ID0gMDtcclxuICB9XHJcblxyXG4gIENvcHkobzogYjJNYW5pZm9sZCk6IGIyTWFuaWZvbGQge1xyXG4gICAgdGhpcy5wb2ludENvdW50ID0gby5wb2ludENvdW50O1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBiMl9tYXhNYW5pZm9sZFBvaW50czsgKytpKSB7XHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5wb2ludHNbaV0gaW5zdGFuY2VvZiBiMk1hbmlmb2xkUG9pbnQpO1xyXG4gICAgICB0aGlzLnBvaW50c1tpXS5Db3B5KG8ucG9pbnRzW2ldKTtcclxuICAgIH1cclxuICAgIHRoaXMubG9jYWxOb3JtYWwuQ29weShvLmxvY2FsTm9ybWFsKTtcclxuICAgIHRoaXMubG9jYWxQb2ludC5Db3B5KG8ubG9jYWxQb2ludCk7XHJcbiAgICB0aGlzLnR5cGUgPSBvLnR5cGU7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIENsb25lKCk6IGIyTWFuaWZvbGQge1xyXG4gICAgcmV0dXJuIG5ldyBiMk1hbmlmb2xkKCkuQ29weSh0aGlzKTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMldvcmxkTWFuaWZvbGQge1xyXG4gIHJlYWRvbmx5IG5vcm1hbDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IHBvaW50czogYjJWZWMyW10gPSBiMlZlYzIuTWFrZUFycmF5KGIyX21heE1hbmlmb2xkUG9pbnRzKTtcclxuICByZWFkb25seSBzZXBhcmF0aW9uczogbnVtYmVyW10gPSBiMk1ha2VOdW1iZXJBcnJheShiMl9tYXhNYW5pZm9sZFBvaW50cyk7XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIEluaXRpYWxpemVfc19wb2ludEEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdGlhbGl6ZV9zX3BvaW50QiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBJbml0aWFsaXplX3NfY0EgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdGlhbGl6ZV9zX2NCID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRpYWxpemVfc19wbGFuZVBvaW50ID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRpYWxpemVfc19jbGlwUG9pbnQgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIEluaXRpYWxpemUoXHJcbiAgICBtYW5pZm9sZDogYjJNYW5pZm9sZCxcclxuICAgIHhmQTogYjJUcmFuc2Zvcm0sXHJcbiAgICByYWRpdXNBOiBudW1iZXIsXHJcbiAgICB4ZkI6IGIyVHJhbnNmb3JtLFxyXG4gICAgcmFkaXVzQjogbnVtYmVyLFxyXG4gICk6IHZvaWQge1xyXG4gICAgaWYgKG1hbmlmb2xkLnBvaW50Q291bnQgPT09IDApIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChtYW5pZm9sZC50eXBlID09PSBiMk1hbmlmb2xkVHlwZS5lX2NpcmNsZXMpIHtcclxuICAgICAgdGhpcy5ub3JtYWwuU2V0KDEsIDApO1xyXG4gICAgICBjb25zdCBwb2ludEE6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKFxyXG4gICAgICAgIHhmQSxcclxuICAgICAgICBtYW5pZm9sZC5sb2NhbFBvaW50LFxyXG4gICAgICAgIGIyV29ybGRNYW5pZm9sZC5Jbml0aWFsaXplX3NfcG9pbnRBLFxyXG4gICAgICApO1xyXG4gICAgICBjb25zdCBwb2ludEI6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKFxyXG4gICAgICAgIHhmQixcclxuICAgICAgICBtYW5pZm9sZC5wb2ludHNbMF0ubG9jYWxQb2ludCxcclxuICAgICAgICBiMldvcmxkTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX3BvaW50QixcclxuICAgICAgKTtcclxuICAgICAgaWYgKGIyVmVjMi5EaXN0YW5jZVNxdWFyZWRWVihwb2ludEEsIHBvaW50QikgPiBiMl9lcHNpbG9uX3NxKSB7XHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHBvaW50QiwgcG9pbnRBLCB0aGlzLm5vcm1hbCkuU2VsZk5vcm1hbGl6ZSgpO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBjb25zdCBjQTogYjJWZWMyID0gYjJWZWMyLkFkZFZNdWxTVihcclxuICAgICAgICBwb2ludEEsXHJcbiAgICAgICAgcmFkaXVzQSxcclxuICAgICAgICB0aGlzLm5vcm1hbCxcclxuICAgICAgICBiMldvcmxkTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX2NBLFxyXG4gICAgICApO1xyXG4gICAgICBjb25zdCBjQjogYjJWZWMyID0gYjJWZWMyLlN1YlZNdWxTVihcclxuICAgICAgICBwb2ludEIsXHJcbiAgICAgICAgcmFkaXVzQixcclxuICAgICAgICB0aGlzLm5vcm1hbCxcclxuICAgICAgICBiMldvcmxkTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX2NCLFxyXG4gICAgICApO1xyXG4gICAgICBiMlZlYzIuTWlkVlYoY0EsIGNCLCB0aGlzLnBvaW50c1swXSk7XHJcbiAgICAgIHRoaXMuc2VwYXJhdGlvbnNbMF0gPSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKGNCLCBjQSwgYjJWZWMyLnNfdDApLCB0aGlzLm5vcm1hbCk7IC8vIGIyRG90KGNCIC0gY0EsIG5vcm1hbCk7XHJcbiAgICB9IGVsc2UgaWYgKG1hbmlmb2xkLnR5cGUgPT09IGIyTWFuaWZvbGRUeXBlLmVfZmFjZUEpIHtcclxuICAgICAgYjJSb3QuTXVsUlYoeGZBLnEsIG1hbmlmb2xkLmxvY2FsTm9ybWFsLCB0aGlzLm5vcm1hbCk7XHJcbiAgICAgIGNvbnN0IHBsYW5lUG9pbnQ6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKFxyXG4gICAgICAgIHhmQSxcclxuICAgICAgICBtYW5pZm9sZC5sb2NhbFBvaW50LFxyXG4gICAgICAgIGIyV29ybGRNYW5pZm9sZC5Jbml0aWFsaXplX3NfcGxhbmVQb2ludCxcclxuICAgICAgKTtcclxuXHJcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbWFuaWZvbGQucG9pbnRDb3VudDsgKytpKSB7XHJcbiAgICAgICAgY29uc3QgY2xpcFBvaW50OiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVihcclxuICAgICAgICAgIHhmQixcclxuICAgICAgICAgIG1hbmlmb2xkLnBvaW50c1tpXS5sb2NhbFBvaW50LFxyXG4gICAgICAgICAgYjJXb3JsZE1hbmlmb2xkLkluaXRpYWxpemVfc19jbGlwUG9pbnQsXHJcbiAgICAgICAgKTtcclxuICAgICAgICBjb25zdCBzOiBudW1iZXIgPVxyXG4gICAgICAgICAgcmFkaXVzQSAtIGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYoY2xpcFBvaW50LCBwbGFuZVBvaW50LCBiMlZlYzIuc190MCksIHRoaXMubm9ybWFsKTtcclxuICAgICAgICBjb25zdCBjQTogYjJWZWMyID0gYjJWZWMyLkFkZFZNdWxTVihcclxuICAgICAgICAgIGNsaXBQb2ludCxcclxuICAgICAgICAgIHMsXHJcbiAgICAgICAgICB0aGlzLm5vcm1hbCxcclxuICAgICAgICAgIGIyV29ybGRNYW5pZm9sZC5Jbml0aWFsaXplX3NfY0EsXHJcbiAgICAgICAgKTtcclxuICAgICAgICBjb25zdCBjQjogYjJWZWMyID0gYjJWZWMyLlN1YlZNdWxTVihcclxuICAgICAgICAgIGNsaXBQb2ludCxcclxuICAgICAgICAgIHJhZGl1c0IsXHJcbiAgICAgICAgICB0aGlzLm5vcm1hbCxcclxuICAgICAgICAgIGIyV29ybGRNYW5pZm9sZC5Jbml0aWFsaXplX3NfY0IsXHJcbiAgICAgICAgKTtcclxuICAgICAgICBiMlZlYzIuTWlkVlYoY0EsIGNCLCB0aGlzLnBvaW50c1tpXSk7XHJcbiAgICAgICAgdGhpcy5zZXBhcmF0aW9uc1tpXSA9IGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYoY0IsIGNBLCBiMlZlYzIuc190MCksIHRoaXMubm9ybWFsKTsgLy8gYjJEb3QoY0IgLSBjQSwgbm9ybWFsKTtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIGlmIChtYW5pZm9sZC50eXBlID09PSBiMk1hbmlmb2xkVHlwZS5lX2ZhY2VCKSB7XHJcbiAgICAgIGIyUm90Lk11bFJWKHhmQi5xLCBtYW5pZm9sZC5sb2NhbE5vcm1hbCwgdGhpcy5ub3JtYWwpO1xyXG4gICAgICBjb25zdCBwbGFuZVBvaW50OiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVihcclxuICAgICAgICB4ZkIsXHJcbiAgICAgICAgbWFuaWZvbGQubG9jYWxQb2ludCxcclxuICAgICAgICBiMldvcmxkTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX3BsYW5lUG9pbnQsXHJcbiAgICAgICk7XHJcblxyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IG1hbmlmb2xkLnBvaW50Q291bnQ7ICsraSkge1xyXG4gICAgICAgIGNvbnN0IGNsaXBQb2ludDogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoXHJcbiAgICAgICAgICB4ZkEsXHJcbiAgICAgICAgICBtYW5pZm9sZC5wb2ludHNbaV0ubG9jYWxQb2ludCxcclxuICAgICAgICAgIGIyV29ybGRNYW5pZm9sZC5Jbml0aWFsaXplX3NfY2xpcFBvaW50LFxyXG4gICAgICAgICk7XHJcbiAgICAgICAgY29uc3QgczogbnVtYmVyID1cclxuICAgICAgICAgIHJhZGl1c0IgLSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKGNsaXBQb2ludCwgcGxhbmVQb2ludCwgYjJWZWMyLnNfdDApLCB0aGlzLm5vcm1hbCk7XHJcbiAgICAgICAgY29uc3QgY0I6IGIyVmVjMiA9IGIyVmVjMi5BZGRWTXVsU1YoXHJcbiAgICAgICAgICBjbGlwUG9pbnQsXHJcbiAgICAgICAgICBzLFxyXG4gICAgICAgICAgdGhpcy5ub3JtYWwsXHJcbiAgICAgICAgICBiMldvcmxkTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX2NCLFxyXG4gICAgICAgICk7XHJcbiAgICAgICAgY29uc3QgY0E6IGIyVmVjMiA9IGIyVmVjMi5TdWJWTXVsU1YoXHJcbiAgICAgICAgICBjbGlwUG9pbnQsXHJcbiAgICAgICAgICByYWRpdXNBLFxyXG4gICAgICAgICAgdGhpcy5ub3JtYWwsXHJcbiAgICAgICAgICBiMldvcmxkTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX2NBLFxyXG4gICAgICAgICk7XHJcbiAgICAgICAgYjJWZWMyLk1pZFZWKGNBLCBjQiwgdGhpcy5wb2ludHNbaV0pO1xyXG4gICAgICAgIHRoaXMuc2VwYXJhdGlvbnNbaV0gPSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKGNBLCBjQiwgYjJWZWMyLnNfdDApLCB0aGlzLm5vcm1hbCk7IC8vIGIyRG90KGNBIC0gY0IsIG5vcm1hbCk7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIEVuc3VyZSBub3JtYWwgcG9pbnRzIGZyb20gQSB0byBCLlxyXG4gICAgICB0aGlzLm5vcm1hbC5TZWxmTmVnKCk7XHJcbiAgICB9XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gVGhpcyBpcyB1c2VkIGZvciBkZXRlcm1pbmluZyB0aGUgc3RhdGUgb2YgY29udGFjdCBwb2ludHMuXHJcbmV4cG9ydCBjb25zdCBlbnVtIGIyUG9pbnRTdGF0ZSB7XHJcbiAgYjJfbnVsbFN0YXRlID0gMCwgLy8vPCBwb2ludCBkb2VzIG5vdCBleGlzdFxyXG4gIGIyX2FkZFN0YXRlID0gMSwgLy8vPCBwb2ludCB3YXMgYWRkZWQgaW4gdGhlIHVwZGF0ZVxyXG4gIGIyX3BlcnNpc3RTdGF0ZSA9IDIsIC8vLzwgcG9pbnQgcGVyc2lzdGVkIGFjcm9zcyB0aGUgdXBkYXRlXHJcbiAgYjJfcmVtb3ZlU3RhdGUgPSAzLCAvLy88IHBvaW50IHdhcyByZW1vdmVkIGluIHRoZSB1cGRhdGVcclxufVxyXG5cclxuLy8vIENvbXB1dGUgdGhlIHBvaW50IHN0YXRlcyBnaXZlbiB0d28gbWFuaWZvbGRzLiBUaGUgc3RhdGVzIHBlcnRhaW4gdG8gdGhlIHRyYW5zaXRpb24gZnJvbSBtYW5pZm9sZDFcclxuLy8vIHRvIG1hbmlmb2xkMi4gU28gc3RhdGUxIGlzIGVpdGhlciBwZXJzaXN0IG9yIHJlbW92ZSB3aGlsZSBzdGF0ZTIgaXMgZWl0aGVyIGFkZCBvciBwZXJzaXN0LlxyXG5leHBvcnQgZnVuY3Rpb24gYjJHZXRQb2ludFN0YXRlcyhcclxuICBzdGF0ZTE6IGIyUG9pbnRTdGF0ZVtdLFxyXG4gIHN0YXRlMjogYjJQb2ludFN0YXRlW10sXHJcbiAgbWFuaWZvbGQxOiBiMk1hbmlmb2xkLFxyXG4gIG1hbmlmb2xkMjogYjJNYW5pZm9sZCxcclxuKTogdm9pZCB7XHJcbiAgLy8gRGV0ZWN0IHBlcnNpc3RzIGFuZCByZW1vdmVzLlxyXG4gIGxldCBpOiBudW1iZXI7XHJcbiAgZm9yIChpID0gMDsgaSA8IG1hbmlmb2xkMS5wb2ludENvdW50OyArK2kpIHtcclxuICAgIGNvbnN0IGlkOiBiMkNvbnRhY3RJRCA9IG1hbmlmb2xkMS5wb2ludHNbaV0uaWQ7XHJcbiAgICBjb25zdCBrZXk6IG51bWJlciA9IGlkLmtleTtcclxuXHJcbiAgICBzdGF0ZTFbaV0gPSBiMlBvaW50U3RhdGUuYjJfcmVtb3ZlU3RhdGU7XHJcblxyXG4gICAgZm9yIChsZXQgaiA9IDAsIGpjdCA9IG1hbmlmb2xkMi5wb2ludENvdW50OyBqIDwgamN0OyArK2opIHtcclxuICAgICAgaWYgKG1hbmlmb2xkMi5wb2ludHNbal0uaWQua2V5ID09PSBrZXkpIHtcclxuICAgICAgICBzdGF0ZTFbaV0gPSBiMlBvaW50U3RhdGUuYjJfcGVyc2lzdFN0YXRlO1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG4gIGZvciAoOyBpIDwgYjJfbWF4TWFuaWZvbGRQb2ludHM7ICsraSkge1xyXG4gICAgc3RhdGUxW2ldID0gYjJQb2ludFN0YXRlLmIyX251bGxTdGF0ZTtcclxuICB9XHJcblxyXG4gIC8vIERldGVjdCBwZXJzaXN0cyBhbmQgYWRkcy5cclxuICBmb3IgKGkgPSAwOyBpIDwgbWFuaWZvbGQyLnBvaW50Q291bnQ7ICsraSkge1xyXG4gICAgY29uc3QgaWQ6IGIyQ29udGFjdElEID0gbWFuaWZvbGQyLnBvaW50c1tpXS5pZDtcclxuICAgIGNvbnN0IGtleTogbnVtYmVyID0gaWQua2V5O1xyXG5cclxuICAgIHN0YXRlMltpXSA9IGIyUG9pbnRTdGF0ZS5iMl9hZGRTdGF0ZTtcclxuXHJcbiAgICBmb3IgKGxldCBqID0gMCwgamN0ID0gbWFuaWZvbGQxLnBvaW50Q291bnQ7IGogPCBqY3Q7ICsraikge1xyXG4gICAgICBpZiAobWFuaWZvbGQxLnBvaW50c1tqXS5pZC5rZXkgPT09IGtleSkge1xyXG4gICAgICAgIHN0YXRlMltpXSA9IGIyUG9pbnRTdGF0ZS5iMl9wZXJzaXN0U3RhdGU7XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcbiAgZm9yICg7IGkgPCBiMl9tYXhNYW5pZm9sZFBvaW50czsgKytpKSB7XHJcbiAgICBzdGF0ZTJbaV0gPSBiMlBvaW50U3RhdGUuYjJfbnVsbFN0YXRlO1xyXG4gIH1cclxufVxyXG5cclxuLy8vIFVzZWQgZm9yIGNvbXB1dGluZyBjb250YWN0IG1hbmlmb2xkcy5cclxuZXhwb3J0IGNsYXNzIGIyQ2xpcFZlcnRleCB7XHJcbiAgcmVhZG9ubHkgdjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IGlkOiBiMkNvbnRhY3RJRCA9IG5ldyBiMkNvbnRhY3RJRCgpO1xyXG5cclxuICBzdGF0aWMgTWFrZUFycmF5KGxlbmd0aDogbnVtYmVyKTogYjJDbGlwVmVydGV4W10ge1xyXG4gICAgcmV0dXJuIGIyTWFrZUFycmF5KGxlbmd0aCwgKGk6IG51bWJlcik6IGIyQ2xpcFZlcnRleCA9PiBuZXcgYjJDbGlwVmVydGV4KCkpO1xyXG4gIH1cclxuXHJcbiAgQ29weShvdGhlcjogYjJDbGlwVmVydGV4KTogYjJDbGlwVmVydGV4IHtcclxuICAgIHRoaXMudi5Db3B5KG90aGVyLnYpO1xyXG4gICAgdGhpcy5pZC5Db3B5KG90aGVyLmlkKTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxufVxyXG5cclxuLy8vIFJheS1jYXN0IGlucHV0IGRhdGEuIFRoZSByYXkgZXh0ZW5kcyBmcm9tIHAxIHRvIHAxICsgbWF4RnJhY3Rpb24gKiAocDIgLSBwMSkuXHJcbmV4cG9ydCBjbGFzcyBiMlJheUNhc3RJbnB1dCB7XHJcbiAgcmVhZG9ubHkgcDE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBwMjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1heEZyYWN0aW9uID0gTmFOO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMubWF4RnJhY3Rpb24gPSAxLjA7XHJcbiAgfVxyXG5cclxuICBDb3B5KG86IGIyUmF5Q2FzdElucHV0KTogYjJSYXlDYXN0SW5wdXQge1xyXG4gICAgdGhpcy5wMS5Db3B5KG8ucDEpO1xyXG4gICAgdGhpcy5wMi5Db3B5KG8ucDIpO1xyXG4gICAgdGhpcy5tYXhGcmFjdGlvbiA9IG8ubWF4RnJhY3Rpb247XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcbn1cclxuXHJcbi8vLyBSYXktY2FzdCBvdXRwdXQgZGF0YS4gVGhlIHJheSBoaXRzIGF0IHAxICsgZnJhY3Rpb24gKiAocDIgLSBwMSksIHdoZXJlIHAxIGFuZCBwMlxyXG4vLy8gY29tZSBmcm9tIGIyUmF5Q2FzdElucHV0LlxyXG5leHBvcnQgY2xhc3MgYjJSYXlDYXN0T3V0cHV0IHtcclxuICByZWFkb25seSBub3JtYWw6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBmcmFjdGlvbiA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLmZyYWN0aW9uID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgQ29weShvOiBiMlJheUNhc3RPdXRwdXQpOiBiMlJheUNhc3RPdXRwdXQge1xyXG4gICAgdGhpcy5ub3JtYWwuQ29weShvLm5vcm1hbCk7XHJcbiAgICB0aGlzLmZyYWN0aW9uID0gby5mcmFjdGlvbjtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxufVxyXG5cclxuLy8vIEFuIGF4aXMgYWxpZ25lZCBib3VuZGluZyBib3guXHJcbmV4cG9ydCBjbGFzcyBiMkFBQkIge1xyXG4gIHJlYWRvbmx5IGxvd2VyQm91bmQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTsgLy8vPCB0aGUgbG93ZXIgdmVydGV4XHJcbiAgcmVhZG9ubHkgdXBwZXJCb3VuZDogYjJWZWMyID0gbmV3IGIyVmVjMigpOyAvLy88IHRoZSB1cHBlciB2ZXJ0ZXhcclxuXHJcbiAgcmVhZG9ubHkgbV9jYWNoZV9jZW50ZXI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTsgLy8gYWNjZXNzIHVzaW5nIEdldENlbnRlcigpXHJcbiAgcmVhZG9ubHkgbV9jYWNoZV9leHRlbnQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTsgLy8gYWNjZXNzIHVzaW5nIEdldEV4dGVudHMoKVxyXG5cclxuICBDb3B5KG86IGIyQUFCQik6IGIyQUFCQiB7XHJcbiAgICB0aGlzLmxvd2VyQm91bmQuQ29weShvLmxvd2VyQm91bmQpO1xyXG4gICAgdGhpcy51cHBlckJvdW5kLkNvcHkoby51cHBlckJvdW5kKTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgLy8vIFZlcmlmeSB0aGF0IHRoZSBib3VuZHMgYXJlIHNvcnRlZC5cclxuICBJc1ZhbGlkKCk6IGJvb2xlYW4ge1xyXG4gICAgaWYgKCF0aGlzLmxvd2VyQm91bmQuSXNWYWxpZCgpKSB7XHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuICAgIGlmICghdGhpcy51cHBlckJvdW5kLklzVmFsaWQoKSkge1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy51cHBlckJvdW5kLnggPCB0aGlzLmxvd2VyQm91bmQueCkge1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy51cHBlckJvdW5kLnkgPCB0aGlzLmxvd2VyQm91bmQueSkge1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gdHJ1ZTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGNlbnRlciBvZiB0aGUgQUFCQi5cclxuICBHZXRDZW50ZXIoKTogYjJWZWMyIHtcclxuICAgIHJldHVybiBiMlZlYzIuTWlkVlYodGhpcy5sb3dlckJvdW5kLCB0aGlzLnVwcGVyQm91bmQsIHRoaXMubV9jYWNoZV9jZW50ZXIpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgZXh0ZW50cyBvZiB0aGUgQUFCQiAoaGFsZi13aWR0aHMpLlxyXG4gIEdldEV4dGVudHMoKTogYjJWZWMyIHtcclxuICAgIHJldHVybiBiMlZlYzIuRXh0VlYodGhpcy5sb3dlckJvdW5kLCB0aGlzLnVwcGVyQm91bmQsIHRoaXMubV9jYWNoZV9leHRlbnQpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgcGVyaW1ldGVyIGxlbmd0aFxyXG4gIEdldFBlcmltZXRlcigpOiBudW1iZXIge1xyXG4gICAgY29uc3Qgd3g6IG51bWJlciA9IHRoaXMudXBwZXJCb3VuZC54IC0gdGhpcy5sb3dlckJvdW5kLng7XHJcbiAgICBjb25zdCB3eTogbnVtYmVyID0gdGhpcy51cHBlckJvdW5kLnkgLSB0aGlzLmxvd2VyQm91bmQueTtcclxuICAgIHJldHVybiAyICogKHd4ICsgd3kpO1xyXG4gIH1cclxuXHJcbiAgLy8vIENvbWJpbmUgYW4gQUFCQiBpbnRvIHRoaXMgb25lLlxyXG4gIENvbWJpbmUxKGFhYmI6IGIyQUFCQik6IGIyQUFCQiB7XHJcbiAgICB0aGlzLmxvd2VyQm91bmQueCA9IGIyTWluKHRoaXMubG93ZXJCb3VuZC54LCBhYWJiLmxvd2VyQm91bmQueCk7XHJcbiAgICB0aGlzLmxvd2VyQm91bmQueSA9IGIyTWluKHRoaXMubG93ZXJCb3VuZC55LCBhYWJiLmxvd2VyQm91bmQueSk7XHJcbiAgICB0aGlzLnVwcGVyQm91bmQueCA9IGIyTWF4KHRoaXMudXBwZXJCb3VuZC54LCBhYWJiLnVwcGVyQm91bmQueCk7XHJcbiAgICB0aGlzLnVwcGVyQm91bmQueSA9IGIyTWF4KHRoaXMudXBwZXJCb3VuZC55LCBhYWJiLnVwcGVyQm91bmQueSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIC8vLyBDb21iaW5lIHR3byBBQUJCcyBpbnRvIHRoaXMgb25lLlxyXG4gIENvbWJpbmUyKGFhYmIxOiBiMkFBQkIsIGFhYmIyOiBiMkFBQkIpOiBiMkFBQkIge1xyXG4gICAgdGhpcy5sb3dlckJvdW5kLnggPSBiMk1pbihhYWJiMS5sb3dlckJvdW5kLngsIGFhYmIyLmxvd2VyQm91bmQueCk7XHJcbiAgICB0aGlzLmxvd2VyQm91bmQueSA9IGIyTWluKGFhYmIxLmxvd2VyQm91bmQueSwgYWFiYjIubG93ZXJCb3VuZC55KTtcclxuICAgIHRoaXMudXBwZXJCb3VuZC54ID0gYjJNYXgoYWFiYjEudXBwZXJCb3VuZC54LCBhYWJiMi51cHBlckJvdW5kLngpO1xyXG4gICAgdGhpcy51cHBlckJvdW5kLnkgPSBiMk1heChhYWJiMS51cHBlckJvdW5kLnksIGFhYmIyLnVwcGVyQm91bmQueSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIHN0YXRpYyBDb21iaW5lKGFhYmIxOiBiMkFBQkIsIGFhYmIyOiBiMkFBQkIsIG91dDogYjJBQUJCKTogYjJBQUJCIHtcclxuICAgIG91dC5Db21iaW5lMihhYWJiMSwgYWFiYjIpO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIC8vLyBEb2VzIHRoaXMgYWFiYiBjb250YWluIHRoZSBwcm92aWRlZCBBQUJCLlxyXG4gIENvbnRhaW5zKGFhYmI6IGIyQUFCQik6IGJvb2xlYW4ge1xyXG4gICAgaWYgKHRoaXMubG93ZXJCb3VuZC54IDw9IGFhYmIubG93ZXJCb3VuZC54KSB7XHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLmxvd2VyQm91bmQueSA8PSBhYWJiLmxvd2VyQm91bmQueSkge1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbiAgICBpZiAoYWFiYi51cHBlckJvdW5kLnggPD0gdGhpcy51cHBlckJvdW5kLngpIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgaWYgKGFhYmIudXBwZXJCb3VuZC55IDw9IHRoaXMudXBwZXJCb3VuZC55KSB7XHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgLy8gRnJvbSBSZWFsLXRpbWUgQ29sbGlzaW9uIERldGVjdGlvbiwgcDE3OS5cclxuICBSYXlDYXN0KG91dHB1dDogYjJSYXlDYXN0T3V0cHV0LCBpbnB1dDogYjJSYXlDYXN0SW5wdXQpOiBib29sZWFuIHtcclxuICAgIGxldCB0bWluOiBudW1iZXIgPSAtYjJfbWF4RmxvYXQ7XHJcbiAgICBsZXQgdG1heDogbnVtYmVyID0gYjJfbWF4RmxvYXQ7XHJcblxyXG4gICAgY29uc3QgcF94OiBudW1iZXIgPSBpbnB1dC5wMS54O1xyXG4gICAgY29uc3QgcF95OiBudW1iZXIgPSBpbnB1dC5wMS55O1xyXG4gICAgY29uc3QgZF94OiBudW1iZXIgPSBpbnB1dC5wMi54IC0gaW5wdXQucDEueDtcclxuICAgIGNvbnN0IGRfeTogbnVtYmVyID0gaW5wdXQucDIueSAtIGlucHV0LnAxLnk7XHJcbiAgICBjb25zdCBhYnNEX3g6IG51bWJlciA9IGIyQWJzKGRfeCk7XHJcbiAgICBjb25zdCBhYnNEX3k6IG51bWJlciA9IGIyQWJzKGRfeSk7XHJcblxyXG4gICAgY29uc3Qgbm9ybWFsOiBiMlZlYzIgPSBvdXRwdXQubm9ybWFsO1xyXG5cclxuICAgIGlmIChhYnNEX3ggPCBiMl9lcHNpbG9uKSB7XHJcbiAgICAgIC8vIFBhcmFsbGVsLlxyXG4gICAgICBpZiAocF94IDwgdGhpcy5sb3dlckJvdW5kLnggfHwgdGhpcy51cHBlckJvdW5kLnggPCBwX3gpIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgIH1cclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIGNvbnN0IGludl9kOiBudW1iZXIgPSAxIC8gZF94O1xyXG4gICAgICBsZXQgdDE6IG51bWJlciA9ICh0aGlzLmxvd2VyQm91bmQueCAtIHBfeCkgKiBpbnZfZDtcclxuICAgICAgbGV0IHQyOiBudW1iZXIgPSAodGhpcy51cHBlckJvdW5kLnggLSBwX3gpICogaW52X2Q7XHJcblxyXG4gICAgICAvLyBTaWduIG9mIHRoZSBub3JtYWwgdmVjdG9yLlxyXG4gICAgICBsZXQgcyA9IC0xO1xyXG5cclxuICAgICAgaWYgKHQxID4gdDIpIHtcclxuICAgICAgICBjb25zdCB0MzogbnVtYmVyID0gdDE7XHJcbiAgICAgICAgdDEgPSB0MjtcclxuICAgICAgICB0MiA9IHQzO1xyXG4gICAgICAgIHMgPSAxO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBQdXNoIHRoZSBtaW4gdXBcclxuICAgICAgaWYgKHQxID4gdG1pbikge1xyXG4gICAgICAgIG5vcm1hbC54ID0gcztcclxuICAgICAgICBub3JtYWwueSA9IDA7XHJcbiAgICAgICAgdG1pbiA9IHQxO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBQdWxsIHRoZSBtYXggZG93blxyXG4gICAgICB0bWF4ID0gYjJNaW4odG1heCwgdDIpO1xyXG5cclxuICAgICAgaWYgKHRtaW4gPiB0bWF4KSB7XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGFic0RfeSA8IGIyX2Vwc2lsb24pIHtcclxuICAgICAgLy8gUGFyYWxsZWwuXHJcbiAgICAgIGlmIChwX3kgPCB0aGlzLmxvd2VyQm91bmQueSB8fCB0aGlzLnVwcGVyQm91bmQueSA8IHBfeSkge1xyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgY29uc3QgaW52X2Q6IG51bWJlciA9IDEgLyBkX3k7XHJcbiAgICAgIGxldCB0MTogbnVtYmVyID0gKHRoaXMubG93ZXJCb3VuZC55IC0gcF95KSAqIGludl9kO1xyXG4gICAgICBsZXQgdDI6IG51bWJlciA9ICh0aGlzLnVwcGVyQm91bmQueSAtIHBfeSkgKiBpbnZfZDtcclxuXHJcbiAgICAgIC8vIFNpZ24gb2YgdGhlIG5vcm1hbCB2ZWN0b3IuXHJcbiAgICAgIGxldCBzID0gLTE7XHJcblxyXG4gICAgICBpZiAodDEgPiB0Mikge1xyXG4gICAgICAgIGNvbnN0IHQzOiBudW1iZXIgPSB0MTtcclxuICAgICAgICB0MSA9IHQyO1xyXG4gICAgICAgIHQyID0gdDM7XHJcbiAgICAgICAgcyA9IDE7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIFB1c2ggdGhlIG1pbiB1cFxyXG4gICAgICBpZiAodDEgPiB0bWluKSB7XHJcbiAgICAgICAgbm9ybWFsLnggPSAwO1xyXG4gICAgICAgIG5vcm1hbC55ID0gcztcclxuICAgICAgICB0bWluID0gdDE7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIFB1bGwgdGhlIG1heCBkb3duXHJcbiAgICAgIHRtYXggPSBiMk1pbih0bWF4LCB0Mik7XHJcblxyXG4gICAgICBpZiAodG1pbiA+IHRtYXgpIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAvLyBEb2VzIHRoZSByYXkgc3RhcnQgaW5zaWRlIHRoZSBib3g/XHJcbiAgICAvLyBEb2VzIHRoZSByYXkgaW50ZXJzZWN0IGJleW9uZCB0aGUgbWF4IGZyYWN0aW9uP1xyXG4gICAgaWYgKHRtaW4gPCAwIHx8IGlucHV0Lm1heEZyYWN0aW9uIDwgdG1pbikge1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gSW50ZXJzZWN0aW9uLlxyXG4gICAgb3V0cHV0LmZyYWN0aW9uID0gdG1pbjtcclxuXHJcbiAgICByZXR1cm4gdHJ1ZTtcclxuICB9XHJcblxyXG4gIFRlc3RDb250YWluKHBvaW50OiBYWSk6IGJvb2xlYW4ge1xyXG4gICAgaWYgKHBvaW50LnggPCB0aGlzLmxvd2VyQm91bmQueCB8fCB0aGlzLnVwcGVyQm91bmQueCA8IHBvaW50LngpIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgaWYgKHBvaW50LnkgPCB0aGlzLmxvd2VyQm91bmQueSB8fCB0aGlzLnVwcGVyQm91bmQueSA8IHBvaW50LnkpIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRydWU7XHJcbiAgfVxyXG5cclxuICBUZXN0T3ZlcmxhcChvdGhlcjogYjJBQUJCKTogYm9vbGVhbiB7XHJcbiAgICBpZiAodGhpcy51cHBlckJvdW5kLnggPCBvdGhlci5sb3dlckJvdW5kLngpIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMudXBwZXJCb3VuZC55IDwgb3RoZXIubG93ZXJCb3VuZC55KSB7XHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuICAgIGlmIChvdGhlci51cHBlckJvdW5kLnggPCB0aGlzLmxvd2VyQm91bmQueCkge1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbiAgICBpZiAob3RoZXIudXBwZXJCb3VuZC55IDwgdGhpcy5sb3dlckJvdW5kLnkpIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRydWU7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJUZXN0T3ZlcmxhcEFBQkIoYTogYjJBQUJCLCBiOiBiMkFBQkIpOiBib29sZWFuIHtcclxuICBpZiAoYS51cHBlckJvdW5kLnggPCBiLmxvd2VyQm91bmQueCkge1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxuICBpZiAoYS51cHBlckJvdW5kLnkgPCBiLmxvd2VyQm91bmQueSkge1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxuICBpZiAoYi51cHBlckJvdW5kLnggPCBhLmxvd2VyQm91bmQueCkge1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxuICBpZiAoYi51cHBlckJvdW5kLnkgPCBhLmxvd2VyQm91bmQueSkge1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxuICByZXR1cm4gdHJ1ZTtcclxufVxyXG5cclxuLy8vIENsaXBwaW5nIGZvciBjb250YWN0IG1hbmlmb2xkcy5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyQ2xpcFNlZ21lbnRUb0xpbmUoXHJcbiAgdk91dDogYjJDbGlwVmVydGV4W10sXHJcbiAgdkluOiBiMkNsaXBWZXJ0ZXhbXSxcclxuICBub3JtYWw6IGIyVmVjMixcclxuICBvZmZzZXQ6IG51bWJlcixcclxuICB2ZXJ0ZXhJbmRleEE6IG51bWJlcixcclxuKTogbnVtYmVyIHtcclxuICAvLyBTdGFydCB3aXRoIG5vIG91dHB1dCBwb2ludHNcclxuICBsZXQgbnVtT3V0ID0gMDtcclxuXHJcbiAgY29uc3QgdkluMDogYjJDbGlwVmVydGV4ID0gdkluWzBdO1xyXG4gIGNvbnN0IHZJbjE6IGIyQ2xpcFZlcnRleCA9IHZJblsxXTtcclxuXHJcbiAgLy8gQ2FsY3VsYXRlIHRoZSBkaXN0YW5jZSBvZiBlbmQgcG9pbnRzIHRvIHRoZSBsaW5lXHJcbiAgY29uc3QgZGlzdGFuY2UwOiBudW1iZXIgPSBiMlZlYzIuRG90VlYobm9ybWFsLCB2SW4wLnYpIC0gb2Zmc2V0O1xyXG4gIGNvbnN0IGRpc3RhbmNlMTogbnVtYmVyID0gYjJWZWMyLkRvdFZWKG5vcm1hbCwgdkluMS52KSAtIG9mZnNldDtcclxuXHJcbiAgLy8gSWYgdGhlIHBvaW50cyBhcmUgYmVoaW5kIHRoZSBwbGFuZVxyXG4gIGlmIChkaXN0YW5jZTAgPD0gMCkge1xyXG4gICAgdk91dFtudW1PdXQrK10uQ29weSh2SW4wKTtcclxuICB9XHJcbiAgaWYgKGRpc3RhbmNlMSA8PSAwKSB7XHJcbiAgICB2T3V0W251bU91dCsrXS5Db3B5KHZJbjEpO1xyXG4gIH1cclxuXHJcbiAgLy8gSWYgdGhlIHBvaW50cyBhcmUgb24gZGlmZmVyZW50IHNpZGVzIG9mIHRoZSBwbGFuZVxyXG4gIGlmIChkaXN0YW5jZTAgKiBkaXN0YW5jZTEgPCAwKSB7XHJcbiAgICAvLyBGaW5kIGludGVyc2VjdGlvbiBwb2ludCBvZiBlZGdlIGFuZCBwbGFuZVxyXG4gICAgY29uc3QgaW50ZXJwOiBudW1iZXIgPSBkaXN0YW5jZTAgLyAoZGlzdGFuY2UwIC0gZGlzdGFuY2UxKTtcclxuICAgIGNvbnN0IHY6IGIyVmVjMiA9IHZPdXRbbnVtT3V0XS52O1xyXG4gICAgdi54ID0gdkluMC52LnggKyBpbnRlcnAgKiAodkluMS52LnggLSB2SW4wLnYueCk7XHJcbiAgICB2LnkgPSB2SW4wLnYueSArIGludGVycCAqICh2SW4xLnYueSAtIHZJbjAudi55KTtcclxuXHJcbiAgICAvLyBWZXJ0ZXhBIGlzIGhpdHRpbmcgZWRnZUIuXHJcbiAgICBjb25zdCBpZDogYjJDb250YWN0SUQgPSB2T3V0W251bU91dF0uaWQ7XHJcbiAgICBpZC5jZi5pbmRleEEgPSB2ZXJ0ZXhJbmRleEE7XHJcbiAgICBpZC5jZi5pbmRleEIgPSB2SW4wLmlkLmNmLmluZGV4QjtcclxuICAgIGlkLmNmLnR5cGVBID0gYjJDb250YWN0RmVhdHVyZVR5cGUuZV92ZXJ0ZXg7XHJcbiAgICBpZC5jZi50eXBlQiA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfZmFjZTtcclxuICAgICsrbnVtT3V0O1xyXG4gIH1cclxuXHJcbiAgcmV0dXJuIG51bU91dDtcclxufVxyXG5cclxuLy8vIERldGVybWluZSBpZiB0d28gZ2VuZXJpYyBzaGFwZXMgb3ZlcmxhcC5cclxuY29uc3QgYjJUZXN0T3ZlcmxhcFNoYXBlX3NfaW5wdXQ6IGIyRGlzdGFuY2VJbnB1dCA9IG5ldyBiMkRpc3RhbmNlSW5wdXQoKTtcclxuY29uc3QgYjJUZXN0T3ZlcmxhcFNoYXBlX3Nfc2ltcGxleENhY2hlOiBiMlNpbXBsZXhDYWNoZSA9IG5ldyBiMlNpbXBsZXhDYWNoZSgpO1xyXG5jb25zdCBiMlRlc3RPdmVybGFwU2hhcGVfc19vdXRwdXQ6IGIyRGlzdGFuY2VPdXRwdXQgPSBuZXcgYjJEaXN0YW5jZU91dHB1dCgpO1xyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyVGVzdE92ZXJsYXBTaGFwZShcclxuICBzaGFwZUE6IGIyU2hhcGUsXHJcbiAgaW5kZXhBOiBudW1iZXIsXHJcbiAgc2hhcGVCOiBiMlNoYXBlLFxyXG4gIGluZGV4QjogbnVtYmVyLFxyXG4gIHhmQTogYjJUcmFuc2Zvcm0sXHJcbiAgeGZCOiBiMlRyYW5zZm9ybSxcclxuKTogYm9vbGVhbiB7XHJcbiAgY29uc3QgaW5wdXQ6IGIyRGlzdGFuY2VJbnB1dCA9IGIyVGVzdE92ZXJsYXBTaGFwZV9zX2lucHV0LlJlc2V0KCk7XHJcbiAgaW5wdXQucHJveHlBLlNldFNoYXBlKHNoYXBlQSwgaW5kZXhBKTtcclxuICBpbnB1dC5wcm94eUIuU2V0U2hhcGUoc2hhcGVCLCBpbmRleEIpO1xyXG4gIGlucHV0LnRyYW5zZm9ybUEuQ29weSh4ZkEpO1xyXG4gIGlucHV0LnRyYW5zZm9ybUIuQ29weSh4ZkIpO1xyXG4gIGlucHV0LnVzZVJhZGlpID0gdHJ1ZTtcclxuXHJcbiAgY29uc3Qgc2ltcGxleENhY2hlOiBiMlNpbXBsZXhDYWNoZSA9IGIyVGVzdE92ZXJsYXBTaGFwZV9zX3NpbXBsZXhDYWNoZS5SZXNldCgpO1xyXG4gIHNpbXBsZXhDYWNoZS5jb3VudCA9IDA7XHJcblxyXG4gIGNvbnN0IG91dHB1dDogYjJEaXN0YW5jZU91dHB1dCA9IGIyVGVzdE92ZXJsYXBTaGFwZV9zX291dHB1dC5SZXNldCgpO1xyXG5cclxuICBiMkRpc3RhbmNlKG91dHB1dCwgc2ltcGxleENhY2hlLCBpbnB1dCk7XHJcblxyXG4gIHJldHVybiBvdXRwdXQuZGlzdGFuY2UgPCAxMCAqIGIyX2Vwc2lsb247XHJcbn1cclxuIl19