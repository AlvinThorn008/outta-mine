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
import { b2_epsilon, b2_epsilon_sq, b2_linearSlop, b2_maxFloat, b2_polygonRadius, b2Assert, } from '../../common/b2Settings';
import { b2Rot, b2Transform, b2Vec2 } from '../../common/b2Math';
import { b2MassData, b2Shape } from './b2Shape';
/// @see b2Shape::ComputeDistance
const ComputeDistance_s_pLocal = new b2Vec2();
const ComputeDistance_s_normalForMaxDistance = new b2Vec2();
const ComputeDistance_s_minDistance = new b2Vec2();
const ComputeDistance_s_distance = new b2Vec2();
// ComputeCentroid
const ComputeCentroid_s_pRef = new b2Vec2();
const ComputeCentroid_s_e1 = new b2Vec2();
const ComputeCentroid_s_e2 = new b2Vec2();
function ComputeCentroid(vs, count, out) {
    !!B2_DEBUG && b2Assert(count >= 3);
    const c = out;
    c.SetZero();
    let area = 0.0;
    // s is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    const pRef = ComputeCentroid_s_pRef.SetZero();
    /*
  #if 0
      // This code would put the reference point inside the polygon.
      for (let i = 0; i < count; ++i) {
        pRef.SelfAdd(vs[i]);
      }
      pRef.SelfMul(1 / count);
  #endif
      */
    const inv3 = 1.0 / 3.0;
    for (let i = 0; i < count; ++i) {
        // Triangle vertices.
        const p1 = pRef;
        const p2 = vs[i];
        const p3 = vs[(i + 1) % count];
        const e1 = b2Vec2.SubVV(p2, p1, ComputeCentroid_s_e1);
        const e2 = b2Vec2.SubVV(p3, p1, ComputeCentroid_s_e2);
        const D = b2Vec2.CrossVV(e1, e2);
        const triangleArea = 0.5 * D;
        area += triangleArea;
        // Area weighted centroid
        c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
        c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);
    }
    // Centroid
    !!B2_DEBUG && b2Assert(area > b2_epsilon);
    c.SelfMul(1.0 / area);
    return c;
}
/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// In most cases you should not need many vertices for a convex polygon.
export class b2PolygonShape extends b2Shape {
    constructor() {
        super(2 /* e_polygonShape */, b2_polygonRadius);
        this.m_centroid = new b2Vec2(0, 0);
        this.m_vertices = [new b2Vec2()];
        this.m_normals = [new b2Vec2()];
        this.m_count = 0;
    }
    /// Implement b2Shape.
    Clone() {
        return new b2PolygonShape().Copy(this);
    }
    Copy(other) {
        super.Copy(other);
        !!B2_DEBUG && b2Assert(other instanceof b2PolygonShape);
        this.m_centroid.Copy(other.m_centroid);
        this.m_count = other.m_count;
        this.m_vertices = b2Vec2.MakeArray(this.m_count);
        this.m_normals = b2Vec2.MakeArray(this.m_count);
        for (let i = 0; i < this.m_count; ++i) {
            this.m_vertices[i].Copy(other.m_vertices[i]);
            this.m_normals[i].Copy(other.m_normals[i]);
        }
        return this;
    }
    /// @see b2Shape::GetChildCount
    GetChildCount() {
        return 1;
    }
    Set(...args) {
        if (typeof args[0][0] === 'number') {
            const vertices = args[0];
            if (vertices.length % 2 !== 0) {
                throw new Error();
            }
            return this._Set((index) => ({
                x: vertices[index * 2],
                y: vertices[index * 2 + 1],
            }), vertices.length / 2);
        }
        else {
            const vertices = args[0];
            const count = args[1] || vertices.length;
            return this._Set((index) => vertices[index], count);
        }
    }
    _Set(vertices, count) {
        !!B2_DEBUG && b2Assert(3 <= count);
        if (count < 3) {
            return this.SetAsBox(1, 1);
        }
        let n = count;
        // Perform welding and copy vertices into local buffer.
        const ps = [];
        for (let i = 0; i < n; ++i) {
            const /*b2Vec2*/ v = vertices(i);
            let /*bool*/ unique = true;
            for (let /*int32*/ j = 0; j < ps.length; ++j) {
                if (b2Vec2.DistanceSquaredVV(v, ps[j]) < 0.5 * b2_linearSlop * (0.5 * b2_linearSlop)) {
                    unique = false;
                    break;
                }
            }
            if (unique) {
                ps.push(v);
            }
        }
        n = ps.length;
        if (n < 3) {
            // Polygon is degenerate.
            !!B2_DEBUG && b2Assert(false);
            return this.SetAsBox(1.0, 1.0);
        }
        // Create the convex hull using the Gift wrapping algorithm
        // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
        // Find the right most point on the hull
        let i0 = 0;
        let x0 = ps[0].x;
        for (let i = 1; i < n; ++i) {
            const x = ps[i].x;
            if (x > x0 || (x === x0 && ps[i].y < ps[i0].y)) {
                i0 = i;
                x0 = x;
            }
        }
        const hull = [];
        let m = 0;
        let ih = i0;
        for (;;) {
            hull[m] = ih;
            let ie = 0;
            for (let j = 1; j < n; ++j) {
                if (ie === ih) {
                    ie = j;
                    continue;
                }
                const r = b2Vec2.SubVV(ps[ie], ps[hull[m]], b2PolygonShape.Set_s_r);
                const v = b2Vec2.SubVV(ps[j], ps[hull[m]], b2PolygonShape.Set_s_v);
                const c = b2Vec2.CrossVV(r, v);
                if (c < 0) {
                    ie = j;
                }
                // Collinearity check
                if (c === 0 && v.LengthSquared() > r.LengthSquared()) {
                    ie = j;
                }
            }
            ++m;
            ih = ie;
            if (ie === i0) {
                break;
            }
        }
        this.m_count = m;
        this.m_vertices = b2Vec2.MakeArray(this.m_count);
        this.m_normals = b2Vec2.MakeArray(this.m_count);
        // Copy vertices.
        for (let i = 0; i < m; ++i) {
            this.m_vertices[i].Copy(ps[hull[i]]);
        }
        // Compute normals. Ensure the edges have non-zero length.
        for (let i = 0; i < m; ++i) {
            const vertexi1 = this.m_vertices[i];
            const vertexi2 = this.m_vertices[(i + 1) % m];
            const edge = b2Vec2.SubVV(vertexi2, vertexi1, b2Vec2.s_t0); // edge uses s_t0
            !!B2_DEBUG && b2Assert(edge.LengthSquared() > b2_epsilon_sq);
            b2Vec2.CrossVOne(edge, this.m_normals[i]).SelfNormalize();
        }
        // Compute the polygon centroid.
        ComputeCentroid(this.m_vertices, m, this.m_centroid);
        return this;
    }
    /// Build vertices to represent an axis-aligned box or an oriented box.
    /// @param hx the half-width.
    /// @param hy the half-height.
    /// @param center the center of the box in local coordinates.
    /// @param angle the rotation of the box in local coordinates.
    SetAsBox(hx, hy, center, angle = 0) {
        this.m_count = 4;
        this.m_vertices = b2Vec2.MakeArray(4);
        this.m_normals = b2Vec2.MakeArray(4);
        this.m_vertices[0].Set(-hx, -hy);
        this.m_vertices[1].Set(hx, -hy);
        this.m_vertices[2].Set(hx, hy);
        this.m_vertices[3].Set(-hx, hy);
        this.m_normals[0].Set(0, -1);
        this.m_normals[1].Set(1, 0);
        this.m_normals[2].Set(0, 1);
        this.m_normals[3].Set(-1, 0);
        this.m_centroid.SetZero();
        if (center) {
            this.m_centroid.Copy(center);
            const xf = new b2Transform();
            xf.SetPosition(center);
            xf.SetRotationAngle(angle);
            // Transform vertices and normals.
            for (let i = 0; i < this.m_count; ++i) {
                b2Transform.MulXV(xf, this.m_vertices[i], this.m_vertices[i]);
                b2Rot.MulRV(xf.q, this.m_normals[i], this.m_normals[i]);
            }
        }
        return this;
    }
    TestPoint(xf, p) {
        const pLocal = b2Transform.MulTXV(xf, p, b2PolygonShape.TestPoint_s_pLocal);
        for (let i = 0; i < this.m_count; ++i) {
            const dot = b2Vec2.DotVV(this.m_normals[i], b2Vec2.SubVV(pLocal, this.m_vertices[i], b2Vec2.s_t0));
            if (dot > 0) {
                return false;
            }
        }
        return true;
    }
    ComputeDistance(xf, p, normal, childIndex) {
        if (B2_ENABLE_PARTICLE) {
            const pLocal = b2Transform.MulTXV(xf, p, ComputeDistance_s_pLocal);
            let maxDistance = -b2_maxFloat;
            const normalForMaxDistance = ComputeDistance_s_normalForMaxDistance.Copy(pLocal);
            for (let i = 0; i < this.m_count; ++i) {
                const dot = b2Vec2.DotVV(this.m_normals[i], b2Vec2.SubVV(pLocal, this.m_vertices[i], b2Vec2.s_t0));
                if (dot > maxDistance) {
                    maxDistance = dot;
                    normalForMaxDistance.Copy(this.m_normals[i]);
                }
            }
            if (maxDistance > 0) {
                const minDistance = ComputeDistance_s_minDistance.Copy(normalForMaxDistance);
                let minDistance2 = maxDistance * maxDistance;
                for (let i = 0; i < this.m_count; ++i) {
                    const distance = b2Vec2.SubVV(pLocal, this.m_vertices[i], ComputeDistance_s_distance);
                    const distance2 = distance.LengthSquared();
                    if (minDistance2 > distance2) {
                        minDistance.Copy(distance);
                        minDistance2 = distance2;
                    }
                }
                b2Rot.MulRV(xf.q, minDistance, normal);
                normal.Normalize();
                return Math.sqrt(minDistance2);
            }
            else {
                b2Rot.MulRV(xf.q, normalForMaxDistance, normal);
                return maxDistance;
            }
        }
        else {
            return 0.0;
        }
    }
    RayCast(output, input, xf, childIndex) {
        // Put the ray into the polygon's frame of reference.
        const p1 = b2Transform.MulTXV(xf, input.p1, b2PolygonShape.RayCast_s_p1);
        const p2 = b2Transform.MulTXV(xf, input.p2, b2PolygonShape.RayCast_s_p2);
        const d = b2Vec2.SubVV(p2, p1, b2PolygonShape.RayCast_s_d);
        let lower = 0, upper = input.maxFraction;
        let index = -1;
        for (let i = 0; i < this.m_count; ++i) {
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            const numerator = b2Vec2.DotVV(this.m_normals[i], b2Vec2.SubVV(this.m_vertices[i], p1, b2Vec2.s_t0));
            const denominator = b2Vec2.DotVV(this.m_normals[i], d);
            if (denominator === 0) {
                if (numerator < 0) {
                    return false;
                }
            }
            else {
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower > numerator.
                if (denominator < 0 && numerator < lower * denominator) {
                    // Increase lower.
                    // The segment enters this half-space.
                    lower = numerator / denominator;
                    index = i;
                }
                else if (denominator > 0 && numerator < upper * denominator) {
                    // Decrease upper.
                    // The segment exits this half-space.
                    upper = numerator / denominator;
                }
            }
            // The use of epsilon here causes the assert on lower to trip
            // in some cases. Apparently the use of epsilon was to make edge
            // shapes work, but now those are handled separately.
            // if (upper < lower - b2_epsilon)
            if (upper < lower) {
                return false;
            }
        }
        !!B2_DEBUG && b2Assert(0 <= lower && lower <= input.maxFraction);
        if (index >= 0) {
            output.fraction = lower;
            b2Rot.MulRV(xf.q, this.m_normals[index], output.normal);
            return true;
        }
        return false;
    }
    ComputeAABB(aabb, xf, childIndex) {
        const lower = b2Transform.MulXV(xf, this.m_vertices[0], aabb.lowerBound);
        const upper = aabb.upperBound.Copy(lower);
        for (let i = 0; i < this.m_count; ++i) {
            const v = b2Transform.MulXV(xf, this.m_vertices[i], b2PolygonShape.ComputeAABB_s_v);
            b2Vec2.MinV(v, lower, lower);
            b2Vec2.MaxV(v, upper, upper);
        }
        const r = this.m_radius;
        lower.SelfSubXY(r, r);
        upper.SelfAddXY(r, r);
    }
    ComputeMass(massData, density) {
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.
        !!B2_DEBUG && b2Assert(this.m_count >= 3);
        const center = b2PolygonShape.ComputeMass_s_center.SetZero();
        let area = 0;
        let I = 0;
        // s is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        const s = b2PolygonShape.ComputeMass_s_s.SetZero();
        // This code would put the reference point inside the polygon.
        for (let i = 0; i < this.m_count; ++i) {
            s.SelfAdd(this.m_vertices[i]);
        }
        s.SelfMul(1 / this.m_count);
        const k_inv3 = 1 / 3;
        for (let i = 0; i < this.m_count; ++i) {
            // Triangle vertices.
            const e1 = b2Vec2.SubVV(this.m_vertices[i], s, b2PolygonShape.ComputeMass_s_e1);
            const e2 = b2Vec2.SubVV(this.m_vertices[(i + 1) % this.m_count], s, b2PolygonShape.ComputeMass_s_e2);
            const D = b2Vec2.CrossVV(e1, e2);
            const triangleArea = 0.5 * D;
            area += triangleArea;
            // Area weighted centroid
            center.SelfAdd(b2Vec2.MulSV(triangleArea * k_inv3, b2Vec2.AddVV(e1, e2, b2Vec2.s_t0), b2Vec2.s_t1));
            const ex1 = e1.x;
            const ey1 = e1.y;
            const ex2 = e2.x;
            const ey2 = e2.y;
            const intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            const inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;
            I += 0.25 * k_inv3 * D * (intx2 + inty2);
        }
        // Total mass
        massData.mass = density * area;
        // Center of mass
        !!B2_DEBUG && b2Assert(area > b2_epsilon);
        center.SelfMul(1 / area);
        b2Vec2.AddVV(center, s, massData.center);
        // Inertia tensor relative to the local origin (point s).
        massData.I = density * I;
        // Shift to center of mass then to original body origin.
        massData.I +=
            massData.mass *
                (b2Vec2.DotVV(massData.center, massData.center) - b2Vec2.DotVV(center, center));
    }
    Validate() {
        for (let i = 0; i < this.m_count; ++i) {
            const i1 = i;
            const i2 = (i + 1) % this.m_count;
            const p = this.m_vertices[i1];
            const e = b2Vec2.SubVV(this.m_vertices[i2], p, b2PolygonShape.Validate_s_e);
            for (let j = 0; j < this.m_count; ++j) {
                if (j === i1 || j === i2) {
                    continue;
                }
                const v = b2Vec2.SubVV(this.m_vertices[j], p, b2PolygonShape.Validate_s_v);
                const c = b2Vec2.CrossVV(e, v);
                if (c < 0) {
                    return false;
                }
            }
        }
        return true;
    }
    SetupDistanceProxy(proxy, index) {
        proxy.m_vertices = this.m_vertices;
        proxy.m_count = this.m_count;
        proxy.m_radius = this.m_radius;
    }
    ComputeSubmergedArea(normal, offset, xf, c) {
        // Transform plane into shape co-ordinates
        const normalL = b2Rot.MulTRV(xf.q, normal, b2PolygonShape.ComputeSubmergedArea_s_normalL);
        const offsetL = offset - b2Vec2.DotVV(normal, xf.p);
        const depths = [];
        let diveCount = 0;
        let intoIndex = -1;
        let outoIndex = -1;
        let lastSubmerged = false;
        for (let i = 0; i < this.m_count; ++i) {
            depths[i] = b2Vec2.DotVV(normalL, this.m_vertices[i]) - offsetL;
            const isSubmerged = depths[i] < -b2_epsilon;
            if (i > 0) {
                if (isSubmerged) {
                    if (!lastSubmerged) {
                        intoIndex = i - 1;
                        diveCount++;
                    }
                }
                else {
                    if (lastSubmerged) {
                        outoIndex = i - 1;
                        diveCount++;
                    }
                }
            }
            lastSubmerged = isSubmerged;
        }
        switch (diveCount) {
            case 0:
                if (lastSubmerged) {
                    // Completely submerged
                    const md = b2PolygonShape.ComputeSubmergedArea_s_md;
                    this.ComputeMass(md, 1);
                    b2Transform.MulXV(xf, md.center, c);
                    return md.mass;
                }
                else {
                    // Completely dry
                    return 0;
                }
            case 1:
                if (intoIndex === -1) {
                    intoIndex = this.m_count - 1;
                }
                else {
                    outoIndex = this.m_count - 1;
                }
                break;
        }
        const intoIndex2 = (intoIndex + 1) % this.m_count;
        const outoIndex2 = (outoIndex + 1) % this.m_count;
        const intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
        const outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
        const intoVec = b2PolygonShape.ComputeSubmergedArea_s_intoVec.Set(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda, this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda);
        const outoVec = b2PolygonShape.ComputeSubmergedArea_s_outoVec.Set(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda, this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda);
        // Initialize accumulator
        let area = 0;
        const center = b2PolygonShape.ComputeSubmergedArea_s_center.SetZero();
        let p2 = this.m_vertices[intoIndex2];
        let p3;
        // An awkward loop from intoIndex2+1 to outIndex2
        let i = intoIndex2;
        while (i !== outoIndex2) {
            i = (i + 1) % this.m_count;
            if (i === outoIndex2) {
                p3 = outoVec;
            }
            else {
                p3 = this.m_vertices[i];
            }
            const triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
            area += triangleArea;
            // Area weighted centroid
            center.x += (triangleArea * (intoVec.x + p2.x + p3.x)) / 3;
            center.y += (triangleArea * (intoVec.y + p2.y + p3.y)) / 3;
            p2 = p3;
        }
        // Normalize and transform centroid
        center.SelfMul(1 / area);
        b2Transform.MulXV(xf, center, c);
        return area;
    }
}
/// Create a convex hull from the given array of points.
/// @warning the points may be re-ordered, even if they form a convex polygon
/// @warning collinear points are handled but not removed. Collinear points
/// may lead to poor stacking behavior.
b2PolygonShape.Set_s_r = new b2Vec2();
b2PolygonShape.Set_s_v = new b2Vec2();
/// @see b2Shape::TestPoint
b2PolygonShape.TestPoint_s_pLocal = new b2Vec2();
/// Implement b2Shape.
b2PolygonShape.RayCast_s_p1 = new b2Vec2();
b2PolygonShape.RayCast_s_p2 = new b2Vec2();
b2PolygonShape.RayCast_s_d = new b2Vec2();
/// @see b2Shape::ComputeAABB
b2PolygonShape.ComputeAABB_s_v = new b2Vec2();
/// @see b2Shape::ComputeMass
b2PolygonShape.ComputeMass_s_center = new b2Vec2();
b2PolygonShape.ComputeMass_s_s = new b2Vec2();
b2PolygonShape.ComputeMass_s_e1 = new b2Vec2();
b2PolygonShape.ComputeMass_s_e2 = new b2Vec2();
b2PolygonShape.Validate_s_e = new b2Vec2();
b2PolygonShape.Validate_s_v = new b2Vec2();
b2PolygonShape.ComputeSubmergedArea_s_normalL = new b2Vec2();
b2PolygonShape.ComputeSubmergedArea_s_md = new b2MassData();
b2PolygonShape.ComputeSubmergedArea_s_intoVec = new b2Vec2();
b2PolygonShape.ComputeSubmergedArea_s_outoVec = new b2Vec2();
b2PolygonShape.ComputeSubmergedArea_s_center = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQb2x5Z29uU2hhcGUuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9zcmMvY29sbGlzaW9uL3NoYXBlcy9iMlBvbHlnb25TaGFwZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFDTCxVQUFVLEVBQ1YsYUFBYSxFQUNiLGFBQWEsRUFDYixXQUFXLEVBQ1gsZ0JBQWdCLEVBQ2hCLFFBQVEsR0FDVCxNQUFNLHlCQUF5QixDQUFDO0FBQ2pDLE9BQU8sRUFBRSxLQUFLLEVBQUUsV0FBVyxFQUFFLE1BQU0sRUFBTSxNQUFNLHFCQUFxQixDQUFDO0FBR3JFLE9BQU8sRUFBRSxVQUFVLEVBQUUsT0FBTyxFQUFlLE1BQU0sV0FBVyxDQUFDO0FBRTdELGlDQUFpQztBQUNqQyxNQUFNLHdCQUF3QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDOUMsTUFBTSxzQ0FBc0MsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQzVELE1BQU0sNkJBQTZCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNuRCxNQUFNLDBCQUEwQixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFFaEQsa0JBQWtCO0FBQ2xCLE1BQU0sc0JBQXNCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUM1QyxNQUFNLG9CQUFvQixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDMUMsTUFBTSxvQkFBb0IsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBRTFDLFNBQVMsZUFBZSxDQUFDLEVBQVksRUFBRSxLQUFhLEVBQUUsR0FBVztJQUMvRCxDQUFDLENBQUMsUUFBUSxJQUFJLFFBQVEsQ0FBQyxLQUFLLElBQUksQ0FBQyxDQUFDLENBQUM7SUFFbkMsTUFBTSxDQUFDLEdBQVcsR0FBRyxDQUFDO0lBQ3RCLENBQUMsQ0FBQyxPQUFPLEVBQUUsQ0FBQztJQUNaLElBQUksSUFBSSxHQUFHLEdBQUcs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TUFBTSxFQUFFLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMDYtMjAwOSBFcmluIENhdHRvIGh0dHA6Ly93d3cuYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7XHJcbiAgYjJfZXBzaWxvbixcclxuICBiMl9lcHNpbG9uX3NxLFxyXG4gIGIyX2xpbmVhclNsb3AsXHJcbiAgYjJfbWF4RmxvYXQsXHJcbiAgYjJfcG9seWdvblJhZGl1cyxcclxuICBiMkFzc2VydCxcclxufSBmcm9tICcuLi8uLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7IGIyUm90LCBiMlRyYW5zZm9ybSwgYjJWZWMyLCBYWSB9IGZyb20gJy4uLy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMkFBQkIsIGIyUmF5Q2FzdElucHV0LCBiMlJheUNhc3RPdXRwdXQgfSBmcm9tICcuLi9iMkNvbGxpc2lvbic7XHJcbmltcG9ydCB7IGIyRGlzdGFuY2VQcm94eSB9IGZyb20gJy4uL2IyRGlzdGFuY2UnO1xyXG5pbXBvcnQgeyBiMk1hc3NEYXRhLCBiMlNoYXBlLCBiMlNoYXBlVHlwZSB9IGZyb20gJy4vYjJTaGFwZSc7XHJcblxyXG4vLy8gQHNlZSBiMlNoYXBlOjpDb21wdXRlRGlzdGFuY2VcclxuY29uc3QgQ29tcHV0ZURpc3RhbmNlX3NfcExvY2FsID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBDb21wdXRlRGlzdGFuY2Vfc19ub3JtYWxGb3JNYXhEaXN0YW5jZSA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgQ29tcHV0ZURpc3RhbmNlX3NfbWluRGlzdGFuY2UgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IENvbXB1dGVEaXN0YW5jZV9zX2Rpc3RhbmNlID0gbmV3IGIyVmVjMigpO1xyXG5cclxuLy8gQ29tcHV0ZUNlbnRyb2lkXHJcbmNvbnN0IENvbXB1dGVDZW50cm9pZF9zX3BSZWYgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IENvbXB1dGVDZW50cm9pZF9zX2UxID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBDb21wdXRlQ2VudHJvaWRfc19lMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbmZ1bmN0aW9uIENvbXB1dGVDZW50cm9pZCh2czogYjJWZWMyW10sIGNvdW50OiBudW1iZXIsIG91dDogYjJWZWMyKTogYjJWZWMyIHtcclxuICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGNvdW50ID49IDMpO1xyXG5cclxuICBjb25zdCBjOiBiMlZlYzIgPSBvdXQ7XHJcbiAgYy5TZXRaZXJvKCk7XHJcbiAgbGV0IGFyZWEgPSAwLjA7XHJcblxyXG4gIC8vIHMgaXMgdGhlIHJlZmVyZW5jZSBwb2ludCBmb3IgZm9ybWluZyB0cmlhbmdsZXMuXHJcbiAgLy8gSXQncyBsb2NhdGlvbiBkb2Vzbid0IGNoYW5nZSB0aGUgcmVzdWx0IChleGNlcHQgZm9yIHJvdW5kaW5nIGVycm9yKS5cclxuICBjb25zdCBwUmVmID0gQ29tcHV0ZUNlbnRyb2lkX3NfcFJlZi5TZXRaZXJvKCk7XHJcbiAgLypcclxuI2lmIDBcclxuICAgIC8vIFRoaXMgY29kZSB3b3VsZCBwdXQgdGhlIHJlZmVyZW5jZSBwb2ludCBpbnNpZGUgdGhlIHBvbHlnb24uXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGNvdW50OyArK2kpIHtcclxuICAgICAgcFJlZi5TZWxmQWRkKHZzW2ldKTtcclxuICAgIH1cclxuICAgIHBSZWYuU2VsZk11bCgxIC8gY291bnQpO1xyXG4jZW5kaWZcclxuICAgICovXHJcblxyXG4gIGNvbnN0IGludjMgPSAxLjAgLyAzLjA7XHJcblxyXG4gIGZvciAobGV0IGkgPSAwOyBpIDwgY291bnQ7ICsraSkge1xyXG4gICAgLy8gVHJpYW5nbGUgdmVydGljZXMuXHJcbiAgICBjb25zdCBwMSA9IHBSZWY7XHJcbiAgICBjb25zdCBwMiA9IHZzW2ldO1xyXG4gICAgY29uc3QgcDMgPSB2c1soaSArIDEpICUgY291bnRdO1xyXG5cclxuICAgIGNvbnN0IGUxID0gYjJWZWMyLlN1YlZWKHAyLCBwMSwgQ29tcHV0ZUNlbnRyb2lkX3NfZTEpO1xyXG4gICAgY29uc3QgZTIgPSBiMlZlYzIuU3ViVlYocDMsIHAxLCBDb21wdXRlQ2VudHJvaWRfc19lMik7XHJcblxyXG4gICAgY29uc3QgRCA9IGIyVmVjMi5Dcm9zc1ZWKGUxLCBlMik7XHJcblxyXG4gICAgY29uc3QgdHJpYW5nbGVBcmVhOiBudW1iZXIgPSAwLjUgKiBEO1xyXG4gICAgYXJlYSArPSB0cmlhbmdsZUFyZWE7XHJcblxyXG4gICAgLy8gQXJlYSB3ZWlnaHRlZCBjZW50cm9pZFxyXG4gICAgYy54ICs9IHRyaWFuZ2xlQXJlYSAqIGludjMgKiAocDEueCArIHAyLnggKyBwMy54KTtcclxuICAgIGMueSArPSB0cmlhbmdsZUFyZWEgKiBpbnYzICogKHAxLnkgKyBwMi55ICsgcDMueSk7XHJcbiAgfVxyXG5cclxuICAvLyBDZW50cm9pZFxyXG4gICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYXJlYSA+IGIyX2Vwc2lsb24pO1xyXG4gIGMuU2VsZk11bCgxLjAgLyBhcmVhKTtcclxuICByZXR1cm4gYztcclxufVxyXG5cclxuLy8vIEEgY29udmV4IHBvbHlnb24uIEl0IGlzIGFzc3VtZWQgdGhhdCB0aGUgaW50ZXJpb3Igb2YgdGhlIHBvbHlnb24gaXMgdG9cclxuLy8vIHRoZSBsZWZ0IG9mIGVhY2ggZWRnZS5cclxuLy8vIEluIG1vc3QgY2FzZXMgeW91IHNob3VsZCBub3QgbmVlZCBtYW55IHZlcnRpY2VzIGZvciBhIGNvbnZleCBwb2x5Z29uLlxyXG5leHBvcnQgY2xhc3MgYjJQb2x5Z29uU2hhcGUgZXh0ZW5kcyBiMlNoYXBlIHtcclxuICByZWFkb25seSBtX2NlbnRyb2lkOiBiMlZlYzIgPSBuZXcgYjJWZWMyKDAsIDApO1xyXG4gIG1fdmVydGljZXM6IGIyVmVjMltdID0gW25ldyBiMlZlYzIoKV07XHJcbiAgbV9ub3JtYWxzOiBiMlZlYzJbXSA9IFtuZXcgYjJWZWMyKCldO1xyXG4gIG1fY291bnQgPSAwO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHN1cGVyKGIyU2hhcGVUeXBlLmVfcG9seWdvblNoYXBlLCBiMl9wb2x5Z29uUmFkaXVzKTtcclxuICB9XHJcblxyXG4gIC8vLyBJbXBsZW1lbnQgYjJTaGFwZS5cclxuICBDbG9uZSgpOiBiMlBvbHlnb25TaGFwZSB7XHJcbiAgICByZXR1cm4gbmV3IGIyUG9seWdvblNoYXBlKCkuQ29weSh0aGlzKTtcclxuICB9XHJcblxyXG4gIENvcHkob3RoZXI6IGIyUG9seWdvblNoYXBlKTogYjJQb2x5Z29uU2hhcGUge1xyXG4gICAgc3VwZXIuQ29weShvdGhlcik7XHJcblxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChvdGhlciBpbnN0YW5jZW9mIGIyUG9seWdvblNoYXBlKTtcclxuXHJcbiAgICB0aGlzLm1fY2VudHJvaWQuQ29weShvdGhlci5tX2NlbnRyb2lkKTtcclxuICAgIHRoaXMubV9jb3VudCA9IG90aGVyLm1fY291bnQ7XHJcbiAgICB0aGlzLm1fdmVydGljZXMgPSBiMlZlYzIuTWFrZUFycmF5KHRoaXMubV9jb3VudCk7XHJcbiAgICB0aGlzLm1fbm9ybWFscyA9IGIyVmVjMi5NYWtlQXJyYXkodGhpcy5tX2NvdW50KTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgdGhpcy5tX3ZlcnRpY2VzW2ldLkNvcHkob3RoZXIubV92ZXJ0aWNlc1tpXSk7XHJcbiAgICAgIHRoaXMubV9ub3JtYWxzW2ldLkNvcHkob3RoZXIubV9ub3JtYWxzW2ldKTtcclxuICAgIH1cclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgLy8vIEBzZWUgYjJTaGFwZTo6R2V0Q2hpbGRDb3VudFxyXG4gIEdldENoaWxkQ291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiAxO1xyXG4gIH1cclxuXHJcbiAgLy8vIENyZWF0ZSBhIGNvbnZleCBodWxsIGZyb20gdGhlIGdpdmVuIGFycmF5IG9mIHBvaW50cy5cclxuICAvLy8gQHdhcm5pbmcgdGhlIHBvaW50cyBtYXkgYmUgcmUtb3JkZXJlZCwgZXZlbiBpZiB0aGV5IGZvcm0gYSBjb252ZXggcG9seWdvblxyXG4gIC8vLyBAd2FybmluZyBjb2xsaW5lYXIgcG9pbnRzIGFyZSBoYW5kbGVkIGJ1dCBub3QgcmVtb3ZlZC4gQ29sbGluZWFyIHBvaW50c1xyXG4gIC8vLyBtYXkgbGVhZCB0byBwb29yIHN0YWNraW5nIGJlaGF2aW9yLlxyXG4gIHByaXZhdGUgc3RhdGljIFNldF9zX3IgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU2V0X3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU2V0KHZlcnRpY2VzOiBYWVtdKTogYjJQb2x5Z29uU2hhcGU7XHJcbiAgU2V0KHZlcnRpY2VzOiBYWVtdLCBjb3VudDogbnVtYmVyKTogYjJQb2x5Z29uU2hhcGU7XHJcbiAgU2V0KHZlcnRpY2VzOiBudW1iZXJbXSk6IGIyUG9seWdvblNoYXBlO1xyXG4gIFNldCguLi5hcmdzOiBhbnlbXSk6IGIyUG9seWdvblNoYXBlIHtcclxuICAgIGlmICh0eXBlb2YgYXJnc1swXVswXSA9PT0gJ251bWJlcicpIHtcclxuICAgICAgY29uc3QgdmVydGljZXM6IG51bWJlcltdID0gYXJnc1swXTtcclxuICAgICAgaWYgKHZlcnRpY2VzLmxlbmd0aCAlIDIgIT09IDApIHtcclxuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgICAgfVxyXG4gICAgICByZXR1cm4gdGhpcy5fU2V0KFxyXG4gICAgICAgIChpbmRleDogbnVtYmVyKTogWFkgPT4gKHtcclxuICAgICAgICAgIHg6IHZlcnRpY2VzW2luZGV4ICogMl0sXHJcbiAgICAgICAgICB5OiB2ZXJ0aWNlc1tpbmRleCAqIDIgKyAxXSxcclxuICAgICAgICB9KSxcclxuICAgICAgICB2ZXJ0aWNlcy5sZW5ndGggLyAyLFxyXG4gICAgICApO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgY29uc3QgdmVydGljZXM6IFhZW10gPSBhcmdzWzBdO1xyXG4gICAgICBjb25zdCBjb3VudDogbnVtYmVyID0gYXJnc1sxXSB8fCB2ZXJ0aWNlcy5sZW5ndGg7XHJcbiAgICAgIHJldHVybiB0aGlzLl9TZXQoKGluZGV4OiBudW1iZXIpOiBYWSA9PiB2ZXJ0aWNlc1tpbmRleF0sIGNvdW50KTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIF9TZXQodmVydGljZXM6IChpbmRleDogbnVtYmVyKSA9PiBYWSwgY291bnQ6IG51bWJlcik6IGIyUG9seWdvblNoYXBlIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoMyA8PSBjb3VudCk7XHJcbiAgICBpZiAoY291bnQgPCAzKSB7XHJcbiAgICAgIHJldHVybiB0aGlzLlNldEFzQm94KDEsIDEpO1xyXG4gICAgfVxyXG5cclxuICAgIGxldCBuOiBudW1iZXIgPSBjb3VudDtcclxuXHJcbiAgICAvLyBQZXJmb3JtIHdlbGRpbmcgYW5kIGNvcHkgdmVydGljZXMgaW50byBsb2NhbCBidWZmZXIuXHJcbiAgICBjb25zdCBwczogWFlbXSA9IFtdO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBuOyArK2kpIHtcclxuICAgICAgY29uc3QgLypiMlZlYzIqLyB2ID0gdmVydGljZXMoaSk7XHJcblxyXG4gICAgICBsZXQgLypib29sKi8gdW5pcXVlID0gdHJ1ZTtcclxuICAgICAgZm9yIChsZXQgLyppbnQzMiovIGogPSAwOyBqIDwgcHMubGVuZ3RoOyArK2opIHtcclxuICAgICAgICBpZiAoYjJWZWMyLkRpc3RhbmNlU3F1YXJlZFZWKHYsIHBzW2pdKSA8IDAuNSAqIGIyX2xpbmVhclNsb3AgKiAoMC41ICogYjJfbGluZWFyU2xvcCkpIHtcclxuICAgICAgICAgIHVuaXF1ZSA9IGZhbHNlO1xyXG4gICAgICAgICAgYnJlYWs7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAodW5pcXVlKSB7XHJcbiAgICAgICAgcHMucHVzaCh2KTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIG4gPSBwcy5sZW5ndGg7XHJcbiAgICBpZiAobiA8IDMpIHtcclxuICAgICAgLy8gUG9seWdvbiBpcyBkZWdlbmVyYXRlLlxyXG4gICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZhbHNlKTtcclxuICAgICAgcmV0dXJuIHRoaXMuU2V0QXNCb3goMS4wLCAxLjApO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIENyZWF0ZSB0aGUgY29udmV4IGh1bGwgdXNpbmcgdGhlIEdpZnQgd3JhcHBpbmcgYWxnb3JpdGhtXHJcbiAgICAvLyBodHRwOi8vZW4ud2lraXBlZGlhLm9yZy93aWtpL0dpZnRfd3JhcHBpbmdfYWxnb3JpdGhtXHJcblxyXG4gICAgLy8gRmluZCB0aGUgcmlnaHQgbW9zdCBwb2ludCBvbiB0aGUgaHVsbFxyXG4gICAgbGV0IGkwID0gMDtcclxuICAgIGxldCB4MDogbnVtYmVyID0gcHNbMF0ueDtcclxuICAgIGZvciAobGV0IGkgPSAxOyBpIDwgbjsgKytpKSB7XHJcbiAgICAgIGNvbnN0IHg6IG51bWJlciA9IHBzW2ldLng7XHJcbiAgICAgIGlmICh4ID4geDAgfHwgKHggPT09IHgwICYmIHBzW2ldLnkgPCBwc1tpMF0ueSkpIHtcclxuICAgICAgICBpMCA9IGk7XHJcbiAgICAgICAgeDAgPSB4O1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgaHVsbDogbnVtYmVyW10gPSBbXTtcclxuICAgIGxldCBtID0gMDtcclxuICAgIGxldCBpaDogbnVtYmVyID0gaTA7XHJcblxyXG4gICAgZm9yICg7Oykge1xyXG4gICAgICBodWxsW21dID0gaWg7XHJcblxyXG4gICAgICBsZXQgaWUgPSAwO1xyXG4gICAgICBmb3IgKGxldCBqID0gMTsgaiA8IG47ICsraikge1xyXG4gICAgICAgIGlmIChpZSA9PT0gaWgpIHtcclxuICAgICAgICAgIGllID0gajtcclxuICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgY29uc3QgcjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHBzW2llXSwgcHNbaHVsbFttXV0sIGIyUG9seWdvblNoYXBlLlNldF9zX3IpO1xyXG4gICAgICAgIGNvbnN0IHY6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihwc1tqXSwgcHNbaHVsbFttXV0sIGIyUG9seWdvblNoYXBlLlNldF9zX3YpO1xyXG4gICAgICAgIGNvbnN0IGM6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHIsIHYpO1xyXG4gICAgICAgIGlmIChjIDwgMCkge1xyXG4gICAgICAgICAgaWUgPSBqO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy8gQ29sbGluZWFyaXR5IGNoZWNrXHJcbiAgICAgICAgaWYgKGMgPT09IDAgJiYgdi5MZW5ndGhTcXVhcmVkKCkgPiByLkxlbmd0aFNxdWFyZWQoKSkge1xyXG4gICAgICAgICAgaWUgPSBqO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgKyttO1xyXG4gICAgICBpaCA9IGllO1xyXG5cclxuICAgICAgaWYgKGllID09PSBpMCkge1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5tX2NvdW50ID0gbTtcclxuICAgIHRoaXMubV92ZXJ0aWNlcyA9IGIyVmVjMi5NYWtlQXJyYXkodGhpcy5tX2NvdW50KTtcclxuICAgIHRoaXMubV9ub3JtYWxzID0gYjJWZWMyLk1ha2VBcnJheSh0aGlzLm1fY291bnQpO1xyXG5cclxuICAgIC8vIENvcHkgdmVydGljZXMuXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IG07ICsraSkge1xyXG4gICAgICB0aGlzLm1fdmVydGljZXNbaV0uQ29weShwc1todWxsW2ldXSk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gQ29tcHV0ZSBub3JtYWxzLiBFbnN1cmUgdGhlIGVkZ2VzIGhhdmUgbm9uLXplcm8gbGVuZ3RoLlxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBtOyArK2kpIHtcclxuICAgICAgY29uc3QgdmVydGV4aTE6IGIyVmVjMiA9IHRoaXMubV92ZXJ0aWNlc1tpXTtcclxuICAgICAgY29uc3QgdmVydGV4aTI6IGIyVmVjMiA9IHRoaXMubV92ZXJ0aWNlc1soaSArIDEpICUgbV07XHJcbiAgICAgIGNvbnN0IGVkZ2U6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVih2ZXJ0ZXhpMiwgdmVydGV4aTEsIGIyVmVjMi5zX3QwKTsgLy8gZWRnZSB1c2VzIHNfdDBcclxuICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChlZGdlLkxlbmd0aFNxdWFyZWQoKSA+IGIyX2Vwc2lsb25fc3EpO1xyXG4gICAgICBiMlZlYzIuQ3Jvc3NWT25lKGVkZ2UsIHRoaXMubV9ub3JtYWxzW2ldKS5TZWxmTm9ybWFsaXplKCk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gQ29tcHV0ZSB0aGUgcG9seWdvbiBjZW50cm9pZC5cclxuICAgIENvbXB1dGVDZW50cm9pZCh0aGlzLm1fdmVydGljZXMsIG0sIHRoaXMubV9jZW50cm9pZCk7XHJcblxyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICAvLy8gQnVpbGQgdmVydGljZXMgdG8gcmVwcmVzZW50IGFuIGF4aXMtYWxpZ25lZCBib3ggb3IgYW4gb3JpZW50ZWQgYm94LlxyXG4gIC8vLyBAcGFyYW0gaHggdGhlIGhhbGYtd2lkdGguXHJcbiAgLy8vIEBwYXJhbSBoeSB0aGUgaGFsZi1oZWlnaHQuXHJcbiAgLy8vIEBwYXJhbSBjZW50ZXIgdGhlIGNlbnRlciBvZiB0aGUgYm94IGluIGxvY2FsIGNvb3JkaW5hdGVzLlxyXG4gIC8vLyBAcGFyYW0gYW5nbGUgdGhlIHJvdGF0aW9uIG9mIHRoZSBib3ggaW4gbG9jYWwgY29vcmRpbmF0ZXMuXHJcbiAgU2V0QXNCb3goaHg6IG51bWJlciwgaHk6IG51bWJlciwgY2VudGVyPzogWFksIGFuZ2xlID0gMCk6IGIyUG9seWdvblNoYXBlIHtcclxuICAgIHRoaXMubV9jb3VudCA9IDQ7XHJcbiAgICB0aGlzLm1fdmVydGljZXMgPSBiMlZlYzIuTWFrZUFycmF5KDQpO1xyXG4gICAgdGhpcy5tX25vcm1hbHMgPSBiMlZlYzIuTWFrZUFycmF5KDQpO1xyXG4gICAgdGhpcy5tX3ZlcnRpY2VzWzBdLlNldCgtaHgsIC1oeSk7XHJcbiAgICB0aGlzLm1fdmVydGljZXNbMV0uU2V0KGh4LCAtaHkpO1xyXG4gICAgdGhpcy5tX3ZlcnRpY2VzWzJdLlNldChoeCwgaHkpO1xyXG4gICAgdGhpcy5tX3ZlcnRpY2VzWzNdLlNldCgtaHgsIGh5KTtcclxuICAgIHRoaXMubV9ub3JtYWxzWzBdLlNldCgwLCAtMSk7XHJcbiAgICB0aGlzLm1fbm9ybWFsc1sxXS5TZXQoMSwgMCk7XHJcbiAgICB0aGlzLm1fbm9ybWFsc1syXS5TZXQoMCwgMSk7XHJcbiAgICB0aGlzLm1fbm9ybWFsc1szXS5TZXQoLTEsIDApO1xyXG4gICAgdGhpcy5tX2NlbnRyb2lkLlNldFplcm8oKTtcclxuXHJcbiAgICBpZiAoY2VudGVyKSB7XHJcbiAgICAgIHRoaXMubV9jZW50cm9pZC5Db3B5KGNlbnRlcik7XHJcblxyXG4gICAgICBjb25zdCB4ZjogYjJUcmFuc2Zvcm0gPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICAgICAgeGYuU2V0UG9zaXRpb24oY2VudGVyKTtcclxuICAgICAgeGYuU2V0Um90YXRpb25BbmdsZShhbmdsZSk7XHJcblxyXG4gICAgICAvLyBUcmFuc2Zvcm0gdmVydGljZXMgYW5kIG5vcm1hbHMuXHJcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgICBiMlRyYW5zZm9ybS5NdWxYVih4ZiwgdGhpcy5tX3ZlcnRpY2VzW2ldLCB0aGlzLm1fdmVydGljZXNbaV0pO1xyXG4gICAgICAgIGIyUm90Lk11bFJWKHhmLnEsIHRoaXMubV9ub3JtYWxzW2ldLCB0aGlzLm1fbm9ybWFsc1tpXSk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIC8vLyBAc2VlIGIyU2hhcGU6OlRlc3RQb2ludFxyXG4gIHByaXZhdGUgc3RhdGljIFRlc3RQb2ludF9zX3BMb2NhbCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgVGVzdFBvaW50KHhmOiBiMlRyYW5zZm9ybSwgcDogWFkpOiBib29sZWFuIHtcclxuICAgIGNvbnN0IHBMb2NhbDogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsVFhWKHhmLCBwLCBiMlBvbHlnb25TaGFwZS5UZXN0UG9pbnRfc19wTG9jYWwpO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgZG90OiBudW1iZXIgPSBiMlZlYzIuRG90VlYoXHJcbiAgICAgICAgdGhpcy5tX25vcm1hbHNbaV0sXHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHBMb2NhbCwgdGhpcy5tX3ZlcnRpY2VzW2ldLCBiMlZlYzIuc190MCksXHJcbiAgICAgICk7XHJcbiAgICAgIGlmIChkb3QgPiAwKSB7XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIHRydWU7XHJcbiAgfVxyXG5cclxuICBDb21wdXRlRGlzdGFuY2UoeGY6IGIyVHJhbnNmb3JtLCBwOiBiMlZlYzIsIG5vcm1hbDogYjJWZWMyLCBjaGlsZEluZGV4OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgaWYgKEIyX0VOQUJMRV9QQVJUSUNMRSkge1xyXG4gICAgICBjb25zdCBwTG9jYWwgPSBiMlRyYW5zZm9ybS5NdWxUWFYoeGYsIHAsIENvbXB1dGVEaXN0YW5jZV9zX3BMb2NhbCk7XHJcbiAgICAgIGxldCBtYXhEaXN0YW5jZSA9IC1iMl9tYXhGbG9hdDtcclxuICAgICAgY29uc3Qgbm9ybWFsRm9yTWF4RGlzdGFuY2UgPSBDb21wdXRlRGlzdGFuY2Vfc19ub3JtYWxGb3JNYXhEaXN0YW5jZS5Db3B5KHBMb2NhbCk7XHJcblxyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgICAgY29uc3QgZG90ID0gYjJWZWMyLkRvdFZWKFxyXG4gICAgICAgICAgdGhpcy5tX25vcm1hbHNbaV0sXHJcbiAgICAgICAgICBiMlZlYzIuU3ViVlYocExvY2FsLCB0aGlzLm1fdmVydGljZXNbaV0sIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICApO1xyXG4gICAgICAgIGlmIChkb3QgPiBtYXhEaXN0YW5jZSkge1xyXG4gICAgICAgICAgbWF4RGlzdGFuY2UgPSBkb3Q7XHJcbiAgICAgICAgICBub3JtYWxGb3JNYXhEaXN0YW5jZS5Db3B5KHRoaXMubV9ub3JtYWxzW2ldKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGlmIChtYXhEaXN0YW5jZSA+IDApIHtcclxuICAgICAgICBjb25zdCBtaW5EaXN0YW5jZSA9IENvbXB1dGVEaXN0YW5jZV9zX21pbkRpc3RhbmNlLkNvcHkobm9ybWFsRm9yTWF4RGlzdGFuY2UpO1xyXG4gICAgICAgIGxldCBtaW5EaXN0YW5jZTIgPSBtYXhEaXN0YW5jZSAqIG1heERpc3RhbmNlO1xyXG4gICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgICAgIGNvbnN0IGRpc3RhbmNlID0gYjJWZWMyLlN1YlZWKHBMb2NhbCwgdGhpcy5tX3ZlcnRpY2VzW2ldLCBDb21wdXRlRGlzdGFuY2Vfc19kaXN0YW5jZSk7XHJcbiAgICAgICAgICBjb25zdCBkaXN0YW5jZTIgPSBkaXN0YW5jZS5MZW5ndGhTcXVhcmVkKCk7XHJcbiAgICAgICAgICBpZiAobWluRGlzdGFuY2UyID4gZGlzdGFuY2UyKSB7XHJcbiAgICAgICAgICAgIG1pbkRpc3RhbmNlLkNvcHkoZGlzdGFuY2UpO1xyXG4gICAgICAgICAgICBtaW5EaXN0YW5jZTIgPSBkaXN0YW5jZTI7XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBiMlJvdC5NdWxSVih4Zi5xLCBtaW5EaXN0YW5jZSwgbm9ybWFsKTtcclxuICAgICAgICBub3JtYWwuTm9ybWFsaXplKCk7XHJcbiAgICAgICAgcmV0dXJuIE1hdGguc3FydChtaW5EaXN0YW5jZTIpO1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIGIyUm90Lk11bFJWKHhmLnEsIG5vcm1hbEZvck1heERpc3RhbmNlLCBub3JtYWwpO1xyXG4gICAgICAgIHJldHVybiBtYXhEaXN0YW5jZTtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgcmV0dXJuIDAuMDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBJbXBsZW1lbnQgYjJTaGFwZS5cclxuICBwcml2YXRlIHN0YXRpYyBSYXlDYXN0X3NfcDEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgUmF5Q2FzdF9zX3AyID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFJheUNhc3Rfc19kID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBSYXlDYXN0KFxyXG4gICAgb3V0cHV0OiBiMlJheUNhc3RPdXRwdXQsXHJcbiAgICBpbnB1dDogYjJSYXlDYXN0SW5wdXQsXHJcbiAgICB4ZjogYjJUcmFuc2Zvcm0sXHJcbiAgICBjaGlsZEluZGV4OiBudW1iZXIsXHJcbiAgKTogYm9vbGVhbiB7XHJcbiAgICAvLyBQdXQgdGhlIHJheSBpbnRvIHRoZSBwb2x5Z29uJ3MgZnJhbWUgb2YgcmVmZXJlbmNlLlxyXG4gICAgY29uc3QgcDE6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFRYVih4ZiwgaW5wdXQucDEsIGIyUG9seWdvblNoYXBlLlJheUNhc3Rfc19wMSk7XHJcbiAgICBjb25zdCBwMjogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsVFhWKHhmLCBpbnB1dC5wMiwgYjJQb2x5Z29uU2hhcGUuUmF5Q2FzdF9zX3AyKTtcclxuICAgIGNvbnN0IGQ6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihwMiwgcDEsIGIyUG9seWdvblNoYXBlLlJheUNhc3Rfc19kKTtcclxuXHJcbiAgICBsZXQgbG93ZXIgPSAwLFxyXG4gICAgICB1cHBlciA9IGlucHV0Lm1heEZyYWN0aW9uO1xyXG5cclxuICAgIGxldCBpbmRleCA9IC0xO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgLy8gcCA9IHAxICsgYSAqIGRcclxuICAgICAgLy8gZG90KG5vcm1hbCwgcCAtIHYpID0gMFxyXG4gICAgICAvLyBkb3Qobm9ybWFsLCBwMSAtIHYpICsgYSAqIGRvdChub3JtYWwsIGQpID0gMFxyXG4gICAgICBjb25zdCBudW1lcmF0b3I6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgICB0aGlzLm1fbm9ybWFsc1tpXSxcclxuICAgICAgICBiMlZlYzIuU3ViVlYodGhpcy5tX3ZlcnRpY2VzW2ldLCBwMSwgYjJWZWMyLnNfdDApLFxyXG4gICAgICApO1xyXG4gICAgICBjb25zdCBkZW5vbWluYXRvcjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHRoaXMubV9ub3JtYWxzW2ldLCBkKTtcclxuXHJcbiAgICAgIGlmIChkZW5vbWluYXRvciA9PT0gMCkge1xyXG4gICAgICAgIGlmIChudW1lcmF0b3IgPCAwKSB7XHJcbiAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgfVxyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIC8vIE5vdGU6IHdlIHdhbnQgdGhpcyBwcmVkaWNhdGUgd2l0aG91dCBkaXZpc2lvbjpcclxuICAgICAgICAvLyBsb3dlciA8IG51bWVyYXRvciAvIGRlbm9taW5hdG9yLCB3aGVyZSBkZW5vbWluYXRvciA8IDBcclxuICAgICAgICAvLyBTaW5jZSBkZW5vbWluYXRvciA8IDAsIHdlIGhhdmUgdG8gZmxpcCB0aGUgaW5lcXVhbGl0eTpcclxuICAgICAgICAvLyBsb3dlciA8IG51bWVyYXRvciAvIGRlbm9taW5hdG9yIDw9PT4gZGVub21pbmF0b3IgKiBsb3dlciA+IG51bWVyYXRvci5cclxuICAgICAgICBpZiAoZGVub21pbmF0b3IgPCAwICYmIG51bWVyYXRvciA8IGxvd2VyICogZGVub21pbmF0b3IpIHtcclxuICAgICAgICAgIC8vIEluY3JlYXNlIGxvd2VyLlxyXG4gICAgICAgICAgLy8gVGhlIHNlZ21lbnQgZW50ZXJzIHRoaXMgaGFsZi1zcGFjZS5cclxuICAgICAgICAgIGxvd2VyID0gbnVtZXJhdG9yIC8gZGVub21pbmF0b3I7XHJcbiAgICAgICAgICBpbmRleCA9IGk7XHJcbiAgICAgICAgfSBlbHNlIGlmIChkZW5vbWluYXRvciA+IDAgJiYgbnVtZXJhdG9yIDwgdXBwZXIgKiBkZW5vbWluYXRvcikge1xyXG4gICAgICAgICAgLy8gRGVjcmVhc2UgdXBwZXIuXHJcbiAgICAgICAgICAvLyBUaGUgc2VnbWVudCBleGl0cyB0aGlzIGhhbGYtc3BhY2UuXHJcbiAgICAgICAgICB1cHBlciA9IG51bWVyYXRvciAvIGRlbm9taW5hdG9yO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gVGhlIHVzZSBvZiBlcHNpbG9uIGhlcmUgY2F1c2VzIHRoZSBhc3NlcnQgb24gbG93ZXIgdG8gdHJpcFxyXG4gICAgICAvLyBpbiBzb21lIGNhc2VzLiBBcHBhcmVudGx5IHRoZSB1c2Ugb2YgZXBzaWxvbiB3YXMgdG8gbWFrZSBlZGdlXHJcbiAgICAgIC8vIHNoYXBlcyB3b3JrLCBidXQgbm93IHRob3NlIGFyZSBoYW5kbGVkIHNlcGFyYXRlbHkuXHJcbiAgICAgIC8vIGlmICh1cHBlciA8IGxvd2VyIC0gYjJfZXBzaWxvbilcclxuICAgICAgaWYgKHVwcGVyIDwgbG93ZXIpIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KDAgPD0gbG93ZXIgJiYgbG93ZXIgPD0gaW5wdXQubWF4RnJhY3Rpb24pO1xyXG5cclxuICAgIGlmIChpbmRleCA+PSAwKSB7XHJcbiAgICAgIG91dHB1dC5mcmFjdGlvbiA9IGxvd2VyO1xyXG4gICAgICBiMlJvdC5NdWxSVih4Zi5xLCB0aGlzLm1fbm9ybWFsc1tpbmRleF0sIG91dHB1dC5ub3JtYWwpO1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbiAgfVxyXG5cclxuICAvLy8gQHNlZSBiMlNoYXBlOjpDb21wdXRlQUFCQlxyXG4gIHByaXZhdGUgc3RhdGljIENvbXB1dGVBQUJCX3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgQ29tcHV0ZUFBQkIoYWFiYjogYjJBQUJCLCB4ZjogYjJUcmFuc2Zvcm0sIGNoaWxkSW5kZXg6IG51bWJlcik6IHZvaWQge1xyXG4gICAgY29uc3QgbG93ZXI6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmLCB0aGlzLm1fdmVydGljZXNbMF0sIGFhYmIubG93ZXJCb3VuZCk7XHJcbiAgICBjb25zdCB1cHBlcjogYjJWZWMyID0gYWFiYi51cHBlckJvdW5kLkNvcHkobG93ZXIpO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgdjogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGYsIHRoaXMubV92ZXJ0aWNlc1tpXSwgYjJQb2x5Z29uU2hhcGUuQ29tcHV0ZUFBQkJfc192KTtcclxuICAgICAgYjJWZWMyLk1pblYodiwgbG93ZXIsIGxvd2VyKTtcclxuICAgICAgYjJWZWMyLk1heFYodiwgdXBwZXIsIHVwcGVyKTtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCByOiBudW1iZXIgPSB0aGlzLm1fcmFkaXVzO1xyXG4gICAgbG93ZXIuU2VsZlN1YlhZKHIsIHIpO1xyXG4gICAgdXBwZXIuU2VsZkFkZFhZKHIsIHIpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEBzZWUgYjJTaGFwZTo6Q29tcHV0ZU1hc3NcclxuICBwcml2YXRlIHN0YXRpYyBDb21wdXRlTWFzc19zX2NlbnRlciA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBDb21wdXRlTWFzc19zX3MgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgQ29tcHV0ZU1hc3Nfc19lMSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBDb21wdXRlTWFzc19zX2UyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBDb21wdXRlTWFzcyhtYXNzRGF0YTogYjJNYXNzRGF0YSwgZGVuc2l0eTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICAvLyBQb2x5Z29uIG1hc3MsIGNlbnRyb2lkLCBhbmQgaW5lcnRpYS5cclxuICAgIC8vIExldCByaG8gYmUgdGhlIHBvbHlnb24gZGVuc2l0eSBpbiBtYXNzIHBlciB1bml0IGFyZWEuXHJcbiAgICAvLyBUaGVuOlxyXG4gICAgLy8gbWFzcyA9IHJobyAqIGludChkQSlcclxuICAgIC8vIGNlbnRyb2lkLnggPSAoMS9tYXNzKSAqIHJobyAqIGludCh4ICogZEEpXHJcbiAgICAvLyBjZW50cm9pZC55ID0gKDEvbWFzcykgKiByaG8gKiBpbnQoeSAqIGRBKVxyXG4gICAgLy8gSSA9IHJobyAqIGludCgoeCp4ICsgeSp5KSAqIGRBKVxyXG4gICAgLy9cclxuICAgIC8vIFdlIGNhbiBjb21wdXRlIHRoZXNlIGludGVncmFscyBieSBzdW1taW5nIGFsbCB0aGUgaW50ZWdyYWxzXHJcbiAgICAvLyBmb3IgZWFjaCB0cmlhbmdsZSBvZiB0aGUgcG9seWdvbi4gVG8gZXZhbHVhdGUgdGhlIGludGVncmFsXHJcbiAgICAvLyBmb3IgYSBzaW5nbGUgdHJpYW5nbGUsIHdlIG1ha2UgYSBjaGFuZ2Ugb2YgdmFyaWFibGVzIHRvXHJcbiAgICAvLyB0aGUgKHUsdikgY29vcmRpbmF0ZXMgb2YgdGhlIHRyaWFuZ2xlOlxyXG4gICAgLy8geCA9IHgwICsgZTF4ICogdSArIGUyeCAqIHZcclxuICAgIC8vIHkgPSB5MCArIGUxeSAqIHUgKyBlMnkgKiB2XHJcbiAgICAvLyB3aGVyZSAwIDw9IHUgJiYgMCA8PSB2ICYmIHUgKyB2IDw9IDEuXHJcbiAgICAvL1xyXG4gICAgLy8gV2UgaW50ZWdyYXRlIHUgZnJvbSBbMCwxLXZdIGFuZCB0aGVuIHYgZnJvbSBbMCwxXS5cclxuICAgIC8vIFdlIGFsc28gbmVlZCB0byB1c2UgdGhlIEphY29iaWFuIG9mIHRoZSB0cmFuc2Zvcm1hdGlvbjpcclxuICAgIC8vIEQgPSBjcm9zcyhlMSwgZTIpXHJcbiAgICAvL1xyXG4gICAgLy8gU2ltcGxpZmljYXRpb246IHRyaWFuZ2xlIGNlbnRyb2lkID0gKDEvMykgKiAocDEgKyBwMiArIHAzKVxyXG4gICAgLy9cclxuICAgIC8vIFRoZSByZXN0IG9mIHRoZSBkZXJpdmF0aW9uIGlzIGhhbmRsZWQgYnkgY29tcHV0ZXIgYWxnZWJyYS5cclxuXHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV9jb3VudCA+PSAzKTtcclxuXHJcbiAgICBjb25zdCBjZW50ZXI6IGIyVmVjMiA9IGIyUG9seWdvblNoYXBlLkNvbXB1dGVNYXNzX3NfY2VudGVyLlNldFplcm8oKTtcclxuICAgIGxldCBhcmVhID0gMDtcclxuICAgIGxldCBJID0gMDtcclxuXHJcbiAgICAvLyBzIGlzIHRoZSByZWZlcmVuY2UgcG9pbnQgZm9yIGZvcm1pbmcgdHJpYW5nbGVzLlxyXG4gICAgLy8gSXQncyBsb2NhdGlvbiBkb2Vzbid0IGNoYW5nZSB0aGUgcmVzdWx0IChleGNlcHQgZm9yIHJvdW5kaW5nIGVycm9yKS5cclxuICAgIGNvbnN0IHM6IGIyVmVjMiA9IGIyUG9seWdvblNoYXBlLkNvbXB1dGVNYXNzX3Nfcy5TZXRaZXJvKCk7XHJcblxyXG4gICAgLy8gVGhpcyBjb2RlIHdvdWxkIHB1dCB0aGUgcmVmZXJlbmNlIHBvaW50IGluc2lkZSB0aGUgcG9seWdvbi5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgcy5TZWxmQWRkKHRoaXMubV92ZXJ0aWNlc1tpXSk7XHJcbiAgICB9XHJcbiAgICBzLlNlbGZNdWwoMSAvIHRoaXMubV9jb3VudCk7XHJcblxyXG4gICAgY29uc3Qga19pbnYzOiBudW1iZXIgPSAxIC8gMztcclxuXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIC8vIFRyaWFuZ2xlIHZlcnRpY2VzLlxyXG4gICAgICBjb25zdCBlMTogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHRoaXMubV92ZXJ0aWNlc1tpXSwgcywgYjJQb2x5Z29uU2hhcGUuQ29tcHV0ZU1hc3Nfc19lMSk7XHJcbiAgICAgIGNvbnN0IGUyOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgdGhpcy5tX3ZlcnRpY2VzWyhpICsgMSkgJSB0aGlzLm1fY291bnRdLFxyXG4gICAgICAgIHMsXHJcbiAgICAgICAgYjJQb2x5Z29uU2hhcGUuQ29tcHV0ZU1hc3Nfc19lMixcclxuICAgICAgKTtcclxuXHJcbiAgICAgIGNvbnN0IEQ6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKGUxLCBlMik7XHJcblxyXG4gICAgICBjb25zdCB0cmlhbmdsZUFyZWE6IG51bWJlciA9IDAuNSAqIEQ7XHJcbiAgICAgIGFyZWEgKz0gdHJpYW5nbGVBcmVhO1xyXG5cclxuICAgICAgLy8gQXJlYSB3ZWlnaHRlZCBjZW50cm9pZFxyXG4gICAgICBjZW50ZXIuU2VsZkFkZChcclxuICAgICAgICBiMlZlYzIuTXVsU1YodHJpYW5nbGVBcmVhICoga19pbnYzLCBiMlZlYzIuQWRkVlYoZTEsIGUyLCBiMlZlYzIuc190MCksIGIyVmVjMi5zX3QxKSxcclxuICAgICAgKTtcclxuXHJcbiAgICAgIGNvbnN0IGV4MTogbnVtYmVyID0gZTEueDtcclxuICAgICAgY29uc3QgZXkxOiBudW1iZXIgPSBlMS55O1xyXG4gICAgICBjb25zdCBleDI6IG51bWJlciA9IGUyLng7XHJcbiAgICAgIGNvbnN0IGV5MjogbnVtYmVyID0gZTIueTtcclxuXHJcbiAgICAgIGNvbnN0IGludHgyOiBudW1iZXIgPSBleDEgKiBleDEgKyBleDIgKiBleDEgKyBleDIgKiBleDI7XHJcbiAgICAgIGNvbnN0IGludHkyOiBudW1iZXIgPSBleTEgKiBleTEgKyBleTIgKiBleTEgKyBleTIgKiBleTI7XHJcblxyXG4gICAgICBJICs9IDAuMjUgKiBrX2ludjMgKiBEICogKGludHgyICsgaW50eTIpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFRvdGFsIG1hc3NcclxuICAgIG1hc3NEYXRhLm1hc3MgPSBkZW5zaXR5ICogYXJlYTtcclxuXHJcbiAgICAvLyBDZW50ZXIgb2YgbWFzc1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChhcmVhID4gYjJfZXBzaWxvbik7XHJcbiAgICBjZW50ZXIuU2VsZk11bCgxIC8gYXJlYSk7XHJcbiAgICBiMlZlYzIuQWRkVlYoY2VudGVyLCBzLCBtYXNzRGF0YS5jZW50ZXIpO1xyXG5cclxuICAgIC8vIEluZXJ0aWEgdGVuc29yIHJlbGF0aXZlIHRvIHRoZSBsb2NhbCBvcmlnaW4gKHBvaW50IHMpLlxyXG4gICAgbWFzc0RhdGEuSSA9IGRlbnNpdHkgKiBJO1xyXG5cclxuICAgIC8vIFNoaWZ0IHRvIGNlbnRlciBvZiBtYXNzIHRoZW4gdG8gb3JpZ2luYWwgYm9keSBvcmlnaW4uXHJcbiAgICBtYXNzRGF0YS5JICs9XHJcbiAgICAgIG1hc3NEYXRhLm1hc3MgKlxyXG4gICAgICAoYjJWZWMyLkRvdFZWKG1hc3NEYXRhLmNlbnRlciwgbWFzc0RhdGEuY2VudGVyKSAtIGIyVmVjMi5Eb3RWVihjZW50ZXIsIGNlbnRlcikpO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgVmFsaWRhdGVfc19lID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFZhbGlkYXRlX3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgVmFsaWRhdGUoKTogYm9vbGVhbiB7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIGNvbnN0IGkxID0gaTtcclxuICAgICAgY29uc3QgaTIgPSAoaSArIDEpICUgdGhpcy5tX2NvdW50O1xyXG4gICAgICBjb25zdCBwOiBiMlZlYzIgPSB0aGlzLm1fdmVydGljZXNbaTFdO1xyXG4gICAgICBjb25zdCBlOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYodGhpcy5tX3ZlcnRpY2VzW2kyXSwgcCwgYjJQb2x5Z29uU2hhcGUuVmFsaWRhdGVfc19lKTtcclxuXHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5tX2NvdW50OyArK2opIHtcclxuICAgICAgICBpZiAoaiA9PT0gaTEgfHwgaiA9PT0gaTIpIHtcclxuICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgY29uc3QgdjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHRoaXMubV92ZXJ0aWNlc1tqXSwgcCwgYjJQb2x5Z29uU2hhcGUuVmFsaWRhdGVfc192KTtcclxuICAgICAgICBjb25zdCBjOiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVihlLCB2KTtcclxuICAgICAgICBpZiAoYyA8IDApIHtcclxuICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gdHJ1ZTtcclxuICB9XHJcblxyXG4gIFNldHVwRGlzdGFuY2VQcm94eShwcm94eTogYjJEaXN0YW5jZVByb3h5LCBpbmRleDogbnVtYmVyKTogdm9pZCB7XHJcbiAgICBwcm94eS5tX3ZlcnRpY2VzID0gdGhpcy5tX3ZlcnRpY2VzO1xyXG4gICAgcHJveHkubV9jb3VudCA9IHRoaXMubV9jb3VudDtcclxuICAgIHByb3h5Lm1fcmFkaXVzID0gdGhpcy5tX3JhZGl1cztcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIENvbXB1dGVTdWJtZXJnZWRBcmVhX3Nfbm9ybWFsTCA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBDb21wdXRlU3VibWVyZ2VkQXJlYV9zX21kID0gbmV3IGIyTWFzc0RhdGEoKTtcclxuICBwcml2YXRlIHN0YXRpYyBDb21wdXRlU3VibWVyZ2VkQXJlYV9zX2ludG9WZWMgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgQ29tcHV0ZVN1Ym1lcmdlZEFyZWFfc19vdXRvVmVjID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIENvbXB1dGVTdWJtZXJnZWRBcmVhX3NfY2VudGVyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBDb21wdXRlU3VibWVyZ2VkQXJlYShub3JtYWw6IGIyVmVjMiwgb2Zmc2V0OiBudW1iZXIsIHhmOiBiMlRyYW5zZm9ybSwgYzogYjJWZWMyKTogbnVtYmVyIHtcclxuICAgIC8vIFRyYW5zZm9ybSBwbGFuZSBpbnRvIHNoYXBlIGNvLW9yZGluYXRlc1xyXG4gICAgY29uc3Qgbm9ybWFsTDogYjJWZWMyID0gYjJSb3QuTXVsVFJWKFxyXG4gICAgICB4Zi5xLFxyXG4gICAgICBub3JtYWwsXHJcbiAgICAgIGIyUG9seWdvblNoYXBlLkNvbXB1dGVTdWJtZXJnZWRBcmVhX3Nfbm9ybWFsTCxcclxuICAgICk7XHJcbiAgICBjb25zdCBvZmZzZXRMOiBudW1iZXIgPSBvZmZzZXQgLSBiMlZlYzIuRG90VlYobm9ybWFsLCB4Zi5wKTtcclxuXHJcbiAgICBjb25zdCBkZXB0aHM6IG51bWJlcltdID0gW107XHJcbiAgICBsZXQgZGl2ZUNvdW50ID0gMDtcclxuICAgIGxldCBpbnRvSW5kZXggPSAtMTtcclxuICAgIGxldCBvdXRvSW5kZXggPSAtMTtcclxuXHJcbiAgICBsZXQgbGFzdFN1Ym1lcmdlZCA9IGZhbHNlO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7ICsraSkge1xyXG4gICAgICBkZXB0aHNbaV0gPSBiMlZlYzIuRG90VlYobm9ybWFsTCwgdGhpcy5tX3ZlcnRpY2VzW2ldKSAtIG9mZnNldEw7XHJcbiAgICAgIGNvbnN0IGlzU3VibWVyZ2VkOiBib29sZWFuID0gZGVwdGhzW2ldIDwgLWIyX2Vwc2lsb247XHJcbiAgICAgIGlmIChpID4gMCkge1xyXG4gICAgICAgIGlmIChpc1N1Ym1lcmdlZCkge1xyXG4gICAgICAgICAgaWYgKCFsYXN0U3VibWVyZ2VkKSB7XHJcbiAgICAgICAgICAgIGludG9JbmRleCA9IGkgLSAxO1xyXG4gICAgICAgICAgICBkaXZlQ291bnQrKztcclxuICAgICAgICAgIH1cclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgaWYgKGxhc3RTdWJtZXJnZWQpIHtcclxuICAgICAgICAgICAgb3V0b0luZGV4ID0gaSAtIDE7XHJcbiAgICAgICAgICAgIGRpdmVDb3VudCsrO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgICBsYXN0U3VibWVyZ2VkID0gaXNTdWJtZXJnZWQ7XHJcbiAgICB9XHJcbiAgICBzd2l0Y2ggKGRpdmVDb3VudCkge1xyXG4gICAgICBjYXNlIDA6XHJcbiAgICAgICAgaWYgKGxhc3RTdWJtZXJnZWQpIHtcclxuICAgICAgICAgIC8vIENvbXBsZXRlbHkgc3VibWVyZ2VkXHJcbiAgICAgICAgICBjb25zdCBtZDogYjJNYXNzRGF0YSA9IGIyUG9seWdvblNoYXBlLkNvbXB1dGVTdWJtZXJnZWRBcmVhX3NfbWQ7XHJcbiAgICAgICAgICB0aGlzLkNvbXB1dGVNYXNzKG1kLCAxKTtcclxuICAgICAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHhmLCBtZC5jZW50ZXIsIGMpO1xyXG4gICAgICAgICAgcmV0dXJuIG1kLm1hc3M7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIC8vIENvbXBsZXRlbHkgZHJ5XHJcbiAgICAgICAgICByZXR1cm4gMDtcclxuICAgICAgICB9XHJcbiAgICAgIGNhc2UgMTpcclxuICAgICAgICBpZiAoaW50b0luZGV4ID09PSAtMSkge1xyXG4gICAgICAgICAgaW50b0luZGV4ID0gdGhpcy5tX2NvdW50IC0gMTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgb3V0b0luZGV4ID0gdGhpcy5tX2NvdW50IC0gMTtcclxuICAgICAgICB9XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICB9XHJcbiAgICBjb25zdCBpbnRvSW5kZXgyOiBudW1iZXIgPSAoaW50b0luZGV4ICsgMSkgJSB0aGlzLm1fY291bnQ7XHJcbiAgICBjb25zdCBvdXRvSW5kZXgyOiBudW1iZXIgPSAob3V0b0luZGV4ICsgMSkgJSB0aGlzLm1fY291bnQ7XHJcbiAgICBjb25zdCBpbnRvTGFtZGRhOiBudW1iZXIgPSAoMCAtIGRlcHRoc1tpbnRvSW5kZXhdKSAvIChkZXB0aHNbaW50b0luZGV4Ml0gLSBkZXB0aHNbaW50b0luZGV4XSk7XHJcbiAgICBjb25zdCBvdXRvTGFtZGRhOiBudW1iZXIgPSAoMCAtIGRlcHRoc1tvdXRvSW5kZXhdKSAvIChkZXB0aHNbb3V0b0luZGV4Ml0gLSBkZXB0aHNbb3V0b0luZGV4XSk7XHJcblxyXG4gICAgY29uc3QgaW50b1ZlYzogYjJWZWMyID0gYjJQb2x5Z29uU2hhcGUuQ29tcHV0ZVN1Ym1lcmdlZEFyZWFfc19pbnRvVmVjLlNldChcclxuICAgICAgdGhpcy5tX3ZlcnRpY2VzW2ludG9JbmRleF0ueCAqICgxIC0gaW50b0xhbWRkYSkgKyB0aGlzLm1fdmVydGljZXNbaW50b0luZGV4Ml0ueCAqIGludG9MYW1kZGEsXHJcbiAgICAgIHRoaXMubV92ZXJ0aWNlc1tpbnRvSW5kZXhdLnkgKiAoMSAtIGludG9MYW1kZGEpICsgdGhpcy5tX3ZlcnRpY2VzW2ludG9JbmRleDJdLnkgKiBpbnRvTGFtZGRhLFxyXG4gICAgKTtcclxuICAgIGNvbnN0IG91dG9WZWM6IGIyVmVjMiA9IGIyUG9seWdvblNoYXBlLkNvbXB1dGVTdWJtZXJnZWRBcmVhX3Nfb3V0b1ZlYy5TZXQoXHJcbiAgICAgIHRoaXMubV92ZXJ0aWNlc1tvdXRvSW5kZXhdLnggKiAoMSAtIG91dG9MYW1kZGEpICsgdGhpcy5tX3ZlcnRpY2VzW291dG9JbmRleDJdLnggKiBvdXRvTGFtZGRhLFxyXG4gICAgICB0aGlzLm1fdmVydGljZXNbb3V0b0luZGV4XS55ICogKDEgLSBvdXRvTGFtZGRhKSArIHRoaXMubV92ZXJ0aWNlc1tvdXRvSW5kZXgyXS55ICogb3V0b0xhbWRkYSxcclxuICAgICk7XHJcblxyXG4gICAgLy8gSW5pdGlhbGl6ZSBhY2N1bXVsYXRvclxyXG4gICAgbGV0IGFyZWEgPSAwO1xyXG4gICAgY29uc3QgY2VudGVyOiBiMlZlYzIgPSBiMlBvbHlnb25TaGFwZS5Db21wdXRlU3VibWVyZ2VkQXJlYV9zX2NlbnRlci5TZXRaZXJvKCk7XHJcbiAgICBsZXQgcDI6IGIyVmVjMiA9IHRoaXMubV92ZXJ0aWNlc1tpbnRvSW5kZXgyXTtcclxuICAgIGxldCBwMzogYjJWZWMyO1xyXG5cclxuICAgIC8vIEFuIGF3a3dhcmQgbG9vcCBmcm9tIGludG9JbmRleDIrMSB0byBvdXRJbmRleDJcclxuICAgIGxldCBpOiBudW1iZXIgPSBpbnRvSW5kZXgyO1xyXG4gICAgd2hpbGUgKGkgIT09IG91dG9JbmRleDIpIHtcclxuICAgICAgaSA9IChpICsgMSkgJSB0aGlzLm1fY291bnQ7XHJcbiAgICAgIGlmIChpID09PSBvdXRvSW5kZXgyKSB7XHJcbiAgICAgICAgcDMgPSBvdXRvVmVjO1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIHAzID0gdGhpcy5tX3ZlcnRpY2VzW2ldO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBjb25zdCB0cmlhbmdsZUFyZWE6IG51bWJlciA9XHJcbiAgICAgICAgMC41ICogKChwMi54IC0gaW50b1ZlYy54KSAqIChwMy55IC0gaW50b1ZlYy55KSAtIChwMi55IC0gaW50b1ZlYy55KSAqIChwMy54IC0gaW50b1ZlYy54KSk7XHJcbiAgICAgIGFyZWEgKz0gdHJpYW5nbGVBcmVhO1xyXG4gICAgICAvLyBBcmVhIHdlaWdodGVkIGNlbnRyb2lkXHJcbiAgICAgIGNlbnRlci54ICs9ICh0cmlhbmdsZUFyZWEgKiAoaW50b1ZlYy54ICsgcDIueCArIHAzLngpKSAvIDM7XHJcbiAgICAgIGNlbnRlci55ICs9ICh0cmlhbmdsZUFyZWEgKiAoaW50b1ZlYy55ICsgcDIueSArIHAzLnkpKSAvIDM7XHJcblxyXG4gICAgICBwMiA9IHAzO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIE5vcm1hbGl6ZSBhbmQgdHJhbnNmb3JtIGNlbnRyb2lkXHJcbiAgICBjZW50ZXIuU2VsZk11bCgxIC8gYXJlYSk7XHJcbiAgICBiMlRyYW5zZm9ybS5NdWxYVih4ZiwgY2VudGVyLCBjKTtcclxuXHJcbiAgICByZXR1cm4gYXJlYTtcclxuICB9XHJcblxyXG4gIC8qXHJcbiAgICBzdGF0aWMgQ29tcHV0ZU9CQihvYmIsIHZzLCBjb3VudCkge1xyXG4gICAgICBjb25zdCBpOiBudW1iZXIgPSAwO1xyXG4gICAgICBjb25zdCBwOiBBcnJheSA9IFtjb3VudCArIDFdO1xyXG4gICAgICBmb3IgKGkgPSAwOyBpIDwgY291bnQ7ICsraSkge1xyXG4gICAgICAgIHBbaV0gPSB2c1tpXTtcclxuICAgICAgfVxyXG4gICAgICBwW2NvdW50XSA9IHBbMF07XHJcbiAgICAgIGNvbnN0IG1pbkFyZWEgPSBiMl9tYXhGbG9hdDtcclxuICAgICAgZm9yIChpID0gMTsgaSA8PSBjb3VudDsgKytpKSB7XHJcbiAgICAgICAgY29uc3Qgcm9vdCA9IHBbaSAtIDFdO1xyXG4gICAgICAgIGNvbnN0IHV4WCA9IHBbaV0ueCAtIHJvb3QueDtcclxuICAgICAgICBjb25zdCB1eFkgPSBwW2ldLnkgLSByb290Lnk7XHJcbiAgICAgICAgY29uc3QgbGVuZ3RoID0gYjJTcXJ0KHV4WCAqIHV4WCArIHV4WSAqIHV4WSk7XHJcbiAgICAgICAgdXhYIC89IGxlbmd0aDtcclxuICAgICAgICB1eFkgLz0gbGVuZ3RoO1xyXG4gICAgICAgIGNvbnN0IHV5WCA9ICgtdXhZKTtcclxuICAgICAgICBjb25zdCB1eVkgPSB1eFg7XHJcbiAgICAgICAgY29uc3QgbG93ZXJYID0gYjJfbWF4RmxvYXQ7XHJcbiAgICAgICAgY29uc3QgbG93ZXJZID0gYjJfbWF4RmxvYXQ7XHJcbiAgICAgICAgY29uc3QgdXBwZXJYID0gKC1iMl9tYXhGbG9hdCk7XHJcbiAgICAgICAgY29uc3QgdXBwZXJZID0gKC1iMl9tYXhGbG9hdCk7XHJcbiAgICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBjb3VudDsgKytqKSB7XHJcbiAgICAgICAgICBjb25zdCBkWCA9IHBbal0ueCAtIHJvb3QueDtcclxuICAgICAgICAgIGNvbnN0IGRZID0gcFtqXS55IC0gcm9vdC55O1xyXG4gICAgICAgICAgY29uc3QgclggPSAodXhYICogZFggKyB1eFkgKiBkWSk7XHJcbiAgICAgICAgICBjb25zdCByWSA9ICh1eVggKiBkWCArIHV5WSAqIGRZKTtcclxuICAgICAgICAgIGlmIChyWCA8IGxvd2VyWCkgbG93ZXJYID0gclg7XHJcbiAgICAgICAgICBpZiAoclkgPCBsb3dlclkpIGxvd2VyWSA9IHJZO1xyXG4gICAgICAgICAgaWYgKHJYID4gdXBwZXJYKSB1cHBlclggPSByWDtcclxuICAgICAgICAgIGlmIChyWSA+IHVwcGVyWSkgdXBwZXJZID0gclk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGNvbnN0IGFyZWEgPSAodXBwZXJYIC0gbG93ZXJYKSAqICh1cHBlclkgLSBsb3dlclkpO1xyXG4gICAgICAgIGlmIChhcmVhIDwgMC45NSAqIG1pbkFyZWEpIHtcclxuICAgICAgICAgIG1pbkFyZWEgPSBhcmVhO1xyXG4gICAgICAgICAgb2JiLlIuZXgueCA9IHV4WDtcclxuICAgICAgICAgIG9iYi5SLmV4LnkgPSB1eFk7XHJcbiAgICAgICAgICBvYmIuUi5leS54ID0gdXlYO1xyXG4gICAgICAgICAgb2JiLlIuZXkueSA9IHV5WTtcclxuICAgICAgICAgIGNvbnN0IGNlbnRlcl94OiBudW1iZXIgPSAwLjUgKiAobG93ZXJYICsgdXBwZXJYKTtcclxuICAgICAgICAgIGNvbnN0IGNlbnRlcl95OiBudW1iZXIgPSAwLjUgKiAobG93ZXJZICsgdXBwZXJZKTtcclxuICAgICAgICAgIGNvbnN0IHRNYXQgPSBvYmIuUjtcclxuICAgICAgICAgIG9iYi5jZW50ZXIueCA9IHJvb3QueCArICh0TWF0LmV4LnggKiBjZW50ZXJfeCArIHRNYXQuZXkueCAqIGNlbnRlcl95KTtcclxuICAgICAgICAgIG9iYi5jZW50ZXIueSA9IHJvb3QueSArICh0TWF0LmV4LnkgKiBjZW50ZXJfeCArIHRNYXQuZXkueSAqIGNlbnRlcl95KTtcclxuICAgICAgICAgIG9iYi5leHRlbnRzLnggPSAwLjUgKiAodXBwZXJYIC0gbG93ZXJYKTtcclxuICAgICAgICAgIG9iYi5leHRlbnRzLnkgPSAwLjUgKiAodXBwZXJZIC0gbG93ZXJZKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgICovXHJcbn1cclxuIl19