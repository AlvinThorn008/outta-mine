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
import { b2_epsilon, b2_epsilon_sq, b2_linearSlop, b2_polygonRadius, b2Assert, } from '../common/b2Settings';
import { b2Abs, b2Max, b2MaxInt, b2Rot, b2Transform, b2Vec2 } from '../common/b2Math';
/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
export class b2DistanceProxy {
    constructor() {
        this.m_buffer = b2Vec2.MakeArray(2);
        this.m_vertices = this.m_buffer;
        this.m_count = 0;
        this.m_radius = NaN;
        this.m_radius = 0.0;
    }
    Copy(other) {
        if (other.m_vertices === other.m_buffer) {
            this.m_vertices = this.m_buffer;
            this.m_buffer[0].Copy(other.m_buffer[0]);
            this.m_buffer[1].Copy(other.m_buffer[1]);
        }
        else {
            this.m_vertices = other.m_vertices;
        }
        this.m_count = other.m_count;
        this.m_radius = other.m_radius;
        return this;
    }
    Reset() {
        this.m_vertices = this.m_buffer;
        this.m_count = 0;
        this.m_radius = 0;
        return this;
    }
    SetShape(shape, index) {
        shape.SetupDistanceProxy(this, index);
    }
    SetVerticesRadius(vertices, count, radius) {
        this.m_vertices = vertices;
        this.m_count = count;
        this.m_radius = radius;
    }
    GetSupport(d) {
        let bestIndex = 0;
        let bestValue = b2Vec2.DotVV(this.m_vertices[0], d);
        for (let i = 1; i < this.m_count; ++i) {
            const value = b2Vec2.DotVV(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }
        return bestIndex;
    }
    GetSupportVertex(d) {
        let bestIndex = 0;
        let bestValue = b2Vec2.DotVV(this.m_vertices[0], d);
        for (let i = 1; i < this.m_count; ++i) {
            const value = b2Vec2.DotVV(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }
        return this.m_vertices[bestIndex];
    }
    GetVertexCount() {
        return this.m_count;
    }
    GetVertex(index) {
        !!B2_DEBUG && b2Assert(0 <= index && index < this.m_count);
        return this.m_vertices[index];
    }
}
export class b2SimplexCache {
    constructor() {
        this.metric = NaN;
        this.count = 0;
        this.indexA = [0, 0, 0];
        this.indexB = [0, 0, 0];
        this.metric = 0.0;
    }
    Reset() {
        this.metric = 0;
        this.count = 0;
        return this;
    }
}
export class b2DistanceInput {
    constructor() {
        this.proxyA = new b2DistanceProxy();
        this.proxyB = new b2DistanceProxy();
        this.transformA = new b2Transform();
        this.transformB = new b2Transform();
        this.useRadii = false;
    }
    Reset() {
        this.proxyA.Reset();
        this.proxyB.Reset();
        this.transformA.SetIdentity();
        this.transformB.SetIdentity();
        this.useRadii = false;
        return this;
    }
}
export class b2DistanceOutput {
    constructor() {
        this.pointA = new b2Vec2();
        this.pointB = new b2Vec2();
        this.distance = NaN;
        this.iterations = 0; ///< number of GJK iterations used
        this.distance = 0.0;
    }
    Reset() {
        this.pointA.SetZero();
        this.pointB.SetZero();
        this.distance = 0;
        this.iterations = 0;
        return this;
    }
}
/// Input parameters for b2ShapeCast
export class b2ShapeCastInput {
    constructor() {
        this.proxyA = new b2DistanceProxy();
        this.proxyB = new b2DistanceProxy();
        this.transformA = new b2Transform();
        this.transformB = new b2Transform();
        this.translationB = new b2Vec2();
    }
}
/// Output results for b2ShapeCast
export class b2ShapeCastOutput {
    constructor() {
        this.point = new b2Vec2();
        this.normal = new b2Vec2();
        this.lambda = NaN;
        this.iterations = 0;
        this.lambda = 0.0;
    }
}
class GJKStats {
    constructor() {
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
    }
    Reset() {
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
    }
}
export const b2_gjkStats = new GJKStats();
export class b2SimplexVertex {
    constructor() {
        this.wA = new b2Vec2(); // support point in proxyA
        this.wB = new b2Vec2(); // support point in proxyB
        this.w = new b2Vec2(); // wB - wA
        this.a = NaN; // barycentric coordinate for closest point
        this.indexA = 0; // wA index
        this.indexB = 0; // wB index
        this.a = 0.0;
    }
    Copy(other) {
        this.wA.Copy(other.wA); // support point in proxyA
        this.wB.Copy(other.wB); // support point in proxyB
        this.w.Copy(other.w); // wB - wA
        this.a = other.a; // barycentric coordinate for closest point
        this.indexA = other.indexA; // wA index
        this.indexB = other.indexB; // wB index
        return this;
    }
}
export class b2Simplex {
    constructor() {
        this.m_count = 0;
        this.m_v1 = new b2SimplexVertex();
        this.m_v2 = new b2SimplexVertex();
        this.m_v3 = new b2SimplexVertex();
        this.m_vertices = [this.m_v1, this.m_v2, this.m_v3];
    }
    ReadCache(cache, proxyA, transformA, proxyB, transformB) {
        !!B2_DEBUG && b2Assert(0 <= cache.count && cache.count <= 3);
        // Copy data from cache.
        this.m_count = cache.count;
        const vertices = this.m_vertices;
        for (let i = 0; i < this.m_count; ++i) {
            const v = vertices[i];
            v.indexA = cache.indexA[i];
            v.indexB = cache.indexB[i];
            const wALocal = proxyA.GetVertex(v.indexA);
            const wBLocal = proxyB.GetVertex(v.indexB);
            b2Transform.MulXV(transformA, wALocal, v.wA);
            b2Transform.MulXV(transformB, wBLocal, v.wB);
            b2Vec2.SubVV(v.wB, v.wA, v.w);
            v.a = 0.0;
        }
        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (this.m_count > 1) {
            const metric1 = cache.metric;
            const metric2 = this.GetMetric();
            if (metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < b2_epsilon) {
                // Reset the simplex.
                this.m_count = 0;
            }
        }
        // If the cache is empty or invalid ...
        if (this.m_count === 0) {
            const v = vertices[0];
            v.indexA = 0;
            v.indexB = 0;
            const wALocal = proxyA.GetVertex(0);
            const wBLocal = proxyB.GetVertex(0);
            b2Transform.MulXV(transformA, wALocal, v.wA);
            b2Transform.MulXV(transformB, wBLocal, v.wB);
            b2Vec2.SubVV(v.wB, v.wA, v.w);
            v.a = 1.0;
            this.m_count = 1;
        }
    }
    WriteCache(cache) {
        cache.metric = this.GetMetric();
        cache.count = this.m_count;
        const vertices = this.m_vertices;
        for (let i = 0; i < this.m_count; ++i) {
            cache.indexA[i] = vertices[i].indexA;
            cache.indexB[i] = vertices[i].indexB;
        }
    }
    GetSearchDirection(out) {
        const count = this.m_count;
        if (count === 1) {
            b2Vec2.NegV(this.m_v1.w, out);
        }
        else if (count === 2) {
            const e12 = b2Vec2.SubVV(this.m_v2.w, this.m_v1.w, out);
            const sgn = b2Vec2.CrossVV(e12, b2Vec2.NegV(this.m_v1.w, b2Vec2.s_t0));
            if (sgn > 0) {
                // Origin is left of e12.
                b2Vec2.CrossOneV(e12, out);
            }
            else {
                // Origin is right of e12.
                b2Vec2.CrossVOne(e12, out);
            }
        }
        else {
            out.SetZero();
        }
        return out;
        // switch (this.m_count) {
        // case 1:
        //   return b2Vec2.NegV(this.m_v1.w, out);
        //
        // case 2: {
        //     const e12: b2Vec2 = b2Vec2.SubVV(this.m_v2.w, this.m_v1.w, out);
        //     const sgn: number = b2Vec2.CrossVV(e12, b2Vec2.NegV(this.m_v1.w, b2Vec2.s_t0));
        //     if (sgn > 0) {
        //       // Origin is left of e12.
        //       return b2Vec2.CrossOneV(e12, out);
        //     } else {
        //       // Origin is right of e12.
        //       return b2Vec2.CrossVOne(e12, out);
        //     }
        //   }
        //
        // default:
        //   !!B2_DEBUG && b2Assert(false);
        //   return out.SetZero();
        // }
    }
    GetClosestPoint(out) {
        const count = this.m_count;
        // if(count === 0) {
        //     !!B2_DEBUG && b2Assert(false);
        //     out.SetZero();
        // }
        // else
        if (count === 1) {
            out.Copy(this.m_v1.w);
        }
        else if (count === 2) {
            out.Set(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
        }
        else {
            // 3 and others
            out.SetZero();
        }
        return out;
    }
    GetWitnessPoints(pA, pB) {
        switch (this.m_count) {
            case 0:
                !!B2_DEBUG && b2Assert(false);
                break;
            case 1:
                pA.Copy(this.m_v1.wA);
                pB.Copy(this.m_v1.wB);
                break;
            case 2:
                pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
                pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
                pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
                pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
                break;
            case 3:
                pB.x = pA.x =
                    this.m_v1.a * this.m_v1.wA.x +
                        this.m_v2.a * this.m_v2.wA.x +
                        this.m_v3.a * this.m_v3.wA.x;
                pB.y = pA.y =
                    this.m_v1.a * this.m_v1.wA.y +
                        this.m_v2.a * this.m_v2.wA.y +
                        this.m_v3.a * this.m_v3.wA.y;
                break;
            default:
                !!B2_DEBUG && b2Assert(false);
                break;
        }
    }
    GetMetric() {
        if (this.m_count === 2) {
            return b2Vec2.DistanceVV(this.m_v1.w, this.m_v2.w);
        }
        else if (this.m_count === 3) {
            return b2Vec2.CrossVV(b2Vec2.SubVV(this.m_v2.w, this.m_v1.w, b2Vec2.s_t0), b2Vec2.SubVV(this.m_v3.w, this.m_v1.w, b2Vec2.s_t1));
        }
        return 0.0;
    }
    Solve2() {
        const w1 = this.m_v1.w;
        const w2 = this.m_v2.w;
        const e12 = b2Vec2.SubVV(w2, w1, b2Simplex.s_e12);
        // w1 region
        const d12_2 = -b2Vec2.DotVV(w1, e12);
        if (d12_2 <= 0) {
            // a2 <= 0, so we clamp it to 0
            this.m_v1.a = 1;
            this.m_count = 1;
            return;
        }
        // w2 region
        const d12_1 = b2Vec2.DotVV(w2, e12);
        if (d12_1 <= 0) {
            // a1 <= 0, so we clamp it to 0
            this.m_v2.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v2);
            return;
        }
        // Must be in e12 region.
        const inv_d12 = 1 / (d12_1 + d12_2);
        this.m_v1.a = d12_1 * inv_d12;
        this.m_v2.a = d12_2 * inv_d12;
        this.m_count = 2;
    }
    Solve3() {
        const w1 = this.m_v1.w;
        const w2 = this.m_v2.w;
        const w3 = this.m_v3.w;
        // Edge12
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        // a3 = 0
        const e12 = b2Vec2.SubVV(w2, w1, b2Simplex.s_e12);
        const w1e12 = b2Vec2.DotVV(w1, e12);
        const w2e12 = b2Vec2.DotVV(w2, e12);
        const d12_1 = w2e12;
        const d12_2 = -w1e12;
        // Edge13
        // [1      1     ][a1] = [1]
        // [w1.e13 w3.e13][a3] = [0]
        // a2 = 0
        const e13 = b2Vec2.SubVV(w3, w1, b2Simplex.s_e13);
        const w1e13 = b2Vec2.DotVV(w1, e13);
        const w3e13 = b2Vec2.DotVV(w3, e13);
        const d13_1 = w3e13;
        const d13_2 = -w1e13;
        // Edge23
        // [1      1     ][a2] = [1]
        // [w2.e23 w3.e23][a3] = [0]
        // a1 = 0
        const e23 = b2Vec2.SubVV(w3, w2, b2Simplex.s_e23);
        const w2e23 = b2Vec2.DotVV(w2, e23);
        const w3e23 = b2Vec2.DotVV(w3, e23);
        const d23_1 = w3e23;
        const d23_2 = -w2e23;
        // Triangle123
        const n123 = b2Vec2.CrossVV(e12, e13);
        const d123_1 = n123 * b2Vec2.CrossVV(w2, w3);
        const d123_2 = n123 * b2Vec2.CrossVV(w3, w1);
        const d123_3 = n123 * b2Vec2.CrossVV(w1, w2);
        // w1 region
        if (d12_2 <= 0 && d13_2 <= 0) {
            this.m_v1.a = 1.0;
            this.m_count = 1;
        }
        // e12
        else if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
            const inv_d12 = 1.0 / (d12_1 + d12_2);
            this.m_v1.a = d12_1 * inv_d12;
            this.m_v2.a = d12_2 * inv_d12;
            this.m_count = 2;
        }
        // e13
        else if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
            const inv_d13 = 1.0 / (d13_1 + d13_2);
            this.m_v1.a = d13_1 * inv_d13;
            this.m_v3.a = d13_2 * inv_d13;
            this.m_count = 2;
            this.m_v2.Copy(this.m_v3);
        }
        // w2 region
        else if (d12_1 <= 0 && d23_2 <= 0) {
            this.m_v2.a = 1.0;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v2);
        }
        // w3 region
        else if (d13_1 <= 0 && d23_1 <= 0) {
            this.m_v3.a = 1.0;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v3);
        }
        // e23
        else if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
            const inv_d23 = 1.0 / (d23_1 + d23_2);
            this.m_v2.a = d23_1 * inv_d23;
            this.m_v3.a = d23_2 * inv_d23;
            this.m_count = 2;
            this.m_v1.Copy(this.m_v3);
        }
        else {
            // Must be in triangle123
            const inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
            this.m_v1.a = d123_1 * inv_d123;
            this.m_v2.a = d123_2 * inv_d123;
            this.m_v3.a = d123_3 * inv_d123;
            this.m_count = 3;
        }
    }
}
b2Simplex.s_e12 = new b2Vec2();
b2Simplex.s_e13 = new b2Vec2();
b2Simplex.s_e23 = new b2Vec2();
const b2Distance_s_simplex = new b2Simplex();
const b2Distance_s_saveA = [0, 0, 0];
const b2Distance_s_saveB = [0, 0, 0];
const b2Distance_s_p = new b2Vec2();
const b2Distance_s_d = new b2Vec2();
const b2Distance_s_normal = new b2Vec2();
const b2Distance_s_supportA = new b2Vec2();
const b2Distance_s_supportB = new b2Vec2();
export function b2Distance(output, cache, input) {
    ++b2_gjkStats.calls;
    const proxyA = input.proxyA;
    const proxyB = input.proxyB;
    const transformA = input.transformA;
    const transformB = input.transformB;
    // Initialize the simplex.
    const simplex = b2Distance_s_simplex;
    simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
    // Get simplex vertices as an array.
    const vertices = simplex.m_vertices;
    const k_maxIters = 20;
    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    const saveA = b2Distance_s_saveA;
    const saveB = b2Distance_s_saveB;
    let saveCount = 0;
    // Main iteration loop.
    let iter = 0;
    while (iter < k_maxIters) {
        // Copy simplex so we can identify duplicates.
        saveCount = simplex.m_count;
        for (let i = 0; i < saveCount; ++i) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
        }
        switch (simplex.m_count) {
            case 1:
                break;
            case 2:
                simplex.Solve2();
                break;
            case 3:
                simplex.Solve3();
                break;
            default:
                !!B2_DEBUG && b2Assert(false);
                break;
        }
        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count === 3) {
            break;
        }
        // Get search direction.
        const d = simplex.GetSearchDirection(b2Distance_s_d);
        // Ensure the search direction is numerically fit.
        if (d.LengthSquared() < b2_epsilon_sq) {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.
            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }
        // Compute a tentative new simplex vertex using support points.
        const vertex = vertices[simplex.m_count];
        vertex.indexA = proxyA.GetSupport(b2Rot.MulTRV(transformA.q, b2Vec2.NegV(d, b2Vec2.s_t0), b2Distance_s_supportA));
        b2Transform.MulXV(transformA, proxyA.GetVertex(vertex.indexA), vertex.wA);
        vertex.indexB = proxyB.GetSupport(b2Rot.MulTRV(transformB.q, d, b2Distance_s_supportB));
        b2Transform.MulXV(transformB, proxyB.GetVertex(vertex.indexB), vertex.wB);
        b2Vec2.SubVV(vertex.wB, vertex.wA, vertex.w);
        // Iteration count is equated to the number of support point calls.
        ++iter;
        ++b2_gjkStats.iters;
        // Check for duplicate support points. This is the main termination criteria.
        let duplicate = false;
        for (let i = 0; i < saveCount; ++i) {
            if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
                duplicate = true;
                break;
            }
        }
        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate) {
            break;
        }
        // New vertex is ok and needed.
        ++simplex.m_count;
    }
    b2_gjkStats.maxIters = b2MaxInt(b2_gjkStats.maxIters, iter);
    // Prepare output.
    simplex.GetWitnessPoints(output.pointA, output.pointB);
    output.distance = b2Vec2.DistanceVV(output.pointA, output.pointB);
    output.iterations = iter;
    // Cache the simplex.
    simplex.WriteCache(cache);
    // Apply radii if requested.
    if (input.useRadii) {
        const rA = proxyA.m_radius;
        const rB = proxyB.m_radius;
        if (output.distance > rA + rB && output.distance > b2_epsilon) {
            // Shapes are still no overlapped.
            // Move the witness points to the outer surface.
            output.distance -= rA + rB;
            const normal = b2Vec2.SubVV(output.pointB, output.pointA, b2Distance_s_normal);
            normal.Normalize();
            output.pointA.SelfMulAdd(rA, normal);
            output.pointB.SelfMulSub(rB, normal);
        }
        else {
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            const p = b2Vec2.MidVV(output.pointA, output.pointB, b2Distance_s_p);
            output.pointA.Copy(p);
            output.pointB.Copy(p);
            output.distance = 0;
        }
    }
}
/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
// bool b2ShapeCast(b2ShapeCastOutput* output, const b2ShapeCastInput* input);
const b2ShapeCast_s_n = new b2Vec2();
const b2ShapeCast_s_simplex = new b2Simplex();
const b2ShapeCast_s_wA = new b2Vec2();
const b2ShapeCast_s_wB = new b2Vec2();
const b2ShapeCast_s_v = new b2Vec2();
const b2ShapeCast_s_p = new b2Vec2();
const b2ShapeCast_s_pointA = new b2Vec2();
const b2ShapeCast_s_pointB = new b2Vec2();
export function b2ShapeCast(output, input) {
    output.iterations = 0;
    output.lambda = 1.0;
    output.normal.SetZero();
    output.point.SetZero();
    // const b2DistanceProxy* proxyA = &input.proxyA;
    const proxyA = input.proxyA;
    // const b2DistanceProxy* proxyB = &input.proxyB;
    const proxyB = input.proxyB;
    // float32 radiusA = b2Max(proxyA.m_radius, b2_polygonRadius);
    const radiusA = b2Max(proxyA.m_radius, b2_polygonRadius);
    // float32 radiusB = b2Max(proxyB.m_radius, b2_polygonRadius);
    const radiusB = b2Max(proxyB.m_radius, b2_polygonRadius);
    // float32 radius = radiusA + radiusB;
    const radius = radiusA + radiusB;
    // b2Transform xfA = input.transformA;
    const xfA = input.transformA;
    // b2Transform xfB = input.transformB;
    const xfB = input.transformB;
    // b2Vec2 r = input.translationB;
    const r = input.translationB;
    // b2Vec2 n(0.0f, 0.0f);
    const n = b2ShapeCast_s_n.Set(0.0, 0.0);
    // float32 lambda = 0.0f;
    let lambda = 0.0;
    // Initial simplex
    const simplex = b2ShapeCast_s_simplex;
    simplex.m_count = 0;
    // Get simplex vertices as an array.
    // b2SimplexVertex* vertices = &simplex.m_v1;
    const vertices = simplex.m_vertices;
    // Get support point in -r direction
    // int32 indexA = proxyA.GetSupport(b2MulT(xfA.q, -r));
    let indexA = proxyA.GetSupport(b2Rot.MulTRV(xfA.q, b2Vec2.NegV(r, b2Vec2.s_t1), b2Vec2.s_t0));
    // b2Vec2 wA = b2Mul(xfA, proxyA.GetVertex(indexA));
    let wA = b2Transform.MulXV(xfA, proxyA.GetVertex(indexA), b2ShapeCast_s_wA);
    // int32 indexB = proxyB.GetSupport(b2MulT(xfB.q, r));
    let indexB = proxyB.GetSupport(b2Rot.MulTRV(xfB.q, r, b2Vec2.s_t0));
    // b2Vec2 wB = b2Mul(xfB, proxyB.GetVertex(indexB));
    let wB = b2Transform.MulXV(xfB, proxyB.GetVertex(indexB), b2ShapeCast_s_wB);
    // b2Vec2 v = wA - wB;
    const v = b2Vec2.SubVV(wA, wB, b2ShapeCast_s_v);
    // Sigma is the target distance between polygons
    // float32 sigma = b2Max(b2_polygonRadius, radius - b2_polygonRadius);
    const sigma = b2Max(b2_polygonRadius, radius - b2_polygonRadius);
    // const float32 tolerance = 0.5f * b2_linearSlop;
    const tolerance = 0.5 * b2_linearSlop;
    // Main iteration loop.
    // const int32 k_maxIters = 20;
    const k_maxIters = 20;
    // int32 iter = 0;
    let iter = 0;
    // while (iter < k_maxIters && b2Abs(v.Length() - sigma) > tolerance)
    while (iter < k_maxIters && b2Abs(v.Length() - sigma) > tolerance) {
        !!B2_DEBUG && b2Assert(simplex.m_count < 3);
        ++output.iterations;
        // Support in direction -v (A - B)
        // indexA = proxyA.GetSupport(b2MulT(xfA.q, -v));
        indexA = proxyA.GetSupport(b2Rot.MulTRV(xfA.q, b2Vec2.NegV(v, b2Vec2.s_t1), b2Vec2.s_t0));
        // wA = b2Mul(xfA, proxyA.GetVertex(indexA));
        wA = b2Transform.MulXV(xfA, proxyA.GetVertex(indexA), b2ShapeCast_s_wA);
        // indexB = proxyB.GetSupport(b2MulT(xfB.q, v));
        indexB = proxyB.GetSupport(b2Rot.MulTRV(xfB.q, v, b2Vec2.s_t0));
        // wB = b2Mul(xfB, proxyB.GetVertex(indexB));
        wB = b2Transform.MulXV(xfB, proxyB.GetVertex(indexB), b2ShapeCast_s_wB);
        // b2Vec2 p = wA - wB;
        const p = b2Vec2.SubVV(wA, wB, b2ShapeCast_s_p);
        // -v is a normal at p
        v.Normalize();
        // Intersect ray with plane
        const vp = b2Vec2.DotVV(v, p);
        const vr = b2Vec2.DotVV(v, r);
        if (vp - sigma > lambda * vr) {
            if (vr <= 0.0) {
                return false;
            }
            lambda = (vp - sigma) / vr;
            if (lambda > 1.0) {
                return false;
            }
            // n = -v;
            n.Copy(v).SelfNeg();
            simplex.m_count = 0;
        }
        // Reverse simplex since it works with B - A.
        // Shift by lambda * r because we want the closest point to the current clip point.
        // Note that the support point p is not shifted because we want the plane equation
        // to be formed in unshifted space.
        // b2SimplexVertex* vertex = vertices + simplex.m_count;
        const vertex = vertices[simplex.m_count];
        vertex.indexA = indexB;
        // vertex.wA = wB + lambda * r;
        vertex.wA.Copy(wB).SelfMulAdd(lambda, r);
        vertex.indexB = indexA;
        // vertex.wB = wA;
        vertex.wB.Copy(wA);
        // vertex.w = vertex.wB - vertex.wA;
        vertex.w.Copy(vertex.wB).SelfSub(vertex.wA);
        vertex.a = 1.0;
        ++simplex.m_count;
        switch (simplex.m_count) {
            case 1:
                break;
            case 2:
                simplex.Solve2();
                break;
            case 3:
                simplex.Solve3();
                break;
            default:
                !!B2_DEBUG && b2Assert(false);
        }
        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count === 3) {
            // Overlap
            return false;
        }
        // Get search direction.
        // v = simplex.GetClosestPoint();
        simplex.GetClosestPoint(v);
        // Iteration count is equated to the number of support point calls.
        ++iter;
    }
    // Prepare output.
    const pointA = b2ShapeCast_s_pointA;
    const pointB = b2ShapeCast_s_pointB;
    simplex.GetWitnessPoints(pointA, pointB);
    if (v.LengthSquared() > 0.0) {
        // n = -v;
        n.Copy(v).SelfNeg();
        n.Normalize();
    }
    // output.point = pointA + radiusA * n;
    output.normal.Copy(n);
    output.lambda = lambda;
    output.iterations = iter;
    return true;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJEaXN0YW5jZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb2xsaXNpb24vYjJEaXN0YW5jZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFDTCxVQUFVLEVBQ1YsYUFBYSxFQUNiLGFBQWEsRUFDYixnQkFBZ0IsRUFDaEIsUUFBUSxHQUNULE1BQU0sc0JBQXNCLENBQUM7QUFDOUIsT0FBTyxFQUFFLEtBQUssRUFBRSxLQUFLLEVBQUUsUUFBUSxFQUFFLEtBQUssRUFBRSxXQUFXLEVBQUUsTUFBTSxFQUFFLE1BQU0sa0JBQWtCLENBQUM7QUFHdEYsa0RBQWtEO0FBQ2xELDhCQUE4QjtBQUM5QixNQUFNLE9BQU8sZUFBZTtJQU0xQjtRQUxTLGFBQVEsR0FBYSxNQUFNLENBQUMsU0FBUyxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ2xELGVBQVUsR0FBYSxJQUFJLENBQUMsUUFBUSxDQUFDO1FBQ3JDLFlBQU8sR0FBRyxDQUFDLENBQUM7UUFDWixhQUFRLEdBQUcsR0FBRyxDQUFDO1FBR2IsSUFBSSxDQUFDLFFBQVEsR0FBRyxHQUFHLENBQUM7SUFDdEIsQ0FBQztJQUVELElBQUksQ0FBQyxLQUFnQztRQUNuQyxJQUFJLEtBQUssQ0FBQyxVQUFVLEtBQUssS0FBSyxDQUFDLFFBQVEsRUFBRTtZQUN2QyxJQUFJLENBQUMsVUFBVSxHQUFHLElBQUksQ0FBQyxRQUFRLENBQUM7WUFDaEMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQyxJQUFJLENBQUMsS0FBSyxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBQ3pDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUMsSUFBSSxDQUFDLEtBQUssQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztTQUMxQzthQUFNO1lBQ0wsSUFBSSxDQUFDLFVBQVUsR0FBRyxLQUFLLENBQUMsVUFBVSxDQUFDO1NBQ3BDO1FBQ0QsSUFBSSxDQUFDLE9BQU8sR0FBRyxLQUFLLENBQUMsT0FBTyxDQUFDO1FBQzdCLElBQUksQ0FBQyxRQUFRLEdBQUcsS0FBSyxDQUFDLFFBQVEsQ0FBQztRQUMvQixPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7SUFFRCxLQUFLO1FBQ0gsSUFBSSxDQUFDLFVBQVUsR0FBRyxJQUFJLENBQUMsUUFBUSxDQUFDO1FBQ2hDLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxDQUFDO1FBQ2pCLElBQUksQ0FBQyxRQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2xCLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztJQUVELFFBQVEsQ0FBQyxLQUFjLEVBQUUsS0FBYTtRQUNwQyxLQUFLLENBQUMsa0JBQWtCLENBQUMsSUFBSSxFQUFFLEtBQUssQ0FBQyxDQUFDO0lBQ3hDLENBQUM7SUFFRCxpQkFBaUIsQ0FBQyxRQUFrQixFQUFFLEtBQWEsRUFBRSxNQUFjO1FBQ2pFLElBQUksQ0FBQyxVQUFVLEdBQUcsUUFBUSxDQUFDO1FBQzNCLElBQUksQ0FBQyxPQUFPLEdBQUcsS0FBSyxDQUFDO1FBQ3JCLElBQUksQ0FBQyxRQUFRLEdBQUcsTUFBTSxDQUFDO0lBQ3pCLENBQUM7SUFFRCxVQUFVLENBQUMsQ0FBUztRQUNsQixJQUFJLFNBQVMsR0FBRyxDQUFDLENBQUM7UUFDbEIsSUFBSSxTQUFTLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsVUFBVSxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQzVELEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxJQUFJLENBQUMsT0FBTyxFQUFFLEVBQUUsQ0FBQyxFQUFFO1lBQ3JDLE1BQU0sS0FBSyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLFVBQVUsQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztZQUMxRCxJQUFJLEtBQUssR0FBRyxTQUFTLEVBQUU7Z0JBQ3JCLFNBQVMsR0FBRyxDQUFDLENBQUM7Z0JBQ2QsU0FBUyxHQUFHLEtBQUssQ0FBQzthQUNuQjtTQUNGO1FBRUQsT0FBTyxTQUFTLENBQUM7SUFDbkIsQ0FBQztJQUVELGdCQUFnQixDQUFDLENBQVM7UUFDeEIsSUFBSSxTQUFTLEdBQUcsQ0FBQyxDQUFDO1FBQ2xCLElBQUksU0FBUyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLFVBQVUsQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUM1RCxLQUFLLElBQUksQ0FBQyxHQUFHLENBQUMsRUFBRSxDQUFDLEdBQUcsSUFBSSxDQUFDLE9BQU8sRUFBRSxFQUFFLENBQUMsRUFBRTtZQUNyQyxNQUFNLEtBQUssR0FBVyxNQUFNLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxVQUFVLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDMUQsSUFBSSxLQUFLLEdBQUcsU0FBUyxFQUFFO2dCQUNyQixTQUFTLEdBQUcsQ0FBQyxDQUFDO2dCQUNkLFNBQVMsR0FBRyxLQUFLLENBQUM7YUFDbkI7U0FDRjtRQUVELE9BQU8sSUFBSSxDQUFDLFVBQVUsQ0FBQyxTQUFTLENBQUMsQ0FBQztJQUNwQyxDQUFDO0lBRUQsY0FBYztRQUNaLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQztJQUN0QixDQUFDO0lBRUQsU0FBUyxDQUFDLEtBQWE7UUFDckIsQ0FBQyxDQUFDLFFBQVEsSUFBSSxRQUFRLENBQUMsQ0FBQyxJQUFJLEtBQUssSUFBSSxLQUFLLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQzNELE9BQU8sSUFBSSxDQUFDLFVBQVUsQ0FBQyxLQUFLLENBQUMsQ0FBQztJQUNoQyxDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sY0FBYztJQU16QjtRQUxBLFdBQU0sR0FBRyxHQUFHLENBQUM7UUFDYixVQUFLLEdBQUcsQ0FBQyxDQUFDO1FBQ0QsV0FBTSxHQUE2QixDQUFDLENBQUMsRUFBRSxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDN0MsV0FBTSxHQUE2QixDQUFDLENBQUMsRUFBRSxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFHcEQsSUFBSSxDQUFDLE1BQU0sR0FBRyxHQUFHLENBQUM7SUFDcEIsQ0FBQztJQUVELEtBQUs7UUFDSCxJQUFJLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztRQUNoQixJQUFJLENBQUMsS0FBSyxHQUFHLENBQUMsQ0FBQztRQUNmLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztDQUNGO0FBRUQsTUFBTSxPQUFPLGVBQWU7SUFBNUI7UUFDVyxXQUFNLEdBQW9CLElBQUksZUFBZSxFQUFFLENBQUM7UUFDaEQsV0FBTSxHQUFvQixJQUFJLGVBQWUsRUFBRSxDQUFDO1FBQ2hELGVBQVUsR0FBZ0IsSUFBSSxXQUFXLEVBQUUsQ0FBQztRQUM1QyxlQUFVLEdBQWdCLElBQUksV0FBVyxFQUFFLENBQUM7UUFDckQsYUFBUSxHQUFHLEtBQUssQ0FBQztJQVVuQixDQUFDO0lBUkMsS0FBSztRQUNILElBQUksQ0FBQyxNQUFNLENBQUMsS0FBSyxFQUFFLENBQUM7UUFDcEIsSUFBSSxDQUFDLE1BQU0sQ0FBQyxLQUFLLEVBQUUsQ0FBQztRQUNwQixJQUFJLENBQUMsVUFBVSxDQUFDLFdBQVcsRUFBRSxDQUFDO1FBQzlCLElBQUksQ0FBQyxVQUFVLENBQUMsV0FBVyxFQUFFLENBQUM7UUFDOUIsSUFBSSxDQUFDLFFBQVEsR0FBRyxLQUFLLENBQUM7UUFDdEIsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sZ0JBQWdCO0lBTTNCO1FBTFMsV0FBTSxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDOUIsV0FBTSxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDdkMsYUFBUSxHQUFHLEdBQUcsQ0FBQztRQUNmLGVBQVUsR0FBRyxDQUFDLENBQUMsQ0FBQyxrQ0FBa0M7UUFHaEQsSUFBSSxDQUFDLFFBQVEsR0FBRyxHQUFHLENBQUM7SUFDdEIsQ0FBQztJQUVELEtBQUs7UUFDSCxJQUFJLENBQUMsTUFBTSxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBQ3RCLElBQUksQ0FBQyxNQUFNLENBQUMsT0FBTyxFQUFFLENBQUM7UUFDdEIsSUFBSSxDQUFDLFFBQVEsR0FBRyxDQUFDLENBQUM7UUFDbEIsSUFBSSxDQUFDLFVBQVUsR0FBRyxDQUFDLENBQUM7UUFDcEIsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0NBQ0Y7QUFFRCxvQ0FBb0M7QUFDcEMsTUFBTSxPQUFPLGdCQUFnQjtJQUE3QjtRQUNXLFdBQU0sR0FBb0IsSUFBSSxlQUFlLEVBQUUsQ0FBQztRQUNoRCxXQUFNLEdBQW9CLElBQUksZUFBZSxFQUFFLENBQUM7UUFDaEQsZUFBVSxHQUFnQixJQUFJLFdBQVcsRUFBRSxDQUFDO1FBQzVDLGVBQVUsR0FBZ0IsSUFBSSxXQUFXLEVBQUUsQ0FBQztRQUM1QyxpQkFBWSxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7SUFDL0MsQ0FBQztDQUFBO0FBRUQsa0NBQWtDO0FBQ2xDLE1BQU0sT0FBTyxpQkFBaUI7SUFNNUI7UUFMUyxVQUFLLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUM3QixXQUFNLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUN2QyxXQUFNLEdBQUcsR0FBRyxDQUFDO1FBQ2IsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUdiLElBQUksQ0FBQyxNQUFNLEdBQUcsR0FBRyxDQUFDO0lBQ3BCLENBQUM7Q0FDRjtBQUVELE1BQU0sUUFBUTtJQUFkO1FBQ0UsVUFBSyxHQUFHLENBQUMsQ0FBQztRQUNWLFVBQUssR0FBRyxDQUFDLENBQUM7UUFDVixhQUFRLEdBQUcsQ0FBQyxDQUFDO0lBT2YsQ0FBQztJQUxDLEtBQUs7UUFDSCxJQUFJLENBQUMsS0FBSyxHQUFHLENBQUMsQ0FBQztRQUNmLElBQUksQ0FBQyxLQUFLLEdBQUcsQ0FBQyxDQUFDO1FBQ2YsSUFBSSxDQUFDLFFBQVEsR0FBRyxDQUFDLENBQUM7SUFDcEIsQ0FBQztDQUNGO0FBRUQsTUFBTSxDQUFDLE1BQU0sV0FBVyxHQUFHLElBQUksUUFBUSxFQUFFLENBQUM7QUFFMUMsTUFBTSxPQUFPLGVBQWU7SUFRMUI7UUFQUyxPQUFFLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQyxDQUFDLDBCQUEwQjtRQUNyRCxPQUFFLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQyxDQUFDLDBCQUEwQjtRQUNyRCxNQUFDLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQyxDQUFDLFVBQVU7UUFDN0MsTUFBQyxHQUFHLEdBQUcsQ0FBQyxDQUFDLDJDQUEyQztRQUNwRCxXQUFNLEdBQUcsQ0FBQyxDQUFDLENBQUMsV0FBVztRQUN2QixXQUFNLEdBQUcsQ0FBQyxDQUFDLENBQUMsV0FBVztRQUdyQixJQUFJLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztJQUNmLENBQUM7SUFFRCxJQUFJLENBQUMsS0FBc0I7UUFDekIsSUFBSSxDQUFDLEVBQUUsQ0FBQyxJQUFJLENBQUMsS0FBSyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUMsMEJBQTBCO1FBQ2xELElBQUksQ0FBQyxFQUFFLENBQUMsSUFBSSxDQUFDLEtBQUssQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLDBCQUEwQjtRQUNsRCxJQUFJLENBQUMsQ0FBQyxDQUFDLElBQUksQ0FBQyxLQUFLLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxVQUFVO1FBQ2hDLElBQUksQ0FBQyxDQUFDLEdBQUcsS0FBSyxDQUFDLENBQUMsQ0FBQyxDQUFDLDJDQUEyQztRQUM3RCxJQUFJLENBQUMsTUFBTSxHQUFHLEtBQUssQ0FBQyxNQUFNLENBQUMsQ0FBQyxXQUFXO1FBQ3ZDLElBQUksQ0FBQyxNQUFNLEdBQUcsS0FBSyxDQUFDLE1BQU0sQ0FBQyxDQUFDLFdBQVc7UUFDdkMsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sU0FBUztJQU9wQjtRQU5BLFlBQU8sR0FBRyxDQUFDLENBQUM7UUFDSCxTQUFJLEdBQW9CLElBQUksZUFBZSxFQUFFLENBQUM7UUFDOUMsU0FBSSxHQUFvQixJQUFJLGVBQWUsRUFBRSxDQUFDO1FBQzlDLFNBQUksR0FBb0IsSUFBSSxlQUFlLEVBQUUsQ0FBQztRQUlyRCxJQUFJLENBQUMsVUFBVSxHQUFHLENBQUMsSUFBSSxDQUFDLElBQUksRUFBRSxJQUFJLENBQUMsSUFBSSxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztJQUN0RCxDQUFDO0lBRUQsU0FBUyxDQUNQLEtBQXFCLEVBQ3JCLE1BQXVCLEVBQ3ZCLFVBQXVCLEVBQ3ZCLE1BQXVCLEVBQ3ZCLFVBQXVCO1FBRXZCLENBQUMsQ0FBQyxRQUFRLElBQUksUUFBUSxDQUFDLENBQUMsSUFBSSxLQUFLLENBQUMsS0FBSyxJQUFJLEtBQUssQ0FBQyxLQUFLLElBQUksQ0FBQyxDQUFDLENBQUM7UUFFN0Qsd0JBQXdCO1FBQ3hCLElBQUksQ0FBQyxPQUFPLEdBQUcsS0FBSyxDQUFDLEtBQUssQ0FBQztRQUMzQixNQUFNLFFBQVEsR0FBc0IsSUFBSSxDQUFDLFVBQVUsQ0FBQztRQUNwRCxLQUFLLElBQUksQ0FBQyxHQUFHLENBQUMsRUFBRSxDQUFDLEdBQUcsSUFBSSxDQUFDLE9BQU8sRUFBRSxFQUFFLENBQUMsRUFBRTtZQUNyQyxNQUFNLENBQUMsR0FBb0IsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBQ3ZDLENBQUMsQ0FBQyxNQUFNLEdBQUcsS0FBSyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUMzQixDQUFDLENBQUMsTUFBTSxHQUFHLEtBQUssQ0FBQyxNQUFNLENBQUMsQ0FBQyxDQUFDLENBQUM7WUFDM0IsTUFBTSxPQUFPLEdBQVcsTUFBTSxDQUFDLFNBQVMsQ0FBQyxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUM7WUFDbkQsTUFBTSxPQUFPLEdBQVcsTUFBTSxDQUFDLFNBQVMsQ0FBQyxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUM7WUFDbkQsV0FBVyxDQUFDLEtBQUssQ0FBQyxVQUFVLEVBQUUsT0FBTyxFQUFFLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQztZQUM3QyxXQUFXLENBQUMsS0FBSyxDQUFDLFVBQVUsRUFBRSxPQUFPLEVBQUUsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDO1lBQzdDLE1BQU0sQ0FBQyxLQUFLLENBQUMsQ0FBQyxDQUFDLEVBQUUsRUFBRSxDQUFDLENBQUMsRUFBRSxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM5QixDQUFDLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztTQUNYO1FBRUQsd0VBQXdFO1FBQ3hFLHFDQUFxQztRQUNyQyxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxFQUFFO1lBQ3BCLE1BQU0sT0FBTyxHQUFXLEtBQUssQ0FBQyxNQUFNLENBQUM7WUFDckMsTUFBTSxPQUFPLEdBQVcsSUFBSSxDQUFDLFNBQVMsRUFBRSxDQUFDO1lBQ3pDLElBQUksT0FBTyxHQUFHLEdBQUcsR0FBRyxPQUFPLElBQUksQ0FBQyxHQUFHLE9BQU8sR0FBRyxPQUFPLElBQUksT0FBTyxHQUFHLFVBQVUsRUFBRTtnQkFDNUUscUJBQXFCO2dCQUNyQixJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsQ0FBQzthQUNsQjtTQUNGO1FBRUQsdUNBQXVDO1FBQ3ZDLElBQUksSUFBSSxDQUFDLE9BQU8sS0FBSyxDQUFDLEVBQUU7WUFDdEIsTUFBTSxDQUFDLEdBQW9CLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUN2QyxDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztZQUNiLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO1lBQ2IsTUFBTSxPQUFPLEdBQVcsTUFBTSxDQUFDLFNBQVMsQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM1QyxNQUFNLE9BQU8sR0FBVyxNQUFNLENBQUMsU0FBUyxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBQzVDLFdBQVcsQ0FBQyxLQUFLLENBQUMsVUFBVSxFQUFFLE9BQU8sRUFBRSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUM7WUFDN0MsV0FBVyxDQUFDLEtBQUssQ0FBQyxVQUFVLEVBQUUsT0FBTyxFQUFFLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQztZQUM3QyxNQUFNLENBQUMsS0FBSyxDQUFDLENBQUMsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLEVBQUUsRUFBRSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUM7WUFDOUIsQ0FBQyxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7WUFDVixJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsQ0FBQztTQUNsQjtJQUNILENBQUM7SUFFRCxVQUFVLENBQUMsS0FBcUI7UUFDOUIsS0FBSyxDQUFDLE1BQU0sR0FBRyxJQUFJLENBQUMsU0FBUyxFQUFFLENBQUM7UUFDaEMsS0FBSyxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDO1FBQzNCLE1BQU0sUUFBUSxHQUFzQixJQUFJLENBQUMsVUFBVSxDQUFDO1FBQ3BELEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxJQUFJLENBQUMsT0FBTyxFQUFFLEVBQUUsQ0FBQyxFQUFFO1lBQ3JDLEtBQUssQ0FBQyxNQUFNLENBQUMsQ0FBQyxDQUFDLEdBQUcsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDLE1BQU0sQ0FBQztZQUNyQyxLQUFLLENBQUMsTUFBTSxDQUFDLENBQUMsQ0FBQyxHQUFHLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQyxNQUFNLENBQUM7U0FDdEM7SUFDSCxDQUFDO0lBRUQsa0JBQWtCLENBQUMsR0FBVztRQUM1QixNQUFNLEtBQUssR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDO1FBQzNCLElBQUksS0FBSyxLQUFLLENBQUMsRUFBRTtZQUNmLE1BQU0sQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDLEVBQUUsR0FBRyxDQUFDLENBQUM7U0FDL0I7YUFBTSxJQUFJLEtBQUssS0FBSyxDQUFDLEVBQUU7WUFDdEIsTUFBTSxHQUFHLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsRUFB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LFNBQVM7UUFDVCw0QkFBNEI7UUFDNUIsNEJBQTRCO1FBQzVCLFNBQVM7UUFDVCxNQUFNLEdBQUcsR0FBVyxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsU0FBUyxDQUFDLEtBQUssQ0FBQyxDQUFDO1FBQzFELE1BQU0sS0FBSyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEdBQUcsQ0FBQyxDQUFDO1FBQzVDLE1BQU0sS0FBSyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEdBQUcsQ0FBQyxDQUFDO1FBQzVDLE1BQU0sS0FBSyxHQUFXLEtBQUssQ0FBQztRQUM1QixNQUFNLEtBQUssR0FBVyxDQUFDLEtBQUssQ0FBQztRQUU3QixTQUFTO1FBQ1QsNEJBQTRCO1FBQzVCLDRCQUE0QjtRQUM1QixTQUFTO1FBQ1QsTUFBTSxHQUFHLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsRUFBRSxFQUFFLFNBQVMsQ0FBQyxLQUFLLENBQUMsQ0FBQztRQUMxRCxNQUFNLEtBQUssR0FBVyxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxHQUFHLENBQUMsQ0FBQztRQUM1QyxNQUFNLEtBQUssR0FBVyxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxHQUFHLENBQUMsQ0FBQztRQUM1QyxNQUFNLEtBQUssR0FBVyxLQUFLLENBQUM7UUFDNUIsTUFBTSxLQUFLLEdBQVcsQ0FBQyxLQUFLLENBQUM7UUFFN0IsU0FBUztRQUNULDRCQUE0QjtRQUM1Qiw0QkFBNEI7UUFDNUIsU0FBUztRQUNULE1BQU0sR0FBRyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxTQUFTLENBQUMsS0FBSyxDQUFDLENBQUM7UUFDMUQsTUFBTSxLQUFLLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsR0FBRyxDQUFDLENBQUM7UUFDNUMsTUFBTSxLQUFLLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxFQUFF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QUFLLENBQUMsQ0FBQztTQUNqQztRQUVELHlFQUF5RTtRQUN6RSxJQUFJLE9BQU8sQ0FBQyxPQUFPLEtBQUssQ0FBQyxFQUFFO1lBQ3pCLFVBQVU7WUFDVixPQUFPLEtBQUssQ0FBQztTQUNkO1FBRUQsd0JBQXdCO1FBQ3hCLGlDQUFpQztRQUNqQyxPQUFPLENBQUMsZUFBZSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBRTNCLG1FQUFtRTtRQUNuRSxFQUFFLElBQUksQ0FBQztLQUNSO0lBRUQsa0JBQWtCO0lBQ2xCLE1BQU0sTUFBTSxHQUFHLG9CQUFvQixDQUFDO0lBQ3BDLE1BQU0sTUFBTSxHQUFHLG9CQUFvQixDQUFDO0lBQ3BDLE9BQU8sQ0FBQyxnQkFBZ0IsQ0FBQyxNQUFNLEVBQUUsTUFBTSxDQUFDLENBQUM7SUFFekMsSUFBSSxDQUFDLENBQUMsYUFBYSxFQUFFLEdBQUcsR0FBRyxFQUFFO1FBQzNCLFVBQVU7UUFDVixDQUFDLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQyxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBQ3BCLENBQUMsQ0FBQyxTQUFTLEVBQUUsQ0FBQztLQUNmO0lBRUQsdUNBQXVDO0lBQ3ZDLE1BQU0sQ0FBQyxNQUFNLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ3RCLE1BQU0sQ0FBQyxNQUFNLEdBQUcsTUFBTSxDQUFDO0lBQ3ZCLE1BQU0sQ0FBQyxVQUFVLEdBQUcsSUFBSSxDQUFDO0lBQ3pCLE9BQU8sSUFBSSxDQUFDO0FBQ2QsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHtcclxuICBiMl9lcHNpbG9uLFxyXG4gIGIyX2Vwc2lsb25fc3EsXHJcbiAgYjJfbGluZWFyU2xvcCxcclxuICBiMl9wb2x5Z29uUmFkaXVzLFxyXG4gIGIyQXNzZXJ0LFxyXG59IGZyb20gJy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJBYnMsIGIyTWF4LCBiMk1heEludCwgYjJSb3QsIGIyVHJhbnNmb3JtLCBiMlZlYzIgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJTaGFwZSB9IGZyb20gJy4vc2hhcGVzL2IyU2hhcGUnO1xyXG5cclxuLy8vIEEgZGlzdGFuY2UgcHJveHkgaXMgdXNlZCBieSB0aGUgR0pLIGFsZ29yaXRobS5cclxuLy8vIEl0IGVuY2Fwc3VsYXRlcyBhbnkgc2hhcGUuXHJcbmV4cG9ydCBjbGFzcyBiMkRpc3RhbmNlUHJveHkge1xyXG4gIHJlYWRvbmx5IG1fYnVmZmVyOiBiMlZlYzJbXSA9IGIyVmVjMi5NYWtlQXJyYXkoMik7XHJcbiAgbV92ZXJ0aWNlczogYjJWZWMyW10gPSB0aGlzLm1fYnVmZmVyO1xyXG4gIG1fY291bnQgPSAwO1xyXG4gIG1fcmFkaXVzID0gTmFOO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMubV9yYWRpdXMgPSAwLjA7XHJcbiAgfVxyXG5cclxuICBDb3B5KG90aGVyOiBSZWFkb25seTxiMkRpc3RhbmNlUHJveHk+KTogdGhpcyB7XHJcbiAgICBpZiAob3RoZXIubV92ZXJ0aWNlcyA9PT0gb3RoZXIubV9idWZmZXIpIHtcclxuICAgICAgdGhpcy5tX3ZlcnRpY2VzID0gdGhpcy5tX2J1ZmZlcjtcclxuICAgICAgdGhpcy5tX2J1ZmZlclswXS5Db3B5KG90aGVyLm1fYnVmZmVyWzBdKTtcclxuICAgICAgdGhpcy5tX2J1ZmZlclsxXS5Db3B5KG90aGVyLm1fYnVmZmVyWzFdKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV92ZXJ0aWNlcyA9IG90aGVyLm1fdmVydGljZXM7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fY291bnQgPSBvdGhlci5tX2NvdW50O1xyXG4gICAgdGhpcy5tX3JhZGl1cyA9IG90aGVyLm1fcmFkaXVzO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBSZXNldCgpOiBiMkRpc3RhbmNlUHJveHkge1xyXG4gICAgdGhpcy5tX3ZlcnRpY2VzID0gdGhpcy5tX2J1ZmZlcjtcclxuICAgIHRoaXMubV9jb3VudCA9IDA7XHJcbiAgICB0aGlzLm1fcmFkaXVzID0gMDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0U2hhcGUoc2hhcGU6IGIyU2hhcGUsIGluZGV4OiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHNoYXBlLlNldHVwRGlzdGFuY2VQcm94eSh0aGlzLCBpbmRleCk7XHJcbiAgfVxyXG5cclxuICBTZXRWZXJ0aWNlc1JhZGl1cyh2ZXJ0aWNlczogYjJWZWMyW10sIGNvdW50OiBudW1iZXIsIHJhZGl1czogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fdmVydGljZXMgPSB2ZXJ0aWNlcztcclxuICAgIHRoaXMubV9jb3VudCA9IGNvdW50O1xyXG4gICAgdGhpcy5tX3JhZGl1cyA9IHJhZGl1cztcclxuICB9XHJcblxyXG4gIEdldFN1cHBvcnQoZDogYjJWZWMyKTogbnVtYmVyIHtcclxuICAgIGxldCBiZXN0SW5kZXggPSAwO1xyXG4gICAgbGV0IGJlc3RWYWx1ZTogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHRoaXMubV92ZXJ0aWNlc1swXSwgZCk7XHJcbiAgICBmb3IgKGxldCBpID0gMTsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIGNvbnN0IHZhbHVlOiBudW1iZXIgPSBiMlZlYzIuRG90VlYodGhpcy5tX3ZlcnRpY2VzW2ldLCBkKTtcclxuICAgICAgaWYgKHZhbHVlID4gYmVzdFZhbHVlKSB7XHJcbiAgICAgICAgYmVzdEluZGV4ID0gaTtcclxuICAgICAgICBiZXN0VmFsdWUgPSB2YWx1ZTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBiZXN0SW5kZXg7XHJcbiAgfVxyXG5cclxuICBHZXRTdXBwb3J0VmVydGV4KGQ6IGIyVmVjMik6IGIyVmVjMiB7XHJcbiAgICBsZXQgYmVzdEluZGV4ID0gMDtcclxuICAgIGxldCBiZXN0VmFsdWU6IG51bWJlciA9IGIyVmVjMi5Eb3RWVih0aGlzLm1fdmVydGljZXNbMF0sIGQpO1xyXG4gICAgZm9yIChsZXQgaSA9IDE7IGkgPCB0aGlzLm1fY291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCB2YWx1ZTogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHRoaXMubV92ZXJ0aWNlc1tpXSwgZCk7XHJcbiAgICAgIGlmICh2YWx1ZSA+IGJlc3RWYWx1ZSkge1xyXG4gICAgICAgIGJlc3RJbmRleCA9IGk7XHJcbiAgICAgICAgYmVzdFZhbHVlID0gdmFsdWU7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gdGhpcy5tX3ZlcnRpY2VzW2Jlc3RJbmRleF07XHJcbiAgfVxyXG5cclxuICBHZXRWZXJ0ZXhDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9jb3VudDtcclxuICB9XHJcblxyXG4gIEdldFZlcnRleChpbmRleDogbnVtYmVyKTogYjJWZWMyIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoMCA8PSBpbmRleCAmJiBpbmRleCA8IHRoaXMubV9jb3VudCk7XHJcbiAgICByZXR1cm4gdGhpcy5tX3ZlcnRpY2VzW2luZGV4XTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlNpbXBsZXhDYWNoZSB7XHJcbiAgbWV0cmljID0gTmFOO1xyXG4gIGNvdW50ID0gMDtcclxuICByZWFkb25seSBpbmRleEE6IFtudW1iZXIsIG51bWJlciwgbnVtYmVyXSA9IFswLCAwLCAwXTtcclxuICByZWFkb25seSBpbmRleEI6IFtudW1iZXIsIG51bWJlciwgbnVtYmVyXSA9IFswLCAwLCAwXTtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLm1ldHJpYyA9IDAuMDtcclxuICB9XHJcblxyXG4gIFJlc2V0KCk6IGIyU2ltcGxleENhY2hlIHtcclxuICAgIHRoaXMubWV0cmljID0gMDtcclxuICAgIHRoaXMuY291bnQgPSAwO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJEaXN0YW5jZUlucHV0IHtcclxuICByZWFkb25seSBwcm94eUE6IGIyRGlzdGFuY2VQcm94eSA9IG5ldyBiMkRpc3RhbmNlUHJveHkoKTtcclxuICByZWFkb25seSBwcm94eUI6IGIyRGlzdGFuY2VQcm94eSA9IG5ldyBiMkRpc3RhbmNlUHJveHkoKTtcclxuICByZWFkb25seSB0cmFuc2Zvcm1BOiBiMlRyYW5zZm9ybSA9IG5ldyBiMlRyYW5zZm9ybSgpO1xyXG4gIHJlYWRvbmx5IHRyYW5zZm9ybUI6IGIyVHJhbnNmb3JtID0gbmV3IGIyVHJhbnNmb3JtKCk7XHJcbiAgdXNlUmFkaWkgPSBmYWxzZTtcclxuXHJcbiAgUmVzZXQoKTogYjJEaXN0YW5jZUlucHV0IHtcclxuICAgIHRoaXMucHJveHlBLlJlc2V0KCk7XHJcbiAgICB0aGlzLnByb3h5Qi5SZXNldCgpO1xyXG4gICAgdGhpcy50cmFuc2Zvcm1BLlNldElkZW50aXR5KCk7XHJcbiAgICB0aGlzLnRyYW5zZm9ybUIuU2V0SWRlbnRpdHkoKTtcclxuICAgIHRoaXMudXNlUmFkaWkgPSBmYWxzZTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyRGlzdGFuY2VPdXRwdXQge1xyXG4gIHJlYWRvbmx5IHBvaW50QTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IHBvaW50QjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIGRpc3RhbmNlID0gTmFOO1xyXG4gIGl0ZXJhdGlvbnMgPSAwOyAvLy88IG51bWJlciBvZiBHSksgaXRlcmF0aW9ucyB1c2VkXHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5kaXN0YW5jZSA9IDAuMDtcclxuICB9XHJcblxyXG4gIFJlc2V0KCk6IGIyRGlzdGFuY2VPdXRwdXQge1xyXG4gICAgdGhpcy5wb2ludEEuU2V0WmVybygpO1xyXG4gICAgdGhpcy5wb2ludEIuU2V0WmVybygpO1xyXG4gICAgdGhpcy5kaXN0YW5jZSA9IDA7XHJcbiAgICB0aGlzLml0ZXJhdGlvbnMgPSAwO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gSW5wdXQgcGFyYW1ldGVycyBmb3IgYjJTaGFwZUNhc3RcclxuZXhwb3J0IGNsYXNzIGIyU2hhcGVDYXN0SW5wdXQge1xyXG4gIHJlYWRvbmx5IHByb3h5QTogYjJEaXN0YW5jZVByb3h5ID0gbmV3IGIyRGlzdGFuY2VQcm94eSgpO1xyXG4gIHJlYWRvbmx5IHByb3h5QjogYjJEaXN0YW5jZVByb3h5ID0gbmV3IGIyRGlzdGFuY2VQcm94eSgpO1xyXG4gIHJlYWRvbmx5IHRyYW5zZm9ybUE6IGIyVHJhbnNmb3JtID0gbmV3IGIyVHJhbnNmb3JtKCk7XHJcbiAgcmVhZG9ubHkgdHJhbnNmb3JtQjogYjJUcmFuc2Zvcm0gPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICByZWFkb25seSB0cmFuc2xhdGlvbkI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxufVxyXG5cclxuLy8vIE91dHB1dCByZXN1bHRzIGZvciBiMlNoYXBlQ2FzdFxyXG5leHBvcnQgY2xhc3MgYjJTaGFwZUNhc3RPdXRwdXQge1xyXG4gIHJlYWRvbmx5IHBvaW50OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbm9ybWFsOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgbGFtYmRhID0gTmFOO1xyXG4gIGl0ZXJhdGlvbnMgPSAwO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMubGFtYmRhID0gMC4wO1xyXG4gIH1cclxufVxyXG5cclxuY2xhc3MgR0pLU3RhdHMge1xyXG4gIGNhbGxzID0gMDtcclxuICBpdGVycyA9IDA7XHJcbiAgbWF4SXRlcnMgPSAwO1xyXG5cclxuICBSZXNldCgpIHtcclxuICAgIHRoaXMuY2FsbHMgPSAwO1xyXG4gICAgdGhpcy5pdGVycyA9IDA7XHJcbiAgICB0aGlzLm1heEl0ZXJzID0gMDtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjb25zdCBiMl9namtTdGF0cyA9IG5ldyBHSktTdGF0cygpO1xyXG5cclxuZXhwb3J0IGNsYXNzIGIyU2ltcGxleFZlcnRleCB7XHJcbiAgcmVhZG9ubHkgd0E6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTsgLy8gc3VwcG9ydCBwb2ludCBpbiBwcm94eUFcclxuICByZWFkb25seSB3QjogYjJWZWMyID0gbmV3IGIyVmVjMigpOyAvLyBzdXBwb3J0IHBvaW50IGluIHByb3h5QlxyXG4gIHJlYWRvbmx5IHc6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTsgLy8gd0IgLSB3QVxyXG4gIGEgPSBOYU47IC8vIGJhcnljZW50cmljIGNvb3JkaW5hdGUgZm9yIGNsb3Nlc3QgcG9pbnRcclxuICBpbmRleEEgPSAwOyAvLyB3QSBpbmRleFxyXG4gIGluZGV4QiA9IDA7IC8vIHdCIGluZGV4XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5hID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgQ29weShvdGhlcjogYjJTaW1wbGV4VmVydGV4KTogYjJTaW1wbGV4VmVydGV4IHtcclxuICAgIHRoaXMud0EuQ29weShvdGhlci53QSk7IC8vIHN1cHBvcnQgcG9pbnQgaW4gcHJveHlBXHJcbiAgICB0aGlzLndCLkNvcHkob3RoZXIud0IpOyAvLyBzdXBwb3J0IHBvaW50IGluIHByb3h5QlxyXG4gICAgdGhpcy53LkNvcHkob3RoZXIudyk7IC8vIHdCIC0gd0FcclxuICAgIHRoaXMuYSA9IG90aGVyLmE7IC8vIGJhcnljZW50cmljIGNvb3JkaW5hdGUgZm9yIGNsb3Nlc3QgcG9pbnRcclxuICAgIHRoaXMuaW5kZXhBID0gb3RoZXIuaW5kZXhBOyAvLyB3QSBpbmRleFxyXG4gICAgdGhpcy5pbmRleEIgPSBvdGhlci5pbmRleEI7IC8vIHdCIGluZGV4XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlNpbXBsZXgge1xyXG4gIG1fY291bnQgPSAwO1xyXG4gIHJlYWRvbmx5IG1fdjE6IGIyU2ltcGxleFZlcnRleCA9IG5ldyBiMlNpbXBsZXhWZXJ0ZXgoKTtcclxuICByZWFkb25seSBtX3YyOiBiMlNpbXBsZXhWZXJ0ZXggPSBuZXcgYjJTaW1wbGV4VmVydGV4KCk7XHJcbiAgcmVhZG9ubHkgbV92MzogYjJTaW1wbGV4VmVydGV4ID0gbmV3IGIyU2ltcGxleFZlcnRleCgpO1xyXG4gIHJlYWRvbmx5IG1fdmVydGljZXM6IGIyU2ltcGxleFZlcnRleFtdO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMubV92ZXJ0aWNlcyA9IFt0aGlzLm1fdjEsIHRoaXMubV92MiwgdGhpcy5tX3YzXTtcclxuICB9XHJcblxyXG4gIFJlYWRDYWNoZShcclxuICAgIGNhY2hlOiBiMlNpbXBsZXhDYWNoZSxcclxuICAgIHByb3h5QTogYjJEaXN0YW5jZVByb3h5LFxyXG4gICAgdHJhbnNmb3JtQTogYjJUcmFuc2Zvcm0sXHJcbiAgICBwcm94eUI6IGIyRGlzdGFuY2VQcm94eSxcclxuICAgIHRyYW5zZm9ybUI6IGIyVHJhbnNmb3JtLFxyXG4gICk6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCgwIDw9IGNhY2hlLmNvdW50ICYmIGNhY2hlLmNvdW50IDw9IDMpO1xyXG5cclxuICAgIC8vIENvcHkgZGF0YSBmcm9tIGNhY2hlLlxyXG4gICAgdGhpcy5tX2NvdW50ID0gY2FjaGUuY291bnQ7XHJcbiAgICBjb25zdCB2ZXJ0aWNlczogYjJTaW1wbGV4VmVydGV4W10gPSB0aGlzLm1fdmVydGljZXM7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIGNvbnN0IHY6IGIyU2ltcGxleFZlcnRleCA9IHZlcnRpY2VzW2ldO1xyXG4gICAgICB2LmluZGV4QSA9IGNhY2hlLmluZGV4QVtpXTtcclxuICAgICAgdi5pbmRleEIgPSBjYWNoZS5pbmRleEJbaV07XHJcbiAgICAgIGNvbnN0IHdBTG9jYWw6IGIyVmVjMiA9IHByb3h5QS5HZXRWZXJ0ZXgodi5pbmRleEEpO1xyXG4gICAgICBjb25zdCB3QkxvY2FsOiBiMlZlYzIgPSBwcm94eUIuR2V0VmVydGV4KHYuaW5kZXhCKTtcclxuICAgICAgYjJUcmFuc2Zvcm0uTXVsWFYodHJhbnNmb3JtQSwgd0FMb2NhbCwgdi53QSk7XHJcbiAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHRyYW5zZm9ybUIsIHdCTG9jYWwsIHYud0IpO1xyXG4gICAgICBiMlZlYzIuU3ViVlYodi53Qiwgdi53QSwgdi53KTtcclxuICAgICAgdi5hID0gMC4wO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIENvbXB1dGUgdGhlIG5ldyBzaW1wbGV4IG1ldHJpYywgaWYgaXQgaXMgc3Vic3RhbnRpYWxseSBkaWZmZXJlbnQgdGhhblxyXG4gICAgLy8gb2xkIG1ldHJpYyB0aGVuIGZsdXNoIHRoZSBzaW1wbGV4LlxyXG4gICAgaWYgKHRoaXMubV9jb3VudCA+IDEpIHtcclxuICAgICAgY29uc3QgbWV0cmljMTogbnVtYmVyID0gY2FjaGUubWV0cmljO1xyXG4gICAgICBjb25zdCBtZXRyaWMyOiBudW1iZXIgPSB0aGlzLkdldE1ldHJpYygpO1xyXG4gICAgICBpZiAobWV0cmljMiA8IDAuNSAqIG1ldHJpYzEgfHwgMiAqIG1ldHJpYzEgPCBtZXRyaWMyIHx8IG1ldHJpYzIgPCBiMl9lcHNpbG9uKSB7XHJcbiAgICAgICAgLy8gUmVzZXQgdGhlIHNpbXBsZXguXHJcbiAgICAgICAgdGhpcy5tX2NvdW50ID0gMDtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIElmIHRoZSBjYWNoZSBpcyBlbXB0eSBvciBpbnZhbGlkIC4uLlxyXG4gICAgaWYgKHRoaXMubV9jb3VudCA9PT0gMCkge1xyXG4gICAgICBjb25zdCB2OiBiMlNpbXBsZXhWZXJ0ZXggPSB2ZXJ0aWNlc1swXTtcclxuICAgICAgdi5pbmRleEEgPSAwO1xyXG4gICAgICB2LmluZGV4QiA9IDA7XHJcbiAgICAgIGNvbnN0IHdBTG9jYWw6IGIyVmVjMiA9IHByb3h5QS5HZXRWZXJ0ZXgoMCk7XHJcbiAgICAgIGNvbnN0IHdCTG9jYWw6IGIyVmVjMiA9IHByb3h5Qi5HZXRWZXJ0ZXgoMCk7XHJcbiAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHRyYW5zZm9ybUEsIHdBTG9jYWwsIHYud0EpO1xyXG4gICAgICBiMlRyYW5zZm9ybS5NdWxYVih0cmFuc2Zvcm1CLCB3QkxvY2FsLCB2LndCKTtcclxuICAgICAgYjJWZWMyLlN1YlZWKHYud0IsIHYud0EsIHYudyk7XHJcbiAgICAgIHYuYSA9IDEuMDtcclxuICAgICAgdGhpcy5tX2NvdW50ID0gMTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIFdyaXRlQ2FjaGUoY2FjaGU6IGIyU2ltcGxleENhY2hlKTogdm9pZCB7XHJcbiAgICBjYWNoZS5tZXRyaWMgPSB0aGlzLkdldE1ldHJpYygpO1xyXG4gICAgY2FjaGUuY291bnQgPSB0aGlzLm1fY291bnQ7XHJcbiAgICBjb25zdCB2ZXJ0aWNlczogYjJTaW1wbGV4VmVydGV4W10gPSB0aGlzLm1fdmVydGljZXM7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIGNhY2hlLmluZGV4QVtpXSA9IHZlcnRpY2VzW2ldLmluZGV4QTtcclxuICAgICAgY2FjaGUuaW5kZXhCW2ldID0gdmVydGljZXNbaV0uaW5kZXhCO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgR2V0U2VhcmNoRGlyZWN0aW9uKG91dDogYjJWZWMyKTogYjJWZWMyIHtcclxuICAgIGNvbnN0IGNvdW50ID0gdGhpcy5tX2NvdW50O1xyXG4gICAgaWYgKGNvdW50ID09PSAxKSB7XHJcbiAgICAgIGIyVmVjMi5OZWdWKHRoaXMubV92MS53LCBvdXQpO1xyXG4gICAgfSBlbHNlIGlmIChjb3VudCA9PT0gMikge1xyXG4gICAgICBjb25zdCBlMTI6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVih0aGlzLm1fdjIudywgdGhpcy5tX3YxLncsIG91dCk7XHJcbiAgICAgIGNvbnN0IHNnbjogbnVtYmVyID0gYjJWZWMyLkNyb3NzVlYoZTEyLCBiMlZlYzIuTmVnVih0aGlzLm1fdjEudywgYjJWZWMyLnNfdDApKTtcclxuICAgICAgaWYgKHNnbiA+IDApIHtcclxuICAgICAgICAvLyBPcmlnaW4gaXMgbGVmdCBvZiBlMTIuXHJcbiAgICAgICAgYjJWZWMyLkNyb3NzT25lVihlMTIsIG91dCk7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgLy8gT3JpZ2luIGlzIHJpZ2h0IG9mIGUxMi5cclxuICAgICAgICBiMlZlYzIuQ3Jvc3NWT25lKGUxMiwgb3V0KTtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgb3V0LlNldFplcm8oKTtcclxuICAgIH1cclxuICAgIHJldHVybiBvdXQ7XHJcbiAgICAvLyBzd2l0Y2ggKHRoaXMubV9jb3VudCkge1xyXG4gICAgLy8gY2FzZSAxOlxyXG4gICAgLy8gICByZXR1cm4gYjJWZWMyLk5lZ1YodGhpcy5tX3YxLncsIG91dCk7XHJcbiAgICAvL1xyXG4gICAgLy8gY2FzZSAyOiB7XHJcbiAgICAvLyAgICAgY29uc3QgZTEyOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYodGhpcy5tX3YyLncsIHRoaXMubV92MS53LCBvdXQpO1xyXG4gICAgLy8gICAgIGNvbnN0IHNnbjogbnVtYmVyID0gYjJWZWMyLkNyb3NzVlYoZTEyLCBiMlZlYzIuTmVnVih0aGlzLm1fdjEudywgYjJWZWMyLnNfdDApKTtcclxuICAgIC8vICAgICBpZiAoc2duID4gMCkge1xyXG4gICAgLy8gICAgICAgLy8gT3JpZ2luIGlzIGxlZnQgb2YgZTEyLlxyXG4gICAgLy8gICAgICAgcmV0dXJuIGIyVmVjMi5Dcm9zc09uZVYoZTEyLCBvdXQpO1xyXG4gICAgLy8gICAgIH0gZWxzZSB7XHJcbiAgICAvLyAgICAgICAvLyBPcmlnaW4gaXMgcmlnaHQgb2YgZTEyLlxyXG4gICAgLy8gICAgICAgcmV0dXJuIGIyVmVjMi5Dcm9zc1ZPbmUoZTEyLCBvdXQpO1xyXG4gICAgLy8gICAgIH1cclxuICAgIC8vICAgfVxyXG4gICAgLy9cclxuICAgIC8vIGRlZmF1bHQ6XHJcbiAgICAvLyAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpO1xyXG4gICAgLy8gICByZXR1cm4gb3V0LlNldFplcm8oKTtcclxuICAgIC8vIH1cclxuICB9XHJcblxyXG4gIEdldENsb3Nlc3RQb2ludChvdXQ6IGIyVmVjMik6IGIyVmVjMiB7XHJcbiAgICBjb25zdCBjb3VudCA9IHRoaXMubV9jb3VudDtcclxuICAgIC8vIGlmKGNvdW50ID09PSAwKSB7XHJcbiAgICAvLyAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgICAvLyAgICAgb3V0LlNldFplcm8oKTtcclxuICAgIC8vIH1cclxuICAgIC8vIGVsc2VcclxuICAgIGlmIChjb3VudCA9PT0gMSkge1xyXG4gICAgICBvdXQuQ29weSh0aGlzLm1fdjEudyk7XHJcbiAgICB9IGVsc2UgaWYgKGNvdW50ID09PSAyKSB7XHJcbiAgICAgIG91dC5TZXQoXHJcbiAgICAgICAgdGhpcy5tX3YxLmEgKiB0aGlzLm1fdjEudy54ICsgdGhpcy5tX3YyLmEgKiB0aGlzLm1fdjIudy54LFxyXG4gICAgICAgIHRoaXMubV92MS5hICogdGhpcy5tX3YxLncueSArIHRoaXMubV92Mi5hICogdGhpcy5tX3YyLncueSxcclxuICAgICAgKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIDMgYW5kIG90aGVyc1xyXG4gICAgICBvdXQuU2V0WmVybygpO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIEdldFdpdG5lc3NQb2ludHMocEE6IGIyVmVjMiwgcEI6IGIyVmVjMik6IHZvaWQge1xyXG4gICAgc3dpdGNoICh0aGlzLm1fY291bnQpIHtcclxuICAgICAgY2FzZSAwOlxyXG4gICAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpO1xyXG4gICAgICAgIGJyZWFrO1xyXG5cclxuICAgICAgY2FzZSAxOlxyXG4gICAgICAgIHBBLkNvcHkodGhpcy5tX3YxLndBKTtcclxuICAgICAgICBwQi5Db3B5KHRoaXMubV92MS53Qik7XHJcbiAgICAgICAgYnJlYWs7XHJcblxyXG4gICAgICBjYXNlIDI6XHJcbiAgICAgICAgcEEueCA9IHRoaXMubV92MS5hICogdGhpcy5tX3YxLndBLnggKyB0aGlzLm1fdjIuYSAqIHRoaXMubV92Mi53QS54O1xyXG4gICAgICAgIHBBLnkgPSB0aGlzLm1fdjEuYSAqIHRoaXMubV92MS53QS55ICsgdGhpcy5tX3YyLmEgKiB0aGlzLm1fdjIud0EueTtcclxuICAgICAgICBwQi54ID0gdGhpcy5tX3YxLmEgKiB0aGlzLm1fdjEud0IueCArIHRoaXMubV92Mi5hICogdGhpcy5tX3YyLndCLng7XHJcbiAgICAgICAgcEIueSA9IHRoaXMubV92MS5hICogdGhpcy5tX3YxLndCLnkgKyB0aGlzLm1fdjIuYSAqIHRoaXMubV92Mi53Qi55O1xyXG4gICAgICAgIGJyZWFrO1xyXG5cclxuICAgICAgY2FzZSAzOlxyXG4gICAgICAgIHBCLnggPSBwQS54ID1cclxuICAgICAgICAgIHRoaXMubV92MS5hICogdGhpcy5tX3YxLndBLnggK1xyXG4gICAgICAgICAgdGhpcy5tX3YyLmEgKiB0aGlzLm1fdjIud0EueCArXHJcbiAgICAgICAgICB0aGlzLm1fdjMuYSAqIHRoaXMubV92My53QS54O1xyXG4gICAgICAgIHBCLnkgPSBwQS55ID1cclxuICAgICAgICAgIHRoaXMubV92MS5hICogdGhpcy5tX3YxLndBLnkgK1xyXG4gICAgICAgICAgdGhpcy5tX3YyLmEgKiB0aGlzLm1fdjIud0EueSArXHJcbiAgICAgICAgICB0aGlzLm1fdjMuYSAqIHRoaXMubV92My53QS55O1xyXG4gICAgICAgIGJyZWFrO1xyXG5cclxuICAgICAgZGVmYXVsdDpcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZhbHNlKTtcclxuICAgICAgICBicmVhaztcclxuICAgIH1cclxuICB9XHJcblxyXG4gIEdldE1ldHJpYygpOiBudW1iZXIge1xyXG4gICAgaWYgKHRoaXMubV9jb3VudCA9PT0gMikge1xyXG4gICAgICByZXR1cm4gYjJWZWMyLkRpc3RhbmNlVlYodGhpcy5tX3YxLncsIHRoaXMubV92Mi53KTtcclxuICAgIH0gZWxzZSBpZiAodGhpcy5tX2NvdW50ID09PSAzKSB7XHJcbiAgICAgIHJldHVybiBiMlZlYzIuQ3Jvc3NWVihcclxuICAgICAgICBiMlZlYzIuU3ViVlYodGhpcy5tX3YyLncsIHRoaXMubV92MS53LCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV92My53LCB0aGlzLm1fdjEudywgYjJWZWMyLnNfdDEpLFxyXG4gICAgICApO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIDAuMDtcclxuICB9XHJcblxyXG4gIFNvbHZlMigpOiB2b2lkIHtcclxuICAgIGNvbnN0IHcxOiBiMlZlYzIgPSB0aGlzLm1fdjEudztcclxuICAgIGNvbnN0IHcyOiBiMlZlYzIgPSB0aGlzLm1fdjIudztcclxuICAgIGNvbnN0IGUxMjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHcyLCB3MSwgYjJTaW1wbGV4LnNfZTEyKTtcclxuXHJcbiAgICAvLyB3MSByZWdpb25cclxuICAgIGNvbnN0IGQxMl8yOiBudW1iZXIgPSAtYjJWZWMyLkRvdFZWKHcxLCBlMTIpO1xyXG4gICAgaWYgKGQxMl8yIDw9IDApIHtcclxuICAgICAgLy8gYTIgPD0gMCwgc28gd2UgY2xhbXAgaXQgdG8gMFxyXG4gICAgICB0aGlzLm1fdjEuYSA9IDE7XHJcbiAgICAgIHRoaXMubV9jb3VudCA9IDE7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICAvLyB3MiByZWdpb25cclxuICAgIGNvbnN0IGQxMl8xOiBudW1iZXIgPSBiMlZlYzIuRG90VlYodzIsIGUxMik7XHJcbiAgICBpZiAoZDEyXzEgPD0gMCkge1xyXG4gICAgICAvLyBhMSA8PSAwLCBzbyB3ZSBjbGFtcCBpdCB0byAwXHJcbiAgICAgIHRoaXMubV92Mi5hID0gMTtcclxuICAgICAgdGhpcy5tX2NvdW50ID0gMTtcclxuICAgICAgdGhpcy5tX3YxLkNvcHkodGhpcy5tX3YyKTtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIE11c3QgYmUgaW4gZTEyIHJlZ2lvbi5cclxuICAgIGNvbnN0IGludl9kMTI6IG51bWJlciA9IDEgLyAoZDEyXzEgKyBkMTJfMik7XHJcbiAgICB0aGlzLm1fdjEuYSA9IGQxMl8xICogaW52X2QxMjtcclxuICAgIHRoaXMubV92Mi5hID0gZDEyXzIgKiBpbnZfZDEyO1xyXG4gICAgdGhpcy5tX2NvdW50ID0gMjtcclxuICB9XHJcblxyXG4gIFNvbHZlMygpOiB2b2lkIHtcclxuICAgIGNvbnN0IHcxOiBiMlZlYzIgPSB0aGlzLm1fdjEudztcclxuICAgIGNvbnN0IHcyOiBiMlZlYzIgPSB0aGlzLm1fdjIudztcclxuICAgIGNvbnN0IHczOiBiMlZlYzIgPSB0aGlzLm1fdjMudztcclxuXHJcbiAgICAvLyBFZGdlMTJcclxuICAgIC8vIFsxICAgICAgMSAgICAgXVthMV0gPSBbMV1cclxuICAgIC8vIFt3MS5lMTIgdzIuZTEyXVthMl0gPSBbMF1cclxuICAgIC8vIGEzID0gMFxyXG4gICAgY29uc3QgZTEyOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYodzIsIHcxLCBiMlNpbXBsZXguc19lMTIpO1xyXG4gICAgY29uc3QgdzFlMTI6IG51bWJlciA9IGIyVmVjMi5Eb3RWVih3MSwgZTEyKTtcclxuICAgIGNvbnN0IHcyZTEyOiBudW1iZXIgPSBiMlZlYzIuRG90VlYodzIsIGUxMik7XHJcbiAgICBjb25zdCBkMTJfMTogbnVtYmVyID0gdzJlMTI7XHJcbiAgICBjb25zdCBkMTJfMjogbnVtYmVyID0gLXcxZTEyO1xyXG5cclxuICAgIC8vIEVkZ2UxM1xyXG4gICAgLy8gWzEgICAgICAxICAgICBdW2ExXSA9IFsxXVxyXG4gICAgLy8gW3cxLmUxMyB3My5lMTNdW2EzXSA9IFswXVxyXG4gICAgLy8gYTIgPSAwXHJcbiAgICBjb25zdCBlMTM6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVih3MywgdzEsIGIyU2ltcGxleC5zX2UxMyk7XHJcbiAgICBjb25zdCB3MWUxMzogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHcxLCBlMTMpO1xyXG4gICAgY29uc3QgdzNlMTM6IG51bWJlciA9IGIyVmVjMi5Eb3RWVih3MywgZTEzKTtcclxuICAgIGNvbnN0IGQxM18xOiBudW1iZXIgPSB3M2UxMztcclxuICAgIGNvbnN0IGQxM18yOiBudW1iZXIgPSAtdzFlMTM7XHJcblxyXG4gICAgLy8gRWRnZTIzXHJcbiAgICAvLyBbMSAgICAgIDEgICAgIF1bYTJdID0gWzFdXHJcbiAgICAvLyBbdzIuZTIzIHczLmUyM11bYTNdID0gWzBdXHJcbiAgICAvLyBhMSA9IDBcclxuICAgIGNvbnN0IGUyMzogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHczLCB3MiwgYjJTaW1wbGV4LnNfZTIzKTtcclxuICAgIGNvbnN0IHcyZTIzOiBudW1iZXIgPSBiMlZlYzIuRG90VlYodzIsIGUyMyk7XHJcbiAgICBjb25zdCB3M2UyMzogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHczLCBlMjMpO1xyXG4gICAgY29uc3QgZDIzXzE6IG51bWJlciA9IHczZTIzO1xyXG4gICAgY29uc3QgZDIzXzI6IG51bWJlciA9IC13MmUyMztcclxuXHJcbiAgICAvLyBUcmlhbmdsZTEyM1xyXG4gICAgY29uc3QgbjEyMzogbnVtYmVyID0gYjJWZWMyLkNyb3NzVlYoZTEyLCBlMTMpO1xyXG5cclxuICAgIGNvbnN0IGQxMjNfMTogbnVtYmVyID0gbjEyMyAqIGIyVmVjMi5Dcm9zc1ZWKHcyLCB3Myk7XHJcbiAgICBjb25zdCBkMTIzXzI6IG51bWJlciA9IG4xMjMgKiBiMlZlYzIuQ3Jvc3NWVih3MywgdzEpO1xyXG4gICAgY29uc3QgZDEyM18zOiBudW1iZXIgPSBuMTIzICogYjJWZWMyLkNyb3NzVlYodzEsIHcyKTtcclxuXHJcbiAgICAvLyB3MSByZWdpb25cclxuICAgIGlmIChkMTJfMiA8PSAwICYmIGQxM18yIDw9IDApIHtcclxuICAgICAgdGhpcy5tX3YxLmEgPSAxLjA7XHJcbiAgICAgIHRoaXMubV9jb3VudCA9IDE7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZTEyXHJcbiAgICBlbHNlIGlmIChkMTJfMSA+IDAgJiYgZDEyXzIgPiAwICYmIGQxMjNfMyA8PSAwKSB7XHJcbiAgICAgIGNvbnN0IGludl9kMTI6IG51bWJlciA9IDEuMCAvIChkMTJfMSArIGQxMl8yKTtcclxuICAgICAgdGhpcy5tX3YxLmEgPSBkMTJfMSAqIGludl9kMTI7XHJcbiAgICAgIHRoaXMubV92Mi5hID0gZDEyXzIgKiBpbnZfZDEyO1xyXG4gICAgICB0aGlzLm1fY291bnQgPSAyO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGUxM1xyXG4gICAgZWxzZSBpZiAoZDEzXzEgPiAwICYmIGQxM18yID4gMCAmJiBkMTIzXzIgPD0gMCkge1xyXG4gICAgICBjb25zdCBpbnZfZDEzOiBudW1iZXIgPSAxLjAgLyAoZDEzXzEgKyBkMTNfMik7XHJcbiAgICAgIHRoaXMubV92MS5hID0gZDEzXzEgKiBpbnZfZDEzO1xyXG4gICAgICB0aGlzLm1fdjMuYSA9IGQxM18yICogaW52X2QxMztcclxuICAgICAgdGhpcy5tX2NvdW50ID0gMjtcclxuICAgICAgdGhpcy5tX3YyLkNvcHkodGhpcy5tX3YzKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyB3MiByZWdpb25cclxuICAgIGVsc2UgaWYgKGQxMl8xIDw9IDAgJiYgZDIzXzIgPD0gMCkge1xyXG4gICAgICB0aGlzLm1fdjIuYSA9IDEuMDtcclxuICAgICAgdGhpcy5tX2NvdW50ID0gMTtcclxuICAgICAgdGhpcy5tX3YxLkNvcHkodGhpcy5tX3YyKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyB3MyByZWdpb25cclxuICAgIGVsc2UgaWYgKGQxM18xIDw9IDAgJiYgZDIzXzEgPD0gMCkge1xyXG4gICAgICB0aGlzLm1fdjMuYSA9IDEuMDtcclxuICAgICAgdGhpcy5tX2NvdW50ID0gMTtcclxuICAgICAgdGhpcy5tX3YxLkNvcHkodGhpcy5tX3YzKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBlMjNcclxuICAgIGVsc2UgaWYgKGQyM18xID4gMCAmJiBkMjNfMiA+IDAgJiYgZDEyM18xIDw9IDApIHtcclxuICAgICAgY29uc3QgaW52X2QyMzogbnVtYmVyID0gMS4wIC8gKGQyM18xICsgZDIzXzIpO1xyXG4gICAgICB0aGlzLm1fdjIuYSA9IGQyM18xICogaW52X2QyMztcclxuICAgICAgdGhpcy5tX3YzLmEgPSBkMjNfMiAqIGludl9kMjM7XHJcbiAgICAgIHRoaXMubV9jb3VudCA9IDI7XHJcbiAgICAgIHRoaXMubV92MS5Db3B5KHRoaXMubV92Myk7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICAvLyBNdXN0IGJlIGluIHRyaWFuZ2xlMTIzXHJcbiAgICAgIGNvbnN0IGludl9kMTIzOiBudW1iZXIgPSAxLjAgLyAoZDEyM18xICsgZDEyM18yICsgZDEyM18zKTtcclxuICAgICAgdGhpcy5tX3YxLmEgPSBkMTIzXzEgKiBpbnZfZDEyMztcclxuICAgICAgdGhpcy5tX3YyLmEgPSBkMTIzXzIgKiBpbnZfZDEyMztcclxuICAgICAgdGhpcy5tX3YzLmEgPSBkMTIzXzMgKiBpbnZfZDEyMztcclxuICAgICAgdGhpcy5tX2NvdW50ID0gMztcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIHNfZTEyOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19lMTM6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBzX2UyMzogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG59XHJcblxyXG5jb25zdCBiMkRpc3RhbmNlX3Nfc2ltcGxleDogYjJTaW1wbGV4ID0gbmV3IGIyU2ltcGxleCgpO1xyXG5jb25zdCBiMkRpc3RhbmNlX3Nfc2F2ZUE6IFtudW1iZXIsIG51bWJlciwgbnVtYmVyXSA9IFswLCAwLCAwXTtcclxuY29uc3QgYjJEaXN0YW5jZV9zX3NhdmVCOiBbbnVtYmVyLCBudW1iZXIsIG51bWJlcl0gPSBbMCwgMCwgMF07XHJcbmNvbnN0IGIyRGlzdGFuY2Vfc19wOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyRGlzdGFuY2Vfc19kOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyRGlzdGFuY2Vfc19ub3JtYWw6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJEaXN0YW5jZV9zX3N1cHBvcnRBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyRGlzdGFuY2Vfc19zdXBwb3J0QjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyRGlzdGFuY2UoXHJcbiAgb3V0cHV0OiBiMkRpc3RhbmNlT3V0cHV0LFxyXG4gIGNhY2hlOiBiMlNpbXBsZXhDYWNoZSxcclxuICBpbnB1dDogYjJEaXN0YW5jZUlucHV0LFxyXG4pOiB2b2lkIHtcclxuICArK2IyX2dqa1N0YXRzLmNhbGxzO1xyXG5cclxuICBjb25zdCBwcm94eUE6IGIyRGlzdGFuY2VQcm94eSA9IGlucHV0LnByb3h5QTtcclxuICBjb25zdCBwcm94eUI6IGIyRGlzdGFuY2VQcm94eSA9IGlucHV0LnByb3h5QjtcclxuXHJcbiAgY29uc3QgdHJhbnNmb3JtQTogYjJUcmFuc2Zvcm0gPSBpbnB1dC50cmFuc2Zvcm1BO1xyXG4gIGNvbnN0IHRyYW5zZm9ybUI6IGIyVHJhbnNmb3JtID0gaW5wdXQudHJhbnNmb3JtQjtcclxuXHJcbiAgLy8gSW5pdGlhbGl6ZSB0aGUgc2ltcGxleC5cclxuICBjb25zdCBzaW1wbGV4OiBiMlNpbXBsZXggPSBiMkRpc3RhbmNlX3Nfc2ltcGxleDtcclxuICBzaW1wbGV4LlJlYWRDYWNoZShjYWNoZSwgcHJveHlBLCB0cmFuc2Zvcm1BLCBwcm94eUIsIHRyYW5zZm9ybUIpO1xyXG5cclxuICAvLyBHZXQgc2ltcGxleCB2ZXJ0aWNlcyBhcyBhbiBhcnJheS5cclxuICBjb25zdCB2ZXJ0aWNlczogYjJTaW1wbGV4VmVydGV4W10gPSBzaW1wbGV4Lm1fdmVydGljZXM7XHJcbiAgY29uc3Qga19tYXhJdGVycyA9IDIwO1xyXG5cclxuICAvLyBUaGVzZSBzdG9yZSB0aGUgdmVydGljZXMgb2YgdGhlIGxhc3Qgc2ltcGxleCBzbyB0aGF0IHdlXHJcbiAgLy8gY2FuIGNoZWNrIGZvciBkdXBsaWNhdGVzIGFuZCBwcmV2ZW50IGN5Y2xpbmcuXHJcbiAgY29uc3Qgc2F2ZUE6IFtudW1iZXIsIG51bWJlciwgbnVtYmVyXSA9IGIyRGlzdGFuY2Vfc19zYXZlQTtcclxuICBjb25zdCBzYXZlQjogW251bWJlciwgbnVtYmVyLCBudW1iZXJdID0gYjJEaXN0YW5jZV9zX3NhdmVCO1xyXG4gIGxldCBzYXZlQ291bnQgPSAwO1xyXG5cclxuICAvLyBNYWluIGl0ZXJhdGlvbiBsb29wLlxyXG4gIGxldCBpdGVyID0gMDtcclxuICB3aGlsZSAoaXRlciA8IGtfbWF4SXRlcnMpIHtcclxuICAgIC8vIENvcHkgc2ltcGxleCBzbyB3ZSBjYW4gaWRlbnRpZnkgZHVwbGljYXRlcy5cclxuICAgIHNhdmVDb3VudCA9IHNpbXBsZXgubV9jb3VudDtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgc2F2ZUNvdW50OyArK2kpIHtcclxuICAgICAgc2F2ZUFbaV0gPSB2ZXJ0aWNlc1tpXS5pbmRleEE7XHJcbiAgICAgIHNhdmVCW2ldID0gdmVydGljZXNbaV0uaW5kZXhCO1xyXG4gICAgfVxyXG5cclxuICAgIHN3aXRjaCAoc2ltcGxleC5tX2NvdW50KSB7XHJcbiAgICAgIGNhc2UgMTpcclxuICAgICAgICBicmVhaztcclxuXHJcbiAgICAgIGNhc2UgMjpcclxuICAgICAgICBzaW1wbGV4LlNvbHZlMigpO1xyXG4gICAgICAgIGJyZWFrO1xyXG5cclxuICAgICAgY2FzZSAzOlxyXG4gICAgICAgIHNpbXBsZXguU29sdmUzKCk7XHJcbiAgICAgICAgYnJlYWs7XHJcblxyXG4gICAgICBkZWZhdWx0OlxyXG4gICAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpO1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIElmIHdlIGhhdmUgMyBwb2ludHMsIHRoZW4gdGhlIG9yaWdpbiBpcyBpbiB0aGUgY29ycmVzcG9uZGluZyB0cmlhbmdsZS5cclxuICAgIGlmIChzaW1wbGV4Lm1fY291bnQgPT09IDMpIHtcclxuICAgICAgYnJlYWs7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gR2V0IHNlYXJjaCBkaXJlY3Rpb24uXHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBzaW1wbGV4LkdldFNlYXJjaERpcmVjdGlvbihiMkRpc3RhbmNlX3NfZCk7XHJcblxyXG4gICAgLy8gRW5zdXJlIHRoZSBzZWFyY2ggZGlyZWN0aW9uIGlzIG51bWVyaWNhbGx5IGZpdC5cclxuICAgIGlmIChkLkxlbmd0aFNxdWFyZWQoKSA8IGIyX2Vwc2lsb25fc3EpIHtcclxuICAgICAgLy8gVGhlIG9yaWdpbiBpcyBwcm9iYWJseSBjb250YWluZWQgYnkgYSBsaW5lIHNlZ21lbnRcclxuICAgICAgLy8gb3IgdHJpYW5nbGUuIFRodXMgdGhlIHNoYXBlcyBhcmUgb3ZlcmxhcHBlZC5cclxuXHJcbiAgICAgIC8vIFdlIGNhbid0IHJldHVybiB6ZXJvIGhlcmUgZXZlbiB0aG91Z2ggdGhlcmUgbWF5IGJlIG92ZXJsYXAuXHJcbiAgICAgIC8vIEluIGNhc2UgdGhlIHNpbXBsZXggaXMgYSBwb2ludCwgc2VnbWVudCwgb3IgdHJpYW5nbGUgaXQgaXMgZGlmZmljdWx0XHJcbiAgICAgIC8vIHRvIGRldGVybWluZSBpZiB0aGUgb3JpZ2luIGlzIGNvbnRhaW5lZCBpbiB0aGUgQ1NPIG9yIHZlcnkgY2xvc2UgdG8gaXQuXHJcbiAgICAgIGJyZWFrO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIENvbXB1dGUgYSB0ZW50YXRpdmUgbmV3IHNpbXBsZXggdmVydGV4IHVzaW5nIHN1cHBvcnQgcG9pbnRzLlxyXG4gICAgY29uc3QgdmVydGV4OiBiMlNpbXBsZXhWZXJ0ZXggPSB2ZXJ0aWNlc1tzaW1wbGV4Lm1fY291bnRdO1xyXG4gICAgdmVydGV4LmluZGV4QSA9IHByb3h5QS5HZXRTdXBwb3J0KFxyXG4gICAgICBiMlJvdC5NdWxUUlYodHJhbnNmb3JtQS5xLCBiMlZlYzIuTmVnVihkLCBiMlZlYzIuc190MCksIGIyRGlzdGFuY2Vfc19zdXBwb3J0QSksXHJcbiAgICApO1xyXG4gICAgYjJUcmFuc2Zvcm0uTXVsWFYodHJhbnNmb3JtQSwgcHJveHlBLkdldFZlcnRleCh2ZXJ0ZXguaW5kZXhBKSwgdmVydGV4LndBKTtcclxuICAgIHZlcnRleC5pbmRleEIgPSBwcm94eUIuR2V0U3VwcG9ydChiMlJvdC5NdWxUUlYodHJhbnNmb3JtQi5xLCBkLCBiMkRpc3RhbmNlX3Nfc3VwcG9ydEIpKTtcclxuICAgIGIyVHJhbnNmb3JtLk11bFhWKHRyYW5zZm9ybUIsIHByb3h5Qi5HZXRWZXJ0ZXgodmVydGV4LmluZGV4QiksIHZlcnRleC53Qik7XHJcbiAgICBiMlZlYzIuU3ViVlYodmVydGV4LndCLCB2ZXJ0ZXgud0EsIHZlcnRleC53KTtcclxuXHJcbiAgICAvLyBJdGVyYXRpb24gY291bnQgaXMgZXF1YXRlZCB0byB0aGUgbnVtYmVyIG9mIHN1cHBvcnQgcG9pbnQgY2FsbHMuXHJcbiAgICArK2l0ZXI7XHJcbiAgICArK2IyX2dqa1N0YXRzLml0ZXJzO1xyXG5cclxuICAgIC8vIENoZWNrIGZvciBkdXBsaWNhdGUgc3VwcG9ydCBwb2ludHMuIFRoaXMgaXMgdGhlIG1haW4gdGVybWluYXRpb24gY3JpdGVyaWEuXHJcbiAgICBsZXQgZHVwbGljYXRlID0gZmFsc2U7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHNhdmVDb3VudDsgKytpKSB7XHJcbiAgICAgIGlmICh2ZXJ0ZXguaW5kZXhBID09PSBzYXZlQVtpXSAmJiB2ZXJ0ZXguaW5kZXhCID09PSBzYXZlQltpXSkge1xyXG4gICAgICAgIGR1cGxpY2F0ZSA9IHRydWU7XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAvLyBJZiB3ZSBmb3VuZCBhIGR1cGxpY2F0ZSBzdXBwb3J0IHBvaW50IHdlIG11c3QgZXhpdCB0byBhdm9pZCBjeWNsaW5nLlxyXG4gICAgaWYgKGR1cGxpY2F0ZSkge1xyXG4gICAgICBicmVhaztcclxuICAgIH1cclxuXHJcbiAgICAvLyBOZXcgdmVydGV4IGlzIG9rIGFuZCBuZWVkZWQuXHJcbiAgICArK3NpbXBsZXgubV9jb3VudDtcclxuICB9XHJcblxyXG4gIGIyX2dqa1N0YXRzLm1heEl0ZXJzID0gYjJNYXhJbnQoYjJfZ2prU3RhdHMubWF4SXRlcnMsIGl0ZXIpO1xyXG5cclxuICAvLyBQcmVwYXJlIG91dHB1dC5cclxuICBzaW1wbGV4LkdldFdpdG5lc3NQb2ludHMob3V0cHV0LnBvaW50QSwgb3V0cHV0LnBvaW50Qik7XHJcbiAgb3V0cHV0LmRpc3RhbmNlID0gYjJWZWMyLkRpc3RhbmNlVlYob3V0cHV0LnBvaW50QSwgb3V0cHV0LnBvaW50Qik7XHJcbiAgb3V0cHV0Lml0ZXJhdGlvbnMgPSBpdGVyO1xyXG5cclxuICAvLyBDYWNoZSB0aGUgc2ltcGxleC5cclxuICBzaW1wbGV4LldyaXRlQ2FjaGUoY2FjaGUpO1xyXG5cclxuICAvLyBBcHBseSByYWRpaSBpZiByZXF1ZXN0ZWQuXHJcbiAgaWYgKGlucHV0LnVzZVJhZGlpKSB7XHJcbiAgICBjb25zdCByQTogbnVtYmVyID0gcHJveHlBLm1fcmFkaXVzO1xyXG4gICAgY29uc3QgckI6IG51bWJlciA9IHByb3h5Qi5tX3JhZGl1cztcclxuXHJcbiAgICBpZiAob3V0cHV0LmRpc3RhbmNlID4gckEgKyByQiAmJiBvdXRwdXQuZGlzdGFuY2UgPiBiMl9lcHNpbG9uKSB7XHJcbiAgICAgIC8vIFNoYXBlcyBhcmUgc3RpbGwgbm8gb3ZlcmxhcHBlZC5cclxuICAgICAgLy8gTW92ZSB0aGUgd2l0bmVzcyBwb2ludHMgdG8gdGhlIG91dGVyIHN1cmZhY2UuXHJcbiAgICAgIG91dHB1dC5kaXN0YW5jZSAtPSByQSArIHJCO1xyXG4gICAgICBjb25zdCBub3JtYWw6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihvdXRwdXQucG9pbnRCLCBvdXRwdXQucG9pbnRBLCBiMkRpc3RhbmNlX3Nfbm9ybWFsKTtcclxuICAgICAgbm9ybWFsLk5vcm1hbGl6ZSgpO1xyXG4gICAgICBvdXRwdXQucG9pbnRBLlNlbGZNdWxBZGQockEsIG5vcm1hbCk7XHJcbiAgICAgIG91dHB1dC5wb2ludEIuU2VsZk11bFN1YihyQiwgbm9ybWFsKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIFNoYXBlcyBhcmUgb3ZlcmxhcHBlZCB3aGVuIHJhZGlpIGFyZSBjb25zaWRlcmVkLlxyXG4gICAgICAvLyBNb3ZlIHRoZSB3aXRuZXNzIHBvaW50cyB0byB0aGUgbWlkZGxlLlxyXG4gICAgICBjb25zdCBwOiBiMlZlYzIgPSBiMlZlYzIuTWlkVlYob3V0cHV0LnBvaW50QSwgb3V0cHV0LnBvaW50QiwgYjJEaXN0YW5jZV9zX3ApO1xyXG4gICAgICBvdXRwdXQucG9pbnRBLkNvcHkocCk7XHJcbiAgICAgIG91dHB1dC5wb2ludEIuQ29weShwKTtcclxuICAgICAgb3V0cHV0LmRpc3RhbmNlID0gMDtcclxuICAgIH1cclxuICB9XHJcbn1cclxuXHJcbi8vLyBQZXJmb3JtIGEgbGluZWFyIHNoYXBlIGNhc3Qgb2Ygc2hhcGUgQiBtb3ZpbmcgYW5kIHNoYXBlIEEgZml4ZWQuIERldGVybWluZXMgdGhlIGhpdCBwb2ludCwgbm9ybWFsLCBhbmQgdHJhbnNsYXRpb24gZnJhY3Rpb24uXHJcblxyXG4vLyBHSkstcmF5Y2FzdFxyXG4vLyBBbGdvcml0aG0gYnkgR2lubyB2YW4gZGVuIEJlcmdlbi5cclxuLy8gXCJTbW9vdGggTWVzaCBDb250YWN0cyB3aXRoIEdKS1wiIGluIEdhbWUgUGh5c2ljcyBQZWFybHMuIDIwMTBcclxuLy8gYm9vbCBiMlNoYXBlQ2FzdChiMlNoYXBlQ2FzdE91dHB1dCogb3V0cHV0LCBjb25zdCBiMlNoYXBlQ2FzdElucHV0KiBpbnB1dCk7XHJcbmNvbnN0IGIyU2hhcGVDYXN0X3NfbiA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJTaGFwZUNhc3Rfc19zaW1wbGV4ID0gbmV3IGIyU2ltcGxleCgpO1xyXG5jb25zdCBiMlNoYXBlQ2FzdF9zX3dBID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBiMlNoYXBlQ2FzdF9zX3dCID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBiMlNoYXBlQ2FzdF9zX3YgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyU2hhcGVDYXN0X3NfcCA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJTaGFwZUNhc3Rfc19wb2ludEEgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyU2hhcGVDYXN0X3NfcG9pbnRCID0gbmV3IGIyVmVjMigpO1xyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyU2hhcGVDYXN0KG91dHB1dDogYjJTaGFwZUNhc3RPdXRwdXQsIGlucHV0OiBiMlNoYXBlQ2FzdElucHV0KTogYm9vbGVhbiB7XHJcbiAgb3V0cHV0Lml0ZXJhdGlvbnMgPSAwO1xyXG4gIG91dHB1dC5sYW1iZGEgPSAxLjA7XHJcbiAgb3V0cHV0Lm5vcm1hbC5TZXRaZXJvKCk7XHJcbiAgb3V0cHV0LnBvaW50LlNldFplcm8oKTtcclxuXHJcbiAgLy8gY29uc3QgYjJEaXN0YW5jZVByb3h5KiBwcm94eUEgPSAmaW5wdXQucHJveHlBO1xyXG4gIGNvbnN0IHByb3h5QSA9IGlucHV0LnByb3h5QTtcclxuICAvLyBjb25zdCBiMkRpc3RhbmNlUHJveHkqIHByb3h5QiA9ICZpbnB1dC5wcm94eUI7XHJcbiAgY29uc3QgcHJveHlCID0gaW5wdXQucHJveHlCO1xyXG5cclxuICAvLyBmbG9hdDMyIHJhZGl1c0EgPSBiMk1heChwcm94eUEubV9yYWRpdXMsIGIyX3BvbHlnb25SYWRpdXMpO1xyXG4gIGNvbnN0IHJhZGl1c0EgPSBiMk1heChwcm94eUEubV9yYWRpdXMsIGIyX3BvbHlnb25SYWRpdXMpO1xyXG4gIC8vIGZsb2F0MzIgcmFkaXVzQiA9IGIyTWF4KHByb3h5Qi5tX3JhZGl1cywgYjJfcG9seWdvblJhZGl1cyk7XHJcbiAgY29uc3QgcmFkaXVzQiA9IGIyTWF4KHByb3h5Qi5tX3JhZGl1cywgYjJfcG9seWdvblJhZGl1cyk7XHJcbiAgLy8gZmxvYXQzMiByYWRpdXMgPSByYWRpdXNBICsgcmFkaXVzQjtcclxuICBjb25zdCByYWRpdXMgPSByYWRpdXNBICsgcmFkaXVzQjtcclxuXHJcbiAgLy8gYjJUcmFuc2Zvcm0geGZBID0gaW5wdXQudHJhbnNmb3JtQTtcclxuICBjb25zdCB4ZkEgPSBpbnB1dC50cmFuc2Zvcm1BO1xyXG4gIC8vIGIyVHJhbnNmb3JtIHhmQiA9IGlucHV0LnRyYW5zZm9ybUI7XHJcbiAgY29uc3QgeGZCID0gaW5wdXQudHJhbnNmb3JtQjtcclxuXHJcbiAgLy8gYjJWZWMyIHIgPSBpbnB1dC50cmFuc2xhdGlvbkI7XHJcbiAgY29uc3QgciA9IGlucHV0LnRyYW5zbGF0aW9uQjtcclxuICAvLyBiMlZlYzIgbigwLjBmLCAwLjBmKTtcclxuICBjb25zdCBuID0gYjJTaGFwZUNhc3Rfc19uLlNldCgwLjAsIDAuMCk7XHJcbiAgLy8gZmxvYXQzMiBsYW1iZGEgPSAwLjBmO1xyXG4gIGxldCBsYW1iZGEgPSAwLjA7XHJcblxyXG4gIC8vIEluaXRpYWwgc2ltcGxleFxyXG4gIGNvbnN0IHNpbXBsZXggPSBiMlNoYXBlQ2FzdF9zX3NpbXBsZXg7XHJcbiAgc2ltcGxleC5tX2NvdW50ID0gMDtcclxuXHJcbiAgLy8gR2V0IHNpbXBsZXggdmVydGljZXMgYXMgYW4gYXJyYXkuXHJcbiAgLy8gYjJTaW1wbGV4VmVydGV4KiB2ZXJ0aWNlcyA9ICZzaW1wbGV4Lm1fdjE7XHJcbiAgY29uc3QgdmVydGljZXMgPSBzaW1wbGV4Lm1fdmVydGljZXM7XHJcblxyXG4gIC8vIEdldCBzdXBwb3J0IHBvaW50IGluIC1yIGRpcmVjdGlvblxyXG4gIC8vIGludDMyIGluZGV4QSA9IHByb3h5QS5HZXRTdXBwb3J0KGIyTXVsVCh4ZkEucSwgLXIpKTtcclxuICBsZXQgaW5kZXhBID0gcHJveHlBLkdldFN1cHBvcnQoYjJSb3QuTXVsVFJWKHhmQS5xLCBiMlZlYzIuTmVnVihyLCBiMlZlYzIuc190MSksIGIyVmVjMi5zX3QwKSk7XHJcbiAgLy8gYjJWZWMyIHdBID0gYjJNdWwoeGZBLCBwcm94eUEuR2V0VmVydGV4KGluZGV4QSkpO1xyXG4gIGxldCB3QSA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQSwgcHJveHlBLkdldFZlcnRleChpbmRleEEpLCBiMlNoYXBlQ2FzdF9zX3dBKTtcclxuICAvLyBpbnQzMiBpbmRleEIgPSBwcm94eUIuR2V0U3VwcG9ydChiMk11bFQoeGZCLnEsIHIpKTtcclxuICBsZXQgaW5kZXhCID0gcHJveHlCLkdldFN1cHBvcnQoYjJSb3QuTXVsVFJWKHhmQi5xLCByLCBiMlZlYzIuc190MCkpO1xyXG4gIC8vIGIyVmVjMiB3QiA9IGIyTXVsKHhmQiwgcHJveHlCLkdldFZlcnRleChpbmRleEIpKTtcclxuICBsZXQgd0IgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIHByb3h5Qi5HZXRWZXJ0ZXgoaW5kZXhCKSwgYjJTaGFwZUNhc3Rfc193Qik7XHJcbiAgLy8gYjJWZWMyIHYgPSB3QSAtIHdCO1xyXG4gIGNvbnN0IHYgPSBiMlZlYzIuU3ViVlYod0EsIHdCLCBiMlNoYXBlQ2FzdF9zX3YpO1xyXG5cclxuICAvLyBTaWdtYSBpcyB0aGUgdGFyZ2V0IGRpc3RhbmNlIGJldHdlZW4gcG9seWdvbnNcclxuICAvLyBmbG9hdDMyIHNpZ21hID0gYjJNYXgoYjJfcG9seWdvblJhZGl1cywgcmFkaXVzIC0gYjJfcG9seWdvblJhZGl1cyk7XHJcbiAgY29uc3Qgc2lnbWEgPSBiMk1heChiMl9wb2x5Z29uUmFkaXVzLCByYWRpdXMgLSBiMl9wb2x5Z29uUmFkaXVzKTtcclxuICAvLyBjb25zdCBmbG9hdDMyIHRvbGVyYW5jZSA9IDAuNWYgKiBiMl9saW5lYXJTbG9wO1xyXG4gIGNvbnN0IHRvbGVyYW5jZSA9IDAuNSAqIGIyX2xpbmVhclNsb3A7XHJcblxyXG4gIC8vIE1haW4gaXRlcmF0aW9uIGxvb3AuXHJcbiAgLy8gY29uc3QgaW50MzIga19tYXhJdGVycyA9IDIwO1xyXG4gIGNvbnN0IGtfbWF4SXRlcnMgPSAyMDtcclxuICAvLyBpbnQzMiBpdGVyID0gMDtcclxuICBsZXQgaXRlciA9IDA7XHJcbiAgLy8gd2hpbGUgKGl0ZXIgPCBrX21heEl0ZXJzICYmIGIyQWJzKHYuTGVuZ3RoKCkgLSBzaWdtYSkgPiB0b2xlcmFuY2UpXHJcbiAgd2hpbGUgKGl0ZXIgPCBrX21heEl0ZXJzICYmIGIyQWJzKHYuTGVuZ3RoKCkgLSBzaWdtYSkgPiB0b2xlcmFuY2UpIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoc2ltcGxleC5tX2NvdW50IDwgMyk7XHJcblxyXG4gICAgKytvdXRwdXQuaXRlcmF0aW9ucztcclxuXHJcbiAgICAvLyBTdXBwb3J0IGluIGRpcmVjdGlvbiAtdiAoQSAtIEIpXHJcbiAgICAvLyBpbmRleEEgPSBwcm94eUEuR2V0U3VwcG9ydChiMk11bFQoeGZBLnEsIC12KSk7XHJcbiAgICBpbmRleEEgPSBwcm94eUEuR2V0U3VwcG9ydChiMlJvdC5NdWxUUlYoeGZBLnEsIGIyVmVjMi5OZWdWKHYsIGIyVmVjMi5zX3QxKSwgYjJWZWMyLnNfdDApKTtcclxuICAgIC8vIHdBID0gYjJNdWwoeGZBLCBwcm94eUEuR2V0VmVydGV4KGluZGV4QSkpO1xyXG4gICAgd0EgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkEsIHByb3h5QS5HZXRWZXJ0ZXgoaW5kZXhBKSwgYjJTaGFwZUNhc3Rfc193QSk7XHJcbiAgICAvLyBpbmRleEIgPSBwcm94eUIuR2V0U3VwcG9ydChiMk11bFQoeGZCLnEsIHYpKTtcclxuICAgIGluZGV4QiA9IHByb3h5Qi5HZXRTdXBwb3J0KGIyUm90Lk11bFRSVih4ZkIucSwgdiwgYjJWZWMyLnNfdDApKTtcclxuICAgIC8vIHdCID0gYjJNdWwoeGZCLCBwcm94eUIuR2V0VmVydGV4KGluZGV4QikpO1xyXG4gICAgd0IgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIHByb3h5Qi5HZXRWZXJ0ZXgoaW5kZXhCKSwgYjJTaGFwZUNhc3Rfc193Qik7XHJcbiAgICAvLyBiMlZlYzIgcCA9IHdBIC0gd0I7XHJcbiAgICBjb25zdCBwID0gYjJWZWMyLlN1YlZWKHdBLCB3QiwgYjJTaGFwZUNhc3Rfc19wKTtcclxuXHJcbiAgICAvLyAtdiBpcyBhIG5vcm1hbCBhdCBwXHJcbiAgICB2Lk5vcm1hbGl6ZSgpO1xyXG5cclxuICAgIC8vIEludGVyc2VjdCByYXkgd2l0aCBwbGFuZVxyXG4gICAgY29uc3QgdnAgPSBiMlZlYzIuRG90VlYodiwgcCk7XHJcbiAgICBjb25zdCB2ciA9IGIyVmVjMi5Eb3RWVih2LCByKTtcclxuICAgIGlmICh2cCAtIHNpZ21hID4gbGFtYmRhICogdnIpIHtcclxuICAgICAgaWYgKHZyIDw9IDAuMCkge1xyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgICAgfVxyXG5cclxuICAgICAgbGFtYmRhID0gKHZwIC0gc2lnbWEpIC8gdnI7XHJcbiAgICAgIGlmIChsYW1iZGEgPiAxLjApIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIG4gPSAtdjtcclxuICAgICAgbi5Db3B5KHYpLlNlbGZOZWcoKTtcclxuICAgICAgc2ltcGxleC5tX2NvdW50ID0gMDtcclxuICAgIH1cclxuXHJcbiAgICAvLyBSZXZlcnNlIHNpbXBsZXggc2luY2UgaXQgd29ya3Mgd2l0aCBCIC0gQS5cclxuICAgIC8vIFNoaWZ0IGJ5IGxhbWJkYSAqIHIgYmVjYXVzZSB3ZSB3YW50IHRoZSBjbG9zZXN0IHBvaW50IHRvIHRoZSBjdXJyZW50IGNsaXAgcG9pbnQuXHJcbiAgICAvLyBOb3RlIHRoYXQgdGhlIHN1cHBvcnQgcG9pbnQgcCBpcyBub3Qgc2hpZnRlZCBiZWNhdXNlIHdlIHdhbnQgdGhlIHBsYW5lIGVxdWF0aW9uXHJcbiAgICAvLyB0byBiZSBmb3JtZWQgaW4gdW5zaGlmdGVkIHNwYWNlLlxyXG4gICAgLy8gYjJTaW1wbGV4VmVydGV4KiB2ZXJ0ZXggPSB2ZXJ0aWNlcyArIHNpbXBsZXgubV9jb3VudDtcclxuICAgIGNvbnN0IHZlcnRleDogYjJTaW1wbGV4VmVydGV4ID0gdmVydGljZXNbc2ltcGxleC5tX2NvdW50XTtcclxuICAgIHZlcnRleC5pbmRleEEgPSBpbmRleEI7XHJcbiAgICAvLyB2ZXJ0ZXgud0EgPSB3QiArIGxhbWJkYSAqIHI7XHJcbiAgICB2ZXJ0ZXgud0EuQ29weSh3QikuU2VsZk11bEFkZChsYW1iZGEsIHIpO1xyXG4gICAgdmVydGV4LmluZGV4QiA9IGluZGV4QTtcclxuICAgIC8vIHZlcnRleC53QiA9IHdBO1xyXG4gICAgdmVydGV4LndCLkNvcHkod0EpO1xyXG4gICAgLy8gdmVydGV4LncgPSB2ZXJ0ZXgud0IgLSB2ZXJ0ZXgud0E7XHJcbiAgICB2ZXJ0ZXgudy5Db3B5KHZlcnRleC53QikuU2VsZlN1Yih2ZXJ0ZXgud0EpO1xyXG4gICAgdmVydGV4LmEgPSAxLjA7XHJcbiAgICArK3NpbXBsZXgubV9jb3VudDtcclxuXHJcbiAgICBzd2l0Y2ggKHNpbXBsZXgubV9jb3VudCkge1xyXG4gICAgICBjYXNlIDE6XHJcbiAgICAgICAgYnJlYWs7XHJcblxyXG4gICAgICBjYXNlIDI6XHJcbiAgICAgICAgc2ltcGxleC5Tb2x2ZTIoKTtcclxuICAgICAgICBicmVhaztcclxuXHJcbiAgICAgIGNhc2UgMzpcclxuICAgICAgICBzaW1wbGV4LlNvbHZlMygpO1xyXG4gICAgICAgIGJyZWFrO1xyXG5cclxuICAgICAgZGVmYXVsdDpcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZhbHNlKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBJZiB3ZSBoYXZlIDMgcG9pbnRzLCB0aGVuIHRoZSBvcmlnaW4gaXMgaW4gdGhlIGNvcnJlc3BvbmRpbmcgdHJpYW5nbGUuXHJcbiAgICBpZiAoc2ltcGxleC5tX2NvdW50ID09PSAzKSB7XHJcbiAgICAgIC8vIE92ZXJsYXBcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIEdldCBzZWFyY2ggZGlyZWN0aW9uLlxyXG4gICAgLy8gdiA9IHNpbXBsZXguR2V0Q2xvc2VzdFBvaW50KCk7XHJcbiAgICBzaW1wbGV4LkdldENsb3Nlc3RQb2ludCh2KTtcclxuXHJcbiAgICAvLyBJdGVyYXRpb24gY291bnQgaXMgZXF1YXRlZCB0byB0aGUgbnVtYmVyIG9mIHN1cHBvcnQgcG9pbnQgY2FsbHMuXHJcbiAgICArK2l0ZXI7XHJcbiAgfVxyXG5cclxuICAvLyBQcmVwYXJlIG91dHB1dC5cclxuICBjb25zdCBwb2ludEEgPSBiMlNoYXBlQ2FzdF9zX3BvaW50QTtcclxuICBjb25zdCBwb2ludEIgPSBiMlNoYXBlQ2FzdF9zX3BvaW50QjtcclxuICBzaW1wbGV4LkdldFdpdG5lc3NQb2ludHMocG9pbnRBLCBwb2ludEIpO1xyXG5cclxuICBpZiAodi5MZW5ndGhTcXVhcmVkKCkgPiAwLjApIHtcclxuICAgIC8vIG4gPSAtdjtcclxuICAgIG4uQ29weSh2KS5TZWxmTmVnKCk7XHJcbiAgICBuLk5vcm1hbGl6ZSgpO1xyXG4gIH1cclxuXHJcbiAgLy8gb3V0cHV0LnBvaW50ID0gcG9pbnRBICsgcmFkaXVzQSAqIG47XHJcbiAgb3V0cHV0Lm5vcm1hbC5Db3B5KG4pO1xyXG4gIG91dHB1dC5sYW1iZGEgPSBsYW1iZGE7XHJcbiAgb3V0cHV0Lml0ZXJhdGlvbnMgPSBpdGVyO1xyXG4gIHJldHVybiB0cnVlO1xyXG59XHJcbiJdfQ==