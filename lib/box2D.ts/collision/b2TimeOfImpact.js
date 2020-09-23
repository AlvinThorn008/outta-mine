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
import { b2_linearSlop, b2_maxPolygonVertices, b2Assert } from '../common/b2Settings';
import { b2Abs, b2Max, b2MaxInt, b2Rot, b2Sweep, b2Transform, b2Vec2 } from '../common/b2Math';
import { b2Timer } from '../common/b2Timer';
import { b2Distance, b2DistanceInput, b2DistanceOutput, b2DistanceProxy, b2SimplexCache, } from './b2Distance';
class TOIStats {
    constructor() {
        this.time = 0;
        this.maxTime = 0;
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
        this.rootIters = 0;
        this.maxRootIters = 0;
        this.time = 0.0;
        this.maxTime = 0.0;
    }
    Reset() {
        this.time = 0.0;
        this.maxTime = 0.0;
        this.calls = 0;
        this.iters = 0;
        this.maxIters = 0;
        this.rootIters = 0;
        this.maxRootIters = 0;
    }
}
export const b2_toiStats = new TOIStats();
const b2TimeOfImpact_s_xfA = new b2Transform();
const b2TimeOfImpact_s_xfB = new b2Transform();
const b2TimeOfImpact_s_pointA = new b2Vec2();
const b2TimeOfImpact_s_pointB = new b2Vec2();
const b2TimeOfImpact_s_normal = new b2Vec2();
const b2TimeOfImpact_s_axisA = new b2Vec2();
const b2TimeOfImpact_s_axisB = new b2Vec2();
/// Input parameters for b2TimeOfImpact
export class b2TOIInput {
    constructor() {
        this.proxyA = new b2DistanceProxy();
        this.proxyB = new b2DistanceProxy();
        this.sweepA = new b2Sweep();
        this.sweepB = new b2Sweep();
        this.tMax = NaN; // defines sweep interval [0, tMax]
        this.tMax = 0.0;
    }
}
export class b2TOIOutput {
    constructor() {
        this.state = 0 /* e_unknown */;
        this.t = NaN;
        this.t = 0.0;
    }
}
export class b2SeparationFunction {
    constructor() {
        this.m_sweepA = new b2Sweep();
        this.m_sweepB = new b2Sweep();
        this.m_type = -1 /* e_unknown */;
        this.m_localPoint = new b2Vec2();
        this.m_axis = new b2Vec2();
    }
    Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1) {
        this.m_proxyA = proxyA;
        this.m_proxyB = proxyB;
        const count = cache.count;
        !!B2_DEBUG && b2Assert(0 < count && count < 3);
        this.m_sweepA.Copy(sweepA);
        this.m_sweepB.Copy(sweepB);
        const xfA = b2TimeOfImpact_s_xfA;
        const xfB = b2TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t1);
        this.m_sweepB.GetTransform(xfB, t1);
        if (count === 1) {
            this.m_type = 0 /* e_points */;
            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            const pointA = b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
            const pointB = b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
            b2Vec2.SubVV(pointB, pointA, this.m_axis);
            const s = this.m_axis.Normalize();
            if (B2_ENABLE_PARTICLE) {
                this.m_localPoint.SetZero();
            }
            return s;
        }
        else if (cache.indexA[0] === cache.indexA[1]) {
            // Two points on B and one on A.
            this.m_type = 2 /* e_faceB */;
            const localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
            const localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
            b2Vec2
                .CrossVOne(b2Vec2.SubVV(localPointB2, localPointB1, b2Vec2.s_t0), this.m_axis)
                .SelfNormalize();
            const normal = b2Rot.MulRV(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
            b2Vec2.MidVV(localPointB1, localPointB2, this.m_localPoint);
            const pointB = b2Transform.MulXV(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);
            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const pointA = b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
            let s = b2Vec2.DotVV(b2Vec2.SubVV(pointA, pointB, b2Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.SelfNeg();
                s = -s;
            }
            return s;
        }
        else {
            // Two points on A and one or two points on B.
            this.m_type = 1 /* e_faceA */;
            const localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
            const localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
            b2Vec2
                .CrossVOne(b2Vec2.SubVV(localPointA2, localPointA1, b2Vec2.s_t0), this.m_axis)
                .SelfNormalize();
            const normal = b2Rot.MulRV(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
            b2Vec2.MidVV(localPointA1, localPointA2, this.m_localPoint);
            const pointA = b2Transform.MulXV(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);
            const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            const pointB = b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
            let s = b2Vec2.DotVV(b2Vec2.SubVV(pointB, pointA, b2Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.SelfNeg();
                s = -s;
            }
            return s;
        }
    }
    FindMinSeparation(indexA, indexB, t) {
        const xfA = b2TimeOfImpact_s_xfA;
        const xfB = b2TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        if (this.m_type === 0 /* e_points */) {
            const axisA = b2Rot.MulTRV(xfA.q, this.m_axis, b2TimeOfImpact_s_axisA);
            const axisB = b2Rot.MulTRV(xfB.q, b2Vec2.NegV(this.m_axis, b2Vec2.s_t0), b2TimeOfImpact_s_axisB);
            indexA[0] = this.m_proxyA.GetSupport(axisA);
            indexB[0] = this.m_proxyB.GetSupport(axisB);
            const localPointA = this.m_proxyA.GetVertex(indexA[0]);
            const localPointB = this.m_proxyB.GetVertex(indexB[0]);
            const pointA = b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
            const pointB = b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
            const separation = b2Vec2.DotVV(b2Vec2.SubVV(pointB, pointA, b2Vec2.s_t0), this.m_axis);
            return separation;
        }
        else if (this.m_type === 1 /* e_faceA */) {
            const normal = b2Rot.MulRV(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
            const pointA = b2Transform.MulXV(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);
            const axisB = b2Rot.MulTRV(xfB.q, b2Vec2.NegV(normal, b2Vec2.s_t0), b2TimeOfImpact_s_axisB);
            indexA[0] = -1;
            indexB[0] = this.m_proxyB.GetSupport(axisB);
            const localPointB = this.m_proxyB.GetVertex(indexB[0]);
            const pointB = b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
            const separation = b2Vec2.DotVV(b2Vec2.SubVV(pointB, pointA, b2Vec2.s_t0), normal);
            return separation;
        }
        else if (this.m_type === 2 /* e_faceB */) {
            const normal = b2Rot.MulRV(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
            const pointB = b2Transform.MulXV(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);
            const axisA = b2Rot.MulTRV(xfA.q, b2Vec2.NegV(normal, b2Vec2.s_t0), b2TimeOfImpact_s_axisA);
            indexB[0] = -1;
            indexA[0] = this.m_proxyA.GetSupport(axisA);
            const localPointA = this.m_proxyA.GetVertex(indexA[0]);
            const pointA = b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
            const separation = b2Vec2.DotVV(b2Vec2.SubVV(pointA, pointB, b2Vec2.s_t0), normal);
            return separation;
        }
        !!B2_DEBUG && b2Assert(false);
        indexA[0] = -1;
        indexB[0] = -1;
        return 0;
    }
    Evaluate(indexA, indexB, t) {
        const xfA = b2TimeOfImpact_s_xfA;
        const xfB = b2TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        switch (this.m_type) {
            case 0 /* e_points */: {
                const localPointA = this.m_proxyA.GetVertex(indexA);
                const localPointB = this.m_proxyB.GetVertex(indexB);
                const pointA = b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
                const pointB = b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
                const separation = b2Vec2.DotVV(b2Vec2.SubVV(pointB, pointA, b2Vec2.s_t0), this.m_axis);
                return separation;
            }
            case 1 /* e_faceA */: {
                const normal = b2Rot.MulRV(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
                const pointA = b2Transform.MulXV(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);
                const localPointB = this.m_proxyB.GetVertex(indexB);
                const pointB = b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
                const separation = b2Vec2.DotVV(b2Vec2.SubVV(pointB, pointA, b2Vec2.s_t0), normal);
                return separation;
            }
            case 2 /* e_faceB */: {
                const normal = b2Rot.MulRV(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
                const pointB = b2Transform.MulXV(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);
                const localPointA = this.m_proxyA.GetVertex(indexA);
                const pointA = b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
                const separation = b2Vec2.DotVV(b2Vec2.SubVV(pointA, pointB, b2Vec2.s_t0), normal);
                return separation;
            }
            default:
                !!B2_DEBUG && b2Assert(false);
                return 0;
        }
    }
}
const b2TimeOfImpact_s_timer = new b2Timer();
const b2TimeOfImpact_s_cache = new b2SimplexCache();
const b2TimeOfImpact_s_distanceInput = new b2DistanceInput();
const b2TimeOfImpact_s_distanceOutput = new b2DistanceOutput();
const b2TimeOfImpact_s_fcn = new b2SeparationFunction();
const b2TimeOfImpact_s_indexA = [0];
const b2TimeOfImpact_s_indexB = [0];
const b2TimeOfImpact_s_sweepA = new b2Sweep();
const b2TimeOfImpact_s_sweepB = new b2Sweep();
export function b2TimeOfImpact(output, input) {
    const timer = b2TimeOfImpact_s_timer.Reset();
    ++b2_toiStats.calls;
    output.state = 0 /* e_unknown */;
    output.t = input.tMax;
    const proxyA = input.proxyA;
    const proxyB = input.proxyB;
    const maxVertices = b2MaxInt(b2_maxPolygonVertices, b2MaxInt(proxyA.m_count, proxyB.m_count));
    const sweepA = b2TimeOfImpact_s_sweepA.Copy(input.sweepA);
    const sweepB = b2TimeOfImpact_s_sweepB.Copy(input.sweepB);
    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.Normalize();
    sweepB.Normalize();
    const tMax = input.tMax;
    const totalRadius = proxyA.m_radius + proxyB.m_radius;
    const target = b2Max(b2_linearSlop, totalRadius - 3 * b2_linearSlop);
    const tolerance = 0.25 * b2_linearSlop;
    !!B2_DEBUG && b2Assert(target > tolerance);
    let t1 = 0;
    const k_maxIterations = 20; // TODO_ERIN b2Settings
    let iter = 0;
    // Prepare input for distance query.
    const cache = b2TimeOfImpact_s_cache;
    cache.count = 0;
    const distanceInput = b2TimeOfImpact_s_distanceInput;
    distanceInput.proxyA.Copy(input.proxyA);
    distanceInput.proxyB.Copy(input.proxyB);
    distanceInput.useRadii = false;
    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
        const xfA = b2TimeOfImpact_s_xfA;
        const xfB = b2TimeOfImpact_s_xfB;
        sweepA.GetTransform(xfA, t1);
        sweepB.GetTransform(xfB, t1);
        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA.Copy(xfA);
        distanceInput.transformB.Copy(xfB);
        const distanceOutput = b2TimeOfImpact_s_distanceOutput;
        b2Distance(distanceOutput, cache, distanceInput);
        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0) {
            // Failure!
            output.state = 2 /* e_overlapped */;
            output.t = 0;
            break;
        }
        if (distanceOutput.distance < target + tolerance) {
            // Victory!
            output.state = 3 /* e_touching */;
            output.t = t1;
            break;
        }
        // Initialize the separating axis.
        const fcn = b2TimeOfImpact_s_fcn;
        fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);
        /*
            #if 0
                // Dump the curve seen by the root finder {
                  const int32 N = 100;
                  float32 dx = 1.0f / N;
                  float32 xs[N+1];
                  float32 fs[N+1];
    
                  float32 x = 0.0f;
    
                  for (int32 i = 0; i <= N; ++i) {
                    sweepA.GetTransform(&xfA, x);
                    sweepB.GetTransform(&xfB, x);
                    float32 f = fcn.Evaluate(xfA, xfB) - target;
    
                    printf("%g %g\n", x, f);
    
                    xs[i] = x;
                    fs[i] = f;
    
                    x += dx;
                  }
                }
            #endif
            */
        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        let done = false;
        let t2 = tMax;
        let pushBackIter = 0;
        for (;;) {
            // Find the deepest point at t2. Store the witness point indices.
            const indexA = b2TimeOfImpact_s_indexA;
            const indexB = b2TimeOfImpact_s_indexB;
            let s2 = fcn.FindMinSeparation(indexA, indexB, t2);
            // Is the final configuration separated?
            if (s2 > target + tolerance) {
                // Victory!
                output.state = 4 /* e_separated */;
                output.t = tMax;
                done = true;
                break;
            }
            // Has the separation reached tolerance?
            if (s2 > target - tolerance) {
                // Advance the sweeps
                t1 = t2;
                break;
            }
            // Compute the initial separation of the witness points.
            let s1 = fcn.Evaluate(indexA[0], indexB[0], t1);
            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < target - tolerance) {
                output.state = 1 /* e_failed */;
                output.t = t1;
                done = true;
                break;
            }
            // Check for touching
            if (s1 <= target + tolerance) {
                // Victory! t1 should hold the TOI (could be 0.0).
                output.state = 3 /* e_touching */;
                output.t = t1;
                done = true;
                break;
            }
            // Compute 1D root of: f(x) - target = 0
            let rootIterCount = 0;
            let a1 = t1;
            let a2 = t2;
            for (;;) {
                // Use a mix of the secant rule and bisection.
                let t = 0;
                if (rootIterCount & 1) {
                    // Secant rule to improve convergence.
                    t = a1 + ((target - s1) * (a2 - a1)) / (s2 - s1);
                }
                else {
                    // Bisection to guarantee progress.
                    t = 0.5 * (a1 + a2);
                }
                ++rootIterCount;
                ++b2_toiStats.rootIters;
                const s = fcn.Evaluate(indexA[0], indexB[0], t);
                if (b2Abs(s - target) < tolerance) {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }
                // Ensure we continue to bracket the root.
                if (s > target) {
                    a1 = t;
                    s1 = s;
                }
                else {
                    a2 = t;
                    s2 = s;
                }
                if (rootIterCount === 50) {
                    break;
                }
            }
            b2_toiStats.maxRootIters = b2MaxInt(b2_toiStats.maxRootIters, rootIterCount);
            ++pushBackIter;
            if (pushBackIter === maxVertices) {
                break;
            }
        }
        ++iter;
        ++b2_toiStats.iters;
        if (done) {
            break;
        }
        if (iter === k_maxIterations) {
            // Root finder got stuck. Semi-victory.
            output.state = 1 /* e_failed */;
            output.t = t1;
            break;
        }
    }
    b2_toiStats.maxIters = b2MaxInt(b2_toiStats.maxIters, iter);
    const time = timer.GetMilliseconds();
    b2_toiStats.maxTime = b2Max(b2_toiStats.maxTime, time);
    b2_toiStats.time += time;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUaW1lT2ZJbXBhY3QuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9zcmMvY29sbGlzaW9uL2IyVGltZU9mSW1wYWN0LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLGFBQWEsRUFBRSxxQkFBcUIsRUFBRSxRQUFRLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUN0RixPQUFPLEVBQUUsS0FBSyxFQUFFLEtBQUssRUFBRSxRQUFRLEVBQUUsS0FBSyxFQUFFLE9BQU8sRUFBRSxXQUFXLEVBQUUsTUFBTSxFQUFFLE1BQU0sa0JBQWtCLENBQUM7QUFDL0YsT0FBTyxFQUFFLE9BQU8sRUFBRSxNQUFNLG1CQUFtQixDQUFDO0FBQzVDLE9BQU8sRUFDTCxVQUFVLEVBQ1YsZUFBZSxFQUNmLGdCQUFnQixFQUNoQixlQUFlLEVBQ2YsY0FBYyxHQUNmLE1BQU0sY0FBYyxDQUFDO0FBRXRCLE1BQU0sUUFBUTtJQVNaO1FBUkEsU0FBSSxHQUFHLENBQUMsQ0FBQztRQUNULFlBQU8sR0FBRyxDQUFDLENBQUM7UUFDWixVQUFLLEdBQUcsQ0FBQyxDQUFDO1FBQ1YsVUFBSyxHQUFHLENBQUMsQ0FBQztRQUNWLGFBQVEsR0FBRyxDQUFDLENBQUM7UUFDYixjQUFTLEdBQUcsQ0FBQyxDQUFDO1FBQ2QsaUJBQVksR0FBRyxDQUFDLENBQUM7UUFHZixJQUFJLENBQUMsSUFBSSxHQUFHLEdBQUcsQ0FBQztRQUNoQixJQUFJLENBQUMsT0FBTyxHQUFHLEdBQUcsQ0FBQztJQUNyQixDQUFDO0lBRUQsS0FBSztRQUNILElBQUksQ0FBQyxJQUFJLEdBQUcsR0FBRyxDQUFDO1FBQ2hCLElBQUksQ0FBQyxPQUFPLEdBQUcsR0FBRyxDQUFDO1FBQ25CLElBQUksQ0FBQyxLQUFLLEdBQUcsQ0FBQyxDQUFDO1FBQ2YsSUFBSSxDQUFDLEtBQUssR0FBRyxDQUFDLENBQUM7UUFDZixJQUFJLENBQUMs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QUFFLEdBQUcsQ0FBQyxDQUFDO29CQUNQLEVBQUUsR0FBRyxDQUFDLENBQUM7aUJBQ1I7cUJBQU07b0JBQ0wsRUFBRSxHQUFHLENBQUMsQ0FBQztvQkFDUCxFQUFFLEdBQUcsQ0FBQyxDQUFDO2lCQUNSO2dCQUVELElBQUksYUFBYSxLQUFLLEVBQUUsRUFBRTtvQkFDeEIsTUFBTTtpQkFDUDthQUNGO1lBRUQsV0FBVyxDQUFDLFlBQVksR0FBRyxRQUFRLENBQUMsV0FBVyxDQUFDLFlBQVksRUFBRSxhQUFhLENBQUMsQ0FBQztZQUU3RSxFQUFFLFlBQVksQ0FBQztZQUVmLElBQUksWUFBWSxLQUFLLFdBQVcsRUFBRTtnQkFDaEMsTUFBTTthQUNQO1NBQ0Y7UUFFRCxFQUFFLElBQUksQ0FBQztRQUNQLEVBQUUsV0FBVyxDQUFDLEtBQUssQ0FBQztRQUVwQixJQUFJLElBQUksRUFBRTtZQUNSLE1BQU07U0FDUDtRQUVELElBQUksSUFBSSxLQUFLLGVBQWUsRUFBRTtZQUM1Qix1Q0FBdUM7WUFDdkMsTUFBTSxDQUFDLEtBQUssbUJBQTRCLENBQUM7WUFDekMsTUFBTSxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7WUFDZCxNQUFNO1NBQ1A7S0FDRjtJQUVELFdBQVcsQ0FBQyxRQUFRLEdBQUcsUUFBUSxDQUFDLFdBQVcsQ0FBQyxRQUFRLEVBQUUsSUFBSSxDQUFDLENBQUM7SUFDNUQsTUFBTSxJQUFJLEdBQUcsS0FBSyxDQUFDLGVBQWUsRUFBRSxDQUFDO0lBQ3JDLFdBQVcsQ0FBQyxPQUFPLEdBQUcsS0FBSyxDQUFDLFdBQVcsQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLENBQUM7SUFDdkQsV0FBVyxDQUFDLElBQUksSUFBSSxJQUFJLENBQUM7QUFDM0IsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJfbGluZWFyU2xvcCwgYjJfbWF4UG9seWdvblZlcnRpY2VzLCBiMkFzc2VydCB9IGZyb20gJy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJBYnMsIGIyTWF4LCBiMk1heEludCwgYjJSb3QsIGIyU3dlZXAsIGIyVHJhbnNmb3JtLCBiMlZlYzIgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJUaW1lciB9IGZyb20gJy4uL2NvbW1vbi9iMlRpbWVyJztcclxuaW1wb3J0IHtcclxuICBiMkRpc3RhbmNlLFxyXG4gIGIyRGlzdGFuY2VJbnB1dCxcclxuICBiMkRpc3RhbmNlT3V0cHV0LFxyXG4gIGIyRGlzdGFuY2VQcm94eSxcclxuICBiMlNpbXBsZXhDYWNoZSxcclxufSBmcm9tICcuL2IyRGlzdGFuY2UnO1xyXG5cclxuY2xhc3MgVE9JU3RhdHMge1xyXG4gIHRpbWUgPSAwO1xyXG4gIG1heFRpbWUgPSAwO1xyXG4gIGNhbGxzID0gMDtcclxuICBpdGVycyA9IDA7XHJcbiAgbWF4SXRlcnMgPSAwO1xyXG4gIHJvb3RJdGVycyA9IDA7XHJcbiAgbWF4Um9vdEl0ZXJzID0gMDtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLnRpbWUgPSAwLjA7XHJcbiAgICB0aGlzLm1heFRpbWUgPSAwLjA7XHJcbiAgfVxyXG5cclxuICBSZXNldCgpIHtcclxuICAgIHRoaXMudGltZSA9IDAuMDtcclxuICAgIHRoaXMubWF4VGltZSA9IDAuMDtcclxuICAgIHRoaXMuY2FsbHMgPSAwO1xyXG4gICAgdGhpcy5pdGVycyA9IDA7XHJcbiAgICB0aGlzLm1heEl0ZXJzID0gMDtcclxuICAgIHRoaXMucm9vdEl0ZXJzID0gMDtcclxuICAgIHRoaXMubWF4Um9vdEl0ZXJzID0gMDtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjb25zdCBiMl90b2lTdGF0cyA9IG5ldyBUT0lTdGF0cygpO1xyXG5cclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc194ZkEgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc194ZkIgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEEgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyVGltZU9mSW1wYWN0X3NfcG9pbnRCID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBiMlRpbWVPZkltcGFjdF9zX25vcm1hbCA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc19heGlzQSA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc19heGlzQiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbi8vLyBJbnB1dCBwYXJhbWV0ZXJzIGZvciBiMlRpbWVPZkltcGFjdFxyXG5leHBvcnQgY2xhc3MgYjJUT0lJbnB1dCB7XHJcbiAgcmVhZG9ubHkgcHJveHlBID0gbmV3IGIyRGlzdGFuY2VQcm94eSgpO1xyXG4gIHJlYWRvbmx5IHByb3h5QiA9IG5ldyBiMkRpc3RhbmNlUHJveHkoKTtcclxuICByZWFkb25seSBzd2VlcEEgPSBuZXcgYjJTd2VlcCgpO1xyXG4gIHJlYWRvbmx5IHN3ZWVwQiA9IG5ldyBiMlN3ZWVwKCk7XHJcbiAgdE1heCA9IE5hTjsgLy8gZGVmaW5lcyBzd2VlcCBpbnRlcnZhbCBbMCwgdE1heF1cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMudE1heCA9IDAuMDtcclxuICB9XHJcbn1cclxuXHJcbi8vLyBPdXRwdXQgcGFyYW1ldGVycyBmb3IgYjJUaW1lT2ZJbXBhY3QuXHJcbmV4cG9ydCBjb25zdCBlbnVtIGIyVE9JT3V0cHV0U3RhdGUge1xyXG4gIGVfdW5rbm93biA9IDAsXHJcbiAgZV9mYWlsZWQgPSAxLFxyXG4gIGVfb3ZlcmxhcHBlZCA9IDIsXHJcbiAgZV90b3VjaGluZyA9IDMsXHJcbiAgZV9zZXBhcmF0ZWQgPSA0LFxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJUT0lPdXRwdXQge1xyXG4gIHN0YXRlID0gYjJUT0lPdXRwdXRTdGF0ZS5lX3Vua25vd247XHJcbiAgdCA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLnQgPSAwLjA7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY29uc3QgZW51bSBiMlNlcGFyYXRpb25GdW5jdGlvblR5cGUge1xyXG4gIGVfdW5rbm93biA9IC0xLFxyXG4gIGVfcG9pbnRzID0gMCxcclxuICBlX2ZhY2VBID0gMSxcclxuICBlX2ZhY2VCID0gMixcclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyU2VwYXJhdGlvbkZ1bmN0aW9uIHtcclxuICBtX3Byb3h5QSE6IGIyRGlzdGFuY2VQcm94eTtcclxuICBtX3Byb3h5QiE6IGIyRGlzdGFuY2VQcm94eTtcclxuICByZWFkb25seSBtX3N3ZWVwQTogYjJTd2VlcCA9IG5ldyBiMlN3ZWVwKCk7XHJcbiAgcmVhZG9ubHkgbV9zd2VlcEI6IGIyU3dlZXAgPSBuZXcgYjJTd2VlcCgpO1xyXG4gIG1fdHlwZTogYjJTZXBhcmF0aW9uRnVuY3Rpb25UeXBlID0gYjJTZXBhcmF0aW9uRnVuY3Rpb25UeXBlLmVfdW5rbm93bjtcclxuICByZWFkb25seSBtX2xvY2FsUG9pbnQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2F4aXM6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgSW5pdGlhbGl6ZShcclxuICAgIGNhY2hlOiBiMlNpbXBsZXhDYWNoZSxcclxuICAgIHByb3h5QTogYjJEaXN0YW5jZVByb3h5LFxyXG4gICAgc3dlZXBBOiBiMlN3ZWVwLFxyXG4gICAgcHJveHlCOiBiMkRpc3RhbmNlUHJveHksXHJcbiAgICBzd2VlcEI6IGIyU3dlZXAsXHJcbiAgICB0MTogbnVtYmVyLFxyXG4gICk6IG51bWJlciB7XHJcbiAgICB0aGlzLm1fcHJveHlBID0gcHJveHlBO1xyXG4gICAgdGhpcy5tX3Byb3h5QiA9IHByb3h5QjtcclxuICAgIGNvbnN0IGNvdW50OiBudW1iZXIgPSBjYWNoZS5jb3VudDtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoMCA8IGNvdW50ICYmIGNvdW50IDwgMyk7XHJcblxyXG4gICAgdGhpcy5tX3N3ZWVwQS5Db3B5KHN3ZWVwQSk7XHJcbiAgICB0aGlzLm1fc3dlZXBCLkNvcHkoc3dlZXBCKTtcclxuXHJcbiAgICBjb25zdCB4ZkE6IGIyVHJhbnNmb3JtID0gYjJUaW1lT2ZJbXBhY3Rfc194ZkE7XHJcbiAgICBjb25zdCB4ZkI6IGIyVHJhbnNmb3JtID0gYjJUaW1lT2ZJbXBhY3Rfc194ZkI7XHJcbiAgICB0aGlzLm1fc3dlZXBBLkdldFRyYW5zZm9ybSh4ZkEsIHQxKTtcclxuICAgIHRoaXMubV9zd2VlcEIuR2V0VHJhbnNmb3JtKHhmQiwgdDEpO1xyXG5cclxuICAgIGlmIChjb3VudCA9PT0gMSkge1xyXG4gICAgICB0aGlzLm1fdHlwZSA9IGIyU2VwYXJhdGlvbkZ1bmN0aW9uVHlwZS5lX3BvaW50cztcclxuICAgICAgY29uc3QgbG9jYWxQb2ludEE6IGIyVmVjMiA9IHRoaXMubV9wcm94eUEuR2V0VmVydGV4KGNhY2hlLmluZGV4QVswXSk7XHJcbiAgICAgIGNvbnN0IGxvY2FsUG9pbnRCOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlCLkdldFZlcnRleChjYWNoZS5pbmRleEJbMF0pO1xyXG4gICAgICBjb25zdCBwb2ludEE6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQSwgbG9jYWxQb2ludEEsIGIyVGltZU9mSW1wYWN0X3NfcG9pbnRBKTtcclxuICAgICAgY29uc3QgcG9pbnRCOiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIGxvY2FsUG9pbnRCLCBiMlRpbWVPZkltcGFjdF9zX3BvaW50Qik7XHJcbiAgICAgIGIyVmVjMi5TdWJWVihwb2ludEIsIHBvaW50QSwgdGhpcy5tX2F4aXMpO1xyXG4gICAgICBjb25zdCBzOiBudW1iZXIgPSB0aGlzLm1fYXhpcy5Ob3JtYWxpemUoKTtcclxuICAgICAgaWYgKEIyX0VOQUJMRV9QQVJUSUNMRSkge1xyXG4gICAgICAgIHRoaXMubV9sb2NhbFBvaW50LlNldFplcm8oKTtcclxuICAgICAgfVxyXG4gICAgICByZXR1cm4gcztcclxuICAgIH0gZWxzZSBpZiAoY2FjaGUuaW5kZXhBWzBdID09PSBjYWNoZS5pbmRleEFbMV0pIHtcclxuICAgICAgLy8gVHdvIHBvaW50cyBvbiBCIGFuZCBvbmUgb24gQS5cclxuICAgICAgdGhpcy5tX3R5cGUgPSBiMlNlcGFyYXRpb25GdW5jdGlvblR5cGUuZV9mYWNlQjtcclxuICAgICAgY29uc3QgbG9jYWxQb2ludEIxOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlCLkdldFZlcnRleChjYWNoZS5pbmRleEJbMF0pO1xyXG4gICAgICBjb25zdCBsb2NhbFBvaW50QjI6IGIyVmVjMiA9IHRoaXMubV9wcm94eUIuR2V0VmVydGV4KGNhY2hlLmluZGV4QlsxXSk7XHJcblxyXG4gICAgICBiMlZlYzJcclxuICAgICAgICAuQ3Jvc3NWT25lKGIyVmVjMi5TdWJWVihsb2NhbFBvaW50QjIsIGxvY2FsUG9pbnRCMSwgYjJWZWMyLnNfdDApLCB0aGlzLm1fYXhpcylcclxuICAgICAgICAuU2VsZk5vcm1hbGl6ZSgpO1xyXG4gICAgICBjb25zdCBub3JtYWw6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHhmQi5xLCB0aGlzLm1fYXhpcywgYjJUaW1lT2ZJbXBhY3Rfc19ub3JtYWwpO1xyXG5cclxuICAgICAgYjJWZWMyLk1pZFZWKGxvY2FsUG9pbnRCMSwgbG9jYWxQb2ludEIyLCB0aGlzLm1fbG9jYWxQb2ludCk7XHJcbiAgICAgIGNvbnN0IHBvaW50QjogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZCLCB0aGlzLm1fbG9jYWxQb2ludCwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEIpO1xyXG5cclxuICAgICAgY29uc3QgbG9jYWxQb2ludEE6IGIyVmVjMiA9IHRoaXMubV9wcm94eUEuR2V0VmVydGV4KGNhY2hlLmluZGV4QVswXSk7XHJcbiAgICAgIGNvbnN0IHBvaW50QTogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZBLCBsb2NhbFBvaW50QSwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEEpO1xyXG5cclxuICAgICAgbGV0IHM6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYocG9pbnRBLCBwb2ludEIsIGIyVmVjMi5zX3QwKSwgbm9ybWFsKTtcclxuICAgICAgaWYgKHMgPCAwKSB7XHJcbiAgICAgICAgdGhpcy5tX2F4aXMuU2VsZk5lZygpO1xyXG4gICAgICAgIHMgPSAtcztcclxuICAgICAgfVxyXG4gICAgICByZXR1cm4gcztcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIFR3byBwb2ludHMgb24gQSBhbmQgb25lIG9yIHR3byBwb2ludHMgb24gQi5cclxuICAgICAgdGhpcy5tX3R5cGUgPSBiMlNlcGFyYXRpb25GdW5jdGlvblR5cGUuZV9mYWNlQTtcclxuICAgICAgY29uc3QgbG9jYWxQb2ludEExOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlBLkdldFZlcnRleChjYWNoZS5pbmRleEFbMF0pO1xyXG4gICAgICBjb25zdCBsb2NhbFBvaW50QTI6IGIyVmVjMiA9IHRoaXMubV9wcm94eUEuR2V0VmVydGV4KGNhY2hlLmluZGV4QVsxXSk7XHJcblxyXG4gICAgICBiMlZlYzJcclxuICAgICAgICAuQ3Jvc3NWT25lKGIyVmVjMi5TdWJWVihsb2NhbFBvaW50QTIsIGxvY2FsUG9pbnRBMSwgYjJWZWMyLnNfdDApLCB0aGlzLm1fYXhpcylcclxuICAgICAgICAuU2VsZk5vcm1hbGl6ZSgpO1xyXG4gICAgICBjb25zdCBub3JtYWw6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHhmQS5xLCB0aGlzLm1fYXhpcywgYjJUaW1lT2ZJbXBhY3Rfc19ub3JtYWwpO1xyXG5cclxuICAgICAgYjJWZWMyLk1pZFZWKGxvY2FsUG9pbnRBMSwgbG9jYWxQb2ludEEyLCB0aGlzLm1fbG9jYWxQb2ludCk7XHJcbiAgICAgIGNvbnN0IHBvaW50QTogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZBLCB0aGlzLm1fbG9jYWxQb2ludCwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEEpO1xyXG5cclxuICAgICAgY29uc3QgbG9jYWxQb2ludEI6IGIyVmVjMiA9IHRoaXMubV9wcm94eUIuR2V0VmVydGV4KGNhY2hlLmluZGV4QlswXSk7XHJcbiAgICAgIGNvbnN0IHBvaW50QjogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZCLCBsb2NhbFBvaW50QiwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEIpO1xyXG5cclxuICAgICAgbGV0IHM6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYocG9pbnRCLCBwb2ludEEsIGIyVmVjMi5zX3QwKSwgbm9ybWFsKTtcclxuICAgICAgaWYgKHMgPCAwKSB7XHJcbiAgICAgICAgdGhpcy5tX2F4aXMuU2VsZk5lZygpO1xyXG4gICAgICAgIHMgPSAtcztcclxuICAgICAgfVxyXG4gICAgICByZXR1cm4gcztcclxuICAgIH1cclxuICB9XHJcblxyXG4gIEZpbmRNaW5TZXBhcmF0aW9uKGluZGV4QTogW251bWJlcl0sIGluZGV4QjogW251bWJlcl0sIHQ6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICBjb25zdCB4ZkE6IGIyVHJhbnNmb3JtID0gYjJUaW1lT2ZJbXBhY3Rfc194ZkE7XHJcbiAgICBjb25zdCB4ZkI6IGIyVHJhbnNmb3JtID0gYjJUaW1lT2ZJbXBhY3Rfc194ZkI7XHJcbiAgICB0aGlzLm1fc3dlZXBBLkdldFRyYW5zZm9ybSh4ZkEsIHQpO1xyXG4gICAgdGhpcy5tX3N3ZWVwQi5HZXRUcmFuc2Zvcm0oeGZCLCB0KTtcclxuXHJcbiAgICBpZiAodGhpcy5tX3R5cGUgPT09IGIyU2VwYXJhdGlvbkZ1bmN0aW9uVHlwZS5lX3BvaW50cykge1xyXG4gICAgICBjb25zdCBheGlzQTogYjJWZWMyID0gYjJSb3QuTXVsVFJWKHhmQS5xLCB0aGlzLm1fYXhpcywgYjJUaW1lT2ZJbXBhY3Rfc19heGlzQSk7XHJcbiAgICAgIGNvbnN0IGF4aXNCOiBiMlZlYzIgPSBiMlJvdC5NdWxUUlYoXHJcbiAgICAgICAgeGZCLnEsXHJcbiAgICAgICAgYjJWZWMyLk5lZ1YodGhpcy5tX2F4aXMsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICBiMlRpbWVPZkltcGFjdF9zX2F4aXNCLFxyXG4gICAgICApO1xyXG5cclxuICAgICAgaW5kZXhBWzBdID0gdGhpcy5tX3Byb3h5QS5HZXRTdXBwb3J0KGF4aXNBKTtcclxuICAgICAgaW5kZXhCWzBdID0gdGhpcy5tX3Byb3h5Qi5HZXRTdXBwb3J0KGF4aXNCKTtcclxuXHJcbiAgICAgIGNvbnN0IGxvY2FsUG9pbnRBOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlBLkdldFZlcnRleChpbmRleEFbMF0pO1xyXG4gICAgICBjb25zdCBsb2NhbFBvaW50QjogYjJWZWMyID0gdGhpcy5tX3Byb3h5Qi5HZXRWZXJ0ZXgoaW5kZXhCWzBdKTtcclxuXHJcbiAgICAgIGNvbnN0IHBvaW50QTogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZBLCBsb2NhbFBvaW50QSwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEEpO1xyXG4gICAgICBjb25zdCBwb2ludEI6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQiwgbG9jYWxQb2ludEIsIGIyVGltZU9mSW1wYWN0X3NfcG9pbnRCKTtcclxuXHJcbiAgICAgIGNvbnN0IHNlcGFyYXRpb246IG51bWJlciA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgICBiMlZlYzIuU3ViVlYocG9pbnRCLCBwb2ludEEsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICB0aGlzLm1fYXhpcyxcclxuICAgICAgKTtcclxuICAgICAgcmV0dXJuIHNlcGFyYXRpb247XHJcbiAgICB9IGVsc2UgaWYgKHRoaXMubV90eXBlID09PSBiMlNlcGFyYXRpb25GdW5jdGlvblR5cGUuZV9mYWNlQSkge1xyXG4gICAgICBjb25zdCBub3JtYWw6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHhmQS5xLCB0aGlzLm1fYXhpcywgYjJUaW1lT2ZJbXBhY3Rfc19ub3JtYWwpO1xyXG4gICAgICBjb25zdCBwb2ludEE6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQSwgdGhpcy5tX2xvY2FsUG9pbnQsIGIyVGltZU9mSW1wYWN0X3NfcG9pbnRBKTtcclxuXHJcbiAgICAgIGNvbnN0IGF4aXNCOiBiMlZlYzIgPSBiMlJvdC5NdWxUUlYoXHJcbiAgICAgICAgeGZCLnEsXHJcbiAgICAgICAgYjJWZWMyLk5lZ1Yobm9ybWFsLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgYjJUaW1lT2ZJbXBhY3Rfc19heGlzQixcclxuICAgICAgKTtcclxuXHJcbiAgICAgIGluZGV4QVswXSA9IC0xO1xyXG4gICAgICBpbmRleEJbMF0gPSB0aGlzLm1fcHJveHlCLkdldFN1cHBvcnQoYXhpc0IpO1xyXG5cclxuICAgICAgY29uc3QgbG9jYWxQb2ludEI6IGIyVmVjMiA9IHRoaXMubV9wcm94eUIuR2V0VmVydGV4KGluZGV4QlswXSk7XHJcbiAgICAgIGNvbnN0IHBvaW50QjogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZCLCBsb2NhbFBvaW50QiwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEIpO1xyXG5cclxuICAgICAgY29uc3Qgc2VwYXJhdGlvbjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGIyVmVjMi5TdWJWVihwb2ludEIsIHBvaW50QSwgYjJWZWMyLnNfdDApLCBub3JtYWwpO1xyXG4gICAgICByZXR1cm4gc2VwYXJhdGlvbjtcclxuICAgIH0gZWxzZSBpZiAodGhpcy5tX3R5cGUgPT09IGIyU2VwYXJhdGlvbkZ1bmN0aW9uVHlwZS5lX2ZhY2VCKSB7XHJcbiAgICAgIGNvbnN0IG5vcm1hbDogYjJWZWMyID0gYjJSb3QuTXVsUlYoeGZCLnEsIHRoaXMubV9heGlzLCBiMlRpbWVPZkltcGFjdF9zX25vcm1hbCk7XHJcbiAgICAgIGNvbnN0IHBvaW50QjogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZCLCB0aGlzLm1fbG9jYWxQb2ludCwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEIpO1xyXG5cclxuICAgICAgY29uc3QgYXhpc0E6IGIyVmVjMiA9IGIyUm90Lk11bFRSVihcclxuICAgICAgICB4ZkEucSxcclxuICAgICAgICBiMlZlYzIuTmVnVihub3JtYWwsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICBiMlRpbWVPZkltcGFjdF9zX2F4aXNBLFxyXG4gICAgICApO1xyXG5cclxuICAgICAgaW5kZXhCWzBdID0gLTE7XHJcbiAgICAgIGluZGV4QVswXSA9IHRoaXMubV9wcm94eUEuR2V0U3VwcG9ydChheGlzQSk7XHJcblxyXG4gICAgICBjb25zdCBsb2NhbFBvaW50QTogYjJWZWMyID0gdGhpcy5tX3Byb3h5QS5HZXRWZXJ0ZXgoaW5kZXhBWzBdKTtcclxuICAgICAgY29uc3QgcG9pbnRBOiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkEsIGxvY2FsUG9pbnRBLCBiMlRpbWVPZkltcGFjdF9zX3BvaW50QSk7XHJcblxyXG4gICAgICBjb25zdCBzZXBhcmF0aW9uOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKHBvaW50QSwgcG9pbnRCLCBiMlZlYzIuc190MCksIG5vcm1hbCk7XHJcbiAgICAgIHJldHVybiBzZXBhcmF0aW9uO1xyXG4gICAgfVxyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpO1xyXG4gICAgaW5kZXhBWzBdID0gLTE7XHJcbiAgICBpbmRleEJbMF0gPSAtMTtcclxuICAgIHJldHVybiAwO1xyXG4gIH1cclxuXHJcbiAgRXZhbHVhdGUoaW5kZXhBOiBudW1iZXIsIGluZGV4QjogbnVtYmVyLCB0OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgY29uc3QgeGZBOiBiMlRyYW5zZm9ybSA9IGIyVGltZU9mSW1wYWN0X3NfeGZBO1xyXG4gICAgY29uc3QgeGZCOiBiMlRyYW5zZm9ybSA9IGIyVGltZU9mSW1wYWN0X3NfeGZCO1xyXG4gICAgdGhpcy5tX3N3ZWVwQS5HZXRUcmFuc2Zvcm0oeGZBLCB0KTtcclxuICAgIHRoaXMubV9zd2VlcEIuR2V0VHJhbnNmb3JtKHhmQiwgdCk7XHJcblxyXG4gICAgc3dpdGNoICh0aGlzLm1fdHlwZSkge1xyXG4gICAgICBjYXNlIGIyU2VwYXJhdGlvbkZ1bmN0aW9uVHlwZS5lX3BvaW50czoge1xyXG4gICAgICAgIGNvbnN0IGxvY2FsUG9pbnRBOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlBLkdldFZlcnRleChpbmRleEEpO1xyXG4gICAgICAgIGNvbnN0IGxvY2FsUG9pbnRCOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlCLkdldFZlcnRleChpbmRleEIpO1xyXG5cclxuICAgICAgICBjb25zdCBwb2ludEE6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQSwgbG9jYWxQb2ludEEsIGIyVGltZU9mSW1wYWN0X3NfcG9pbnRBKTtcclxuICAgICAgICBjb25zdCBwb2ludEI6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQiwgbG9jYWxQb2ludEIsIGIyVGltZU9mSW1wYWN0X3NfcG9pbnRCKTtcclxuICAgICAgICBjb25zdCBzZXBhcmF0aW9uOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoXHJcbiAgICAgICAgICBiMlZlYzIuU3ViVlYocG9pbnRCLCBwb2ludEEsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICAgIHRoaXMubV9heGlzLFxyXG4gICAgICAgICk7XHJcblxyXG4gICAgICAgIHJldHVybiBzZXBhcmF0aW9uO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBjYXNlIGIyU2VwYXJhdGlvbkZ1bmN0aW9uVHlwZS5lX2ZhY2VBOiB7XHJcbiAgICAgICAgY29uc3Qgbm9ybWFsOiBiMlZlYzIgPSBiMlJvdC5NdWxSVih4ZkEucSwgdGhpcy5tX2F4aXMsIGIyVGltZU9mSW1wYWN0X3Nfbm9ybWFsKTtcclxuICAgICAgICBjb25zdCBwb2ludEE6IGIyVmVjMiA9IGIyVHJhbnNmb3JtLk11bFhWKHhmQSwgdGhpcy5tX2xvY2FsUG9pbnQsIGIyVGltZU9mSW1wYWN0X3NfcG9pbnRBKTtcclxuXHJcbiAgICAgICAgY29uc3QgbG9jYWxQb2ludEI6IGIyVmVjMiA9IHRoaXMubV9wcm94eUIuR2V0VmVydGV4KGluZGV4Qik7XHJcbiAgICAgICAgY29uc3QgcG9pbnRCOiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIGxvY2FsUG9pbnRCLCBiMlRpbWVPZkltcGFjdF9zX3BvaW50Qik7XHJcblxyXG4gICAgICAgIGNvbnN0IHNlcGFyYXRpb246IG51bWJlciA9IGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYocG9pbnRCLCBwb2ludEEsIGIyVmVjMi5zX3QwKSwgbm9ybWFsKTtcclxuICAgICAgICByZXR1cm4gc2VwYXJhdGlvbjtcclxuICAgICAgfVxyXG5cclxuICAgICAgY2FzZSBiMlNlcGFyYXRpb25GdW5jdGlvblR5cGUuZV9mYWNlQjoge1xyXG4gICAgICAgIGNvbnN0IG5vcm1hbDogYjJWZWMyID0gYjJSb3QuTXVsUlYoeGZCLnEsIHRoaXMubV9heGlzLCBiMlRpbWVPZkltcGFjdF9zX25vcm1hbCk7XHJcbiAgICAgICAgY29uc3QgcG9pbnRCOiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIHRoaXMubV9sb2NhbFBvaW50LCBiMlRpbWVPZkltcGFjdF9zX3BvaW50Qik7XHJcblxyXG4gICAgICAgIGNvbnN0IGxvY2FsUG9pbnRBOiBiMlZlYzIgPSB0aGlzLm1fcHJveHlBLkdldFZlcnRleChpbmRleEEpO1xyXG4gICAgICAgIGNvbnN0IHBvaW50QTogYjJWZWMyID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZBLCBsb2NhbFBvaW50QSwgYjJUaW1lT2ZJbXBhY3Rfc19wb2ludEEpO1xyXG5cclxuICAgICAgICBjb25zdCBzZXBhcmF0aW9uOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKHBvaW50QSwgcG9pbnRCLCBiMlZlYzIuc190MCksIG5vcm1hbCk7XHJcbiAgICAgICAgcmV0dXJuIHNlcGFyYXRpb247XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGRlZmF1bHQ6XHJcbiAgICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgICAgICAgcmV0dXJuIDA7XHJcbiAgICB9XHJcbiAgfVxyXG59XHJcblxyXG5jb25zdCBiMlRpbWVPZkltcGFjdF9zX3RpbWVyID0gbmV3IGIyVGltZXIoKTtcclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc19jYWNoZSA9IG5ldyBiMlNpbXBsZXhDYWNoZSgpO1xyXG5jb25zdCBiMlRpbWVPZkltcGFjdF9zX2Rpc3RhbmNlSW5wdXQgPSBuZXcgYjJEaXN0YW5jZUlucHV0KCk7XHJcbmNvbnN0IGIyVGltZU9mSW1wYWN0X3NfZGlzdGFuY2VPdXRwdXQgPSBuZXcgYjJEaXN0YW5jZU91dHB1dCgpO1xyXG5jb25zdCBiMlRpbWVPZkltcGFjdF9zX2ZjbiA9IG5ldyBiMlNlcGFyYXRpb25GdW5jdGlvbigpO1xyXG5jb25zdCBiMlRpbWVPZkltcGFjdF9zX2luZGV4QTogW251bWJlcl0gPSBbMF07XHJcbmNvbnN0IGIyVGltZU9mSW1wYWN0X3NfaW5kZXhCOiBbbnVtYmVyXSA9IFswXTtcclxuY29uc3QgYjJUaW1lT2ZJbXBhY3Rfc19zd2VlcEEgPSBuZXcgYjJTd2VlcCgpO1xyXG5jb25zdCBiMlRpbWVPZkltcGFjdF9zX3N3ZWVwQiA9IG5ldyBiMlN3ZWVwKCk7XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJUaW1lT2ZJbXBhY3Qob3V0cHV0OiBiMlRPSU91dHB1dCwgaW5wdXQ6IGIyVE9JSW5wdXQpOiB2b2lkIHtcclxuICBjb25zdCB0aW1lciA9IGIyVGltZU9mSW1wYWN0X3NfdGltZXIuUmVzZXQoKTtcclxuXHJcbiAgKytiMl90b2lTdGF0cy5jYWxscztcclxuXHJcbiAgb3V0cHV0LnN0YXRlID0gYjJUT0lPdXRwdXRTdGF0ZS5lX3Vua25vd247XHJcbiAgb3V0cHV0LnQgPSBpbnB1dC50TWF4O1xyXG5cclxuICBjb25zdCBwcm94eUE6IGIyRGlzdGFuY2VQcm94eSA9IGlucHV0LnByb3h5QTtcclxuICBjb25zdCBwcm94eUI6IGIyRGlzdGFuY2VQcm94eSA9IGlucHV0LnByb3h5QjtcclxuICBjb25zdCBtYXhWZXJ0aWNlczogbnVtYmVyID0gYjJNYXhJbnQoXHJcbiAgICBiMl9tYXhQb2x5Z29uVmVydGljZXMsXHJcbiAgICBiMk1heEludChwcm94eUEubV9jb3VudCwgcHJveHlCLm1fY291bnQpLFxyXG4gICk7XHJcblxyXG4gIGNvbnN0IHN3ZWVwQTogYjJTd2VlcCA9IGIyVGltZU9mSW1wYWN0X3Nfc3dlZXBBLkNvcHkoaW5wdXQuc3dlZXBBKTtcclxuICBjb25zdCBzd2VlcEI6IGIyU3dlZXAgPSBiMlRpbWVPZkltcGFjdF9zX3N3ZWVwQi5Db3B5KGlucHV0LnN3ZWVwQik7XHJcblxyXG4gIC8vIExhcmdlIHJvdGF0aW9ucyBjYW4gbWFrZSB0aGUgcm9vdCBmaW5kZXIgZmFpbCwgc28gd2Ugbm9ybWFsaXplIHRoZVxyXG4gIC8vIHN3ZWVwIGFuZ2xlcy5cclxuICBzd2VlcEEuTm9ybWFsaXplKCk7XHJcbiAgc3dlZXBCLk5vcm1hbGl6ZSgpO1xyXG5cclxuICBjb25zdCB0TWF4OiBudW1iZXIgPSBpbnB1dC50TWF4O1xyXG5cclxuICBjb25zdCB0b3RhbFJhZGl1czogbnVtYmVyID0gcHJveHlBLm1fcmFkaXVzICsgcHJveHlCLm1fcmFkaXVzO1xyXG4gIGNvbnN0IHRhcmdldDogbnVtYmVyID0gYjJNYXgoYjJfbGluZWFyU2xvcCwgdG90YWxSYWRpdXMgLSAzICogYjJfbGluZWFyU2xvcCk7XHJcbiAgY29uc3QgdG9sZXJhbmNlOiBudW1iZXIgPSAwLjI1ICogYjJfbGluZWFyU2xvcDtcclxuICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRhcmdldCA+IHRvbGVyYW5jZSk7XHJcblxyXG4gIGxldCB0MSA9IDA7XHJcbiAgY29uc3Qga19tYXhJdGVyYXRpb25zID0gMjA7IC8vIFRPRE9fRVJJTiBiMlNldHRpbmdzXHJcbiAgbGV0IGl0ZXIgPSAwO1xyXG5cclxuICAvLyBQcmVwYXJlIGlucHV0IGZvciBkaXN0YW5jZSBxdWVyeS5cclxuICBjb25zdCBjYWNoZTogYjJTaW1wbGV4Q2FjaGUgPSBiMlRpbWVPZkltcGFjdF9zX2NhY2hlO1xyXG4gIGNhY2hlLmNvdW50ID0gMDtcclxuICBjb25zdCBkaXN0YW5jZUlucHV0OiBiMkRpc3RhbmNlSW5wdXQgPSBiMlRpbWVPZkltcGFjdF9zX2Rpc3RhbmNlSW5wdXQ7XHJcbiAgZGlzdGFuY2VJbnB1dC5wcm94eUEuQ29weShpbnB1dC5wcm94eUEpO1xyXG4gIGRpc3RhbmNlSW5wdXQucHJveHlCLkNvcHkoaW5wdXQucHJveHlCKTtcclxuICBkaXN0YW5jZUlucHV0LnVzZVJhZGlpID0gZmFsc2U7XHJcblxyXG4gIC8vIFRoZSBvdXRlciBsb29wIHByb2dyZXNzaXZlbHkgYXR0ZW1wdHMgdG8gY29tcHV0ZSBuZXcgc2VwYXJhdGluZyBheGVzLlxyXG4gIC8vIFRoaXMgbG9vcCB0ZXJtaW5hdGVzIHdoZW4gYW4gYXhpcyBpcyByZXBlYXRlZCAobm8gcHJvZ3Jlc3MgaXMgbWFkZSkuXHJcbiAgZm9yICg7Oykge1xyXG4gICAgY29uc3QgeGZBOiBiMlRyYW5zZm9ybSA9IGIyVGltZU9mSW1wYWN0X3NfeGZBO1xyXG4gICAgY29uc3QgeGZCOiBiMlRyYW5zZm9ybSA9IGIyVGltZU9mSW1wYWN0X3NfeGZCO1xyXG4gICAgc3dlZXBBLkdldFRyYW5zZm9ybSh4ZkEsIHQxKTtcclxuICAgIHN3ZWVwQi5HZXRUcmFuc2Zvcm0oeGZCLCB0MSk7XHJcblxyXG4gICAgLy8gR2V0IHRoZSBkaXN0YW5jZSBiZXR3ZWVuIHNoYXBlcy4gV2UgY2FuIGFsc28gdXNlIHRoZSByZXN1bHRzXHJcbiAgICAvLyB0byBnZXQgYSBzZXBhcmF0aW5nIGF4aXMuXHJcbiAgICBkaXN0YW5jZUlucHV0LnRyYW5zZm9ybUEuQ29weSh4ZkEpO1xyXG4gICAgZGlzdGFuY2VJbnB1dC50cmFuc2Zvcm1CLkNvcHkoeGZCKTtcclxuICAgIGNvbnN0IGRpc3RhbmNlT3V0cHV0OiBiMkRpc3RhbmNlT3V0cHV0ID0gYjJUaW1lT2ZJbXBhY3Rfc19kaXN0YW5jZU91dHB1dDtcclxuICAgIGIyRGlzdGFuY2UoZGlzdGFuY2VPdXRwdXQsIGNhY2hlLCBkaXN0YW5jZUlucHV0KTtcclxuXHJcbiAgICAvLyBJZiB0aGUgc2hhcGVzIGFyZSBvdmVybGFwcGVkLCB3ZSBnaXZlIHVwIG9uIGNvbnRpbnVvdXMgY29sbGlzaW9uLlxyXG4gICAgaWYgKGRpc3RhbmNlT3V0cHV0LmRpc3RhbmNlIDw9IDApIHtcclxuICAgICAgLy8gRmFpbHVyZSFcclxuICAgICAgb3V0cHV0LnN0YXRlID0gYjJUT0lPdXRwdXRTdGF0ZS5lX292ZXJsYXBwZWQ7XHJcbiAgICAgIG91dHB1dC50ID0gMDtcclxuICAgICAgYnJlYWs7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGRpc3RhbmNlT3V0cHV0LmRpc3RhbmNlIDwgdGFyZ2V0ICsgdG9sZXJhbmNlKSB7XHJcbiAgICAgIC8vIFZpY3RvcnkhXHJcbiAgICAgIG91dHB1dC5zdGF0ZSA9IGIyVE9JT3V0cHV0U3RhdGUuZV90b3VjaGluZztcclxuICAgICAgb3V0cHV0LnQgPSB0MTtcclxuICAgICAgYnJlYWs7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gSW5pdGlhbGl6ZSB0aGUgc2VwYXJhdGluZyBheGlzLlxyXG4gICAgY29uc3QgZmNuOiBiMlNlcGFyYXRpb25GdW5jdGlvbiA9IGIyVGltZU9mSW1wYWN0X3NfZmNuO1xyXG4gICAgZmNuLkluaXRpYWxpemUoY2FjaGUsIHByb3h5QSwgc3dlZXBBLCBwcm94eUIsIHN3ZWVwQiwgdDEpO1xyXG4gICAgLypcclxuICAgICAgICAjaWYgMFxyXG4gICAgICAgICAgICAvLyBEdW1wIHRoZSBjdXJ2ZSBzZWVuIGJ5IHRoZSByb290IGZpbmRlciB7XHJcbiAgICAgICAgICAgICAgY29uc3QgaW50MzIgTiA9IDEwMDtcclxuICAgICAgICAgICAgICBmbG9hdDMyIGR4ID0gMS4wZiAvIE47XHJcbiAgICAgICAgICAgICAgZmxvYXQzMiB4c1tOKzFdO1xyXG4gICAgICAgICAgICAgIGZsb2F0MzIgZnNbTisxXTtcclxuXHJcbiAgICAgICAgICAgICAgZmxvYXQzMiB4ID0gMC4wZjtcclxuXHJcbiAgICAgICAgICAgICAgZm9yIChpbnQzMiBpID0gMDsgaSA8PSBOOyArK2kpIHtcclxuICAgICAgICAgICAgICAgIHN3ZWVwQS5HZXRUcmFuc2Zvcm0oJnhmQSwgeCk7XHJcbiAgICAgICAgICAgICAgICBzd2VlcEIuR2V0VHJhbnNmb3JtKCZ4ZkIsIHgpO1xyXG4gICAgICAgICAgICAgICAgZmxvYXQzMiBmID0gZmNuLkV2YWx1YXRlKHhmQSwgeGZCKSAtIHRhcmdldDtcclxuXHJcbiAgICAgICAgICAgICAgICBwcmludGYoXCIlZyAlZ1xcblwiLCB4LCBmKTtcclxuXHJcbiAgICAgICAgICAgICAgICB4c1tpXSA9IHg7XHJcbiAgICAgICAgICAgICAgICBmc1tpXSA9IGY7XHJcblxyXG4gICAgICAgICAgICAgICAgeCArPSBkeDtcclxuICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAjZW5kaWZcclxuICAgICAgICAqL1xyXG5cclxuICAgIC8vIENvbXB1dGUgdGhlIFRPSSBvbiB0aGUgc2VwYXJhdGluZyBheGlzLiBXZSBkbyB0aGlzIGJ5IHN1Y2Nlc3NpdmVseVxyXG4gICAgLy8gcmVzb2x2aW5nIHRoZSBkZWVwZXN0IHBvaW50LiBUaGlzIGxvb3AgaXMgYm91bmRlZCBieSB0aGUgbnVtYmVyIG9mIHZlcnRpY2VzLlxyXG4gICAgbGV0IGRvbmUgPSBmYWxzZTtcclxuICAgIGxldCB0MjogbnVtYmVyID0gdE1heDtcclxuICAgIGxldCBwdXNoQmFja0l0ZXIgPSAwO1xyXG4gICAgZm9yICg7Oykge1xyXG4gICAgICAvLyBGaW5kIHRoZSBkZWVwZXN0IHBvaW50IGF0IHQyLiBTdG9yZSB0aGUgd2l0bmVzcyBwb2ludCBpbmRpY2VzLlxyXG4gICAgICBjb25zdCBpbmRleEE6IFtudW1iZXJdID0gYjJUaW1lT2ZJbXBhY3Rfc19pbmRleEE7XHJcbiAgICAgIGNvbnN0IGluZGV4QjogW251bWJlcl0gPSBiMlRpbWVPZkltcGFjdF9zX2luZGV4QjtcclxuICAgICAgbGV0IHMyOiBudW1iZXIgPSBmY24uRmluZE1pblNlcGFyYXRpb24oaW5kZXhBLCBpbmRleEIsIHQyKTtcclxuXHJcbiAgICAgIC8vIElzIHRoZSBmaW5hbCBjb25maWd1cmF0aW9uIHNlcGFyYXRlZD9cclxuICAgICAgaWYgKHMyID4gdGFyZ2V0ICsgdG9sZXJhbmNlKSB7XHJcbiAgICAgICAgLy8gVmljdG9yeSFcclxuICAgICAgICBvdXRwdXQuc3RhdGUgPSBiMlRPSU91dHB1dFN0YXRlLmVfc2VwYXJhdGVkO1xyXG4gICAgICAgIG91dHB1dC50ID0gdE1heDtcclxuICAgICAgICBkb25lID0gdHJ1ZTtcclxuICAgICAgICBicmVhaztcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gSGFzIHRoZSBzZXBhcmF0aW9uIHJlYWNoZWQgdG9sZXJhbmNlP1xyXG4gICAgICBpZiAoczIgPiB0YXJnZXQgLSB0b2xlcmFuY2UpIHtcclxuICAgICAgICAvLyBBZHZhbmNlIHRoZSBzd2VlcHNcclxuICAgICAgICB0MSA9IHQyO1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBDb21wdXRlIHRoZSBpbml0aWFsIHNlcGFyYXRpb24gb2YgdGhlIHdpdG5lc3MgcG9pbnRzLlxyXG4gICAgICBsZXQgczE6IG51bWJlciA9IGZjbi5FdmFsdWF0ZShpbmRleEFbMF0sIGluZGV4QlswXSwgdDEpO1xyXG5cclxuICAgICAgLy8gQ2hlY2sgZm9yIGluaXRpYWwgb3ZlcmxhcC4gVGhpcyBtaWdodCBoYXBwZW4gaWYgdGhlIHJvb3QgZmluZGVyXHJcbiAgICAgIC8vIHJ1bnMgb3V0IG9mIGl0ZXJhdGlvbnMuXHJcbiAgICAgIGlmIChzMSA8IHRhcmdldCAtIHRvbGVyYW5jZSkge1xyXG4gICAgICAgIG91dHB1dC5zdGF0ZSA9IGIyVE9JT3V0cHV0U3RhdGUuZV9mYWlsZWQ7XHJcbiAgICAgICAgb3V0cHV0LnQgPSB0MTtcclxuICAgICAgICBkb25lID0gdHJ1ZTtcclxuICAgICAgICBicmVhaztcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gQ2hlY2sgZm9yIHRvdWNoaW5nXHJcbiAgICAgIGlmIChzMSA8PSB0YXJnZXQgKyB0b2xlcmFuY2UpIHtcclxuICAgICAgICAvLyBWaWN0b3J5ISB0MSBzaG91bGQgaG9sZCB0aGUgVE9JIChjb3VsZCBiZSAwLjApLlxyXG4gICAgICAgIG91dHB1dC5zdGF0ZSA9IGIyVE9JT3V0cHV0U3RhdGUuZV90b3VjaGluZztcclxuICAgICAgICBvdXRwdXQudCA9IHQxO1xyXG4gICAgICAgIGRvbmUgPSB0cnVlO1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBDb21wdXRlIDFEIHJvb3Qgb2Y6IGYoeCkgLSB0YXJnZXQgPSAwXHJcbiAgICAgIGxldCByb290SXRlckNvdW50ID0gMDtcclxuICAgICAgbGV0IGExOiBudW1iZXIgPSB0MTtcclxuICAgICAgbGV0IGEyOiBudW1iZXIgPSB0MjtcclxuICAgICAgZm9yICg7Oykge1xyXG4gICAgICAgIC8vIFVzZSBhIG1peCBvZiB0aGUgc2VjYW50IHJ1bGUgYW5kIGJpc2VjdGlvbi5cclxuICAgICAgICBsZXQgdCA9IDA7XHJcbiAgICAgICAgaWYgKHJvb3RJdGVyQ291bnQgJiAxKSB7XHJcbiAgICAgICAgICAvLyBTZWNhbnQgcnVsZSB0byBpbXByb3ZlIGNvbnZlcmdlbmNlLlxyXG4gICAgICAgICAgdCA9IGExICsgKCh0YXJnZXQgLSBzMSkgKiAoYTIgLSBhMSkpIC8gKHMyIC0gczEpO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICAvLyBCaXNlY3Rpb24gdG8gZ3VhcmFudGVlIHByb2dyZXNzLlxyXG4gICAgICAgICAgdCA9IDAuNSAqIChhMSArIGEyKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgICsrcm9vdEl0ZXJDb3VudDtcclxuICAgICAgICArK2IyX3RvaVN0YXRzLnJvb3RJdGVycztcclxuXHJcbiAgICAgICAgY29uc3QgczogbnVtYmVyID0gZmNuLkV2YWx1YXRlKGluZGV4QVswXSwgaW5kZXhCWzBdLCB0KTtcclxuXHJcbiAgICAgICAgaWYgKGIyQWJzKHMgLSB0YXJnZXQpIDwgdG9sZXJhbmNlKSB7XHJcbiAgICAgICAgICAvLyB0MiBob2xkcyBhIHRlbnRhdGl2ZSB2YWx1ZSBmb3IgdDFcclxuICAgICAgICAgIHQyID0gdDtcclxuICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy8gRW5zdXJlIHdlIGNvbnRpbnVlIHRvIGJyYWNrZXQgdGhlIHJvb3QuXHJcbiAgICAgICAgaWYgKHMgPiB0YXJnZXQpIHtcclxuICAgICAgICAgIGExID0gdDtcclxuICAgICAgICAgIHMxID0gcztcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgYTIgPSB0O1xyXG4gICAgICAgICAgczIgPSBzO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKHJvb3RJdGVyQ291bnQgPT09IDUwKSB7XHJcbiAgICAgICAgICBicmVhaztcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGIyX3RvaVN0YXRzLm1heFJvb3RJdGVycyA9IGIyTWF4SW50KGIyX3RvaVN0YXRzLm1heFJvb3RJdGVycywgcm9vdEl0ZXJDb3VudCk7XHJcblxyXG4gICAgICArK3B1c2hCYWNrSXRlcjtcclxuXHJcbiAgICAgIGlmIChwdXNoQmFja0l0ZXIgPT09IG1heFZlcnRpY2VzKSB7XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICArK2l0ZXI7XHJcbiAgICArK2IyX3RvaVN0YXRzLml0ZXJzO1xyXG5cclxuICAgIGlmIChkb25lKSB7XHJcbiAgICAgIGJyZWFrO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChpdGVyID09PSBrX21heEl0ZXJhdGlvbnMpIHtcclxuICAgICAgLy8gUm9vdCBmaW5kZXIgZ290IHN0dWNrLiBTZW1pLXZpY3RvcnkuXHJcbiAgICAgIG91dHB1dC5zdGF0ZSA9IGIyVE9JT3V0cHV0U3RhdGUuZV9mYWlsZWQ7XHJcbiAgICAgIG91dHB1dC50ID0gdDE7XHJcbiAgICAgIGJyZWFrO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgYjJfdG9pU3RhdHMubWF4SXRlcnMgPSBiMk1heEludChiMl90b2lTdGF0cy5tYXhJdGVycywgaXRlcik7XHJcbiAgY29uc3QgdGltZSA9IHRpbWVyLkdldE1pbGxpc2Vjb25kcygpO1xyXG4gIGIyX3RvaVN0YXRzLm1heFRpbWUgPSBiMk1heChiMl90b2lTdGF0cy5tYXhUaW1lLCB0aW1lKTtcclxuICBiMl90b2lTdGF0cy50aW1lICs9IHRpbWU7XHJcbn1cclxuIl19