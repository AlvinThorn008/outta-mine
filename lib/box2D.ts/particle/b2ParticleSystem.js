/*
 * Copyright (c) 2013 Google, Inc.
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
import { b2_linearSlop, b2_maxFloat, b2Assert, b2MakeArray, b2Maybe } from '../common/b2Settings';
import { b2_barrierCollisionTime, b2_invalidParticleIndex, b2_maxParticleForce, b2_maxParticleIndex, b2_maxParticlePressure, b2_maxTriadDistanceSquared, b2_minParticleSystemBufferCapacity, b2_minParticleWeight, b2_particleStride, } from '../common/b2SettingsParticleSystem';
import { b2Abs, b2Clamp, b2InvSqrt, b2Max, b2MaxInt, b2Min, b2MinInt, b2Rot, b2Sqrt, b2Transform, b2Vec2, } from '../common/b2Math';
import { b2Color } from '../common/b2Draw';
import { b2AABB, b2RayCastInput, b2RayCastOutput } from '../collision/b2Collision';
import { b2Shape } from '../collision/shapes/b2Shape';
import { b2EdgeShape } from '../collision/shapes/b2EdgeShape';
import { b2TimeStep } from '../dynamics/b2TimeStep';
import { b2QueryCallback, } from '../dynamics/b2WorldCallbacks';
import { b2ParticleDef, b2ParticleHandle } from './b2Particle';
import { b2ParticleGroup, b2ParticleGroupDef, } from './b2ParticleGroup';
import { b2VoronoiDiagram } from './b2VoronoiDiagram';
function std_iter_swap(array, a, b) {
    const tmp = array[a];
    array[a] = array[b];
    array[b] = tmp;
}
function default_compare(a, b) {
    return a < b;
}
function std_sort(array, first = 0, len = array.length - first, cmp = default_compare) {
    let left = first;
    const stack = [];
    let pos = 0;
    for (;;) {
        /* outer loop */
        for (; left + 1 < len; len++) {
            /* sort left to len-1 */
            const pivot = array[left + Math.floor(Math.random() * (len - left))]; /* pick random pivot */
            stack[pos++] = len; /* sort right part later */
            for (let right = left - 1;;) {
                /* inner loop: partitioning */
                // eslint-disable-next-line no-empty
                while (cmp(array[++right], pivot)) { } /* look for greater element */
                // eslint-disable-next-line no-empty
                while (cmp(pivot, array[--len])) { } /* look for smaller element */
                if (right >= len) {
                    break;
                } /* partition point found? */
                std_iter_swap(array, right, len); /* the only swap */
            } /* partitioned, continue left part */
        }
        if (pos === 0) {
            break;
        } /* stack empty? */
        left = len; /* left to right is sorted */
        len = stack[--pos]; /* get next range to sort */
    }
    return array;
}
function std_stable_sort(array, first = 0, len = array.length - first, cmp = default_compare) {
    return std_sort(array, first, len, cmp);
}
function std_remove_if(array, predicate, length = array.length) {
    let l = 0;
    for (let c = 0; c < length; ++c) {
        // if we can be collapsed, keep l where it is.
        if (predicate(array[c])) {
            continue;
        }
        // this node can't be collapsed; push it back as far as we can.
        if (c === l) {
            ++l;
            continue; // quick exit if we're already in the right spot
        }
        // array[l++] = array[c];
        std_iter_swap(array, l++, c);
    }
    return l;
}
function std_lower_bound(array, first, last, val, cmp) {
    let count = last - first;
    while (count > 0) {
        const step = Math.floor(count / 2);
        let it = first + step;
        if (cmp(array[it], val)) {
            first = ++it;
            count -= step + 1;
        }
        else {
            count = step;
        }
    }
    return first;
}
function std_upper_bound(array, first, last, val, cmp) {
    let count = last - first;
    while (count > 0) {
        const step = Math.floor(count / 2);
        let it = first + step;
        if (!cmp(val, array[it])) {
            first = ++it;
            count -= step + 1;
        }
        else {
            count = step;
        }
    }
    return first;
}
function std_rotate(array, first, n_first, last) {
    let next = n_first;
    while (first !== next) {
        std_iter_swap(array, first++, next++);
        if (next === last) {
            next = n_first;
        }
        else if (first === n_first) {
            n_first = next;
        }
    }
}
function std_unique(array, first, last, cmp) {
    if (first === last) {
        return last;
    }
    let result = first;
    while (++first !== last) {
        if (!cmp(array[result], array[first])) {
            ///array[++result] = array[first];
            std_iter_swap(array, ++result, first);
        }
    }
    return ++result;
}
const newIndices = (i, start, mid, end) => {
    if (i < start) {
        return i;
    }
    else if (i < mid) {
        return i + end - mid;
    }
    else if (i < end) {
        return i + start - mid;
    }
    else {
        return i;
    }
};
export class b2GrowableBuffer {
    constructor(allocator) {
        this.data = [];
        this.count = 0;
        this.capacity = 0;
        this.allocator = allocator;
    }
    Append() {
        if (this.count >= this.capacity) {
            this.Grow();
        }
        return this.count++;
    }
    Reserve(newCapacity) {
        if (this.capacity >= newCapacity) {
            return;
        }
        !!B2_DEBUG && b2Assert(this.capacity === this.data.length);
        for (let i = this.capacity; i < newCapacity; ++i) {
            this.data[i] = this.allocator();
        }
        this.capacity = newCapacity;
    }
    Grow() {
        // Double the capacity.
        const newCapacity = this.capacity ? 2 * this.capacity : b2_minParticleSystemBufferCapacity;
        !!B2_DEBUG && b2Assert(newCapacity > this.capacity);
        this.Reserve(newCapacity);
    }
    Free() {
        if (this.data.length === 0) {
            return;
        }
        this.data = [];
        this.capacity = 0;
        this.count = 0;
    }
    Shorten(newEnd) {
        !!B2_DEBUG && b2Assert(false);
    }
    Data() {
        return this.data;
    }
    GetCount() {
        return this.count;
    }
    SetCount(newCount) {
        !!B2_DEBUG && b2Assert(0 <= newCount && newCount <= this.capacity);
        this.count = newCount;
    }
    GetCapacity() {
        return this.capacity;
    }
    RemoveIf(pred) {
        if (B2_DEBUG) {
            let count = 0;
            for (let i = 0; i < this.count; ++i) {
                if (!pred(this.data[i])) {
                    count++;
                }
            }
            this.count = std_remove_if(this.data, pred, this.count);
            b2Assert(count === this.count);
        }
        else {
            this.count = std_remove_if(this.data, pred, this.count);
        }
    }
    Unique(pred) {
        this.count = std_unique(this.data, 0, this.count, pred);
    }
}
export class b2FixtureParticleQueryCallback extends b2QueryCallback {
    constructor(system) {
        super();
        this.m_system = system;
    }
    ShouldQueryParticleSystem(system) {
        // Skip reporting particles.
        return false;
    }
    ReportFixture(fixture) {
        if (fixture.IsSensor()) {
            return true;
        }
        const shape = fixture.GetShape();
        const childCount = shape.GetChildCount();
        for (let childIndex = 0; childIndex < childCount; childIndex++) {
            const aabb = fixture.GetAABB(childIndex);
            const enumerator = this.m_system.GetInsideBoundsEnumerator(aabb);
            let index;
            while ((index = enumerator.GetNext()) >= 0) {
                this.ReportFixtureAndParticle(fixture, childIndex, index);
            }
        }
        return true;
    }
    ReportParticle(system, index) {
        return false;
    }
    ReportFixtureAndParticle(fixture, childIndex, index) {
        !!B2_DEBUG && b2Assert(false); // pure virtual
    }
}
export class b2ParticleContact {
    constructor() {
        this.indexA = 0;
        this.indexB = 0;
        this.weight = NaN;
        this.normal = new b2Vec2();
        this.flags = 0 /* none */;
        this.weight = 0.0;
    }
    SetIndices(a, b) {
        !!B2_DEBUG && b2Assert(a <= b2_maxParticleIndex && b <= b2_maxParticleIndex);
        this.indexA = a;
        this.indexB = b;
    }
    SetWeight(w) {
        this.weight = w;
    }
    SetNormal(n) {
        this.normal.Copy(n);
    }
    SetFlags(f) {
        this.flags = f;
    }
    GetIndexA() {
        return this.indexA;
    }
    GetIndexB() {
        return this.indexB;
    }
    GetWeight() {
        return this.weight;
    }
    GetNormal() {
        return this.normal;
    }
    GetFlags() {
        return this.flags;
    }
    IsEqual(rhs) {
        return (this.indexA === rhs.indexA &&
            this.indexB === rhs.indexB &&
            this.flags === rhs.flags &&
            this.weight === rhs.weight &&
            this.normal.x === rhs.normal.x &&
            this.normal.y === rhs.normal.y);
    }
    IsNotEqual(rhs) {
        return !this.IsEqual(rhs);
    }
    ApproximatelyEqual(rhs) {
        const MAX_WEIGHT_DIFF = 0.01; // Weight 0 ~ 1, so about 1%
        const MAX_NORMAL_DIFF_SQ = 0.01 * 0.01; // Normal length = 1, so 1%
        return (this.indexA === rhs.indexA &&
            this.indexB === rhs.indexB &&
            this.flags === rhs.flags &&
            b2Abs(this.weight - rhs.weight) < MAX_WEIGHT_DIFF &&
            b2Vec2.DistanceSquaredVV(this.normal, rhs.normal) < MAX_NORMAL_DIFF_SQ);
    }
}
export class b2ParticleBodyContact {
    constructor() {
        this.index = 0; // Index of the particle making contact.
        this.weight = NaN; // Weight of the contact. A value between 0.0f and 1.0f.
        this.normal = new b2Vec2(); // The normalized direction from the particle to the body.
        this.mass = NaN; // The effective mass used in calculating force.
        this.weight = 0.0;
        this.mass = 0.0;
    }
}
export class b2ParticlePair {
    constructor() {
        this.indexA = 0; // Indices of the respective particles making pair.
        this.indexB = 0;
        this.flags = 0 /* none */; // The logical sum of the particle flags. See the b2ParticleFlag enum.
        this.strength = NaN; // The strength of cohesion among the particles.
        this.distance = NaN; // The initial distance of the particles.
        this.strength = 0.0;
        this.distance = 0.0;
    }
}
export class b2ParticleTriad {
    constructor() {
        this.indexA = 0; // Indices of the respective particles making triad.
        this.indexB = 0;
        this.indexC = 0;
        this.flags = 0 /* none */; // The logical sum of the particle flags. See the b2ParticleFlag enum.
        this.strength = NaN; // The strength of cohesion among the particles.
        this.pa = new b2Vec2(); // Values used for calculation.
        this.pb = new b2Vec2();
        this.pc = new b2Vec2();
        this.ka = NaN;
        this.kb = NaN;
        this.kc = NaN;
        this.s = NaN;
        this.strength = 0.0;
        this.ka = 0.0;
        this.kb = 0.0;
        this.kc = 0.0;
        this.s = 0.0;
    }
}
export class b2ParticleSystemDef {
    constructor() {
        // Initialize physical coefficients to the maximum values that
        // maintain numerical stability.
        /**
         * Enable strict Particle/Body contact check.
         * See SetStrictContactCheck for details.
         */
        this.strictContactCheck = false;
        /**
         * Set the particle density.
         * See SetDensity for details.
         */
        this.density = NaN;
        /**
         * Change the particle gravity scale. Adjusts the effect of the
         * global gravity vector on particles. Default value is 1.0f.
         */
        this.gravityScale = NaN;
        /**
         * Particles behave as circles with this radius. In Box2D units.
         */
        this.radius = NaN;
        /**
         * Set the maximum number of particles.
         * By default, there is no maximum. The particle buffers can
         * continue to grow while b2World's block allocator still has
         * memory.
         * See SetMaxParticleCount for details.
         */
        this.maxCount = 0;
        /**
         * Increases pressure in response to compression
         * Smaller values allow more compression
         */
        this.pressureStrength = 0.005;
        /**
         * Reduces velocity along the collision normal
         * Smaller value reduces less
         */
        this.dampingStrength = NaN;
        /**
         * Restores shape of elastic particle groups
         * Larger values increase elastic particle velocity
         */
        this.elasticStrength = 0.25;
        /**
         * Restores length of spring particle groups
         * Larger values increase spring particle velocity
         */
        this.springStrength = 0.25;
        /**
         * Reduces relative velocity of viscous particles
         * Larger values slow down viscous particles more
         */
        this.viscousStrength = 0.25;
        /**
         * Produces pressure on tensile particles
         * 0~0.2. Larger values increase the amount of surface tension.
         */
        this.surfaceTensionPressureStrength = 0.2;
        /**
         * Smoothes outline of tensile particles
         * 0~0.2. Larger values result in rounder, smoother,
         * water-drop-like clusters of particles.
         */
        this.surfaceTensionNormalStrength = 0.2;
        /**
         * Produces additional pressure on repulsive particles
         * Larger values repulse more
         * Negative values mean attraction. The range where particles
         * behave stably is about -0.2 to 2.0.
         */
        this.repulsiveStrength = NaN;
        /**
         * Produces repulsion between powder particles
         * Larger values repulse more
         */
        this.powderStrength = 0.5;
        /**
         * Pushes particles out of solid particle group
         * Larger values repulse more
         */
        this.ejectionStrength = 0.5;
        /**
         * Produces static pressure
         * Larger values increase the pressure on neighboring partilces
         * For a description of static pressure, see
         * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
         */
        this.staticPressureStrength = 0.2;
        /**
         * Reduces instability in static pressure calculation
         * Larger values make stabilize static pressure with fewer
         * iterations
         */
        this.staticPressureRelaxation = 0.2;
        /**
         * Computes static pressure more precisely
         * See SetStaticPressureIterations for details
         */
        this.staticPressureIterations = 8;
        /**
         * Determines how fast colors are mixed
         * 1.0f ==> mixed immediately
         * 0.5f ==> mixed half way each simulation step (see
         * b2World::Step())
         */
        this.colorMixingStrength = 0.5;
        /**
         * Whether to destroy particles by age when no more particles
         * can be created.  See #b2ParticleSystem::SetDestructionByAge()
         * for more information.
         */
        this.destroyByAge = true;
        /**
         * Granularity of particle lifetimes in seconds.  By default
         * this is set to (1.0f / 60.0f) seconds.  b2ParticleSystem uses
         * a 32-bit signed value to track particle lifetimes so the
         * maximum lifetime of a particle is (2^32 - 1) / (1.0f /
         * lifetimeGranularity) seconds. With the value set to 1/60 the
         * maximum lifetime or age of a particle is 2.27 years.
         */
        this.lifetimeGranularity = 1.0 / 60.0;
        this.density = 1.0;
        this.gravityScale = 1.0;
        this.radius = 1.0;
        this.dampingStrength = 1.0;
        this.repulsiveStrength = 1.0;
    }
    Copy(def) {
        this.strictContactCheck = def.strictContactCheck;
        this.density = def.density;
        this.gravityScale = def.gravityScale;
        this.radius = def.radius;
        this.maxCount = def.maxCount;
        this.pressureStrength = def.pressureStrength;
        this.dampingStrength = def.dampingStrength;
        this.elasticStrength = def.elasticStrength;
        this.springStrength = def.springStrength;
        this.viscousStrength = def.viscousStrength;
        this.surfaceTensionPressureStrength = def.surfaceTensionPressureStrength;
        this.surfaceTensionNormalStrength = def.surfaceTensionNormalStrength;
        this.repulsiveStrength = def.repulsiveStrength;
        this.powderStrength = def.powderStrength;
        this.ejectionStrength = def.ejectionStrength;
        this.staticPressureStrength = def.staticPressureStrength;
        this.staticPressureRelaxation = def.staticPressureRelaxation;
        this.staticPressureIterations = def.staticPressureIterations;
        this.colorMixingStrength = def.colorMixingStrength;
        this.destroyByAge = def.destroyByAge;
        this.lifetimeGranularity = def.lifetimeGranularity;
        return this;
    }
    Clone() {
        return new b2ParticleSystemDef().Copy(this);
    }
}
export class b2ParticleSystem {
    constructor(def, world) {
        this.m_paused = false;
        this.m_timestamp = 0;
        this.m_allParticleFlags = 0 /* none */;
        this.m_needsUpdateAllParticleFlags = false;
        this.m_allGroupFlags = 0 /* none */;
        this.m_needsUpdateAllGroupFlags = false;
        this.m_hasForce = false;
        this.m_iterationIndex = 0;
        this.m_inverseDensity = NaN;
        this.m_particleDiameter = NaN;
        this.m_inverseDiameter = NaN;
        this.m_squaredDiameter = NaN;
        this.m_count = 0;
        this.m_internalAllocatedCapacity = 0;
        /**
         * Allocator for b2ParticleHandle instances.
         */
        ///m_handleAllocator: any = null;
        /**
         * Maps particle indicies to handles.
         */
        this.m_handleIndexBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_flagsBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_positionBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_velocityBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_forceBuffer = [];
        /**
         * this.m_weightBuffer is populated in ComputeWeight and used in
         * ComputeDepth(), SolveStaticPressure() and SolvePressure().
         */
        this.m_weightBuffer = [];
        /**
         * When any particles have the flag b2_staticPressureParticle,
         * this.m_staticPressureBuffer is first allocated and used in
         * SolveStaticPressure() and SolvePressure().  It will be
         * reallocated on subsequent CreateParticle() calls.
         */
        this.m_staticPressureBuffer = [];
        /**
         * this.m_accumulationBuffer is used in many functions as a temporary
         * buffer for scalar values.
         */
        this.m_accumulationBuffer = [];
        /**
         * When any particles have the flag b2_tensileParticle,
         * this.m_accumulation2Buffer is first allocated and used in
         * SolveTensile() as a temporary buffer for vector values.  It
         * will be reallocated on subsequent CreateParticle() calls.
         */
        this.m_accumulation2Buffer = [];
        /**
         * When any particle groups have the flag b2_solidParticleGroup,
         * this.m_depthBuffer is first allocated and populated in
         * ComputeDepth() and used in SolveSolid(). It will be
         * reallocated on subsequent CreateParticle() calls.
         */
        this.m_depthBuffer = [];
        this.m_colorBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_groupBuffer = [];
        this.m_userDataBuffer = new b2ParticleSystem_UserOverridableBuffer();
        /**
         * Stuck particle detection parameters and record keeping
         */
        this.m_stuckThreshold = 0;
        this.m_lastBodyContactStepBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_bodyContactCountBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_consecutiveContactStepsBuffer = new b2ParticleSystem_UserOverridableBuffer();
        this.m_stuckParticleBuffer = new b2GrowableBuffer(() => 0);
        this.m_proxyBuffer = new b2GrowableBuffer(() => new b2ParticleSystem_Proxy());
        this.m_contactBuffer = new b2GrowableBuffer(() => new b2ParticleContact());
        this.m_bodyContactBuffer = new b2GrowableBuffer(() => new b2ParticleBodyContact());
        this.m_pairBuffer = new b2GrowableBuffer(() => new b2ParticlePair());
        this.m_triadBuffer = new b2GrowableBuffer(() => new b2ParticleTriad());
        /**
         * Time each particle should be destroyed relative to the last
         * time this.m_timeElapsed was initialized.  Each unit of time
         * corresponds to b2ParticleSystemDef::lifetimeGranularity
         * seconds.
         */
        this.m_expirationTimeBuffer = new b2ParticleSystem_UserOverridableBuffer();
        /**
         * List of particle indices sorted by expiration time.
         */
        this.m_indexByExpirationTimeBuffer = new b2ParticleSystem_UserOverridableBuffer();
        /**
         * Time elapsed in 32:32 fixed point.  Each non-fractional unit
         * of time corresponds to
         * b2ParticleSystemDef::lifetimeGranularity seconds.
         */
        // TODO: check and implement optimized SMI storage?
        this.m_timeElapsed = 0;
        /**
         * Whether the expiration time buffer has been modified and
         * needs to be resorted.
         */
        this.m_expirationTimeBufferRequiresSorting = false;
        this.m_groupCount = 0;
        this.m_groupList = null;
        this.m_def = new b2ParticleSystemDef();
        this.m_prev = null;
        this.m_next = null;
        this.UpdateBodyContacts_callback = null;
        this.SolveCollision_callback = null;
        this.m_inverseDensity = 0.0;
        this.m_particleDiameter = 0.0;
        this.m_inverseDiameter = 0.0;
        this.m_squaredDiameter = 0.0;
        this.SetStrictContactCheck(def.strictContactCheck);
        this.SetDensity(def.density);
        this.SetGravityScale(def.gravityScale);
        this.SetRadius(def.radius);
        this.SetMaxParticleCount(def.maxCount);
        !!B2_DEBUG && b2Assert(def.lifetimeGranularity > 0.0);
        this.m_def = def.Clone();
        this.m_world = world;
        this.SetDestructionByAge(this.m_def.destroyByAge);
    }
    static computeTag(x, y) {
        ///return ((uint32)(y + yOffset) << yShift) + (uint32)(xScale * x + xOffset);
        return (((((y + b2ParticleSystem.yOffset) >>> 0) << b2ParticleSystem.yShift) +
            ((b2ParticleSystem.xScale * x + b2ParticleSystem.xOffset) >>> 0)) >>>
            0);
    }
    static computeRelativeTag(tag, x, y) {
        ///return tag + (y << yShift) + (x << xShift);
        return (tag + (y << b2ParticleSystem.yShift) + (x << b2ParticleSystem.xShift)) >>> 0;
    }
    Drop() {
        while (this.m_groupList) {
            this.DestroyParticleGroup(this.m_groupList);
        }
        this.FreeUserOverridableBuffer(this.m_handleIndexBuffer);
        this.FreeUserOverridableBuffer(this.m_flagsBuffer);
        this.FreeUserOverridableBuffer(this.m_lastBodyContactStepBuffer);
        this.FreeUserOverridableBuffer(this.m_bodyContactCountBuffer);
        this.FreeUserOverridableBuffer(this.m_consecutiveContactStepsBuffer);
        this.FreeUserOverridableBuffer(this.m_positionBuffer);
        this.FreeUserOverridableBuffer(this.m_velocityBuffer);
        this.FreeUserOverridableBuffer(this.m_colorBuffer);
        this.FreeUserOverridableBuffer(this.m_userDataBuffer);
        this.FreeUserOverridableBuffer(this.m_expirationTimeBuffer);
        this.FreeUserOverridableBuffer(this.m_indexByExpirationTimeBuffer);
        this.FreeBuffer(this.m_forceBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_weightBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_staticPressureBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_accumulationBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_accumulation2Buffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_depthBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_groupBuffer, this.m_internalAllocatedCapacity);
    }
    /**
     * Create a particle whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * A simulation step must occur before it's possible to interact
     * with a newly created particle.  For example,
     * DestroyParticleInShape() will not destroy a particle until
     * b2World::Step() has been called.
     *
     * warning: This function is locked during callbacks.
     */
    CreateParticle(def) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (this.m_count >= this.m_internalAllocatedCapacity) {
            // Double the particle capacity.
            const capacity = this.m_count ? 2 * this.m_count : b2_minParticleSystemBufferCapacity;
            this.ReallocateInternalAllocatedBuffers(capacity);
        }
        if (this.m_count >= this.m_internalAllocatedCapacity) {
            // If the oldest particle should be destroyed...
            if (this.m_def.destroyByAge) {
                this.DestroyOldestParticle(0, false);
                // Need to destroy this particle *now* so that it's possible to
                // create a new particle.
                this.SolveZombie();
            }
            else {
                return b2_invalidParticleIndex;
            }
        }
        const index = this.m_count++;
        this.m_flagsBuffer.data[index] = 0;
        if (this.m_lastBodyContactStepBuffer.data) {
            this.m_lastBodyContactStepBuffer.data[index] = 0;
        }
        if (this.m_bodyContactCountBuffer.data) {
            this.m_bodyContactCountBuffer.data[index] = 0;
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            this.m_consecutiveContactStepsBuffer.data[index] = 0;
        }
        this.m_positionBuffer.data[index] = (this.m_positionBuffer.data[index] || new b2Vec2()).Copy(b2Maybe(def.position, b2Vec2.ZERO));
        this.m_velocityBuffer.data[index] = (this.m_velocityBuffer.data[index] || new b2Vec2()).Copy(b2Maybe(def.velocity, b2Vec2.ZERO));
        this.m_weightBuffer[index] = 0;
        this.m_forceBuffer[index] = (this.m_forceBuffer[index] || new b2Vec2()).SetZero();
        if (this.m_staticPressureBuffer) {
            this.m_staticPressureBuffer[index] = 0;
        }
        if (this.m_depthBuffer) {
            this.m_depthBuffer[index] = 0;
        }
        const color = new b2Color().Copy(b2Maybe(def.color, b2Color.ZERO));
        if (this.m_colorBuffer.data || !color.IsZero()) {
            this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
            this.m_colorBuffer.data[index] = (this.m_colorBuffer.data[index] || new b2Color()).Copy(color);
        }
        if (this.m_userDataBuffer.data || def.userData) {
            this.m_userDataBuffer.data = this.RequestBuffer(this.m_userDataBuffer.data);
            this.m_userDataBuffer.data[index] = def.userData;
        }
        if (this.m_handleIndexBuffer.data) {
            this.m_handleIndexBuffer.data[index] = null;
        }
        ///Proxy& proxy = m_proxyBuffer.Append();
        const proxy = this.m_proxyBuffer.data[this.m_proxyBuffer.Append()];
        // If particle lifetimes are enabled or the lifetime is set in the particle
        // definition, initialize the lifetime.
        const lifetime = b2Maybe(def.lifetime, 0.0);
        const finiteLifetime = lifetime > 0.0;
        if (this.m_expirationTimeBuffer.data || finiteLifetime) {
            this.SetParticleLifetime(index, finiteLifetime ? lifetime : this.ExpirationTimeToLifetime(-this.GetQuantizedTimeElapsed()));
            // Add a reference to the newly added particle to the end of the
            // queue.
            this.m_indexByExpirationTimeBuffer.data[index] = index;
        }
        proxy.index = index;
        const group = b2Maybe(def.group, null);
        this.m_groupBuffer[index] = group;
        if (group) {
            if (group.m_firstIndex < group.m_lastIndex) {
                // Move particles in the group just before the new particle.
                this.RotateBuffer(group.m_firstIndex, group.m_lastIndex, index);
                !!B2_DEBUG && b2Assert(group.m_lastIndex === index);
                // Update the index range of the group to contain the new particle.
                group.m_lastIndex = index + 1;
            }
            else {
                // If the group is empty, reset the index range to contain only the
                // new particle.
                group.m_firstIndex = index;
                group.m_lastIndex = index + 1;
            }
        }
        this.SetParticleFlags(index, b2Maybe(def.flags, 0));
        return index;
    }
    /**
     * Retrieve a handle to the particle at the specified index.
     *
     * Please see #b2ParticleHandle for why you might want a handle.
     */
    GetParticleHandleFromIndex(index) {
        !!B2_DEBUG &&
            b2Assert(index >= 0 && index < this.GetParticleCount() && index !== b2_invalidParticleIndex);
        this.m_handleIndexBuffer.data = this.RequestBuffer(this.m_handleIndexBuffer.data);
        let handle = this.m_handleIndexBuffer.data[index];
        if (handle) {
            return handle;
        }
        // Create a handle.
        ///handle = m_handleAllocator.Allocate();
        handle = new b2ParticleHandle();
        !!B2_DEBUG && b2Assert(handle !== null);
        handle.index = index;
        this.m_handleIndexBuffer.data[index] = handle;
        return handle;
    }
    /**
     * Destroy a particle.
     *
     * The particle is removed after the next simulation step (see
     * b2World::Step()).
     *
     * @param index Index of the particle to destroy.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    DestroyParticle(index, callDestructionListener = false) {
        let flags = 2 /* b2_zombieParticle */;
        if (callDestructionListener) {
            flags |= 512 /* b2_destructionListenerParticle */;
        }
        this.SetParticleFlags(index, this.m_flagsBuffer.data[index] | flags);
    }
    /**
     * Destroy the Nth oldest particle in the system.
     *
     * The particle is removed after the next b2World::Step().
     *
     * @param index Index of the Nth oldest particle to
     *      destroy, 0 will destroy the oldest particle in the
     *      system, 1 will destroy the next oldest particle etc.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    DestroyOldestParticle(index, callDestructionListener = false) {
        const particleCount = this.GetParticleCount();
        !!B2_DEBUG && b2Assert(index >= 0 && index < particleCount);
        // Make sure particle lifetime tracking is enabled.
        !!B2_DEBUG && b2Assert(this.m_indexByExpirationTimeBuffer.data !== null);
        // Destroy the oldest particle (preferring to destroy finite
        // lifetime particles first) to free a slot in the buffer.
        const oldestFiniteLifetimeParticle = this.m_indexByExpirationTimeBuffer.data[particleCount - (index + 1)];
        const oldestInfiniteLifetimeParticle = this.m_indexByExpirationTimeBuffer.data[index];
        this.DestroyParticle(this.m_expirationTimeBuffer.data[oldestFiniteLifetimeParticle] > 0.0
            ? oldestFiniteLifetimeParticle
            : oldestInfiniteLifetimeParticle, callDestructionListener);
    }
    /**
     * Destroy particles inside a shape.
     *
     * warning: This function is locked during callbacks.
     *
     * In addition, this function immediately destroys particles in
     * the shape in constrast to DestroyParticle() which defers the
     * destruction until the next simulation step.
     *
     * @return Number of particles destroyed.
     * @param shape Shape which encloses particles
     *      that should be destroyed.
     * @param xf Transform applied to the shape.
     * @param callDestructionListener Whether to call the
     *      world b2DestructionListener for each particle
     *      destroyed.
     */
    DestroyParticlesInShape(shape, xf, callDestructionListener = false) {
        const s_aabb = b2ParticleSystem.DestroyParticlesInShape_s_aabb;
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        const callback = new b2ParticleSystem_DestroyParticlesInShapeCallback(this, shape, xf, callDestructionListener);
        const aabb = s_aabb;
        shape.ComputeAABB(aabb, xf, 0);
        this.m_world.QueryAABB(callback, aabb);
        return callback.Destroyed();
    }
    /**
     * Create a particle group whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * warning: This function is locked during callbacks.
     */
    CreateParticleGroup(groupDef) {
        const s_transform = b2ParticleSystem.CreateParticleGroup_s_transform;
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        const transform = s_transform;
        transform.SetPositionAngle(b2Maybe(groupDef.position, b2Vec2.ZERO), b2Maybe(groupDef.angle, 0));
        const firstIndex = this.m_count;
        if (groupDef.shape) {
            this.CreateParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
        }
        if (groupDef.shapes) {
            this.CreateParticlesWithShapesForGroup(groupDef.shapes, b2Maybe(groupDef.shapeCount, groupDef.shapes.length), groupDef, transform);
        }
        if (groupDef.positionData) {
            const count = b2Maybe(groupDef.particleCount, groupDef.positionData.length);
            for (let i = 0; i < count; i++) {
                const p = groupDef.positionData[i];
                this.CreateParticleForGroup(groupDef, transform, p);
            }
        }
        const lastIndex = this.m_count;
        let group = new b2ParticleGroup(this);
        group.m_firstIndex = firstIndex;
        group.m_lastIndex = lastIndex;
        group.m_strength = b2Maybe(groupDef.strength, 1);
        group.m_userData = groupDef.userData;
        group.m_transform.Copy(transform);
        group.m_prev = null;
        group.m_next = this.m_groupList;
        if (this.m_groupList) {
            this.m_groupList.m_prev = group;
        }
        this.m_groupList = group;
        ++this.m_groupCount;
        for (let i = firstIndex; i < lastIndex; i++) {
            this.m_groupBuffer[i] = group;
        }
        this.SetGroupFlags(group, b2Maybe(groupDef.groupFlags, 0));
        // Create pairs and triads between particles in the group.
        const filter = new b2ParticleSystem_ConnectionFilter();
        this.UpdateContacts(true);
        this.UpdatePairsAndTriads(firstIndex, lastIndex, filter);
        if (groupDef.group) {
            this.JoinParticleGroups(groupDef.group, group);
            group = groupDef.group;
        }
        return group;
    }
    /**
     * Join two particle groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param groupA the first group. Expands to encompass the second group.
     * @param groupB the second group. It is destroyed.
     */
    JoinParticleGroups(groupA, groupB) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        !!B2_DEBUG && b2Assert(groupA !== groupB);
        this.RotateBuffer(groupB.m_firstIndex, groupB.m_lastIndex, this.m_count);
        !!B2_DEBUG && b2Assert(groupB.m_lastIndex === this.m_count);
        this.RotateBuffer(groupA.m_firstIndex, groupA.m_lastIndex, groupB.m_firstIndex);
        !!B2_DEBUG && b2Assert(groupA.m_lastIndex === groupB.m_firstIndex);
        // Create pairs and triads connecting groupA and groupB.
        const filter = new b2ParticleSystem_JoinParticleGroupsFilter(groupB.m_firstIndex);
        this.UpdateContacts(true);
        this.UpdatePairsAndTriads(groupA.m_firstIndex, groupB.m_lastIndex, filter);
        for (let i = groupB.m_firstIndex; i < groupB.m_lastIndex; i++) {
            this.m_groupBuffer[i] = groupA;
        }
        const groupFlags = groupA.m_groupFlags | groupB.m_groupFlags;
        this.SetGroupFlags(groupA, groupFlags);
        groupA.m_lastIndex = groupB.m_lastIndex;
        groupB.m_firstIndex = groupB.m_lastIndex;
        this.DestroyParticleGroup(groupB);
    }
    /**
     * Split particle group into multiple disconnected groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param group the group to be split.
     */
    SplitParticleGroup(group) {
        this.UpdateContacts(true);
        const particleCount = group.GetParticleCount();
        // We create several linked lists. Each list represents a set of connected particles.
        const nodeBuffer = b2MakeArray(particleCount, (index) => new b2ParticleSystem_ParticleListNode());
        b2ParticleSystem.InitializeParticleLists(group, nodeBuffer);
        this.MergeParticleListsInContact(group, nodeBuffer);
        const survivingList = b2ParticleSystem.FindLongestParticleList(group, nodeBuffer);
        this.MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
        this.CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
        this.UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
    }
    /**
     * Get the world particle group list. With the returned group,
     * use b2ParticleGroup::GetNext to get the next group in the
     * world list.
     *
     * A null group indicates the end of the list.
     *
     * @return the head of the world particle group list.
     */
    GetParticleGroupList() {
        return this.m_groupList;
    }
    /**
     * Get the number of particle groups.
     */
    GetParticleGroupCount() {
        return this.m_groupCount;
    }
    /**
     * Get the number of particles.
     */
    GetParticleCount() {
        return this.m_count;
    }
    /**
     * Get the maximum number of particles.
     */
    GetMaxParticleCount() {
        return this.m_def.maxCount;
    }
    /**
     * Set the maximum number of particles.
     *
     * A value of 0 means there is no maximum. The particle buffers
     * can continue to grow while b2World's block allocator still
     * has memory.
     *
     * Note: If you try to CreateParticle() with more than this
     * count, b2_invalidParticleIndex is returned unless
     * SetDestructionByAge() is used to enable the destruction of
     * the oldest particles in the system.
     */
    SetMaxParticleCount(count) {
        !!B2_DEBUG && b2Assert(this.m_count <= count);
        this.m_def.maxCount = count;
    }
    /**
     * Get all existing particle flags.
     */
    GetAllParticleFlags() {
        return this.m_allParticleFlags;
    }
    /**
     * Get all existing particle group flags.
     */
    GetAllGroupFlags() {
        return this.m_allGroupFlags;
    }
    /**
     * Pause or unpause the particle system. When paused,
     * b2World::Step() skips over this particle system. All
     * b2ParticleSystem function calls still work.
     *
     * @param paused paused is true to pause, false to un-pause.
     */
    SetPaused(paused) {
        this.m_paused = paused;
    }
    /**
     * Initially, true, then, the last value passed into
     * SetPaused().
     *
     * @return true if the particle system is being updated in b2World::Step().
     */
    GetPaused() {
        return this.m_paused;
    }
    /**
     * Change the particle density.
     *
     * Particle density affects the mass of the particles, which in
     * turn affects how the particles interact with b2Bodies. Note
     * that the density does not affect how the particles interact
     * with each other.
     */
    SetDensity(density) {
        this.m_def.density = density;
        this.m_inverseDensity = 1 / this.m_def.density;
    }
    /**
     * Get the particle density.
     */
    GetDensity() {
        return this.m_def.density;
    }
    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles.
     */
    SetGravityScale(gravityScale) {
        this.m_def.gravityScale = gravityScale;
    }
    /**
     * Get the particle gravity scale.
     */
    GetGravityScale() {
        return this.m_def.gravityScale;
    }
    /**
     * Damping is used to reduce the velocity of particles. The
     * damping parameter can be larger than 1.0f but the damping
     * effect becomes sensitive to the time step when the damping
     * parameter is large.
     */
    SetDamping(damping) {
        this.m_def.dampingStrength = damping;
    }
    /**
     * Get damping for particles
     */
    GetDamping() {
        return this.m_def.dampingStrength;
    }
    /**
     * Change the number of iterations when calculating the static
     * pressure of particles. By default, 8 iterations. You can
     * reduce the number of iterations down to 1 in some situations,
     * but this may cause instabilities when many particles come
     * together. If you see particles popping away from each other
     * like popcorn, you may have to increase the number of
     * iterations.
     *
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    SetStaticPressureIterations(iterations) {
        this.m_def.staticPressureIterations = iterations;
    }
    /**
     * Get the number of iterations for static pressure of
     * particles.
     */
    GetStaticPressureIterations() {
        return this.m_def.staticPressureIterations;
    }
    /**
     * Change the particle radius.
     *
     * You should set this only once, on world start.
     * If you change the radius during execution, existing particles
     * may explode, shrink, or behave unexpectedly.
     */
    SetRadius(radius) {
        this.m_particleDiameter = 2 * radius;
        this.m_squaredDiameter = this.m_particleDiameter * this.m_particleDiameter;
        this.m_inverseDiameter = 1 / this.m_particleDiameter;
    }
    /**
     * Get the particle radius.
     */
    GetRadius() {
        return this.m_particleDiameter / 2;
    }
    /**
     * Get the position of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    GetPositionBuffer() {
        return this.m_positionBuffer.data;
    }
    /**
     * Get the velocity of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle velocities array.
     */
    GetVelocityBuffer() {
        return this.m_velocityBuffer.data;
    }
    /**
     * Get the color of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle colors array.
     */
    GetColorBuffer() {
        this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
        return this.m_colorBuffer.data;
    }
    /**
     * Get the particle-group of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle group array.
     */
    GetGroupBuffer() {
        return this.m_groupBuffer;
    }
    /**
     * Get the weight of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    GetWeightBuffer() {
        return this.m_weightBuffer;
    }
    /**
     * Get the user-specified data of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle user-data array.
     */
    GetUserDataBuffer() {
        this.m_userDataBuffer.data = this.RequestBuffer(this.m_userDataBuffer.data);
        return this.m_userDataBuffer.data;
    }
    /**
     * Get the flags for each particle. See the b2ParticleFlag enum.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle-flags array.
     */
    GetFlagsBuffer() {
        return this.m_flagsBuffer.data;
    }
    /**
     * Set flags for a particle. See the b2ParticleFlag enum.
     */
    SetParticleFlags(index, newFlags) {
        const oldFlags = this.m_flagsBuffer.data[index];
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.m_needsUpdateAllParticleFlags = true;
        }
        if (~this.m_allParticleFlags & newFlags) {
            // If any flags were added
            if (newFlags & 128 /* b2_tensileParticle */) {
                this.m_accumulation2Buffer = this.RequestBuffer(this.m_accumulation2Buffer);
            }
            if (newFlags & 256 /* b2_colorMixingParticle */) {
                this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
            }
            this.m_allParticleFlags |= newFlags;
        }
        this.m_flagsBuffer.data[index] = newFlags;
    }
    /**
     * Get flags for a particle. See the b2ParticleFlag enum.
     */
    GetParticleFlags(index) {
        return this.m_flagsBuffer.data[index];
    }
    /**
     * Set an external buffer for particle data.
     *
     * Normally, the b2World's block allocator is used for particle
     * data. However, sometimes you may have an OpenGL or Java
     * buffer for particle data. To avoid data duplication, you may
     * supply this external buffer.
     *
     * Note that, when b2World's block allocator is used, the
     * particle data buffers can grow as required. However, when
     * external buffers are used, the maximum number of particles is
     * clamped to the size of the smallest external buffer.
     *
     * @param buffer a pointer to a block of memory.
     * @param capacity the number of values in the block.
     */
    SetFlagsBuffer(buffer) {
        this.SetUserOverridableBuffer(this.m_flagsBuffer, buffer);
    }
    SetPositionBuffer(buffer) {
        if (buffer instanceof Float32Array) {
            if ((buffer.length & 1) !== 0) {
                throw new Error();
            }
            const count = buffer.length / 2;
            const array = [];
            let ptr = 0;
            for (let i = 0; i < count; ++i) {
                array.push(new b2Vec2(buffer[ptr++], buffer[ptr++]));
            }
            buffer = array;
        }
        this.SetUserOverridableBuffer(this.m_positionBuffer, buffer);
    }
    SetVelocityBuffer(buffer) {
        if (buffer instanceof Float32Array) {
            if ((buffer.length & 1) !== 0) {
                throw new Error();
            }
            const count = buffer.length / 2;
            const array = [];
            let ptr = 0;
            for (let i = 0; i < count; ++i) {
                array.push(new b2Vec2(buffer[ptr++], buffer[ptr++]));
            }
            buffer = array;
        }
        this.SetUserOverridableBuffer(this.m_velocityBuffer, buffer);
    }
    SetColorBuffer(buffer) {
        if (buffer instanceof Float32Array) {
            if ((buffer.length & 3) !== 0) {
                throw new Error();
            }
            const count = buffer.length / 4;
            const array = [];
            let ptr = 0;
            for (let i = 0; i < count; ++i) {
                array.push(new b2Color(buffer[ptr++], buffer[ptr++], buffer[ptr++], buffer[ptr++]));
            }
            buffer = array;
        }
        this.SetUserOverridableBuffer(this.m_colorBuffer, buffer);
    }
    SetUserDataBuffer(buffer) {
        this.SetUserOverridableBuffer(this.m_userDataBuffer, buffer);
    }
    /**
     * Get contacts between particles
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    GetContacts() {
        return this.m_contactBuffer.data;
    }
    GetContactCount() {
        return this.m_contactBuffer.count;
    }
    /**
     * Get contacts between particles and bodies
     *
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    GetBodyContacts() {
        return this.m_bodyContactBuffer.data;
    }
    GetBodyContactCount() {
        return this.m_bodyContactBuffer.count;
    }
    /**
     * Get array of particle pairs. The particles in a pair:
     *   (1) are contacting,
     *   (2) are in the same particle group,
     *   (3) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (4) have at least one particle that is a spring or barrier
     *       particle (i.e. one of the types in k_pairFlags),
     *   (5) have at least one particle that returns true for
     *       ConnectionFilter::IsNecessary,
     *   (6) are not zombie particles.
     *
     * Essentially, this is an array of spring or barrier particles
     * that are interacting. The array is sorted by b2ParticlePair's
     * indexA, and then indexB. There are no duplicate entries.
     */
    GetPairs() {
        return this.m_pairBuffer.data;
    }
    GetPairCount() {
        return this.m_pairBuffer.count;
    }
    /**
     * Get array of particle triads. The particles in a triad:
     *   (1) are in the same particle group,
     *   (2) are in a Voronoi triangle together,
     *   (3) are within b2_maxTriadDistance particle diameters of each
     *       other,
     *   (4) return true for ConnectionFilter::ShouldCreateTriad
     *   (5) have at least one particle of type elastic (i.e. one of the
     *       types in k_triadFlags),
     *   (6) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (7) are not zombie particles.
     *
     * Essentially, this is an array of elastic particles that are
     * interacting. The array is sorted by b2ParticleTriad's indexA,
     * then indexB, then indexC. There are no duplicate entries.
     */
    GetTriads() {
        return this.m_triadBuffer.data;
    }
    GetTriadCount() {
        return this.m_triadBuffer.count;
    }
    /**
     * Set an optional threshold for the maximum number of
     * consecutive particle iterations that a particle may contact
     * multiple bodies before it is considered a candidate for being
     * "stuck". Setting to zero or less disables.
     */
    SetStuckThreshold(steps) {
        this.m_stuckThreshold = steps;
        if (steps > 0) {
            this.m_lastBodyContactStepBuffer.data = this.RequestBuffer(this.m_lastBodyContactStepBuffer.data);
            this.m_bodyContactCountBuffer.data = this.RequestBuffer(this.m_bodyContactCountBuffer.data);
            this.m_consecutiveContactStepsBuffer.data = this.RequestBuffer(this.m_consecutiveContactStepsBuffer.data);
        }
    }
    /**
     * Get potentially stuck particles from the last step; the user
     * must decide if they are stuck or not, and if so, delete or
     * move them
     */
    GetStuckCandidates() {
        ///return m_stuckParticleBuffer.Data();
        return this.m_stuckParticleBuffer.Data();
    }
    /**
     * Get the number of stuck particle candidates from the last
     * step.
     */
    GetStuckCandidateCount() {
        ///return m_stuckParticleBuffer.GetCount();
        return this.m_stuckParticleBuffer.GetCount();
    }
    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    ComputeCollisionEnergy() {
        const s_v = b2ParticleSystem.ComputeCollisionEnergy_s_v;
        const vel_data = this.m_velocityBuffer.data;
        let sum_v2 = 0;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const n = contact.normal;
            ///b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
            const v = b2Vec2.SubVV(vel_data[b], vel_data[a], s_v);
            const vn = b2Vec2.DotVV(v, n);
            if (vn < 0) {
                sum_v2 += vn * vn;
            }
        }
        return 0.5 * this.GetParticleMass() * sum_v2;
    }
    /**
     * Set strict Particle/Body contact check.
     *
     * This is an option that will help ensure correct behavior if
     * there are corners in the world model where Particle/Body
     * contact is ambiguous. This option scales at n*log(n) of the
     * number of Particle/Body contacts, so it is best to only
     * enable if it is necessary for your geometry. Enable if you
     * see strange particle behavior around b2Body intersections.
     */
    SetStrictContactCheck(enabled) {
        this.m_def.strictContactCheck = enabled;
    }
    /**
     * Get the status of the strict contact check.
     */
    GetStrictContactCheck() {
        return this.m_def.strictContactCheck;
    }
    /**
     * Set the lifetime (in seconds) of a particle relative to the
     * current time.  A lifetime of less than or equal to 0.0f
     * results in the particle living forever until it's manually
     * destroyed by the application.
     */
    SetParticleLifetime(index, lifetime) {
        !!B2_DEBUG && b2Assert(this.ValidateParticleIndex(index));
        const initializeExpirationTimes = this.m_indexByExpirationTimeBuffer.data === null;
        this.m_expirationTimeBuffer.data = this.RequestBuffer(this.m_expirationTimeBuffer.data);
        this.m_indexByExpirationTimeBuffer.data = this.RequestBuffer(this.m_indexByExpirationTimeBuffer.data);
        // Initialize the inverse mapping buffer.
        if (initializeExpirationTimes) {
            const particleCount = this.GetParticleCount();
            for (let i = 0; i < particleCount; ++i) {
                this.m_indexByExpirationTimeBuffer.data[i] = i;
            }
        }
        ///const int32 quantizedLifetime = (int32)(lifetime / m_def.lifetimeGranularity);
        const quantizedLifetime = lifetime / this.m_def.lifetimeGranularity;
        // Use a negative lifetime so that it's possible to track which
        // of the infinite lifetime particles are older.
        const newExpirationTime = quantizedLifetime > 0.0
            ? this.GetQuantizedTimeElapsed() + quantizedLifetime
            : quantizedLifetime;
        if (newExpirationTime !== this.m_expirationTimeBuffer.data[index]) {
            this.m_expirationTimeBuffer.data[index] = newExpirationTime;
            this.m_expirationTimeBufferRequiresSorting = true;
        }
    }
    /**
     * Get the lifetime (in seconds) of a particle relative to the
     * current time.  A value > 0.0f is returned if the particle is
     * scheduled to be destroyed in the future, values <= 0.0f
     * indicate the particle has an infinite lifetime.
     */
    GetParticleLifetime(index) {
        !!B2_DEBUG && b2Assert(this.ValidateParticleIndex(index));
        return this.ExpirationTimeToLifetime(this.GetExpirationTimeBuffer()[index]);
    }
    /**
     * Enable / disable destruction of particles in CreateParticle()
     * when no more particles can be created due to a prior call to
     * SetMaxParticleCount().  When this is enabled, the oldest
     * particle is destroyed in CreateParticle() favoring the
     * destruction of particles with a finite lifetime over
     * particles with infinite lifetimes. This feature is enabled by
     * default when particle lifetimes are tracked.  Explicitly
     * enabling this feature using this function enables particle
     * lifetime tracking.
     */
    SetDestructionByAge(enable) {
        if (enable) {
            this.GetExpirationTimeBuffer();
        }
        this.m_def.destroyByAge = enable;
    }
    /**
     * Get whether the oldest particle will be destroyed in
     * CreateParticle() when the maximum number of particles are
     * present in the system.
     */
    GetDestructionByAge() {
        return this.m_def.destroyByAge;
    }
    /**
     * Get the array of particle expiration times indexed by
     * particle index.
     *
     * GetParticleCount() items are in the returned array.
     */
    GetExpirationTimeBuffer() {
        this.m_expirationTimeBuffer.data = this.RequestBuffer(this.m_expirationTimeBuffer.data);
        return this.m_expirationTimeBuffer.data;
    }
    /**
     * Convert a expiration time value in returned by
     * GetExpirationTimeBuffer() to a time in seconds relative to
     * the current simulation time.
     */
    ExpirationTimeToLifetime(expirationTime) {
        return ((expirationTime > 0 ? expirationTime - this.GetQuantizedTimeElapsed() : expirationTime) *
            this.m_def.lifetimeGranularity);
    }
    /**
     * Get the array of particle indices ordered by reverse
     * lifetime. The oldest particle indexes are at the end of the
     * array with the newest at the start.  Particles with infinite
     * lifetimes (i.e expiration times less than or equal to 0) are
     * placed at the start of the array.
     * ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]) is
     * equivalent to GetParticleLifetime(index).
     *
     * GetParticleCount() items are in the returned array.
     */
    GetIndexByExpirationTimeBuffer() {
        // If particles are present, initialize / reinitialize the lifetime buffer.
        if (this.GetParticleCount()) {
            this.SetParticleLifetime(0, this.GetParticleLifetime(0));
        }
        else {
            this.m_indexByExpirationTimeBuffer.data = this.RequestBuffer(this.m_indexByExpirationTimeBuffer.data);
        }
        return this.m_indexByExpirationTimeBuffer.data;
    }
    /**
     * Apply an impulse to one particle. This immediately modifies
     * the velocity. Similar to b2Body::ApplyLinearImpulse.
     *
     * @param index the particle that will be modified.
     * @param impulse impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    ParticleApplyLinearImpulse(index, impulse) {
        this.ApplyLinearImpulse(index, index + 1, impulse);
    }
    /**
     * Apply an impulse to all particles between 'firstIndex' and
     * 'lastIndex'. This immediately modifies the velocity. Note
     * that the impulse is applied to the total mass of all
     * particles. So, calling ParticleApplyLinearImpulse(0, impulse)
     * and ParticleApplyLinearImpulse(1, impulse) will impart twice
     * as much velocity as calling just ApplyLinearImpulse(0, 1,
     * impulse).
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    ApplyLinearImpulse(firstIndex, lastIndex, impulse) {
        const vel_data = this.m_velocityBuffer.data;
        const numParticles = lastIndex - firstIndex;
        const totalMass = numParticles * this.GetParticleMass();
        ///const b2Vec2 velocityDelta = impulse / totalMass;
        const velocityDelta = new b2Vec2().Copy(impulse).SelfMul(1 / totalMass);
        for (let i = firstIndex; i < lastIndex; i++) {
            ///m_velocityBuffer.data[i] += velocityDelta;
            vel_data[i].SelfAdd(velocityDelta);
        }
    }
    static IsSignificantForce(force) {
        return force.x !== 0 || force.y !== 0;
    }
    /**
     * Apply a force to the center of a particle.
     *
     * @param index the particle that will be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    ParticleApplyForce(index, force) {
        if (b2ParticleSystem.IsSignificantForce(force) &&
            this.ForceCanBeApplied(this.m_flagsBuffer.data[index])) {
            this.PrepareForceBuffer();
            ///m_forceBuffer[index] += force;
            this.m_forceBuffer[index].SelfAdd(force);
        }
    }
    /**
     * Distribute a force across several particles. The particles
     * must not be wall particles. Note that the force is
     * distributed across all the particles, so calling this
     * function for indices 0..N is not the same as calling
     * ParticleApplyForce(i, force) for i in 0..N.
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    ApplyForce(firstIndex, lastIndex, force) {
        // Ensure we're not trying to apply force to particles that can't move,
        // such as wall particles.
        if (B2_DEBUG) {
            let flags = 0;
            for (let i = firstIndex; i < lastIndex; i++) {
                flags |= this.m_flagsBuffer.data[i];
            }
            b2Assert(this.ForceCanBeApplied(flags));
        }
        // Early out if force does nothing (optimization).
        ///const b2Vec2 distributedForce = force / (float32)(lastIndex - firstIndex);
        const distributedForce = new b2Vec2().Copy(force).SelfMul(1 / (lastIndex - firstIndex));
        if (b2ParticleSystem.IsSignificantForce(distributedForce)) {
            this.PrepareForceBuffer();
            // Distribute the force over all the particles.
            for (let i = firstIndex; i < lastIndex; i++) {
                ///m_forceBuffer[i] += distributedForce;
                this.m_forceBuffer[i].SelfAdd(distributedForce);
            }
        }
    }
    /**
     * Get the next particle-system in the world's particle-system
     * list.
     */
    GetNext() {
        return this.m_next;
    }
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided AABB.
     * b2QueryCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    QueryAABB(callback, aabb) {
        if (this.m_proxyBuffer.count === 0) {
            return;
        }
        const beginProxy = 0;
        const endProxy = this.m_proxyBuffer.count;
        const firstProxy = std_lower_bound(this.m_proxyBuffer.data, beginProxy, endProxy, b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.lowerBound.x, this.m_inverseDiameter * aabb.lowerBound.y), b2ParticleSystem_Proxy.CompareProxyTag);
        const lastProxy = std_upper_bound(this.m_proxyBuffer.data, firstProxy, endProxy, b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.upperBound.x, this.m_inverseDiameter * aabb.upperBound.y), b2ParticleSystem_Proxy.CompareTagProxy);
        const pos_data = this.m_positionBuffer.data;
        for (let k = firstProxy; k < lastProxy; ++k) {
            const proxy = this.m_proxyBuffer.data[k];
            const i = proxy.index;
            const p = pos_data[i];
            if (aabb.lowerBound.x < p.x &&
                p.x < aabb.upperBound.x &&
                aabb.lowerBound.y < p.y &&
                p.y < aabb.upperBound.y) {
                if (!callback.ReportParticle(this, i)) {
                    break;
                }
            }
        }
    }
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided shape's AABB. Calls QueryAABB
     * internally. b2QueryCallback::ShouldQueryParticleSystem is
     * ignored.
     *
     * @param callback a user implemented callback class.
     * @param shape the query shape
     * @param xf the transform of the AABB
     * @param childIndex
     */
    QueryShapeAABB(callback, shape, xf, childIndex = 0) {
        const s_aabb = b2ParticleSystem.QueryShapeAABB_s_aabb;
        const aabb = s_aabb;
        shape.ComputeAABB(aabb, xf, childIndex);
        this.QueryAABB(callback, aabb);
    }
    QueryPointAABB(callback, point, slop = b2_linearSlop) {
        const s_aabb = b2ParticleSystem.QueryPointAABB_s_aabb;
        const aabb = s_aabb;
        aabb.lowerBound.Set(point.x - slop, point.y - slop);
        aabb.upperBound.Set(point.x + slop, point.y + slop);
        this.QueryAABB(callback, aabb);
    }
    /**
     * Ray-cast the particle system for all particles in the path of
     * the ray. Your callback controls whether you get the closest
     * point, any point, or n-points. The ray-cast ignores particles
     * that contain the starting point.
     * b2RayCastCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    RayCast(callback, point1, point2) {
        const s_aabb = b2ParticleSystem.RayCast_s_aabb;
        const s_p = b2ParticleSystem.RayCast_s_p;
        const s_v = b2ParticleSystem.RayCast_s_v;
        const s_n = b2ParticleSystem.RayCast_s_n;
        const s_point = b2ParticleSystem.RayCast_s_point;
        if (this.m_proxyBuffer.count === 0) {
            return;
        }
        const pos_data = this.m_positionBuffer.data;
        const aabb = s_aabb;
        b2Vec2.MinV(point1, point2, aabb.lowerBound);
        b2Vec2.MaxV(point1, point2, aabb.upperBound);
        let fraction = 1;
        // solving the following equation:
        // ((1-t)*point1+t*point2-position)^2=diameter^2
        // where t is a potential fraction
        ///b2Vec2 v = point2 - point1;
        const v = b2Vec2.SubVV(point2, point1, s_v);
        const v2 = b2Vec2.DotVV(v, v);
        const enumerator = this.GetInsideBoundsEnumerator(aabb);
        let i;
        while ((i = enumerator.GetNext()) >= 0) {
            ///b2Vec2 p = point1 - m_positionBuffer.data[i];
            const p = b2Vec2.SubVV(point1, pos_data[i], s_p);
            const pv = b2Vec2.DotVV(p, v);
            const p2 = b2Vec2.DotVV(p, p);
            const determinant = pv * pv - v2 * (p2 - this.m_squaredDiameter);
            if (determinant >= 0) {
                const sqrtDeterminant = b2Sqrt(determinant);
                // find a solution between 0 and fraction
                let t = (-pv - sqrtDeterminant) / v2;
                if (t > fraction) {
                    continue;
                }
                if (t < 0) {
                    t = (-pv + sqrtDeterminant) / v2;
                    if (t < 0 || t > fraction) {
                        continue;
                    }
                }
                ///b2Vec2 n = p + t * v;
                const n = b2Vec2.AddVMulSV(p, t, v, s_n);
                n.Normalize();
                ///float32 f = callback.ReportParticle(this, i, point1 + t * v, n, t);
                const f = callback.ReportParticle(this, i, b2Vec2.AddVMulSV(point1, t, v, s_point), n, t);
                fraction = b2Min(fraction, f);
                if (fraction <= 0) {
                    break;
                }
            }
        }
    }
    /**
     * Compute the axis-aligned bounding box for all particles
     * contained within this particle system.
     * @param aabb Returns the axis-aligned bounding box of the system.
     */
    ComputeAABB(aabb) {
        const particleCount = this.GetParticleCount();
        !!B2_DEBUG && b2Assert(aabb !== null);
        aabb.lowerBound.x = +b2_maxFloat;
        aabb.lowerBound.y = +b2_maxFloat;
        aabb.upperBound.x = -b2_maxFloat;
        aabb.upperBound.y = -b2_maxFloat;
        const pos_data = this.m_positionBuffer.data;
        for (let i = 0; i < particleCount; i++) {
            const p = pos_data[i];
            b2Vec2.MinV(aabb.lowerBound, p, aabb.lowerBound);
            b2Vec2.MaxV(aabb.upperBound, p, aabb.upperBound);
        }
        aabb.lowerBound.x -= this.m_particleDiameter;
        aabb.lowerBound.y -= this.m_particleDiameter;
        aabb.upperBound.x += this.m_particleDiameter;
        aabb.upperBound.y += this.m_particleDiameter;
    }
    FreeBuffer(b, capacity) {
        if (b === null) {
            return;
        }
        b.length = 0;
    }
    FreeUserOverridableBuffer(b) {
        if (b.userSuppliedCapacity === 0) {
            this.FreeBuffer(b.data, this.m_internalAllocatedCapacity);
        }
    }
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer3(oldBuffer, oldCapacity, newCapacity) {
        // b2Assert(newCapacity > oldCapacity);
        if (newCapacity <= oldCapacity) {
            throw new Error();
        }
        const newBuffer = oldBuffer ? oldBuffer.slice() : [];
        newBuffer.length = newCapacity;
        return newBuffer;
    }
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer5(buffer, userSuppliedCapacity, oldCapacity, newCapacity, deferred) {
        // b2Assert(newCapacity > oldCapacity);
        if (newCapacity <= oldCapacity) {
            throw new Error();
        }
        // A 'deferred' buffer is reallocated only if it is not NULL.
        // If 'userSuppliedCapacity' is not zero, buffer is user supplied and must
        // be kept.
        // b2Assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
        if (!(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity)) {
            throw new Error();
        }
        if ((!deferred || buffer) && !userSuppliedCapacity) {
            buffer = this.ReallocateBuffer3(buffer, oldCapacity, newCapacity);
        }
        return buffer; // TODO: fix this
    }
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer4(buffer, oldCapacity, newCapacity, deferred) {
        !!B2_DEBUG && b2Assert(newCapacity > oldCapacity);
        return this.ReallocateBuffer5(buffer.data, buffer.userSuppliedCapacity, oldCapacity, newCapacity, deferred);
    }
    RequestBuffer(buffer) {
        if (!buffer) {
            if (this.m_internalAllocatedCapacity === 0) {
                this.ReallocateInternalAllocatedBuffers(b2_minParticleSystemBufferCapacity);
            }
            buffer = [];
            buffer.length = this.m_internalAllocatedCapacity;
        }
        return buffer;
    }
    /**
     * Reallocate the handle / index map and schedule the allocation
     * of a new pool for handle allocation.
     */
    ReallocateHandleBuffers(newCapacity) {
        !!B2_DEBUG && b2Assert(newCapacity > this.m_internalAllocatedCapacity);
        // Reallocate a new handle / index map buffer, copying old handle pointers
        // is fine since they're kept around.
        this.m_handleIndexBuffer.data = this.ReallocateBuffer4(this.m_handleIndexBuffer, this.m_internalAllocatedCapacity, newCapacity, true);
        // Set the size of the next handle allocation.
        ///this.m_handleAllocator.SetItemsPerSlab(newCapacity - this.m_internalAllocatedCapacity);
    }
    ReallocateInternalAllocatedBuffers(capacity) {
        function LimitCapacity(capacity, maxCount) {
            return maxCount && capacity > maxCount ? maxCount : capacity;
        }
        // Don't increase capacity beyond the smallest user-supplied buffer size.
        capacity = LimitCapacity(capacity, this.m_def.maxCount);
        capacity = LimitCapacity(capacity, this.m_flagsBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_positionBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_velocityBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_colorBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_userDataBuffer.userSuppliedCapacity);
        if (this.m_internalAllocatedCapacity < capacity) {
            this.ReallocateHandleBuffers(capacity);
            this.m_flagsBuffer.data = this.ReallocateBuffer4(this.m_flagsBuffer, this.m_internalAllocatedCapacity, capacity, false);
            // Conditionally defer these as they are optional if the feature is
            // not enabled.
            const stuck = this.m_stuckThreshold > 0;
            this.m_lastBodyContactStepBuffer.data = this.ReallocateBuffer4(this.m_lastBodyContactStepBuffer, this.m_internalAllocatedCapacity, capacity, stuck);
            this.m_bodyContactCountBuffer.data = this.ReallocateBuffer4(this.m_bodyContactCountBuffer, this.m_internalAllocatedCapacity, capacity, stuck);
            this.m_consecutiveContactStepsBuffer.data = this.ReallocateBuffer4(this.m_consecutiveContactStepsBuffer, this.m_internalAllocatedCapacity, capacity, stuck);
            this.m_positionBuffer.data = this.ReallocateBuffer4(this.m_positionBuffer, this.m_internalAllocatedCapacity, capacity, false);
            this.m_velocityBuffer.data = this.ReallocateBuffer4(this.m_velocityBuffer, this.m_internalAllocatedCapacity, capacity, false);
            this.m_forceBuffer = this.ReallocateBuffer5(this.m_forceBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_weightBuffer = this.ReallocateBuffer5(this.m_weightBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_staticPressureBuffer = this.ReallocateBuffer5(this.m_staticPressureBuffer, 0, this.m_internalAllocatedCapacity, capacity, true);
            this.m_accumulationBuffer = this.ReallocateBuffer5(this.m_accumulationBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_accumulation2Buffer = this.ReallocateBuffer5(this.m_accumulation2Buffer, 0, this.m_internalAllocatedCapacity, capacity, true);
            this.m_depthBuffer = this.ReallocateBuffer5(this.m_depthBuffer, 0, this.m_internalAllocatedCapacity, capacity, true);
            this.m_colorBuffer.data = this.ReallocateBuffer4(this.m_colorBuffer, this.m_internalAllocatedCapacity, capacity, true);
            this.m_groupBuffer = this.ReallocateBuffer5(this.m_groupBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_userDataBuffer.data = this.ReallocateBuffer4(this.m_userDataBuffer, this.m_internalAllocatedCapacity, capacity, true);
            this.m_expirationTimeBuffer.data = this.ReallocateBuffer4(this.m_expirationTimeBuffer, this.m_internalAllocatedCapacity, capacity, true);
            this.m_indexByExpirationTimeBuffer.data = this.ReallocateBuffer4(this.m_indexByExpirationTimeBuffer, this.m_internalAllocatedCapacity, capacity, false);
            this.m_internalAllocatedCapacity = capacity;
        }
    }
    CreateParticleForGroup(groupDef, xf, p) {
        const particleDef = new b2ParticleDef();
        particleDef.flags = b2Maybe(groupDef.flags, 0);
        ///particleDef.position = b2Mul(xf, p);
        b2Transform.MulXV(xf, p, particleDef.position);
        ///particleDef.velocity =
        ///  groupDef.linearVelocity +
        ///  b2Cross(groupDef.angularVelocity,
        ///      particleDef.position - groupDef.position);
        b2Vec2.AddVV(b2Maybe(groupDef.linearVelocity, b2Vec2.ZERO), b2Vec2.CrossSV(b2Maybe(groupDef.angularVelocity, 0), b2Vec2.SubVV(particleDef.position, b2Maybe(groupDef.position, b2Vec2.ZERO), b2Vec2.s_t0), b2Vec2.s_t0), particleDef.velocity);
        particleDef.color.Copy(b2Maybe(groupDef.color, b2Color.ZERO));
        particleDef.lifetime = b2Maybe(groupDef.lifetime, 0);
        particleDef.userData = groupDef.userData;
        this.CreateParticle(particleDef);
    }
    CreateParticlesStrokeShapeForGroup(shape, groupDef, xf) {
        const s_edge = b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_edge;
        const s_d = b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_d;
        const s_p = b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_p;
        let stride = b2Maybe(groupDef.stride, 0);
        if (stride === 0) {
            stride = this.GetParticleStride();
        }
        let positionOnEdge = 0;
        const childCount = shape.GetChildCount();
        for (let childIndex = 0; childIndex < childCount; childIndex++) {
            let edge = null;
            if (shape.GetType() === 1 /* e_edgeShape */) {
                edge = shape;
            }
            else {
                !!B2_DEBUG && b2Assert(shape.GetType() === 3 /* e_chainShape */);
                edge = s_edge;
                shape.GetChildEdge(edge, childIndex);
            }
            const d = b2Vec2.SubVV(edge.m_vertex2, edge.m_vertex1, s_d);
            const edgeLength = d.Length();
            while (positionOnEdge < edgeLength) {
                ///b2Vec2 p = edge.m_vertex1 + positionOnEdge / edgeLength * d;
                const p = b2Vec2.AddVMulSV(edge.m_vertex1, positionOnEdge / edgeLength, d, s_p);
                this.CreateParticleForGroup(groupDef, xf, p);
                positionOnEdge += stride;
            }
            positionOnEdge -= edgeLength;
        }
    }
    CreateParticlesFillShapeForGroup(shape, groupDef, xf) {
        const s_aabb = b2ParticleSystem.CreateParticlesFillShapeForGroup_s_aabb;
        const s_p = b2ParticleSystem.CreateParticlesFillShapeForGroup_s_p;
        let stride = b2Maybe(groupDef.stride, 0);
        if (stride === 0) {
            stride = this.GetParticleStride();
        }
        ///b2Transform identity;
        /// identity.SetIdentity();
        const identity = b2Transform.IDENTITY;
        const aabb = s_aabb;
        !!B2_DEBUG && b2Assert(shape.GetChildCount() === 1);
        shape.ComputeAABB(aabb, identity, 0);
        for (let y = Math.floor(aabb.lowerBound.y / stride) * stride; y < aabb.upperBound.y; y += stride) {
            for (let x = Math.floor(aabb.lowerBound.x / stride) * stride; x < aabb.upperBound.x; x += stride) {
                const p = s_p.Set(x, y);
                if (shape.TestPoint(identity, p)) {
                    this.CreateParticleForGroup(groupDef, xf, p);
                }
            }
        }
    }
    CreateParticlesWithShapeForGroup(shape, groupDef, xf) {
        switch (shape.GetType()) {
            case 1 /* e_edgeShape */:
            case 3 /* e_chainShape */:
                this.CreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
                break;
            case 2 /* e_polygonShape */:
            case 0 /* e_circleShape */:
                this.CreateParticlesFillShapeForGroup(shape, groupDef, xf);
                break;
            default:
                !!B2_DEBUG && b2Assert(false);
                break;
        }
    }
    CreateParticlesWithShapesForGroup(shapes, shapeCount, groupDef, xf) {
        const compositeShape = new b2ParticleSystem_CompositeShape(shapes, shapeCount);
        this.CreateParticlesFillShapeForGroup(compositeShape, groupDef, xf);
    }
    CloneParticle(oldIndex, group) {
        const def = new b2ParticleDef();
        def.flags = this.m_flagsBuffer.data[oldIndex];
        def.position.Copy(this.m_positionBuffer.data[oldIndex]);
        def.velocity.Copy(this.m_velocityBuffer.data[oldIndex]);
        if (this.m_colorBuffer.data) {
            def.color.Copy(this.m_colorBuffer.data[oldIndex]);
        }
        if (this.m_userDataBuffer.data) {
            def.userData = this.m_userDataBuffer.data[oldIndex];
        }
        def.group = group;
        const newIndex = this.CreateParticle(def);
        if (this.m_handleIndexBuffer.data) {
            const handle = this.m_handleIndexBuffer.data[oldIndex];
            if (handle) {
                handle.index = newIndex;
            }
            this.m_handleIndexBuffer.data[newIndex] = handle;
            this.m_handleIndexBuffer.data[oldIndex] = null;
        }
        if (this.m_lastBodyContactStepBuffer.data) {
            this.m_lastBodyContactStepBuffer.data[newIndex] = this.m_lastBodyContactStepBuffer.data[oldIndex];
        }
        if (this.m_bodyContactCountBuffer.data) {
            this.m_bodyContactCountBuffer.data[newIndex] = this.m_bodyContactCountBuffer.data[oldIndex];
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            this.m_consecutiveContactStepsBuffer.data[newIndex] = this.m_consecutiveContactStepsBuffer.data[oldIndex];
        }
        if (this.m_hasForce) {
            this.m_forceBuffer[newIndex].Copy(this.m_forceBuffer[oldIndex]);
        }
        if (this.m_staticPressureBuffer) {
            this.m_staticPressureBuffer[newIndex] = this.m_staticPressureBuffer[oldIndex];
        }
        if (this.m_depthBuffer) {
            this.m_depthBuffer[newIndex] = this.m_depthBuffer[oldIndex];
        }
        if (this.m_expirationTimeBuffer.data) {
            this.m_expirationTimeBuffer.data[newIndex] = this.m_expirationTimeBuffer.data[oldIndex];
        }
        return newIndex;
    }
    DestroyParticlesInGroup(group, callDestructionListener = false) {
        for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
            this.DestroyParticle(i, callDestructionListener);
        }
    }
    DestroyParticleGroup(group) {
        !!B2_DEBUG && b2Assert(this.m_groupCount > 0);
        !!B2_DEBUG && b2Assert(group !== null);
        if (this.m_world.m_destructionListener) {
            this.m_world.m_destructionListener.SayGoodbyeParticleGroup(group);
        }
        this.SetGroupFlags(group, 0);
        for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
            this.m_groupBuffer[i] = null;
        }
        if (group.m_prev) {
            group.m_prev.m_next = group.m_next;
        }
        if (group.m_next) {
            group.m_next.m_prev = group.m_prev;
        }
        if (group === this.m_groupList) {
            this.m_groupList = group.m_next;
        }
        --this.m_groupCount;
    }
    static ParticleCanBeConnected(flags, group) {
        return ((flags &
            (4 /* b2_wallParticle */ |
                8 /* b2_springParticle */ |
                16 /* b2_elasticParticle */)) !==
            0 ||
            (group !== null && (group.GetGroupFlags() & 2 /* b2_rigidParticleGroup */) !== 0));
    }
    UpdatePairsAndTriads(firstIndex, lastIndex, filter) {
        const s_dab = b2ParticleSystem.UpdatePairsAndTriads_s_dab;
        const s_dbc = b2ParticleSystem.UpdatePairsAndTriads_s_dbc;
        const s_dca = b2ParticleSystem.UpdatePairsAndTriads_s_dca;
        const pos_data = this.m_positionBuffer.data;
        // Create pairs or triads.
        // All particles in each pair/triad should satisfy the following:
        // * firstIndex <= index < lastIndex
        // * don't have b2_zombieParticle
        // * ParticleCanBeConnected returns true
        // * ShouldCreatePair/ShouldCreateTriad returns true
        // Any particles in each pair/triad should satisfy the following:
        // * filter.IsNeeded returns true
        // * have one of k_pairFlags/k_triadsFlags
        !!B2_DEBUG && b2Assert(firstIndex <= lastIndex);
        let particleFlags = 0;
        for (let i = firstIndex; i < lastIndex; i++) {
            particleFlags |= this.m_flagsBuffer.data[i];
        }
        if (particleFlags & b2ParticleSystem.k_pairFlags) {
            for (let k = 0; k < this.m_contactBuffer.count; k++) {
                const contact = this.m_contactBuffer.data[k];
                const a = contact.indexA;
                const b = contact.indexB;
                const af = this.m_flagsBuffer.data[a];
                const bf = this.m_flagsBuffer.data[b];
                const groupA = this.m_groupBuffer[a];
                const groupB = this.m_groupBuffer[b];
                if (a >= firstIndex &&
                    a < lastIndex &&
                    b >= firstIndex &&
                    b < lastIndex &&
                    !((af | bf) & 2 /* b2_zombieParticle */) &&
                    (af | bf) & b2ParticleSystem.k_pairFlags &&
                    (filter.IsNecessary(a) || filter.IsNecessary(b)) &&
                    b2ParticleSystem.ParticleCanBeConnected(af, groupA) &&
                    b2ParticleSystem.ParticleCanBeConnected(bf, groupB) &&
                    filter.ShouldCreatePair(a, b)) {
                    ///b2ParticlePair& pair = m_pairBuffer.Append();
                    const pair = this.m_pairBuffer.data[this.m_pairBuffer.Append()];
                    pair.indexA = a;
                    pair.indexB = b;
                    pair.flags = contact.flags;
                    pair.strength = b2Min(groupA ? groupA.m_strength : 1, groupB ? groupB.m_strength : 1);
                    ///pair.distance = b2Distance(pos_data[a], pos_data[b]); // TODO: this was wrong!
                    pair.distance = b2Vec2.DistanceVV(pos_data[a], pos_data[b]);
                }
                ///std::stable_sort(m_pairBuffer.Begin(), m_pairBuffer.End(), ComparePairIndices);
                std_stable_sort(this.m_pairBuffer.data, 0, this.m_pairBuffer.count, b2ParticleSystem.ComparePairIndices);
                ///m_pairBuffer.Unique(MatchPairIndices);
                this.m_pairBuffer.Unique(b2ParticleSystem.MatchPairIndices);
            }
        }
        if (particleFlags & b2ParticleSystem.k_triadFlags) {
            const diagram = new b2VoronoiDiagram(lastIndex - firstIndex);
            ///let necessary_count = 0;
            for (let i = firstIndex; i < lastIndex; i++) {
                const flags = this.m_flagsBuffer.data[i];
                const group = this.m_groupBuffer[i];
                if (!(flags & 2 /* b2_zombieParticle */) &&
                    b2ParticleSystem.ParticleCanBeConnected(flags, group)) {
                    ///if (filter.IsNecessary(i)) {
                    ///++necessary_count;
                    ///}
                    diagram.AddGenerator(pos_data[i], i, filter.IsNecessary(i));
                }
            }
            ///if (necessary_count === 0) {
            /////debugger;
            ///for (let i = firstIndex; i < lastIndex; i++) {
            ///  filter.IsNecessary(i);
            ///}
            ///}
            const stride = this.GetParticleStride();
            diagram.Generate(stride / 2, stride * 2);
            const callback = /*UpdateTriadsCallback*/ (a, b, c) => {
                const af = this.m_flagsBuffer.data[a];
                const bf = this.m_flagsBuffer.data[b];
                const cf = this.m_flagsBuffer.data[c];
                if ((af | bf | cf) & b2ParticleSystem.k_triadFlags && filter.ShouldCreateTriad(a, b, c)) {
                    const pa = pos_data[a];
                    const pb = pos_data[b];
                    const pc = pos_data[c];
                    const dab = b2Vec2.SubVV(pa, pb, s_dab);
                    const dbc = b2Vec2.SubVV(pb, pc, s_dbc);
                    const dca = b2Vec2.SubVV(pc, pa, s_dca);
                    const maxDistanceSquared = b2_maxTriadDistanceSquared * this.m_squaredDiameter;
                    if (b2Vec2.DotVV(dab, dab) > maxDistanceSquared ||
                        b2Vec2.DotVV(dbc, dbc) > maxDistanceSquared ||
                        b2Vec2.DotVV(dca, dca) > maxDistanceSquared) {
                        return;
                    }
                    const groupA = this.m_groupBuffer[a];
                    const groupB = this.m_groupBuffer[b];
                    const groupC = this.m_groupBuffer[c];
                    ///b2ParticleTriad& triad = m_system.m_triadBuffer.Append();
                    const triad = this.m_triadBuffer.data[this.m_triadBuffer.Append()];
                    triad.indexA = a;
                    triad.indexB = b;
                    triad.indexC = c;
                    triad.flags = af | bf | cf;
                    triad.strength = b2Min(b2Min(groupA ? groupA.m_strength : 1, groupB ? groupB.m_strength : 1), groupC ? groupC.m_strength : 1);
                    ///let midPoint = b2Vec2.MulSV(1.0 / 3.0, b2Vec2.AddVV(pa, b2Vec2.AddVV(pb, pc, new b2Vec2()), new b2Vec2()), new b2Vec2());
                    const midPoint_x = (pa.x + pb.x + pc.x) / 3.0;
                    const midPoint_y = (pa.y + pb.y + pc.y) / 3.0;
                    ///triad.pa = b2Vec2.SubVV(pa, midPoint, new b2Vec2());
                    triad.pa.x = pa.x - midPoint_x;
                    triad.pa.y = pa.y - midPoint_y;
                    ///triad.pb = b2Vec2.SubVV(pb, midPoint, new b2Vec2());
                    triad.pb.x = pb.x - midPoint_x;
                    triad.pb.y = pb.y - midPoint_y;
                    ///triad.pc = b2Vec2.SubVV(pc, midPoint, new b2Vec2());
                    triad.pc.x = pc.x - midPoint_x;
                    triad.pc.y = pc.y - midPoint_y;
                    triad.ka = -b2Vec2.DotVV(dca, dab);
                    triad.kb = -b2Vec2.DotVV(dab, dbc);
                    triad.kc = -b2Vec2.DotVV(dbc, dca);
                    triad.s = b2Vec2.CrossVV(pa, pb) + b2Vec2.CrossVV(pb, pc) + b2Vec2.CrossVV(pc, pa);
                }
            };
            diagram.GetNodes(callback);
            ///std::stable_sort(m_triadBuffer.Begin(), m_triadBuffer.End(), CompareTriadIndices);
            std_stable_sort(this.m_triadBuffer.data, 0, this.m_triadBuffer.count, b2ParticleSystem.CompareTriadIndices);
            ///m_triadBuffer.Unique(MatchTriadIndices);
            this.m_triadBuffer.Unique(b2ParticleSystem.MatchTriadIndices);
        }
    }
    UpdatePairsAndTriadsWithReactiveParticles() {
        const filter = new b2ParticleSystem_ReactiveFilter(this.m_flagsBuffer);
        this.UpdatePairsAndTriads(0, this.m_count, filter);
        for (let i = 0; i < this.m_count; i++) {
            this.m_flagsBuffer.data[i] &= ~4096 /* b2_reactiveParticle */;
        }
        this.m_allParticleFlags &= ~4096 /* b2_reactiveParticle */;
    }
    static ComparePairIndices(a, b) {
        const diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        return a.indexB < b.indexB;
    }
    static MatchPairIndices(a, b) {
        return a.indexA === b.indexA && a.indexB === b.indexB;
    }
    static CompareTriadIndices(a, b) {
        const diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        const diffB = a.indexB - b.indexB;
        if (diffB !== 0) {
            return diffB < 0;
        }
        return a.indexC < b.indexC;
    }
    static MatchTriadIndices(a, b) {
        return a.indexA === b.indexA && a.indexB === b.indexB && a.indexC === b.indexC;
    }
    static InitializeParticleLists(group, nodeBuffer) {
        const bufferIndex = group.GetBufferIndex();
        const particleCount = group.GetParticleCount();
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            node.list = node;
            node.next = null;
            node.count = 1;
            node.index = i + bufferIndex;
        }
    }
    MergeParticleListsInContact(group, nodeBuffer) {
        const bufferIndex = group.GetBufferIndex();
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            /*const b2ParticleContact&*/
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            if (!group.ContainsParticle(a) || !group.ContainsParticle(b)) {
                continue;
            }
            let listA = nodeBuffer[a - bufferIndex].list;
            let listB = nodeBuffer[b - bufferIndex].list;
            if (listA === listB) {
                continue;
            }
            // To minimize the cost of insertion, make sure listA is longer than
            // listB.
            if (listA.count < listB.count) {
                const _tmp = listA;
                listA = listB;
                listB = _tmp; ///b2Swap(listA, listB);
            }
            !!B2_DEBUG && b2Assert(listA.count >= listB.count);
            b2ParticleSystem.MergeParticleLists(listA, listB);
        }
    }
    static MergeParticleLists(listA, listB) {
        // Insert listB between index 0 and 1 of listA
        // Example:
        //     listA => a1 => a2 => a3 => null
        //     listB => b1 => b2 => null
        // to
        //     listA => listB => b1 => b2 => a1 => a2 => a3 => null
        !!B2_DEBUG && b2Assert(listA !== listB);
        for (let b = listB;;) {
            b.list = listA;
            const nextB = b.next;
            if (nextB) {
                b = nextB;
            }
            else {
                b.next = listA.next;
                break;
            }
        }
        listA.next = listB;
        listA.count += listB.count;
        listB.count = 0;
    }
    static FindLongestParticleList(group, nodeBuffer) {
        const particleCount = group.GetParticleCount();
        let result = nodeBuffer[0];
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            if (result.count < node.count) {
                result = node;
            }
        }
        return result;
    }
    MergeZombieParticleListNodes(group, nodeBuffer, survivingList) {
        const particleCount = group.GetParticleCount();
        for (let i = 0; i < particleCount; i++) {
            const node = nodeBuffer[i];
            if (node !== survivingList &&
                this.m_flagsBuffer.data[node.index] & 2 /* b2_zombieParticle */) {
                b2ParticleSystem.MergeParticleListAndNode(survivingList, node);
            }
        }
    }
    static MergeParticleListAndNode(list, node) {
        // Insert node between index 0 and 1 of list
        // Example:
        //     list => a1 => a2 => a3 => null
        //     node => null
        // to
        //     list => node => a1 => a2 => a3 => null
        if (B2_DEBUG) {
            b2Assert(node !== list);
            b2Assert(node.list === node);
            b2Assert(node.count === 1);
        }
        node.list = list;
        node.next = list.next;
        list.next = node;
        list.count++;
        node.count = 0;
    }
    CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList) {
        const particleCount = group.GetParticleCount();
        const def = new b2ParticleGroupDef();
        def.groupFlags = group.GetGroupFlags();
        def.userData = group.GetUserData();
        for (let i = 0; i < particleCount; i++) {
            const list = nodeBuffer[i];
            if (!list.count || list === survivingList) {
                continue;
            }
            !!B2_DEBUG && b2Assert(list.list === list);
            const newGroup = this.CreateParticleGroup(def);
            for (let node = list; node; node = node.next) {
                const oldIndex = node.index;
                if (B2_DEBUG) {
                    const flags = this.m_flagsBuffer.data[oldIndex];
                    b2Assert(!(flags & 2 /* b2_zombieParticle */));
                }
                const newIndex = this.CloneParticle(oldIndex, newGroup);
                this.m_flagsBuffer.data[oldIndex] |= 2 /* b2_zombieParticle */;
                node.index = newIndex;
            }
        }
    }
    UpdatePairsAndTriadsWithParticleList(group, nodeBuffer) {
        const bufferIndex = group.GetBufferIndex();
        // Update indices in pairs and triads. If an index belongs to the group,
        // replace it with the corresponding value in nodeBuffer.
        // Note that nodeBuffer is allocated only for the group and the index should
        // be shifted by bufferIndex.
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            const a = pair.indexA;
            const b = pair.indexB;
            if (group.ContainsParticle(a)) {
                pair.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.ContainsParticle(b)) {
                pair.indexB = nodeBuffer[b - bufferIndex].index;
            }
        }
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            const a = triad.indexA;
            const b = triad.indexB;
            const c = triad.indexC;
            if (group.ContainsParticle(a)) {
                triad.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.ContainsParticle(b)) {
                triad.indexB = nodeBuffer[b - bufferIndex].index;
            }
            if (group.ContainsParticle(c)) {
                triad.indexC = nodeBuffer[c - bufferIndex].index;
            }
        }
    }
    ComputeDepth() {
        const contactGroups = []; // TODO: static
        let contactGroupsCount = 0;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const groupA = this.m_groupBuffer[a];
            const groupB = this.m_groupBuffer[b];
            if (groupA &&
                groupA === groupB &&
                groupA.m_groupFlags & 16 /* b2_particleGroupNeedsUpdateDepth */) {
                contactGroups[contactGroupsCount++] = contact;
            }
        }
        const groupsToUpdate = []; // TODO: static
        let groupsToUpdateCount = 0;
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            if (group.m_groupFlags & 16 /* b2_particleGroupNeedsUpdateDepth */) {
                groupsToUpdate[groupsToUpdateCount++] = group;
                this.SetGroupFlags(group, group.m_groupFlags & ~16 /* b2_particleGroupNeedsUpdateDepth */);
                for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                    this.m_accumulationBuffer[i] = 0;
                }
            }
        }
        // Compute sum of weight of contacts except between different groups.
        for (let k = 0; k < contactGroupsCount; k++) {
            const contact = contactGroups[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            this.m_accumulationBuffer[a] += w;
            this.m_accumulationBuffer[b] += w;
        }
        !!B2_DEBUG && b2Assert(this.m_depthBuffer !== null);
        for (let i = 0; i < groupsToUpdateCount; i++) {
            const group = groupsToUpdate[i];
            for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                const w = this.m_accumulationBuffer[i];
                this.m_depthBuffer[i] = w < 0.8 ? 0 : b2_maxFloat;
            }
        }
        // The number of iterations is equal to particle number from the deepest
        // particle to the nearest surface particle, and in general it is smaller
        // than sqrt of total particle number.
        ///int32 iterationCount = (int32)b2Sqrt((float)m_count);
        const iterationCount = b2Sqrt(this.m_count) >> 0;
        for (let t = 0; t < iterationCount; t++) {
            let updated = false;
            for (let k = 0; k < contactGroupsCount; k++) {
                const contact = contactGroups[k];
                const a = contact.indexA;
                const b = contact.indexB;
                const r = 1 - contact.weight;
                ///float32& ap0 = m_depthBuffer[a];
                const ap0 = this.m_depthBuffer[a];
                ///float32& bp0 = m_depthBuffer[b];
                const bp0 = this.m_depthBuffer[b];
                const ap1 = bp0 + r;
                const bp1 = ap0 + r;
                if (ap0 > ap1) {
                    ///ap0 = ap1;
                    this.m_depthBuffer[a] = ap1;
                    updated = true;
                }
                if (bp0 > bp1) {
                    ///bp0 = bp1;
                    this.m_depthBuffer[b] = bp1;
                    updated = true;
                }
            }
            if (!updated) {
                break;
            }
        }
        for (let i = 0; i < groupsToUpdateCount; i++) {
            const group = groupsToUpdate[i];
            for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                if (this.m_depthBuffer[i] < b2_maxFloat) {
                    this.m_depthBuffer[i] *= this.m_particleDiameter;
                }
                else {
                    this.m_depthBuffer[i] = 0;
                }
            }
        }
    }
    GetInsideBoundsEnumerator(aabb) {
        const lowerTag = b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.lowerBound.x - 1, this.m_inverseDiameter * aabb.lowerBound.y - 1);
        const upperTag = b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.upperBound.x + 1, this.m_inverseDiameter * aabb.upperBound.y + 1);
        ///const Proxy* beginProxy = m_proxyBuffer.Begin();
        const beginProxy = 0;
        ///const Proxy* endProxy = m_proxyBuffer.End();
        const endProxy = this.m_proxyBuffer.count;
        ///const Proxy* firstProxy = std::lower_bound(beginProxy, endProxy, lowerTag);
        const firstProxy = std_lower_bound(this.m_proxyBuffer.data, beginProxy, endProxy, lowerTag, b2ParticleSystem_Proxy.CompareProxyTag);
        ///const Proxy* lastProxy = std::upper_bound(firstProxy, endProxy, upperTag);
        const lastProxy = std_upper_bound(this.m_proxyBuffer.data, beginProxy, endProxy, upperTag, b2ParticleSystem_Proxy.CompareTagProxy);
        !!B2_DEBUG && b2Assert(beginProxy <= firstProxy);
        !!B2_DEBUG && b2Assert(firstProxy <= lastProxy);
        !!B2_DEBUG && b2Assert(lastProxy <= endProxy);
        return new b2ParticleSystem_InsideBoundsEnumerator(this, lowerTag, upperTag, firstProxy, lastProxy);
    }
    UpdateAllParticleFlags() {
        this.m_allParticleFlags = 0;
        for (let i = 0; i < this.m_count; i++) {
            this.m_allParticleFlags |= this.m_flagsBuffer.data[i];
        }
        this.m_needsUpdateAllParticleFlags = false;
    }
    UpdateAllGroupFlags() {
        this.m_allGroupFlags = 0;
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            this.m_allGroupFlags |= group.m_groupFlags;
        }
        this.m_needsUpdateAllGroupFlags = false;
    }
    AddContact(a, b, contacts) {
        !!B2_DEBUG && b2Assert(contacts === this.m_contactBuffer);
        const flags_data = this.m_flagsBuffer.data;
        const pos_data = this.m_positionBuffer.data;
        ///b2Vec2 d = m_positionBuffer.data[b] - m_positionBuffer.data[a];
        const d = b2Vec2.SubVV(pos_data[b], pos_data[a], b2ParticleSystem.AddContact_s_d);
        const distBtParticlesSq = b2Vec2.DotVV(d, d);
        if (0 < distBtParticlesSq && distBtParticlesSq < this.m_squaredDiameter) {
            const invD = b2InvSqrt(distBtParticlesSq);
            ///b2ParticleContact& contact = contacts.Append();
            const contact = this.m_contactBuffer.data[this.m_contactBuffer.Append()];
            contact.indexA = a;
            contact.indexB = b;
            contact.flags = flags_data[a] | flags_data[b];
            contact.weight = 1 - distBtParticlesSq * invD * this.m_inverseDiameter;
            contact.normal.x = invD * d.x;
            contact.normal.y = invD * d.y;
        }
    }
    FindContacts_Reference(contacts) {
        !!B2_DEBUG && b2Assert(contacts === this.m_contactBuffer);
        const beginProxy = 0;
        const endProxy = this.m_proxyBuffer.count;
        this.m_contactBuffer.count = 0;
        for (let a = beginProxy, c = beginProxy; a < endProxy; a++) {
            const rightTag = b2ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, 1, 0);
            for (let b = a + 1; b < endProxy; b++) {
                if (rightTag < this.m_proxyBuffer.data[b].tag) {
                    break;
                }
                this.AddContact(this.m_proxyBuffer.data[a].index, this.m_proxyBuffer.data[b].index, this.m_contactBuffer);
            }
            const bottomLeftTag = b2ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, -1, 1);
            for (; c < endProxy; c++) {
                if (bottomLeftTag <= this.m_proxyBuffer.data[c].tag) {
                    break;
                }
            }
            const bottomRightTag = b2ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, 1, 1);
            for (let b = c; b < endProxy; b++) {
                if (bottomRightTag < this.m_proxyBuffer.data[b].tag) {
                    break;
                }
                this.AddContact(this.m_proxyBuffer.data[a].index, this.m_proxyBuffer.data[b].index, this.m_contactBuffer);
            }
        }
    }
    ///void ReorderForFindContact(FindContactInput* reordered, int alignedCount) const;
    ///void GatherChecksOneParticle(const uint32 bound, const int startIndex, const int particleIndex, int* nextUncheckedIndex, b2GrowableBuffer<FindContactCheck>& checks) const;
    ///void GatherChecks(b2GrowableBuffer<FindContactCheck>& checks) const;
    ///void FindContacts_Simd(b2GrowableBuffer<b2ParticleContact>& contacts) const;
    FindContacts(contacts) {
        this.FindContacts_Reference(contacts);
    }
    ///static void UpdateProxyTags(const uint32* const tags, b2GrowableBuffer<Proxy>& proxies);
    ///static bool ProxyBufferHasIndex(int32 index, const Proxy* const a, int count);
    ///static int NumProxiesWithSameTag(const Proxy* const a, const Proxy* const b, int count);
    ///static bool AreProxyBuffersTheSame(const b2GrowableBuffer<Proxy>& a, const b2GrowableBuffer<Proxy>& b);
    UpdateProxies_Reference(proxies) {
        !!B2_DEBUG && b2Assert(proxies === this.m_proxyBuffer);
        const pos_data = this.m_positionBuffer.data;
        const inv_diam = this.m_inverseDiameter;
        for (let k = 0; k < this.m_proxyBuffer.count; ++k) {
            const proxy = this.m_proxyBuffer.data[k];
            const i = proxy.index;
            const p = pos_data[i];
            proxy.tag = b2ParticleSystem.computeTag(inv_diam * p.x, inv_diam * p.y);
        }
    }
    ///void UpdateProxies_Simd(b2GrowableBuffer<Proxy>& proxies) const;
    UpdateProxies(proxies) {
        this.UpdateProxies_Reference(proxies);
    }
    SortProxies(proxies) {
        !!B2_DEBUG && b2Assert(proxies === this.m_proxyBuffer);
        ///std::sort(proxies.Begin(), proxies.End());
        std_sort(this.m_proxyBuffer.data, 0, this.m_proxyBuffer.count, b2ParticleSystem_Proxy.CompareProxyProxy);
    }
    FilterContacts(contacts) {
        // Optionally filter the contact.
        const contactFilter = this.GetParticleContactFilter();
        if (contactFilter === null) {
            return;
        }
        /// contacts.RemoveIf(b2ParticleContactRemovePredicate(this, contactFilter));
        !!B2_DEBUG && b2Assert(contacts === this.m_contactBuffer);
        const predicate = (contact) => {
            return ((contact.flags & 131072 /* b2_particleContactFilterParticle */) !== 0 &&
                !contactFilter.ShouldCollideParticleParticle(this, contact.indexA, contact.indexB));
        };
        this.m_contactBuffer.RemoveIf(predicate);
    }
    NotifyContactListenerPreContact(particlePairs) {
        const contactListener = this.GetParticleContactListener();
        if (contactListener === null) {
            return;
        }
        ///particlePairs.Initialize(m_contactBuffer.Begin(), m_contactBuffer.GetCount(), GetFlagsBuffer());
        particlePairs.Initialize(this.m_contactBuffer, this.m_flagsBuffer);
        throw new Error(); // TODO: notify
    }
    NotifyContactListenerPostContact(particlePairs) {
        const contactListener = this.GetParticleContactListener();
        if (contactListener === null) {
            return;
        }
        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        ///const b2ParticleContact* const endContact = m_contactBuffer.End();
        ///for (b2ParticleContact* contact = m_contactBuffer.Begin(); contact < endContact; ++contact)
        for (let k = 0; k < this.m_contactBuffer.count; ++k) {
            const contact = this.m_contactBuffer.data[k];
            ///ParticlePair pair;
            ///pair.first = contact.GetIndexA();
            ///pair.second = contact.GetIndexB();
            ///const int32 itemIndex = particlePairs.Find(pair);
            const itemIndex = -1; // TODO
            if (itemIndex >= 0) {
                // Already touching, ignore this contact.
                particlePairs.Invalidate(itemIndex);
            }
            else {
                // Just started touching, inform the listener.
                contactListener.BeginContactParticleParticle(this, contact);
            }
        }
        // Report particles that are no longer touching.
        // That is, any pairs that were not invalidated above.
        ///const int32 pairCount = particlePairs.GetCount();
        ///const ParticlePair* const pairs = particlePairs.GetBuffer();
        ///const int8* const valid = particlePairs.GetValidBuffer();
        ///for (int32 i = 0; i < pairCount; ++i)
        ///{
        ///  if (valid[i])
        ///  {
        ///    contactListener.EndContactParticleParticle(this, pairs[i].first, pairs[i].second);
        ///  }
        ///}
        throw new Error(); // TODO: notify
    }
    static b2ParticleContactIsZombie(contact) {
        return (contact.flags & 2 /* b2_zombieParticle */) === 2 /* b2_zombieParticle */;
    }
    UpdateContacts(exceptZombie) {
        this.UpdateProxies(this.m_proxyBuffer);
        this.SortProxies(this.m_proxyBuffer);
        const particlePairs = new b2ParticlePairSet(); // TODO: static
        this.NotifyContactListenerPreContact(particlePairs);
        this.FindContacts(this.m_contactBuffer);
        this.FilterContacts(this.m_contactBuffer);
        this.NotifyContactListenerPostContact(particlePairs);
        if (exceptZombie) {
            this.m_contactBuffer.RemoveIf(b2ParticleSystem.b2ParticleContactIsZombie);
        }
    }
    NotifyBodyContactListenerPreContact(fixtureSet) {
        const contactListener = this.GetFixtureContactListener();
        if (contactListener === null) {
            return;
        }
        ///fixtureSet.Initialize(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.GetCount(), GetFlagsBuffer());
        fixtureSet.Initialize(this.m_bodyContactBuffer, this.m_flagsBuffer);
        throw new Error(); // TODO: notify
    }
    NotifyBodyContactListenerPostContact(fixtureSet) {
        const contactListener = this.GetFixtureContactListener();
        if (contactListener === null) {
            return;
        }
        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        ///for (b2ParticleBodyContact* contact = m_bodyContactBuffer.Begin(); contact !== m_bodyContactBuffer.End(); ++contact)
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            !!B2_DEBUG && b2Assert(contact !== null);
            ///FixtureParticle fixtureParticleToFind;
            ///fixtureParticleToFind.first = contact.fixture;
            ///fixtureParticleToFind.second = contact.index;
            ///const int32 index = fixtureSet.Find(fixtureParticleToFind);
            const index = -1; // TODO
            if (index >= 0) {
                // Already touching remove this from the set.
                fixtureSet.Invalidate(index);
            }
            else {
                // Just started touching, report it!
                contactListener.BeginContactFixtureParticle(this, contact);
            }
        }
        // If the contact listener is enabled, report all fixtures that are no
        // longer in contact with particles.
        ///const FixtureParticle* const fixtureParticles = fixtureSet.GetBuffer();
        ///const int8* const fixtureParticlesValid = fixtureSet.GetValidBuffer();
        ///const int32 fixtureParticleCount = fixtureSet.GetCount();
        ///for (int32 i = 0; i < fixtureParticleCount; ++i)
        ///{
        ///  if (fixtureParticlesValid[i])
        ///  {
        ///    const FixtureParticle* const fixtureParticle = &fixtureParticles[i];
        ///    contactListener.EndContactFixtureParticle(fixtureParticle.first, this, fixtureParticle.second);
        ///  }
        ///}
        throw new Error(); // TODO: notify
    }
    UpdateBodyContacts() {
        const s_aabb = b2ParticleSystem.UpdateBodyContacts_s_aabb;
        // If the particle contact listener is enabled, generate a set of
        // fixture / particle contacts.
        const fixtureSet = new b2ParticleSystem_FixtureParticleSet(); // TODO: static
        this.NotifyBodyContactListenerPreContact(fixtureSet);
        if (this.m_stuckThreshold > 0) {
            const particleCount = this.GetParticleCount();
            for (let i = 0; i < particleCount; i++) {
                // Detect stuck particles, see comment in
                // b2ParticleSystem::DetectStuckParticle()
                this.m_bodyContactCountBuffer.data[i] = 0;
                if (this.m_timestamp > this.m_lastBodyContactStepBuffer.data[i] + 1) {
                    this.m_consecutiveContactStepsBuffer.data[i] = 0;
                }
            }
        }
        this.m_bodyContactBuffer.SetCount(0);
        this.m_stuckParticleBuffer.SetCount(0);
        const aabb = s_aabb;
        this.ComputeAABB(aabb);
        if (this.UpdateBodyContacts_callback === null) {
            this.UpdateBodyContacts_callback = new b2ParticleSystem_UpdateBodyContactsCallback(this);
        }
        const callback = this.UpdateBodyContacts_callback;
        callback.m_contactFilter = this.GetFixtureContactFilter();
        this.m_world.QueryAABB(callback, aabb);
        if (this.m_def.strictContactCheck) {
            this.RemoveSpuriousBodyContacts();
        }
        this.NotifyBodyContactListenerPostContact(fixtureSet);
    }
    Solve(step) {
        const s_subStep = b2ParticleSystem.Solve_s_subStep;
        if (this.m_count === 0) {
            return;
        }
        // If particle lifetimes are enabled, destroy particles that are too old.
        if (this.m_expirationTimeBuffer.data) {
            this.SolveLifetimes(step);
        }
        if (this.m_allParticleFlags & 2 /* b2_zombieParticle */) {
            this.SolveZombie();
        }
        if (this.m_needsUpdateAllParticleFlags) {
            this.UpdateAllParticleFlags();
        }
        if (this.m_needsUpdateAllGroupFlags) {
            this.UpdateAllGroupFlags();
        }
        if (this.m_paused) {
            return;
        }
        for (this.m_iterationIndex = 0; this.m_iterationIndex < step.particleIterations; this.m_iterationIndex++) {
            ++this.m_timestamp;
            const subStep = s_subStep.Copy(step);
            subStep.dt /= step.particleIterations;
            subStep.inv_dt *= step.particleIterations;
            this.UpdateContacts(false);
            this.UpdateBodyContacts();
            this.ComputeWeight();
            if (this.m_allGroupFlags & 16 /* b2_particleGroupNeedsUpdateDepth */) {
                this.ComputeDepth();
            }
            if (this.m_allParticleFlags & 4096 /* b2_reactiveParticle */) {
                this.UpdatePairsAndTriadsWithReactiveParticles();
            }
            if (this.m_hasForce) {
                this.SolveForce(subStep);
            }
            if (this.m_allParticleFlags & 32 /* b2_viscousParticle */) {
                this.SolveViscous();
            }
            if (this.m_allParticleFlags & 8192 /* b2_repulsiveParticle */) {
                this.SolveRepulsive(subStep);
            }
            if (this.m_allParticleFlags & 64 /* b2_powderParticle */) {
                this.SolvePowder(subStep);
            }
            if (this.m_allParticleFlags & 128 /* b2_tensileParticle */) {
                this.SolveTensile(subStep);
            }
            if (this.m_allGroupFlags & 1 /* b2_solidParticleGroup */) {
                this.SolveSolid(subStep);
            }
            if (this.m_allParticleFlags & 256 /* b2_colorMixingParticle */) {
                this.SolveColorMixing();
            }
            this.SolveGravity(subStep);
            if (this.m_allParticleFlags & 2048 /* b2_staticPressureParticle */) {
                this.SolveStaticPressure(subStep);
            }
            this.SolvePressure(subStep);
            this.SolveDamping(subStep);
            if (this.m_allParticleFlags & b2ParticleSystem.k_extraDampingFlags) {
                this.SolveExtraDamping();
            }
            // SolveElastic and SolveSpring refer the current velocities for
            // numerical stability, they should be called as late as possible.
            if (this.m_allParticleFlags & 16 /* b2_elasticParticle */) {
                this.SolveElastic(subStep);
            }
            if (this.m_allParticleFlags & 8 /* b2_springParticle */) {
                this.SolveSpring(subStep);
            }
            this.LimitVelocity(subStep);
            if (this.m_allGroupFlags & 2 /* b2_rigidParticleGroup */) {
                this.SolveRigidDamping();
            }
            if (this.m_allParticleFlags & 1024 /* b2_barrierParticle */) {
                this.SolveBarrier(subStep);
            }
            // SolveCollision, SolveRigid and SolveWall should be called after
            // other force functions because they may require particles to have
            // specific velocities.
            this.SolveCollision(subStep);
            if (this.m_allGroupFlags & 2 /* b2_rigidParticleGroup */) {
                this.SolveRigid(subStep);
            }
            if (this.m_allParticleFlags & 4 /* b2_wallParticle */) {
                this.SolveWall();
            }
            // The particle positions can be updated only at the end of substep.
            for (let i = 0; i < this.m_count; i++) {
                ///m_positionBuffer.data[i] += subStep.dt * m_velocityBuffer.data[i];
                this.m_positionBuffer.data[i].SelfMulAdd(subStep.dt, this.m_velocityBuffer.data[i]);
            }
        }
    }
    SolveCollision(step) {
        const s_aabb = b2ParticleSystem.SolveCollision_s_aabb;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // This function detects particles which are crossing boundary of bodies
        // and modifies velocities of them so that they will move just in front of
        // boundary. This function function also applies the reaction force to
        // bodies as precisely as the numerical stability is kept.
        const aabb = s_aabb;
        aabb.lowerBound.x = +b2_maxFloat;
        aabb.lowerBound.y = +b2_maxFloat;
        aabb.upperBound.x = -b2_maxFloat;
        aabb.upperBound.y = -b2_maxFloat;
        for (let i = 0; i < this.m_count; i++) {
            const v = vel_data[i];
            const p1 = pos_data[i];
            ///let p2 = p1 + step.dt * v;
            const p2_x = p1.x + step.dt * v.x;
            const p2_y = p1.y + step.dt * v.y;
            ///aabb.lowerBound = b2Min(aabb.lowerBound, b2Min(p1, p2));
            aabb.lowerBound.x = b2Min(aabb.lowerBound.x, b2Min(p1.x, p2_x));
            aabb.lowerBound.y = b2Min(aabb.lowerBound.y, b2Min(p1.y, p2_y));
            ///aabb.upperBound = b2Max(aabb.upperBound, b2Max(p1, p2));
            aabb.upperBound.x = b2Max(aabb.upperBound.x, b2Max(p1.x, p2_x));
            aabb.upperBound.y = b2Max(aabb.upperBound.y, b2Max(p1.y, p2_y));
        }
        if (this.SolveCollision_callback === null) {
            this.SolveCollision_callback = new b2ParticleSystem_SolveCollisionCallback(this, step);
        }
        const callback = this.SolveCollision_callback;
        callback.m_step = step;
        this.m_world.QueryAABB(callback, aabb);
    }
    LimitVelocity(step) {
        const vel_data = this.m_velocityBuffer.data;
        const criticalVelocitySquared = this.GetCriticalVelocitySquared(step);
        for (let i = 0; i < this.m_count; i++) {
            const v = vel_data[i];
            const v2 = b2Vec2.DotVV(v, v);
            if (v2 > criticalVelocitySquared) {
                ///v *= b2Sqrt(criticalVelocitySquared / v2);
                v.SelfMul(b2Sqrt(criticalVelocitySquared / v2));
            }
        }
    }
    SolveGravity(step) {
        const s_gravity = b2ParticleSystem.SolveGravity_s_gravity;
        const vel_data = this.m_velocityBuffer.data;
        ///b2Vec2 gravity = step.dt * m_def.gravityScale * m_world.GetGravity();
        const gravity = b2Vec2.MulSV(step.dt * this.m_def.gravityScale, this.m_world.GetGravity(), s_gravity);
        for (let i = 0; i < this.m_count; i++) {
            vel_data[i].SelfAdd(gravity);
        }
    }
    SolveBarrier(step) {
        const s_aabb = b2ParticleSystem.SolveBarrier_s_aabb;
        const s_va = b2ParticleSystem.SolveBarrier_s_va;
        const s_vb = b2ParticleSystem.SolveBarrier_s_vb;
        const s_pba = b2ParticleSystem.SolveBarrier_s_pba;
        const s_vba = b2ParticleSystem.SolveBarrier_s_vba;
        const s_vc = b2ParticleSystem.SolveBarrier_s_vc;
        const s_pca = b2ParticleSystem.SolveBarrier_s_pca;
        const s_vca = b2ParticleSystem.SolveBarrier_s_vca;
        const s_qba = b2ParticleSystem.SolveBarrier_s_qba;
        const s_qca = b2ParticleSystem.SolveBarrier_s_qca;
        const s_dv = b2ParticleSystem.SolveBarrier_s_dv;
        const s_f = b2ParticleSystem.SolveBarrier_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // If a particle is passing between paired barrier particles,
        // its velocity will be decelerated to avoid passing.
        for (let i = 0; i < this.m_count; i++) {
            const flags = this.m_flagsBuffer.data[i];
            ///if ((flags & b2ParticleSystem.k_barrierWallFlags) === b2ParticleSystem.k_barrierWallFlags)
            if ((flags & b2ParticleSystem.k_barrierWallFlags) !== 0) {
                vel_data[i].SetZero();
            }
        }
        const tmax = b2_barrierCollisionTime * step.dt;
        const mass = this.GetParticleMass();
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            if (pair.flags & 1024 /* b2_barrierParticle */) {
                const a = pair.indexA;
                const b = pair.indexB;
                const pa = pos_data[a];
                const pb = pos_data[b];
                /// b2AABB aabb;
                const aabb = s_aabb;
                ///aabb.lowerBound = b2Min(pa, pb);
                b2Vec2.MinV(pa, pb, aabb.lowerBound);
                ///aabb.upperBound = b2Max(pa, pb);
                b2Vec2.MaxV(pa, pb, aabb.upperBound);
                const aGroup = this.m_groupBuffer[a];
                const bGroup = this.m_groupBuffer[b];
                ///b2Vec2 va = GetLinearVelocity(aGroup, a, pa);
                const va = this.GetLinearVelocity(aGroup, a, pa, s_va);
                ///b2Vec2 vb = GetLinearVelocity(bGroup, b, pb);
                const vb = this.GetLinearVelocity(bGroup, b, pb, s_vb);
                ///b2Vec2 pba = pb - pa;
                const pba = b2Vec2.SubVV(pb, pa, s_pba);
                ///b2Vec2 vba = vb - va;
                const vba = b2Vec2.SubVV(vb, va, s_vba);
                ///InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
                const enumerator = this.GetInsideBoundsEnumerator(aabb);
                let c;
                while ((c = enumerator.GetNext()) >= 0) {
                    const pc = pos_data[c];
                    const cGroup = this.m_groupBuffer[c];
                    if (aGroup !== cGroup && bGroup !== cGroup) {
                        ///b2Vec2 vc = GetLinearVelocity(cGroup, c, pc);
                        const vc = this.GetLinearVelocity(cGroup, c, pc, s_vc);
                        // Solve the equation below:
                        //   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
                        // which expresses that the particle c will pass a line
                        // connecting the particles a and b at the time of t.
                        // if s is between 0 and 1, c will pass between a and b.
                        ///b2Vec2 pca = pc - pa;
                        const pca = b2Vec2.SubVV(pc, pa, s_pca);
                        ///b2Vec2 vca = vc - va;
                        const vca = b2Vec2.SubVV(vc, va, s_vca);
                        const e2 = b2Vec2.CrossVV(vba, vca);
                        const e1 = b2Vec2.CrossVV(pba, vca) - b2Vec2.CrossVV(pca, vba);
                        const e0 = b2Vec2.CrossVV(pba, pca);
                        let s, t;
                        ///b2Vec2 qba, qca;
                        const qba = s_qba, qca = s_qca;
                        if (e2 === 0) {
                            if (e1 === 0) {
                                continue;
                            }
                            t = -e0 / e1;
                            if (!(t >= 0 && t < tmax)) {
                                continue;
                            }
                            ///qba = pba + t * vba;
                            b2Vec2.AddVMulSV(pba, t, vba, qba);
                            ///qca = pca + t * vca;
                            b2Vec2.AddVMulSV(pca, t, vca, qca);
                            s = b2Vec2.DotVV(qba, qca) / b2Vec2.DotVV(qba, qba);
                            if (!(s >= 0 && s <= 1)) {
                                continue;
                            }
                        }
                        else {
                            const det = e1 * e1 - 4 * e0 * e2;
                            if (det < 0) {
                                continue;
                            }
                            const sqrtDet = b2Sqrt(det);
                            let t1 = (-e1 - sqrtDet) / (2 * e2);
                            let t2 = (-e1 + sqrtDet) / (2 * e2);
                            ///if (t1 > t2) b2Swap(t1, t2);
                            if (t1 > t2) {
                                const tmp = t1;
                                t1 = t2;
                                t2 = tmp;
                            }
                            t = t1;
                            ///qba = pba + t * vba;
                            b2Vec2.AddVMulSV(pba, t, vba, qba);
                            ///qca = pca + t * vca;
                            b2Vec2.AddVMulSV(pca, t, vca, qca);
                            ///s = b2Dot(qba, qca) / b2Dot(qba, qba);
                            s = b2Vec2.DotVV(qba, qca) / b2Vec2.DotVV(qba, qba);
                            if (!(t >= 0 && t < tmax && s >= 0 && s <= 1)) {
                                t = t2;
                                if (!(t >= 0 && t < tmax)) {
                                    continue;
                                }
                                ///qba = pba + t * vba;
                                b2Vec2.AddVMulSV(pba, t, vba, qba);
                                ///qca = pca + t * vca;
                                b2Vec2.AddVMulSV(pca, t, vca, qca);
                                ///s = b2Dot(qba, qca) / b2Dot(qba, qba);
                                s = b2Vec2.DotVV(qba, qca) / b2Vec2.DotVV(qba, qba);
                                if (!(s >= 0 && s <= 1)) {
                                    continue;
                                }
                            }
                        }
                        // Apply a force to particle c so that it will have the
                        // interpolated velocity at the collision point on line ab.
                        ///b2Vec2 dv = va + s * vba - vc;
                        const dv = s_dv;
                        dv.x = va.x + s * vba.x - vc.x;
                        dv.y = va.y + s * vba.y - vc.y;
                        ///b2Vec2 f = GetParticleMass() * dv;
                        const f = b2Vec2.MulSV(mass, dv, s_f);
                        if (cGroup && this.IsRigidGroup(cGroup)) {
                            // If c belongs to a rigid group, the force will be
                            // distributed in the group.
                            const mass = cGroup.GetMass();
                            const inertia = cGroup.GetInertia();
                            if (mass > 0) {
                                ///cGroup.m_linearVelocity += 1 / mass * f;
                                cGroup.m_linearVelocity.SelfMulAdd(1 / mass, f);
                            }
                            if (inertia > 0) {
                                ///cGroup.m_angularVelocity += b2Cross(pc - cGroup.GetCenter(), f) / inertia;
                                cGroup.m_angularVelocity +=
                                    b2Vec2.CrossVV(b2Vec2.SubVV(pc, cGroup.GetCenter(), b2Vec2.s_t0), f) / inertia;
                            }
                        }
                        else {
                            ///m_velocityBuffer.data[c] += dv;
                            vel_data[c].SelfAdd(dv);
                        }
                        // Apply a reversed force to particle c after particle
                        // movement so that momentum will be preserved.
                        ///ParticleApplyForce(c, -step.inv_dt * f);
                        this.ParticleApplyForce(c, f.SelfMul(-step.inv_dt));
                    }
                }
            }
        }
    }
    SolveStaticPressure(step) {
        this.m_staticPressureBuffer = this.RequestBuffer(this.m_staticPressureBuffer);
        const criticalPressure = this.GetCriticalPressure(step);
        const pressurePerWeight = this.m_def.staticPressureStrength * criticalPressure;
        const maxPressure = b2_maxParticlePressure * criticalPressure;
        const relaxation = this.m_def.staticPressureRelaxation;
        /// Compute pressure satisfying the modified Poisson equation:
        ///   Sum_for_j((p_i - p_j) * w_ij) + relaxation * p_i =
        ///   pressurePerWeight * (w_i - b2_minParticleWeight)
        /// by iterating the calculation:
        ///   p_i = (Sum_for_j(p_j * w_ij) + pressurePerWeight *
        ///         (w_i - b2_minParticleWeight)) / (w_i + relaxation)
        /// where
        ///   p_i and p_j are static pressure of particle i and j
        ///   w_ij is contact weight between particle i and j
        ///   w_i is sum of contact weight of particle i
        for (let t = 0; t < this.m_def.staticPressureIterations; t++) {
            ///memset(m_accumulationBuffer, 0, sizeof(*m_accumulationBuffer) * m_count);
            for (let i = 0; i < this.m_count; i++) {
                this.m_accumulationBuffer[i] = 0;
            }
            for (let k = 0; k < this.m_contactBuffer.count; k++) {
                const contact = this.m_contactBuffer.data[k];
                if (contact.flags & 2048 /* b2_staticPressureParticle */) {
                    const a = contact.indexA;
                    const b = contact.indexB;
                    const w = contact.weight;
                    this.m_accumulationBuffer[a] += w * this.m_staticPressureBuffer[b]; // a <- b
                    this.m_accumulationBuffer[b] += w * this.m_staticPressureBuffer[a]; // b <- a
                }
            }
            for (let i = 0; i < this.m_count; i++) {
                const w = this.m_weightBuffer[i];
                if (this.m_flagsBuffer.data[i] & 2048 /* b2_staticPressureParticle */) {
                    const wh = this.m_accumulationBuffer[i];
                    const h = (wh + pressurePerWeight * (w - b2_minParticleWeight)) / (w + relaxation);
                    this.m_staticPressureBuffer[i] = b2Clamp(h, 0.0, maxPressure);
                }
                else {
                    this.m_staticPressureBuffer[i] = 0;
                }
            }
        }
    }
    ComputeWeight() {
        // calculates the sum of contact-weights for each particle
        // that means dimensionless density
        ///memset(m_weightBuffer, 0, sizeof(*m_weightBuffer) * m_count);
        for (let k = 0; k < this.m_count; k++) {
            this.m_weightBuffer[k] = 0;
        }
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const w = contact.weight;
            this.m_weightBuffer[a] += w;
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            this.m_weightBuffer[a] += w;
            this.m_weightBuffer[b] += w;
        }
    }
    SolvePressure(step) {
        const s_f = b2ParticleSystem.SolvePressure_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // calculates pressure as a linear function of density
        const criticalPressure = this.GetCriticalPressure(step);
        const pressurePerWeight = this.m_def.pressureStrength * criticalPressure;
        const maxPressure = b2_maxParticlePressure * criticalPressure;
        for (let i = 0; i < this.m_count; i++) {
            const w = this.m_weightBuffer[i];
            const h = pressurePerWeight * b2Max(0.0, w - b2_minParticleWeight);
            this.m_accumulationBuffer[i] = b2Min(h, maxPressure);
        }
        // ignores particles which have their own repulsive force
        if (this.m_allParticleFlags & b2ParticleSystem.k_noPressureFlags) {
            for (let i = 0; i < this.m_count; i++) {
                if (this.m_flagsBuffer.data[i] & b2ParticleSystem.k_noPressureFlags) {
                    this.m_accumulationBuffer[i] = 0;
                }
            }
        }
        // static pressure
        if (this.m_allParticleFlags & 2048 /* b2_staticPressureParticle */) {
            !!B2_DEBUG && b2Assert(this.m_staticPressureBuffer !== null);
            for (let i = 0; i < this.m_count; i++) {
                if (this.m_flagsBuffer.data[i] & 2048 /* b2_staticPressureParticle */) {
                    this.m_accumulationBuffer[i] += this.m_staticPressureBuffer[i];
                }
            }
        }
        // applies pressure between each particles in contact
        const velocityPerPressure = step.dt / (this.m_def.density * this.m_particleDiameter);
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const b = contact.body;
            const w = contact.weight;
            const m = contact.mass;
            const n = contact.normal;
            const p = pos_data[a];
            const h = this.m_accumulationBuffer[a] + pressurePerWeight * w;
            ///b2Vec2 f = velocityPerPressure * w * m * h * n;
            const f = b2Vec2.MulSV(velocityPerPressure * w * m * h, n, s_f);
            ///m_velocityBuffer.data[a] -= GetParticleInvMass() * f;
            vel_data[a].SelfMulSub(inv_mass, f);
            b.ApplyLinearImpulse(f, p, true);
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            const n = contact.normal;
            const h = this.m_accumulationBuffer[a] + this.m_accumulationBuffer[b];
            ///b2Vec2 f = velocityPerPressure * w * h * n;
            const f = b2Vec2.MulSV(velocityPerPressure * w * h, n, s_f);
            ///m_velocityBuffer.data[a] -= f;
            vel_data[a].SelfSub(f);
            ///m_velocityBuffer.data[b] += f;
            vel_data[b].SelfAdd(f);
        }
    }
    SolveDamping(step) {
        const s_v = b2ParticleSystem.SolveDamping_s_v;
        const s_f = b2ParticleSystem.SolveDamping_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        // reduces normal velocity of each contact
        const linearDamping = this.m_def.dampingStrength;
        const quadraticDamping = 1 / this.GetCriticalVelocity(step);
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const b = contact.body;
            const w = contact.weight;
            const m = contact.mass;
            const n = contact.normal;
            const p = pos_data[a];
            ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - m_velocityBuffer.data[a];
            const v = b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, b2Vec2.s_t0), vel_data[a], s_v);
            const vn = b2Vec2.DotVV(v, n);
            if (vn < 0) {
                const damping = b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5));
                ///b2Vec2 f = damping * m * vn * n;
                const f = b2Vec2.MulSV(damping * m * vn, n, s_f);
                ///m_velocityBuffer.data[a] += GetParticleInvMass() * f;
                vel_data[a].SelfMulAdd(inv_mass, f);
                ///b.ApplyLinearImpulse(-f, p, true);
                b.ApplyLinearImpulse(f.SelfNeg(), p, true);
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const w = contact.weight;
            const n = contact.normal;
            ///b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
            const v = b2Vec2.SubVV(vel_data[b], vel_data[a], s_v);
            const vn = b2Vec2.DotVV(v, n);
            if (vn < 0) {
                ///float32 damping = b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
                const damping = b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5));
                ///b2Vec2 f = damping * vn * n;
                const f = b2Vec2.MulSV(damping * vn, n, s_f);
                ///this.m_velocityBuffer.data[a] += f;
                vel_data[a].SelfAdd(f);
                ///this.m_velocityBuffer.data[b] -= f;
                vel_data[b].SelfSub(f);
            }
        }
    }
    SolveRigidDamping() {
        const s_t0 = b2ParticleSystem.SolveRigidDamping_s_t0;
        const s_t1 = b2ParticleSystem.SolveRigidDamping_s_t1;
        const s_p = b2ParticleSystem.SolveRigidDamping_s_p;
        const s_v = b2ParticleSystem.SolveRigidDamping_s_v;
        const invMassA = [0.0], invInertiaA = [0.0], tangentDistanceA = [0.0]; // TODO: static
        const invMassB = [0.0], invInertiaB = [0.0], tangentDistanceB = [0.0]; // TODO: static
        // Apply impulse to rigid particle groups colliding with other objects
        // to reduce relative velocity at the colliding point.
        const pos_data = this.m_positionBuffer.data;
        const damping = this.m_def.dampingStrength;
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            const aGroup = this.m_groupBuffer[a];
            if (aGroup && this.IsRigidGroup(aGroup)) {
                const b = contact.body;
                const n = contact.normal;
                const w = contact.weight;
                const p = pos_data[a];
                ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - aGroup.GetLinearVelocityFromWorldPoint(p);
                const v = b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, s_t0), aGroup.GetLinearVelocityFromWorldPoint(p, s_t1), s_v);
                const vn = b2Vec2.DotVV(v, n);
                if (vn < 0) {
                    // The group's average velocity at particle position 'p' is pushing
                    // the particle into the body.
                    ///this.InitDampingParameterWithRigidGroupOrParticle(&invMassA, &invInertiaA, &tangentDistanceA, true, aGroup, a, p, n);
                    this.InitDampingParameterWithRigidGroupOrParticle(invMassA, invInertiaA, tangentDistanceA, true, aGroup, a, p, n);
                    // Calculate b.m_I from functions of b2Body.
                    ///this.InitDampingParameter(&invMassB, &invInertiaB, &tangentDistanceB, b.GetMass(), b.GetInertia() - b.GetMass() * b.GetLocalCenter().LengthSquared(), b.GetWorldCenter(), p, n);
                    this.InitDampingParameter(invMassB, invInertiaB, tangentDistanceB, b.GetMass(), b.GetInertia() - b.GetMass() * b.GetLocalCenter().LengthSquared(), b.GetWorldCenter(), p, n);
                    ///float32 f = damping * b2Min(w, 1.0) * this.ComputeDampingImpulse(invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, vn);
                    const f = damping *
                        b2Min(w, 1.0) *
                        this.ComputeDampingImpulse(invMassA[0], invInertiaA[0], tangentDistanceA[0], invMassB[0], invInertiaB[0], tangentDistanceB[0], vn);
                    ///this.ApplyDamping(invMassA, invInertiaA, tangentDistanceA, true, aGroup, a, f, n);
                    this.ApplyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], true, aGroup, a, f, n);
                    ///b.ApplyLinearImpulse(-f * n, p, true);
                    b.ApplyLinearImpulse(b2Vec2.MulSV(-f, n, b2Vec2.s_t0), p, true);
                }
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            const n = contact.normal;
            const w = contact.weight;
            const aGroup = this.m_groupBuffer[a];
            const bGroup = this.m_groupBuffer[b];
            const aRigid = this.IsRigidGroup(aGroup);
            const bRigid = this.IsRigidGroup(bGroup);
            if (aGroup !== bGroup && (aRigid || bRigid)) {
                ///b2Vec2 p = 0.5f * (this.m_positionBuffer.data[a] + this.m_positionBuffer.data[b]);
                const p = b2Vec2.MidVV(pos_data[a], pos_data[b], s_p);
                ///b2Vec2 v = GetLinearVelocity(bGroup, b, p) - GetLinearVelocity(aGroup, a, p);
                const v = b2Vec2.SubVV(this.GetLinearVelocity(bGroup, b, p, s_t0), this.GetLinearVelocity(aGroup, a, p, s_t1), s_v);
                const vn = b2Vec2.DotVV(v, n);
                if (vn < 0) {
                    ///this.InitDampingParameterWithRigidGroupOrParticle(&invMassA, &invInertiaA, &tangentDistanceA, aRigid, aGroup, a, p, n);
                    this.InitDampingParameterWithRigidGroupOrParticle(invMassA, invInertiaA, tangentDistanceA, aRigid, aGroup, a, p, n);
                    ///this.InitDampingParameterWithRigidGroupOrParticle(&invMassB, &invInertiaB, &tangentDistanceB, bRigid, bGroup, b, p, n);
                    this.InitDampingParameterWithRigidGroupOrParticle(invMassB, invInertiaB, tangentDistanceB, bRigid, bGroup, b, p, n);
                    ///float32 f = damping * w * this.ComputeDampingImpulse(invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, vn);
                    const f = damping *
                        w *
                        this.ComputeDampingImpulse(invMassA[0], invInertiaA[0], tangentDistanceA[0], invMassB[0], invInertiaB[0], tangentDistanceB[0], vn);
                    ///this.ApplyDamping(invMassA, invInertiaA, tangentDistanceA, aRigid, aGroup, a, f, n);
                    this.ApplyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], aRigid, aGroup, a, f, n);
                    ///this.ApplyDamping(invMassB, invInertiaB, tangentDistanceB, bRigid, bGroup, b, -f, n);
                    this.ApplyDamping(invMassB[0], invInertiaB[0], tangentDistanceB[0], bRigid, bGroup, b, -f, n);
                }
            }
        }
    }
    SolveExtraDamping() {
        const s_v = b2ParticleSystem.SolveExtraDamping_s_v;
        const s_f = b2ParticleSystem.SolveExtraDamping_s_f;
        const vel_data = this.m_velocityBuffer.data;
        // Applies additional damping force between bodies and particles which can
        // produce strong repulsive force. Applying damping force multiple times
        // is effective in suppressing vibration.
        const pos_data = this.m_positionBuffer.data;
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.m_flagsBuffer.data[a] & b2ParticleSystem.k_extraDampingFlags) {
                const b = contact.body;
                const m = contact.mass;
                const n = contact.normal;
                const p = pos_data[a];
                ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - m_velocityBuffer.data[a];
                const v = b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, b2Vec2.s_t0), vel_data[a], s_v);
                ///float32 vn = b2Dot(v, n);
                const vn = b2Vec2.DotVV(v, n);
                if (vn < 0) {
                    ///b2Vec2 f = 0.5f * m * vn * n;
                    const f = b2Vec2.MulSV(0.5 * m * vn, n, s_f);
                    ///m_velocityBuffer.data[a] += GetParticleInvMass() * f;
                    vel_data[a].SelfMulAdd(inv_mass, f);
                    ///b.ApplyLinearImpulse(-f, p, true);
                    b.ApplyLinearImpulse(f.SelfNeg(), p, true);
                }
            }
        }
    }
    SolveWall() {
        const vel_data = this.m_velocityBuffer.data;
        for (let i = 0; i < this.m_count; i++) {
            if (this.m_flagsBuffer.data[i] & 4 /* b2_wallParticle */) {
                vel_data[i].SetZero();
            }
        }
    }
    SolveRigid(step) {
        const s_position = b2ParticleSystem.SolveRigid_s_position;
        const s_rotation = b2ParticleSystem.SolveRigid_s_rotation;
        const s_transform = b2ParticleSystem.SolveRigid_s_transform;
        const s_velocityTransform = b2ParticleSystem.SolveRigid_s_velocityTransform;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            if (group.m_groupFlags & 2 /* b2_rigidParticleGroup */) {
                group.UpdateStatistics();
                ///b2Rot rotation(step.dt * group.m_angularVelocity);
                const rotation = s_rotation;
                rotation.SetAngle(step.dt * group.m_angularVelocity);
                ///b2Transform transform(group.m_center + step.dt * group.m_linearVelocity - b2Mul(rotation, group.m_center), rotation);
                const position = b2Vec2.AddVV(group.m_center, b2Vec2.SubVV(b2Vec2.MulSV(step.dt, group.m_linearVelocity, b2Vec2.s_t0), b2Rot.MulRV(rotation, group.m_center, b2Vec2.s_t1), b2Vec2.s_t0), s_position);
                const transform = s_transform;
                transform.SetPositionRotation(position, rotation);
                ///group.m_transform = b2Mul(transform, group.m_transform);
                b2Transform.MulXX(transform, group.m_transform, group.m_transform);
                const velocityTransform = s_velocityTransform;
                velocityTransform.p.x = step.inv_dt * transform.p.x;
                velocityTransform.p.y = step.inv_dt * transform.p.y;
                velocityTransform.q.s = step.inv_dt * transform.q.s;
                velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
                for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                    ///m_velocityBuffer.data[i] = b2Mul(velocityTransform, m_positionBuffer.data[i]);
                    b2Transform.MulXV(velocityTransform, pos_data[i], vel_data[i]);
                }
            }
        }
    }
    SolveElastic(step) {
        const s_pa = b2ParticleSystem.SolveElastic_s_pa;
        const s_pb = b2ParticleSystem.SolveElastic_s_pb;
        const s_pc = b2ParticleSystem.SolveElastic_s_pc;
        const s_r = b2ParticleSystem.SolveElastic_s_r;
        const s_t0 = b2ParticleSystem.SolveElastic_s_t0;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const elasticStrength = step.inv_dt * this.m_def.elasticStrength;
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            if (triad.flags & 16 /* b2_elasticParticle */) {
                const a = triad.indexA;
                const b = triad.indexB;
                const c = triad.indexC;
                const oa = triad.pa;
                const ob = triad.pb;
                const oc = triad.pc;
                ///b2Vec2 pa = m_positionBuffer.data[a];
                const pa = s_pa.Copy(pos_data[a]);
                ///b2Vec2 pb = m_positionBuffer.data[b];
                const pb = s_pb.Copy(pos_data[b]);
                ///b2Vec2 pc = m_positionBuffer.data[c];
                const pc = s_pc.Copy(pos_data[c]);
                const va = vel_data[a];
                const vb = vel_data[b];
                const vc = vel_data[c];
                ///pa += step.dt * va;
                pa.SelfMulAdd(step.dt, va);
                ///pb += step.dt * vb;
                pb.SelfMulAdd(step.dt, vb);
                ///pc += step.dt * vc;
                pc.SelfMulAdd(step.dt, vc);
                ///b2Vec2 midPoint = (float32) 1 / 3 * (pa + pb + pc);
                const midPoint_x = (pa.x + pb.x + pc.x) / 3.0;
                const midPoint_y = (pa.y + pb.y + pc.y) / 3.0;
                ///pa -= midPoint;
                pa.x -= midPoint_x;
                pa.y -= midPoint_y;
                ///pb -= midPoint;
                pb.x -= midPoint_x;
                pb.y -= midPoint_y;
                ///pc -= midPoint;
                pc.x -= midPoint_x;
                pc.y -= midPoint_y;
                ///b2Rot r;
                const r = s_r;
                r.s = b2Vec2.CrossVV(oa, pa) + b2Vec2.CrossVV(ob, pb) + b2Vec2.CrossVV(oc, pc);
                r.c = b2Vec2.DotVV(oa, pa) + b2Vec2.DotVV(ob, pb) + b2Vec2.DotVV(oc, pc);
                const r2 = r.s * r.s + r.c * r.c;
                let invR = b2InvSqrt(r2);
                if (!isFinite(invR)) {
                    invR = 1.98177537e19;
                }
                r.s *= invR;
                r.c *= invR;
                ///r.angle = Math.atan2(r.s, r.c); // TODO: optimize
                const strength = elasticStrength * triad.strength;
                ///va += strength * (b2Mul(r, oa) - pa);
                b2Rot.MulRV(r, oa, s_t0);
                b2Vec2.SubVV(s_t0, pa, s_t0);
                b2Vec2.MulSV(strength, s_t0, s_t0);
                va.SelfAdd(s_t0);
                ///vb += strength * (b2Mul(r, ob) - pb);
                b2Rot.MulRV(r, ob, s_t0);
                b2Vec2.SubVV(s_t0, pb, s_t0);
                b2Vec2.MulSV(strength, s_t0, s_t0);
                vb.SelfAdd(s_t0);
                ///vc += strength * (b2Mul(r, oc) - pc);
                b2Rot.MulRV(r, oc, s_t0);
                b2Vec2.SubVV(s_t0, pc, s_t0);
                b2Vec2.MulSV(strength, s_t0, s_t0);
                vc.SelfAdd(s_t0);
            }
        }
    }
    SolveSpring(step) {
        const s_pa = b2ParticleSystem.SolveSpring_s_pa;
        const s_pb = b2ParticleSystem.SolveSpring_s_pb;
        const s_d = b2ParticleSystem.SolveSpring_s_d;
        const s_f = b2ParticleSystem.SolveSpring_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const springStrength = step.inv_dt * this.m_def.springStrength;
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            if (pair.flags & 8 /* b2_springParticle */) {
                ///int32 a = pair.indexA;
                const a = pair.indexA;
                ///int32 b = pair.indexB;
                const b = pair.indexB;
                ///b2Vec2 pa = m_positionBuffer.data[a];
                const pa = s_pa.Copy(pos_data[a]);
                ///b2Vec2 pb = m_positionBuffer.data[b];
                const pb = s_pb.Copy(pos_data[b]);
                ///b2Vec2& va = m_velocityBuffer.data[a];
                const va = vel_data[a];
                ///b2Vec2& vb = m_velocityBuffer.data[b];
                const vb = vel_data[b];
                ///pa += step.dt * va;
                pa.SelfMulAdd(step.dt, va);
                ///pb += step.dt * vb;
                pb.SelfMulAdd(step.dt, vb);
                ///b2Vec2 d = pb - pa;
                const d = b2Vec2.SubVV(pb, pa, s_d);
                ///float32 r0 = pair.distance;
                const r0 = pair.distance;
                ///float32 r1 = d.Length();
                const r1 = d.Length();
                ///float32 strength = springStrength * pair.strength;
                const strength = springStrength * pair.strength;
                ///b2Vec2 f = strength * (r0 - r1) / r1 * d;
                const f = b2Vec2.MulSV((strength * (r0 - r1)) / r1, d, s_f);
                ///va -= f;
                va.SelfSub(f);
                ///vb += f;
                vb.SelfAdd(f);
            }
        }
    }
    SolveTensile(step) {
        const s_weightedNormal = b2ParticleSystem.SolveTensile_s_weightedNormal;
        const s_s = b2ParticleSystem.SolveTensile_s_s;
        const s_f = b2ParticleSystem.SolveTensile_s_f;
        const vel_data = this.m_velocityBuffer.data;
        !!B2_DEBUG && b2Assert(this.m_accumulation2Buffer !== null);
        for (let i = 0; i < this.m_count; i++) {
            this.m_accumulation2Buffer[i] = new b2Vec2();
            this.m_accumulation2Buffer[i].SetZero();
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & 128 /* b2_tensileParticle */) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const n = contact.normal;
                ///b2Vec2 weightedNormal = (1 - w) * w * n;
                const weightedNormal = b2Vec2.MulSV((1 - w) * w, n, s_weightedNormal);
                ///m_accumulation2Buffer[a] -= weightedNormal;
                this.m_accumulation2Buffer[a].SelfSub(weightedNormal);
                ///m_accumulation2Buffer[b] += weightedNormal;
                this.m_accumulation2Buffer[b].SelfAdd(weightedNormal);
            }
        }
        const criticalVelocity = this.GetCriticalVelocity(step);
        const pressureStrength = this.m_def.surfaceTensionPressureStrength * criticalVelocity;
        const normalStrength = this.m_def.surfaceTensionNormalStrength * criticalVelocity;
        const maxVelocityVariation = b2_maxParticleForce * criticalVelocity;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & 128 /* b2_tensileParticle */) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                const n = contact.normal;
                const h = this.m_weightBuffer[a] + this.m_weightBuffer[b];
                ///b2Vec2 s = m_accumulation2Buffer[b] - m_accumulation2Buffer[a];
                const s = b2Vec2.SubVV(this.m_accumulation2Buffer[b], this.m_accumulation2Buffer[a], s_s);
                const fn = b2Min(pressureStrength * (h - 2) + normalStrength * b2Vec2.DotVV(s, n), maxVelocityVariation) * w;
                ///b2Vec2 f = fn * n;
                const f = b2Vec2.MulSV(fn, n, s_f);
                ///m_velocityBuffer.data[a] -= f;
                vel_data[a].SelfSub(f);
                ///m_velocityBuffer.data[b] += f;
                vel_data[b].SelfAdd(f);
            }
        }
    }
    SolveViscous() {
        const s_v = b2ParticleSystem.SolveViscous_s_v;
        const s_f = b2ParticleSystem.SolveViscous_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const viscousStrength = this.m_def.viscousStrength;
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.m_flagsBuffer.data[a] & 32 /* b2_viscousParticle */) {
                const b = contact.body;
                const w = contact.weight;
                const m = contact.mass;
                const p = pos_data[a];
                ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - m_velocityBuffer.data[a];
                const v = b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, b2Vec2.s_t0), vel_data[a], s_v);
                ///b2Vec2 f = viscousStrength * m * w * v;
                const f = b2Vec2.MulSV(viscousStrength * m * w, v, s_f);
                ///m_velocityBuffer.data[a] += GetParticleInvMass() * f;
                vel_data[a].SelfMulAdd(inv_mass, f);
                ///b.ApplyLinearImpulse(-f, p, true);
                b.ApplyLinearImpulse(f.SelfNeg(), p, true);
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & 32 /* b2_viscousParticle */) {
                const a = contact.indexA;
                const b = contact.indexB;
                const w = contact.weight;
                ///b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
                const v = b2Vec2.SubVV(vel_data[b], vel_data[a], s_v);
                ///b2Vec2 f = viscousStrength * w * v;
                const f = b2Vec2.MulSV(viscousStrength * w, v, s_f);
                ///m_velocityBuffer.data[a] += f;
                vel_data[a].SelfAdd(f);
                ///m_velocityBuffer.data[b] -= f;
                vel_data[b].SelfSub(f);
            }
        }
    }
    SolveRepulsive(step) {
        const s_f = b2ParticleSystem.SolveRepulsive_s_f;
        const vel_data = this.m_velocityBuffer.data;
        const repulsiveStrength = this.m_def.repulsiveStrength * this.GetCriticalVelocity(step);
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & 8192 /* b2_repulsiveParticle */) {
                const a = contact.indexA;
                const b = contact.indexB;
                if (this.m_groupBuffer[a] !== this.m_groupBuffer[b]) {
                    const w = contact.weight;
                    const n = contact.normal;
                    ///b2Vec2 f = repulsiveStrength * w * n;
                    const f = b2Vec2.MulSV(repulsiveStrength * w, n, s_f);
                    ///m_velocityBuffer.data[a] -= f;
                    vel_data[a].SelfSub(f);
                    ///m_velocityBuffer.data[b] += f;
                    vel_data[b].SelfAdd(f);
                }
            }
        }
    }
    SolvePowder(step) {
        const s_f = b2ParticleSystem.SolvePowder_s_f;
        const pos_data = this.m_positionBuffer.data;
        const vel_data = this.m_velocityBuffer.data;
        const powderStrength = this.m_def.powderStrength * this.GetCriticalVelocity(step);
        const minWeight = 1.0 - b2_particleStride;
        const inv_mass = this.GetParticleInvMass();
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            const a = contact.index;
            if (this.m_flagsBuffer.data[a] & 64 /* b2_powderParticle */) {
                const w = contact.weight;
                if (w > minWeight) {
                    const b = contact.body;
                    const m = contact.mass;
                    const p = pos_data[a];
                    const n = contact.normal;
                    const f = b2Vec2.MulSV(powderStrength * m * (w - minWeight), n, s_f);
                    vel_data[a].SelfMulSub(inv_mass, f);
                    b.ApplyLinearImpulse(f, p, true);
                }
            }
        }
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            if (contact.flags & 64 /* b2_powderParticle */) {
                const w = contact.weight;
                if (w > minWeight) {
                    const a = contact.indexA;
                    const b = contact.indexB;
                    const n = contact.normal;
                    const f = b2Vec2.MulSV(powderStrength * (w - minWeight), n, s_f);
                    vel_data[a].SelfSub(f);
                    vel_data[b].SelfAdd(f);
                }
            }
        }
    }
    SolveSolid(step) {
        const s_f = b2ParticleSystem.SolveSolid_s_f;
        const vel_data = this.m_velocityBuffer.data;
        // applies extra repulsive force from solid particle groups
        this.m_depthBuffer = this.RequestBuffer(this.m_depthBuffer);
        const ejectionStrength = step.inv_dt * this.m_def.ejectionStrength;
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            const a = contact.indexA;
            const b = contact.indexB;
            if (this.m_groupBuffer[a] !== this.m_groupBuffer[b]) {
                const w = contact.weight;
                const n = contact.normal;
                const h = this.m_depthBuffer[a] + this.m_depthBuffer[b];
                const f = b2Vec2.MulSV(ejectionStrength * h * w, n, s_f);
                vel_data[a].SelfSub(f);
                vel_data[b].SelfAdd(f);
            }
        }
    }
    SolveForce(step) {
        const vel_data = this.m_velocityBuffer.data;
        const velocityPerForce = step.dt * this.GetParticleInvMass();
        for (let i = 0; i < this.m_count; i++) {
            ///m_velocityBuffer.data[i] += velocityPerForce * m_forceBuffer[i];
            vel_data[i].SelfMulAdd(velocityPerForce, this.m_forceBuffer[i]);
        }
        this.m_hasForce = false;
    }
    SolveColorMixing() {
        // mixes color between contacting particles
        const colorMixing = 0.5 * this.m_def.colorMixingStrength;
        if (colorMixing) {
            for (let k = 0; k < this.m_contactBuffer.count; k++) {
                const contact = this.m_contactBuffer.data[k];
                const a = contact.indexA;
                const b = contact.indexB;
                if (this.m_flagsBuffer.data[a] &
                    this.m_flagsBuffer.data[b] &
                    256 /* b2_colorMixingParticle */) {
                    const colorA = this.m_colorBuffer.data[a];
                    const colorB = this.m_colorBuffer.data[b];
                    // Use the static method to ensure certain compilers inline
                    // this correctly.
                    b2Color.MixColors(colorA, colorB, colorMixing);
                }
            }
        }
    }
    SolveZombie() {
        // removes particles with zombie flag
        let newCount = 0;
        const newIndicesArray = []; // TODO: static
        for (let i = 0; i < this.m_count; i++) {
            newIndicesArray[i] = b2_invalidParticleIndex;
        }
        !!B2_DEBUG && b2Assert(newIndicesArray.length === this.m_count);
        let allParticleFlags = 0;
        for (let i = 0; i < this.m_count; i++) {
            const flags = this.m_flagsBuffer.data[i];
            if (flags & 2 /* b2_zombieParticle */) {
                const destructionListener = this.m_world.m_destructionListener;
                if (flags & 512 /* b2_destructionListenerParticle */ && destructionListener) {
                    destructionListener.SayGoodbyeParticle(this, i);
                }
                // Destroy particle handle.
                if (this.m_handleIndexBuffer.data) {
                    const handle = this.m_handleIndexBuffer.data[i];
                    if (handle) {
                        handle.index = b2_invalidParticleIndex;
                        this.m_handleIndexBuffer.data[i] = null;
                        ///m_handleAllocator.Free(handle);
                    }
                }
                newIndicesArray[i] = b2_invalidParticleIndex;
            }
            else {
                newIndicesArray[i] = newCount;
                if (i !== newCount) {
                    // Update handle to reference new particle index.
                    if (this.m_handleIndexBuffer.data) {
                        const handle = this.m_handleIndexBuffer.data[i];
                        if (handle) {
                            handle.index = newCount;
                        }
                        this.m_handleIndexBuffer.data[newCount] = handle;
                    }
                    this.m_flagsBuffer.data[newCount] = this.m_flagsBuffer.data[i];
                    if (this.m_lastBodyContactStepBuffer.data) {
                        this.m_lastBodyContactStepBuffer.data[newCount] = this.m_lastBodyContactStepBuffer.data[i];
                    }
                    if (this.m_bodyContactCountBuffer.data) {
                        this.m_bodyContactCountBuffer.data[newCount] = this.m_bodyContactCountBuffer.data[i];
                    }
                    if (this.m_consecutiveContactStepsBuffer.data) {
                        this.m_consecutiveContactStepsBuffer.data[newCount] = this.m_consecutiveContactStepsBuffer.data[i];
                    }
                    this.m_positionBuffer.data[newCount].Copy(this.m_positionBuffer.data[i]);
                    this.m_velocityBuffer.data[newCount].Copy(this.m_velocityBuffer.data[i]);
                    this.m_groupBuffer[newCount] = this.m_groupBuffer[i];
                    if (this.m_hasForce) {
                        this.m_forceBuffer[newCount].Copy(this.m_forceBuffer[i]);
                    }
                    if (this.m_staticPressureBuffer) {
                        this.m_staticPressureBuffer[newCount] = this.m_staticPressureBuffer[i];
                    }
                    if (this.m_depthBuffer) {
                        this.m_depthBuffer[newCount] = this.m_depthBuffer[i];
                    }
                    if (this.m_colorBuffer.data) {
                        this.m_colorBuffer.data[newCount].Copy(this.m_colorBuffer.data[i]);
                    }
                    if (this.m_userDataBuffer.data) {
                        this.m_userDataBuffer.data[newCount] = this.m_userDataBuffer.data[i];
                    }
                    if (this.m_expirationTimeBuffer.data) {
                        this.m_expirationTimeBuffer.data[newCount] = this.m_expirationTimeBuffer.data[i];
                    }
                }
                newCount++;
                allParticleFlags |= flags;
            }
        }
        // predicate functions
        const Test = {
            ///static bool IsProxyInvalid(const Proxy& proxy)
            IsProxyInvalid: (proxy) => {
                return proxy.index < 0;
            },
            ///static bool IsContactInvalid(const b2ParticleContact& contact)
            IsContactInvalid: (contact) => {
                return contact.indexA < 0 || contact.indexB < 0;
            },
            ///static bool IsBodyContactInvalid(const b2ParticleBodyContact& contact)
            IsBodyContactInvalid: (contact) => {
                return contact.index < 0;
            },
            ///static bool IsPairInvalid(const b2ParticlePair& pair)
            IsPairInvalid: (pair) => {
                return pair.indexA < 0 || pair.indexB < 0;
            },
            ///static bool IsTriadInvalid(const b2ParticleTriad& triad)
            IsTriadInvalid: (triad) => {
                return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
            },
        };
        // update proxies
        for (let k = 0; k < this.m_proxyBuffer.count; k++) {
            const proxy = this.m_proxyBuffer.data[k];
            proxy.index = newIndicesArray[proxy.index];
        }
        this.m_proxyBuffer.RemoveIf(Test.IsProxyInvalid);
        // update contacts
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            contact.indexA = newIndicesArray[contact.indexA];
            contact.indexB = newIndicesArray[contact.indexB];
        }
        this.m_contactBuffer.RemoveIf(Test.IsContactInvalid);
        // update particle-body contacts
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            contact.index = newIndicesArray[contact.index];
        }
        this.m_bodyContactBuffer.RemoveIf(Test.IsBodyContactInvalid);
        // update pairs
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            pair.indexA = newIndicesArray[pair.indexA];
            pair.indexB = newIndicesArray[pair.indexB];
        }
        this.m_pairBuffer.RemoveIf(Test.IsPairInvalid);
        // update triads
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            triad.indexA = newIndicesArray[triad.indexA];
            triad.indexB = newIndicesArray[triad.indexB];
            triad.indexC = newIndicesArray[triad.indexC];
        }
        this.m_triadBuffer.RemoveIf(Test.IsTriadInvalid);
        // Update lifetime indices.
        if (this.m_indexByExpirationTimeBuffer.data) {
            let writeOffset = 0;
            for (let readOffset = 0; readOffset < this.m_count; readOffset++) {
                const newIndex = newIndicesArray[this.m_indexByExpirationTimeBuffer.data[readOffset]];
                if (newIndex !== b2_invalidParticleIndex) {
                    this.m_indexByExpirationTimeBuffer.data[writeOffset++] = newIndex;
                }
            }
        }
        // update groups
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            let firstIndex = newCount;
            let lastIndex = 0;
            let modified = false;
            for (let i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                const j = newIndicesArray[i];
                if (j >= 0) {
                    firstIndex = b2MinInt(firstIndex, j);
                    lastIndex = b2MaxInt(lastIndex, j + 1);
                }
                else {
                    modified = true;
                }
            }
            if (firstIndex < lastIndex) {
                group.m_firstIndex = firstIndex;
                group.m_lastIndex = lastIndex;
                if (modified) {
                    if (group.m_groupFlags & 1 /* b2_solidParticleGroup */) {
                        this.SetGroupFlags(group, group.m_groupFlags | 16 /* b2_particleGroupNeedsUpdateDepth */);
                    }
                }
            }
            else {
                group.m_firstIndex = 0;
                group.m_lastIndex = 0;
                if (!(group.m_groupFlags & 4 /* b2_particleGroupCanBeEmpty */)) {
                    this.SetGroupFlags(group, group.m_groupFlags | 8 /* b2_particleGroupWillBeDestroyed */);
                }
            }
        }
        // update particle count
        this.m_count = newCount;
        this.m_allParticleFlags = allParticleFlags;
        this.m_needsUpdateAllParticleFlags = false;
        // destroy bodies with no particles
        for (let group = this.m_groupList; group;) {
            const next = group.GetNext();
            if (group.m_groupFlags & 8 /* b2_particleGroupWillBeDestroyed */) {
                this.DestroyParticleGroup(group);
            }
            group = next;
        }
    }
    /**
     * Destroy all particles which have outlived their lifetimes set
     * by SetParticleLifetime().
     */
    SolveLifetimes(step) {
        // Update the time elapsed.
        this.m_timeElapsed = this.LifetimeToExpirationTime(step.dt);
        // Get the floor (non-fractional component) of the elapsed time.
        const quantizedTimeElapsed = this.GetQuantizedTimeElapsed();
        const expirationTimes = this.m_expirationTimeBuffer.data;
        const expirationTimeIndices = this.m_indexByExpirationTimeBuffer.data;
        const particleCount = this.GetParticleCount();
        // Sort the lifetime buffer if it's required.
        if (this.m_expirationTimeBufferRequiresSorting) {
            ///const ExpirationTimeComparator expirationTimeComparator(expirationTimes);
            ///std::sort(expirationTimeIndices, expirationTimeIndices + particleCount, expirationTimeComparator);
            /**
             * Compare the lifetime of particleIndexA and particleIndexB
             * returning true if the lifetime of A is greater than B for
             * particles that will expire.  If either particle's lifetime is
             * infinite (<= 0.0f) this function return true if the lifetime
             * of A is lesser than B. When used with std::sort() this
             * results in an array of particle indicies sorted in reverse
             * order by particle lifetime.
             *
             * For example, the set of lifetimes
             * (1.0, 0.7, 0.3, 0.0, -1.0, 2.0)
             * would be sorted as
             * (0.0, 1.0, -2.0, 1.0, 0.7, 0.3)
             */
            const ExpirationTimeComparator = (particleIndexA, particleIndexB) => {
                const expirationTimeA = expirationTimes[particleIndexA];
                const expirationTimeB = expirationTimes[particleIndexB];
                const infiniteExpirationTimeA = expirationTimeA <= 0.0;
                const infiniteExpirationTimeB = expirationTimeB <= 0.0;
                return infiniteExpirationTimeA === infiniteExpirationTimeB
                    ? expirationTimeA > expirationTimeB
                    : infiniteExpirationTimeA;
            };
            std_sort(expirationTimeIndices, 0, particleCount, ExpirationTimeComparator);
            this.m_expirationTimeBufferRequiresSorting = false;
        }
        // Destroy particles which have expired.
        for (let i = particleCount - 1; i >= 0; --i) {
            const particleIndex = expirationTimeIndices[i];
            const expirationTime = expirationTimes[particleIndex];
            // If no particles need to be destroyed, skip this.
            if (quantizedTimeElapsed < expirationTime || expirationTime <= 0) {
                break;
            }
            // Destroy this particle.
            this.DestroyParticle(particleIndex);
        }
    }
    RotateBuffer(start, mid, end) {
        // move the particles assigned to the given group toward the end of array
        if (start === mid || mid === end) {
            return;
        }
        !!B2_DEBUG && b2Assert(mid >= start && mid <= end);
        ///std::rotate(m_flagsBuffer.data + start, m_flagsBuffer.data + mid, m_flagsBuffer.data + end);
        std_rotate(this.m_flagsBuffer.data, start, mid, end);
        if (this.m_lastBodyContactStepBuffer.data) {
            ///std::rotate(m_lastBodyContactStepBuffer.data + start, m_lastBodyContactStepBuffer.data + mid, m_lastBodyContactStepBuffer.data + end);
            std_rotate(this.m_lastBodyContactStepBuffer.data, start, mid, end);
        }
        if (this.m_bodyContactCountBuffer.data) {
            ///std::rotate(m_bodyContactCountBuffer.data + start, m_bodyContactCountBuffer.data + mid, m_bodyContactCountBuffer.data + end);
            std_rotate(this.m_bodyContactCountBuffer.data, start, mid, end);
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            ///std::rotate(m_consecutiveContactStepsBuffer.data + start, m_consecutiveContactStepsBuffer.data + mid, m_consecutiveContactStepsBuffer.data + end);
            std_rotate(this.m_consecutiveContactStepsBuffer.data, start, mid, end);
        }
        ///std::rotate(m_positionBuffer.data + start, m_positionBuffer.data + mid, m_positionBuffer.data + end);
        std_rotate(this.m_positionBuffer.data, start, mid, end);
        ///std::rotate(m_velocityBuffer.data + start, m_velocityBuffer.data + mid, m_velocityBuffer.data + end);
        std_rotate(this.m_velocityBuffer.data, start, mid, end);
        ///std::rotate(m_groupBuffer + start, m_groupBuffer + mid, m_groupBuffer + end);
        std_rotate(this.m_groupBuffer, start, mid, end);
        if (this.m_hasForce) {
            ///std::rotate(m_forceBuffer + start, m_forceBuffer + mid, m_forceBuffer + end);
            std_rotate(this.m_forceBuffer, start, mid, end);
        }
        if (this.m_staticPressureBuffer) {
            ///std::rotate(m_staticPressureBuffer + start, m_staticPressureBuffer + mid, m_staticPressureBuffer + end);
            std_rotate(this.m_staticPressureBuffer, start, mid, end);
        }
        if (this.m_depthBuffer) {
            ///std::rotate(m_depthBuffer + start, m_depthBuffer + mid, m_depthBuffer + end);
            std_rotate(this.m_depthBuffer, start, mid, end);
        }
        if (this.m_colorBuffer.data) {
            ///std::rotate(m_colorBuffer.data + start, m_colorBuffer.data + mid, m_colorBuffer.data + end);
            std_rotate(this.m_colorBuffer.data, start, mid, end);
        }
        if (this.m_userDataBuffer.data) {
            ///std::rotate(m_userDataBuffer.data + start, m_userDataBuffer.data + mid, m_userDataBuffer.data + end);
            std_rotate(this.m_userDataBuffer.data, start, mid, end);
        }
        // Update handle indices.
        if (this.m_handleIndexBuffer.data) {
            ///std::rotate(m_handleIndexBuffer.data + start, m_handleIndexBuffer.data + mid, m_handleIndexBuffer.data + end);
            std_rotate(this.m_handleIndexBuffer.data, start, mid, end);
            for (let i = start; i < end; ++i) {
                const handle = this.m_handleIndexBuffer.data[i];
                if (handle) {
                    handle.index = newIndices(handle.index, start, mid, end);
                }
            }
        }
        if (this.m_expirationTimeBuffer.data) {
            ///std::rotate(m_expirationTimeBuffer.data + start, m_expirationTimeBuffer.data + mid, m_expirationTimeBuffer.data + end);
            std_rotate(this.m_expirationTimeBuffer.data, start, mid, end);
            // Update expiration time buffer indices.
            const particleCount = this.GetParticleCount();
            const indexByExpirationTime = this.m_indexByExpirationTimeBuffer.data;
            for (let i = 0; i < particleCount; ++i) {
                indexByExpirationTime[i] = newIndices(indexByExpirationTime[i], start, mid, end);
            }
        }
        // update proxies
        for (let k = 0; k < this.m_proxyBuffer.count; k++) {
            const proxy = this.m_proxyBuffer.data[k];
            proxy.index = newIndices(proxy.index, start, mid, end);
        }
        // update contacts
        for (let k = 0; k < this.m_contactBuffer.count; k++) {
            const contact = this.m_contactBuffer.data[k];
            contact.indexA = newIndices(contact.indexA, start, mid, end);
            contact.indexB = newIndices(contact.indexB, start, mid, end);
        }
        // update particle-body contacts
        for (let k = 0; k < this.m_bodyContactBuffer.count; k++) {
            const contact = this.m_bodyContactBuffer.data[k];
            contact.index = newIndices(contact.index, start, mid, end);
        }
        // update pairs
        for (let k = 0; k < this.m_pairBuffer.count; k++) {
            const pair = this.m_pairBuffer.data[k];
            pair.indexA = newIndices(pair.indexA, start, mid, end);
            pair.indexB = newIndices(pair.indexB, start, mid, end);
        }
        // update triads
        for (let k = 0; k < this.m_triadBuffer.count; k++) {
            const triad = this.m_triadBuffer.data[k];
            triad.indexA = newIndices(triad.indexA, start, mid, end);
            triad.indexB = newIndices(triad.indexB, start, mid, end);
            triad.indexC = newIndices(triad.indexC, start, mid, end);
        }
        // update groups
        for (let group = this.m_groupList; group; group = group.GetNext()) {
            group.m_firstIndex = newIndices(group.m_firstIndex, start, mid, end);
            group.m_lastIndex = newIndices(group.m_lastIndex - 1, start, mid, end) + 1;
        }
    }
    GetCriticalVelocity(step) {
        return this.m_particleDiameter * step.inv_dt;
    }
    GetCriticalVelocitySquared(step) {
        const velocity = this.GetCriticalVelocity(step);
        return velocity * velocity;
    }
    GetCriticalPressure(step) {
        return this.m_def.density * this.GetCriticalVelocitySquared(step);
    }
    GetParticleStride() {
        return b2_particleStride * this.m_particleDiameter;
    }
    GetParticleMass() {
        const stride = this.GetParticleStride();
        return this.m_def.density * stride * stride;
    }
    GetParticleInvMass() {
        ///return 1.777777 * this.m_inverseDensity * this.m_inverseDiameter * this.m_inverseDiameter;
        // mass = density * stride^2, so we take the inverse of this.
        const inverseStride = this.m_inverseDiameter * (1.0 / b2_particleStride);
        return this.m_inverseDensity * inverseStride * inverseStride;
    }
    /**
     * Get the world's contact filter if any particles with the
     * b2_contactFilterParticle flag are present in the system.
     */
    GetFixtureContactFilter() {
        return this.m_allParticleFlags & 65536 /* b2_fixtureContactFilterParticle */
            ? this.m_world.m_contactManager.m_contactFilter
            : null;
    }
    /**
     * Get the world's contact filter if any particles with the
     * b2_particleContactFilterParticle flag are present in the
     * system.
     */
    GetParticleContactFilter() {
        return this.m_allParticleFlags & 131072 /* b2_particleContactFilterParticle */
            ? this.m_world.m_contactManager.m_contactFilter
            : null;
    }
    /**
     * Get the world's contact listener if any particles with the
     * b2_fixtureContactListenerParticle flag are present in the
     * system.
     */
    GetFixtureContactListener() {
        return this.m_allParticleFlags & 16384 /* b2_fixtureContactListenerParticle */
            ? this.m_world.m_contactManager.m_contactListener
            : null;
    }
    /**
     * Get the world's contact listener if any particles with the
     * b2_particleContactListenerParticle flag are present in the
     * system.
     */
    GetParticleContactListener() {
        return this.m_allParticleFlags & 32768 /* b2_particleContactListenerParticle */
            ? this.m_world.m_contactManager.m_contactListener
            : null;
    }
    SetUserOverridableBuffer(buffer, data) {
        buffer.data = data;
        buffer.userSuppliedCapacity = data.length;
    }
    SetGroupFlags(group, newFlags) {
        const oldFlags = group.m_groupFlags;
        if ((oldFlags ^ newFlags) & 1 /* b2_solidParticleGroup */) {
            // If the b2_solidParticleGroup flag changed schedule depth update.
            newFlags |= 16 /* b2_particleGroupNeedsUpdateDepth */;
        }
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.m_needsUpdateAllGroupFlags = true;
        }
        if (~this.m_allGroupFlags & newFlags) {
            // If any flags were added
            if (newFlags & 1 /* b2_solidParticleGroup */) {
                this.m_depthBuffer = this.RequestBuffer(this.m_depthBuffer);
            }
            this.m_allGroupFlags |= newFlags;
        }
        group.m_groupFlags = newFlags;
    }
    static BodyContactCompare(lhs, rhs) {
        if (lhs.index === rhs.index) {
            // Subsort by weight, decreasing.
            return lhs.weight > rhs.weight;
        }
        return lhs.index < rhs.index;
    }
    RemoveSpuriousBodyContacts() {
        // At this point we have a list of contact candidates based on AABB
        // overlap.The AABB query that  generated this returns all collidable
        // fixtures overlapping particle bounding boxes.  This breaks down around
        // vertices where two shapes intersect, such as a "ground" surface made
        // of multiple b2PolygonShapes; it potentially applies a lot of spurious
        // impulses from normals that should not actually contribute.  See the
        // Ramp example in Testbed.
        //
        // To correct for this, we apply this algorithm:
        //   * sort contacts by particle and subsort by weight (nearest to farthest)
        //   * for each contact per particle:
        //      - project a point at the contact distance along the inverse of the
        //        contact normal
        //      - if this intersects the fixture that generated the contact, apply
        //         it, otherwise discard as impossible
        //      - repeat for up to n nearest contacts, currently we get good results
        //        from n=3.
        ///std::sort(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.End(), b2ParticleSystem::BodyContactCompare);
        std_sort(this.m_bodyContactBuffer.data, 0, this.m_bodyContactBuffer.count, b2ParticleSystem.BodyContactCompare);
        ///int32 discarded = 0;
        ///std::remove_if(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.End(), b2ParticleBodyContactRemovePredicate(this, &discarded));
        ///
        ///m_bodyContactBuffer.SetCount(m_bodyContactBuffer.GetCount() - discarded);
        const s_n = b2ParticleSystem.RemoveSpuriousBodyContacts_s_n;
        const s_pos = b2ParticleSystem.RemoveSpuriousBodyContacts_s_pos;
        const s_normal = b2ParticleSystem.RemoveSpuriousBodyContacts_s_normal;
        // Max number of contacts processed per particle, from nearest to farthest.
        // This must be at least 2 for correctness with concave shapes; 3 was
        // experimentally arrived at as looking reasonable.
        const k_maxContactsPerPoint = 3;
        // Index of last particle processed.
        let lastIndex = -1;
        // Number of contacts processed for the current particle.
        let currentContacts = 0;
        // Output the number of discarded contacts.
        // let discarded = 0;
        const b2ParticleBodyContactRemovePredicate = (contact) => {
            // This implements the selection criteria described in
            // RemoveSpuriousBodyContacts().
            // This functor is iterating through a list of Body contacts per
            // Particle, ordered from near to far.  For up to the maximum number of
            // contacts we allow per point per step, we verify that the contact
            // normal of the Body that genenerated the contact makes physical sense
            // by projecting a point back along that normal and seeing if it
            // intersects the fixture generating the contact.
            if (contact.index !== lastIndex) {
                currentContacts = 0;
                lastIndex = contact.index;
            }
            if (currentContacts++ > k_maxContactsPerPoint) {
                // ++discarded;
                return true;
            }
            // Project along inverse normal (as returned in the contact) to get the
            // point to check.
            ///b2Vec2 n = contact.normal;
            const n = s_n.Copy(contact.normal);
            // weight is 1-(inv(diameter) * distance)
            ///n *= system.m_particleDiameter * (1 - contact.weight);
            n.SelfMul(this.m_particleDiameter * (1 - contact.weight));
            ///b2Vec2 pos = system.m_positionBuffer.data[contact.index] + n;
            const pos = b2Vec2.AddVV(this.m_positionBuffer.data[contact.index], n, s_pos);
            // pos is now a point projected back along the contact normal to the
            // contact distance. If the surface makes sense for a contact, pos will
            // now lie on or in the fixture generating
            if (!contact.fixture.TestPoint(pos)) {
                const childCount = contact.fixture.GetShape().GetChildCount();
                for (let childIndex = 0; childIndex < childCount; childIndex++) {
                    const normal = s_normal;
                    const distance = contact.fixture.ComputeDistance(pos, normal, childIndex);
                    if (distance < b2_linearSlop) {
                        return false;
                    }
                }
                // ++discarded;
                return true;
            }
            return false;
        };
        this.m_bodyContactBuffer.count = std_remove_if(this.m_bodyContactBuffer.data, b2ParticleBodyContactRemovePredicate, this.m_bodyContactBuffer.count);
    }
    DetectStuckParticle(particle) {
        // Detect stuck particles
        //
        // The basic algorithm is to allow the user to specify an optional
        // threshold where we detect whenever a particle is contacting
        // more than one fixture for more than threshold consecutive
        // steps. This is considered to be "stuck", and these are put
        // in a list the user can query per step, if enabled, to deal with
        // such particles.
        if (this.m_stuckThreshold <= 0) {
            return;
        }
        // Get the state variables for this particle.
        ///int32 * const consecutiveCount = &m_consecutiveContactStepsBuffer.data[particle];
        ///int32 * const lastStep = &m_lastBodyContactStepBuffer.data[particle];
        ///int32 * const bodyCount = &m_bodyContactCountBuffer.data[particle];
        // This is only called when there is a body contact for this particle.
        ///++(*bodyCount);
        ++this.m_bodyContactCountBuffer.data[particle];
        // We want to only trigger detection once per step, the first time we
        // contact more than one fixture in a step for a given particle.
        ///if (*bodyCount === 2)
        if (this.m_bodyContactCountBuffer.data[particle] === 2) {
            ///++(*consecutiveCount);
            ++this.m_consecutiveContactStepsBuffer.data[particle];
            ///if (*consecutiveCount > m_stuckThreshold)
            if (this.m_consecutiveContactStepsBuffer.data[particle] > this.m_stuckThreshold) {
                ///int32& newStuckParticle = m_stuckParticleBuffer.Append();
                ///newStuckParticle = particle;
                this.m_stuckParticleBuffer.data[this.m_stuckParticleBuffer.Append()] = particle;
            }
        }
        ///*lastStep = m_timestamp;
        this.m_lastBodyContactStepBuffer.data[particle] = this.m_timestamp;
    }
    /**
     * Determine whether a particle index is valid.
     */
    ValidateParticleIndex(index) {
        return index >= 0 && index < this.GetParticleCount() && index !== b2_invalidParticleIndex;
    }
    /**
     * Get the time elapsed in
     * b2ParticleSystemDef::lifetimeGranularity.
     */
    GetQuantizedTimeElapsed() {
        ///return (int32)(m_timeElapsed >> 32);
        return Math.floor(this.m_timeElapsed / 0x100000000);
    }
    /**
     * Convert a lifetime in seconds to an expiration time.
     */
    LifetimeToExpirationTime(lifetime) {
        ///return m_timeElapsed + (int64)((lifetime / m_def.lifetimeGranularity) * (float32)(1LL << 32));
        return (this.m_timeElapsed + Math.floor((lifetime / this.m_def.lifetimeGranularity) * 0x100000000));
    }
    ForceCanBeApplied(flags) {
        return !(flags & 4 /* b2_wallParticle */);
    }
    PrepareForceBuffer() {
        if (!this.m_hasForce) {
            ///memset(m_forceBuffer, 0, sizeof(*m_forceBuffer) * m_count);
            for (let i = 0; i < this.m_count; i++) {
                this.m_forceBuffer[i].SetZero();
            }
            this.m_hasForce = true;
        }
    }
    IsRigidGroup(group) {
        return group !== null && (group.m_groupFlags & 2 /* b2_rigidParticleGroup */) !== 0;
    }
    GetLinearVelocity(group, particleIndex, point, out) {
        if (group && this.IsRigidGroup(group)) {
            return group.GetLinearVelocityFromWorldPoint(point, out);
        }
        else {
            ///return m_velocityBuffer.data[particleIndex];
            return out.Copy(this.m_velocityBuffer.data[particleIndex]);
        }
    }
    InitDampingParameter(invMass, invInertia, tangentDistance, mass, inertia, center, point, normal) {
        ///*invMass = mass > 0 ? 1 / mass : 0;
        invMass[0] = mass > 0 ? 1 / mass : 0;
        ///*invInertia = inertia > 0 ? 1 / inertia : 0;
        invInertia[0] = inertia > 0 ? 1 / inertia : 0;
        ///*tangentDistance = b2Cross(point - center, normal);
        tangentDistance[0] = b2Vec2.CrossVV(b2Vec2.SubVV(point, center, b2Vec2.s_t0), normal);
    }
    InitDampingParameterWithRigidGroupOrParticle(invMass, invInertia, tangentDistance, isRigidGroup, group, particleIndex, point, normal) {
        if (group && isRigidGroup) {
            this.InitDampingParameter(invMass, invInertia, tangentDistance, group.GetMass(), group.GetInertia(), group.GetCenter(), point, normal);
        }
        else {
            const flags = this.m_flagsBuffer.data[particleIndex];
            this.InitDampingParameter(invMass, invInertia, tangentDistance, flags & 4 /* b2_wallParticle */ ? 0 : this.GetParticleMass(), 0, point, point, normal);
        }
    }
    ComputeDampingImpulse(invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, normalVelocity) {
        const invMass = invMassA +
            invInertiaA * tangentDistanceA * tangentDistanceA +
            invMassB +
            invInertiaB * tangentDistanceB * tangentDistanceB;
        return invMass > 0 ? normalVelocity / invMass : 0;
    }
    ApplyDamping(invMass, invInertia, tangentDistance, isRigidGroup, group, particleIndex, impulse, normal) {
        if (group && isRigidGroup) {
            ///group.m_linearVelocity += impulse * invMass * normal;
            group.m_linearVelocity.SelfMulAdd(impulse * invMass, normal);
            ///group.m_angularVelocity += impulse * tangentDistance * invInertia;
            group.m_angularVelocity += impulse * tangentDistance * invInertia;
        }
        else {
            ///m_velocityBuffer.data[particleIndex] += impulse * invMass * normal;
            this.m_velocityBuffer.data[particleIndex].SelfMulAdd(impulse * invMass, normal);
        }
    }
}
b2ParticleSystem.xTruncBits = 12;
b2ParticleSystem.yTruncBits = 12;
b2ParticleSystem.tagBits = 8 * 4; // 8u * sizeof(uint32);
b2ParticleSystem.yOffset = 1 << (b2ParticleSystem.yTruncBits - 1);
b2ParticleSystem.yShift = b2ParticleSystem.tagBits - b2ParticleSystem.yTruncBits;
b2ParticleSystem.xShift = b2ParticleSystem.tagBits - b2ParticleSystem.yTruncBits - b2ParticleSystem.xTruncBits;
b2ParticleSystem.xScale = 1 << b2ParticleSystem.xShift;
b2ParticleSystem.xOffset = b2ParticleSystem.xScale * (1 << (b2ParticleSystem.xTruncBits - 1));
b2ParticleSystem.yMask = ((1 << b2ParticleSystem.yTruncBits) - 1) << b2ParticleSystem.yShift;
b2ParticleSystem.xMask = ~b2ParticleSystem.yMask;
b2ParticleSystem.DestroyParticlesInShape_s_aabb = new b2AABB();
b2ParticleSystem.CreateParticleGroup_s_transform = new b2Transform();
b2ParticleSystem.ComputeCollisionEnergy_s_v = new b2Vec2();
b2ParticleSystem.QueryShapeAABB_s_aabb = new b2AABB();
b2ParticleSystem.QueryPointAABB_s_aabb = new b2AABB();
b2ParticleSystem.RayCast_s_aabb = new b2AABB();
b2ParticleSystem.RayCast_s_p = new b2Vec2();
b2ParticleSystem.RayCast_s_v = new b2Vec2();
b2ParticleSystem.RayCast_s_n = new b2Vec2();
b2ParticleSystem.RayCast_s_point = new b2Vec2();
/**
 * All particle types that require creating pairs
 */
b2ParticleSystem.k_pairFlags = 8 /* b2_springParticle */;
/**
 * All particle types that require creating triads
 */
b2ParticleSystem.k_triadFlags = 16 /* b2_elasticParticle */;
/**
 * All particle types that do not produce dynamic pressure
 */
b2ParticleSystem.k_noPressureFlags = 64 /* b2_powderParticle */ | 128 /* b2_tensileParticle */;
/**
 * All particle types that apply extra damping force with bodies
 */
b2ParticleSystem.k_extraDampingFlags = 2048 /* b2_staticPressureParticle */;
b2ParticleSystem.k_barrierWallFlags = 1024 /* b2_barrierParticle */ | 4 /* b2_wallParticle */;
b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_edge = new b2EdgeShape();
b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_d = new b2Vec2();
b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_p = new b2Vec2();
b2ParticleSystem.CreateParticlesFillShapeForGroup_s_aabb = new b2AABB();
b2ParticleSystem.CreateParticlesFillShapeForGroup_s_p = new b2Vec2();
b2ParticleSystem.UpdatePairsAndTriads_s_dab = new b2Vec2();
b2ParticleSystem.UpdatePairsAndTriads_s_dbc = new b2Vec2();
b2ParticleSystem.UpdatePairsAndTriads_s_dca = new b2Vec2();
b2ParticleSystem.AddContact_s_d = new b2Vec2();
b2ParticleSystem.UpdateBodyContacts_s_aabb = new b2AABB();
b2ParticleSystem.Solve_s_subStep = new b2TimeStep();
b2ParticleSystem.SolveCollision_s_aabb = new b2AABB();
b2ParticleSystem.SolveGravity_s_gravity = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_aabb = new b2AABB();
b2ParticleSystem.SolveBarrier_s_va = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_vb = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_pba = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_vba = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_vc = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_pca = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_vca = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_qba = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_qca = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_dv = new b2Vec2();
b2ParticleSystem.SolveBarrier_s_f = new b2Vec2();
b2ParticleSystem.SolvePressure_s_f = new b2Vec2();
b2ParticleSystem.SolveDamping_s_v = new b2Vec2();
b2ParticleSystem.SolveDamping_s_f = new b2Vec2();
b2ParticleSystem.SolveRigidDamping_s_t0 = new b2Vec2();
b2ParticleSystem.SolveRigidDamping_s_t1 = new b2Vec2();
b2ParticleSystem.SolveRigidDamping_s_p = new b2Vec2();
b2ParticleSystem.SolveRigidDamping_s_v = new b2Vec2();
b2ParticleSystem.SolveExtraDamping_s_v = new b2Vec2();
b2ParticleSystem.SolveExtraDamping_s_f = new b2Vec2();
b2ParticleSystem.SolveRigid_s_position = new b2Vec2();
b2ParticleSystem.SolveRigid_s_rotation = new b2Rot();
b2ParticleSystem.SolveRigid_s_transform = new b2Transform();
b2ParticleSystem.SolveRigid_s_velocityTransform = new b2Transform();
b2ParticleSystem.SolveElastic_s_pa = new b2Vec2();
b2ParticleSystem.SolveElastic_s_pb = new b2Vec2();
b2ParticleSystem.SolveElastic_s_pc = new b2Vec2();
b2ParticleSystem.SolveElastic_s_r = new b2Rot();
b2ParticleSystem.SolveElastic_s_t0 = new b2Vec2();
b2ParticleSystem.SolveSpring_s_pa = new b2Vec2();
b2ParticleSystem.SolveSpring_s_pb = new b2Vec2();
b2ParticleSystem.SolveSpring_s_d = new b2Vec2();
b2ParticleSystem.SolveSpring_s_f = new b2Vec2();
b2ParticleSystem.SolveTensile_s_weightedNormal = new b2Vec2();
b2ParticleSystem.SolveTensile_s_s = new b2Vec2();
b2ParticleSystem.SolveTensile_s_f = new b2Vec2();
b2ParticleSystem.SolveViscous_s_v = new b2Vec2();
b2ParticleSystem.SolveViscous_s_f = new b2Vec2();
b2ParticleSystem.SolveRepulsive_s_f = new b2Vec2();
b2ParticleSystem.SolvePowder_s_f = new b2Vec2();
b2ParticleSystem.SolveSolid_s_f = new b2Vec2();
b2ParticleSystem.RemoveSpuriousBodyContacts_s_n = new b2Vec2();
b2ParticleSystem.RemoveSpuriousBodyContacts_s_pos = new b2Vec2();
b2ParticleSystem.RemoveSpuriousBodyContacts_s_normal = new b2Vec2();
export class b2ParticleSystem_UserOverridableBuffer {
    constructor() {
        this.userSuppliedCapacity = 0;
        this._data = null;
    }
    get data() {
        return this._data;
    } // HACK: may return null
    set data(value) {
        this._data = value;
    }
}
export class b2ParticleSystem_Proxy {
    constructor() {
        this.index = b2_invalidParticleIndex;
        this.tag = 0;
    }
    static CompareProxyProxy(a, b) {
        return a.tag < b.tag;
    }
    static CompareTagProxy(a, b) {
        return a < b.tag;
    }
    static CompareProxyTag(a, b) {
        return a.tag < b;
    }
}
export class b2ParticleSystem_InsideBoundsEnumerator {
    /**
     * InsideBoundsEnumerator enumerates all particles inside the
     * given bounds.
     *
     * Construct an enumerator with bounds of tags and a range of
     * proxies.
     */
    constructor(system, lower, upper, first, last) {
        this.m_system = system;
        this.m_xLower = (lower & b2ParticleSystem.xMask) >>> 0;
        this.m_xUpper = (upper & b2ParticleSystem.xMask) >>> 0;
        this.m_yLower = (lower & b2ParticleSystem.yMask) >>> 0;
        this.m_yUpper = (upper & b2ParticleSystem.yMask) >>> 0;
        this.m_first = first;
        this.m_last = last;
        !!B2_DEBUG && b2Assert(this.m_first <= this.m_last);
    }
    /**
     * Get index of the next particle. Returns
     * b2_invalidParticleIndex if there are no more particles.
     */
    GetNext() {
        while (this.m_first < this.m_last) {
            const xTag = (this.m_system.m_proxyBuffer.data[this.m_first].tag & b2ParticleSystem.xMask) >>> 0;
            if (!!B2_ASSERT && !!B2_DEBUG) {
                // B2_ASSERT -> B2_ASSERT_ENABLED ??
                const yTag = (this.m_system.m_proxyBuffer.data[this.m_first].tag & b2ParticleSystem.yMask) >>> 0;
                b2Assert(yTag >= this.m_yLower);
                b2Assert(yTag <= this.m_yUpper);
            }
            if (xTag >= this.m_xLower && xTag <= this.m_xUpper) {
                return this.m_system.m_proxyBuffer.data[this.m_first++].index;
            }
            this.m_first++;
        }
        return b2_invalidParticleIndex;
    }
}
export class b2ParticleSystem_ParticleListNode {
    constructor() {
        /**
         * The next node in the list.
         */
        this.next = null;
        /**
         * Number of entries in the list. Valid only for the node at the
         * head of the list.
         */
        this.count = 0;
        /**
         * Particle index.
         */
        this.index = 0;
    }
}
/**
 * @constructor
 */
export class b2ParticleSystem_FixedSetAllocator {
    Allocate(itemSize, count) {
        // TODO
        return count;
    }
    Clear() {
        // TODO
    }
    GetCount() {
        // TODO
        return 0;
    }
    Invalidate(itemIndex) {
        // TODO
    }
    GetValidBuffer() {
        // TODO
        return [];
    }
    GetBuffer() {
        // TODO
        return [];
    }
    SetCount(count) {
        // TODO
    }
}
export class b2ParticleSystem_FixtureParticle {
    constructor(fixture, particle) {
        this.second = b2_invalidParticleIndex;
        this.first = fixture;
        this.second = particle;
    }
}
export class b2ParticleSystem_FixtureParticleSet extends b2ParticleSystem_FixedSetAllocator {
    Initialize(bodyContactBuffer, flagsBuffer) {
        // TODO
    }
    Find(pair) {
        // TODO
        return b2_invalidParticleIndex;
    }
}
export class b2ParticleSystem_ParticlePair {
    constructor(particleA, particleB) {
        this.first = b2_invalidParticleIndex;
        this.second = b2_invalidParticleIndex;
        this.first = particleA;
        this.second = particleB;
    }
}
export class b2ParticlePairSet extends b2ParticleSystem_FixedSetAllocator {
    Initialize(contactBuffer, flagsBuffer) {
        // TODO
    }
    Find(pair) {
        // TODO
        return b2_invalidParticleIndex;
    }
}
export class b2ParticleSystem_ConnectionFilter {
    /**
     * Is the particle necessary for connection?
     * A pair or a triad should contain at least one 'necessary'
     * particle.
     */
    IsNecessary(index) {
        return true;
    }
    /**
     * An additional condition for creating a pair.
     */
    ShouldCreatePair(a, b) {
        return true;
    }
    /**
     * An additional condition for creating a triad.
     */
    ShouldCreateTriad(a, b, c) {
        return true;
    }
}
export class b2ParticleSystem_DestroyParticlesInShapeCallback extends b2QueryCallback {
    constructor(system, shape, xf, callDestructionListener) {
        super();
        this.m_callDestructionListener = false;
        this.m_destroyed = 0;
        this.m_system = system;
        this.m_shape = shape;
        this.m_xf = xf;
        this.m_callDestructionListener = callDestructionListener;
        this.m_destroyed = 0;
    }
    ReportFixture(fixture) {
        return false;
    }
    ReportParticle(particleSystem, index) {
        if (particleSystem !== this.m_system) {
            return false;
        }
        !!B2_DEBUG && b2Assert(index >= 0 && index < this.m_system.m_count);
        if (this.m_shape.TestPoint(this.m_xf, this.m_system.m_positionBuffer.data[index])) {
            this.m_system.DestroyParticle(index, this.m_callDestructionListener);
            this.m_destroyed++;
        }
        return true;
    }
    Destroyed() {
        return this.m_destroyed;
    }
}
export class b2ParticleSystem_JoinParticleGroupsFilter extends b2ParticleSystem_ConnectionFilter {
    constructor(threshold) {
        super();
        this.m_threshold = 0;
        this.m_threshold = threshold;
    }
    /**
     * An additional condition for creating a pair.
     */
    ShouldCreatePair(a, b) {
        return ((a < this.m_threshold && this.m_threshold <= b) ||
            (b < this.m_threshold && this.m_threshold <= a));
    }
    /**
     * An additional condition for creating a triad.
     */
    ShouldCreateTriad(a, b, c) {
        return ((a < this.m_threshold || b < this.m_threshold || c < this.m_threshold) &&
            (this.m_threshold <= a || this.m_threshold <= b || this.m_threshold <= c));
    }
}
export class b2ParticleSystem_CompositeShape extends b2Shape {
    constructor(shapes, shapeCount) {
        super(-1 /* e_unknown */, 0.0);
        this.m_shapeCount = 0;
        this.m_shapes = shapes;
        this.m_shapeCount = shapeCount ?? shapes.length;
    }
    Clone() {
        !!B2_DEBUG && b2Assert(false);
        throw new Error();
    }
    GetChildCount() {
        return 1;
    }
    /**
     * @see b2Shape::TestPoint
     */
    TestPoint(xf, p) {
        for (let i = 0; i < this.m_shapeCount; i++) {
            if (this.m_shapes[i].TestPoint(xf, p)) {
                return true;
            }
        }
        return false;
    }
    /**
     * @see b2Shape::ComputeDistance
     */
    ComputeDistance(xf, p, normal, childIndex) {
        !!B2_DEBUG && b2Assert(false);
        return 0;
    }
    /**
     * Implement b2Shape.
     */
    RayCast(output, input, xf, childIndex) {
        !!B2_DEBUG && b2Assert(false);
        return false;
    }
    /**
     * @see b2Shape::ComputeAABB
     */
    ComputeAABB(aabb, xf, childIndex) {
        const s_subaabb = new b2AABB();
        aabb.lowerBound.x = +b2_maxFloat;
        aabb.lowerBound.y = +b2_maxFloat;
        aabb.upperBound.x = -b2_maxFloat;
        aabb.upperBound.y = -b2_maxFloat;
        !!B2_DEBUG && b2Assert(childIndex === 0);
        for (let i = 0; i < this.m_shapeCount; i++) {
            const childCount = this.m_shapes[i].GetChildCount();
            for (let j = 0; j < childCount; j++) {
                const subaabb = s_subaabb;
                this.m_shapes[i].ComputeAABB(subaabb, xf, j);
                aabb.Combine1(subaabb);
            }
        }
    }
    /**
     * @see b2Shape::ComputeMass
     */
    ComputeMass(massData, density) {
        !!B2_DEBUG && b2Assert(false);
    }
    SetupDistanceProxy(proxy, index) {
        !!B2_DEBUG && b2Assert(false);
    }
    ComputeSubmergedArea(normal, offset, xf, c) {
        !!B2_DEBUG && b2Assert(false);
        return 0;
    }
}
export class b2ParticleSystem_ReactiveFilter extends b2ParticleSystem_ConnectionFilter {
    constructor(flagsBuffer) {
        super();
        this.m_flagsBuffer = flagsBuffer;
    }
    IsNecessary(index) {
        return (this.m_flagsBuffer.data[index] & 4096 /* b2_reactiveParticle */) !== 0;
    }
}
export class b2ParticleSystem_UpdateBodyContactsCallback extends b2FixtureParticleQueryCallback {
    constructor(system, contactFilter = null) {
        super(system); // base class constructor
        this.m_contactFilter = null;
        this.m_contactFilter = contactFilter;
    }
    ShouldCollideFixtureParticle(fixture, particleSystem, particleIndex) {
        // Call the contact filter if it's set, to determine whether to
        // filter this contact.  Returns true if contact calculations should
        // be performed, false otherwise.
        if (this.m_contactFilter) {
            const flags = this.m_system.GetFlagsBuffer();
            if (flags[particleIndex] & 65536 /* b2_fixtureContactFilterParticle */) {
                return this.m_contactFilter.ShouldCollideFixtureParticle(fixture, this.m_system, particleIndex);
            }
        }
        return true;
    }
    ReportFixtureAndParticle(fixture, childIndex, a) {
        const s_n = b2ParticleSystem_UpdateBodyContactsCallback.ReportFixtureAndParticle_s_n;
        const s_rp = b2ParticleSystem_UpdateBodyContactsCallback.ReportFixtureAndParticle_s_rp;
        const ap = this.m_system.m_positionBuffer.data[a];
        const n = s_n;
        const d = fixture.ComputeDistance(ap, n, childIndex);
        if (d < this.m_system.m_particleDiameter &&
            this.ShouldCollideFixtureParticle(fixture, this.m_system, a)) {
            const b = fixture.GetBody();
            const bp = b.GetWorldCenter();
            const bm = b.GetMass();
            const bI = b.GetInertia() - bm * b.GetLocalCenter().LengthSquared();
            const invBm = bm > 0 ? 1 / bm : 0;
            const invBI = bI > 0 ? 1 / bI : 0;
            const invAm = this.m_system.m_flagsBuffer.data[a] & 4 /* b2_wallParticle */
                ? 0
                : this.m_system.GetParticleInvMass();
            ///b2Vec2 rp = ap - bp;
            const rp = b2Vec2.SubVV(ap, bp, s_rp);
            const rpn = b2Vec2.CrossVV(rp, n);
            const invM = invAm + invBm + invBI * rpn * rpn;
            ///b2ParticleBodyContact& contact = m_system.m_bodyContactBuffer.Append();
            const contact = this.m_system.m_bodyContactBuffer.data[this.m_system.m_bodyContactBuffer.Append()];
            contact.index = a;
            contact.body = b;
            contact.fixture = fixture;
            contact.weight = 1 - d * this.m_system.m_inverseDiameter;
            ///contact.normal = -n;
            contact.normal.Copy(n.SelfNeg());
            contact.mass = invM > 0 ? 1 / invM : 0;
            this.m_system.DetectStuckParticle(a);
        }
    }
}
b2ParticleSystem_UpdateBodyContactsCallback.ReportFixtureAndParticle_s_n = new b2Vec2();
b2ParticleSystem_UpdateBodyContactsCallback.ReportFixtureAndParticle_s_rp = new b2Vec2();
export class b2ParticleSystem_SolveCollisionCallback extends b2FixtureParticleQueryCallback {
    constructor(system, step) {
        super(system); // base class constructor
        this.m_step = step;
    }
    ReportFixtureAndParticle(fixture, childIndex, a) {
        const s_p1 = b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_p1;
        const s_output = b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_output;
        const s_input = b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_input;
        const s_p = b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_p;
        const s_v = b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_v;
        const s_f = b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_f;
        const body = fixture.GetBody();
        const ap = this.m_system.m_positionBuffer.data[a];
        const av = this.m_system.m_velocityBuffer.data[a];
        const output = s_output;
        const input = s_input;
        if (this.m_system.m_iterationIndex === 0) {
            // Put 'ap' in the local space of the previous frame
            ///b2Vec2 p1 = b2MulT(body.m_xf0, ap);
            const p1 = b2Transform.MulTXV(body.m_xf0, ap, s_p1);
            if (fixture.GetShape().GetType() === 0 /* e_circleShape */) {
                // Make relative to the center of the circle
                ///p1 -= body.GetLocalCenter();
                p1.SelfSub(body.GetLocalCenter());
                // Re-apply rotation about the center of the circle
                ///p1 = b2Mul(body.m_xf0.q, p1);
                b2Rot.MulRV(body.m_xf0.q, p1, p1);
                // Subtract rotation of the current frame
                ///p1 = b2MulT(body.m_xf.q, p1);
                b2Rot.MulTRV(body.m_xf.q, p1, p1);
                // Return to local space
                ///p1 += body.GetLocalCenter();
                p1.SelfAdd(body.GetLocalCenter());
            }
            // Return to global space and apply rotation of current frame
            ///input.p1 = b2Mul(body.m_xf, p1);
            b2Transform.MulXV(body.m_xf, p1, input.p1);
        }
        else {
            ///input.p1 = ap;
            input.p1.Copy(ap);
        }
        ///input.p2 = ap + m_step.dt * av;
        b2Vec2.AddVMulSV(ap, this.m_step.dt, av, input.p2);
        input.maxFraction = 1;
        if (fixture.RayCast(output, input, childIndex)) {
            const n = output.normal;
            ///b2Vec2 p = (1 - output.fraction) * input.p1 + output.fraction * input.p2 + b2_linearSlop * n;
            const p = s_p;
            p.x = (1 - output.fraction) * input.p1.x + output.fraction * input.p2.x + b2_linearSlop * n.x;
            p.y = (1 - output.fraction) * input.p1.y + output.fraction * input.p2.y + b2_linearSlop * n.y;
            ///b2Vec2 v = m_step.inv_dt * (p - ap);
            const v = s_v;
            v.x = this.m_step.inv_dt * (p.x - ap.x);
            v.y = this.m_step.inv_dt * (p.y - ap.y);
            ///m_system.m_velocityBuffer.data[a] = v;
            this.m_system.m_velocityBuffer.data[a].Copy(v);
            ///b2Vec2 f = m_step.inv_dt * m_system.GetParticleMass() * (av - v);
            const f = s_f;
            f.x = this.m_step.inv_dt * this.m_system.GetParticleMass() * (av.x - v.x);
            f.y = this.m_step.inv_dt * this.m_system.GetParticleMass() * (av.y - v.y);
            this.m_system.ParticleApplyForce(a, f);
        }
    }
    ReportParticle(system, index) {
        return false;
    }
}
b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_p1 = new b2Vec2();
b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_output = new b2RayCastOutput();
b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_input = new b2RayCastInput();
b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_p = new b2Vec2();
b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_v = new b2Vec2();
b2ParticleSystem_SolveCollisionCallback.ReportFixtureAndParticle_s_f = new b2Vec2();
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQYXJ0aWNsZVN5c3RlbS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9wYXJ0aWNsZS9iMlBhcnRpY2xlU3lzdGVtLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLGFBQWEsRUFBRSxXQUFXLEVBQUUsUUFBUSxFQUFFLFdBQVcsRUFBRSxPQUFPLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUNsRyxPQUFPLEVBQ0wsdUJBQXVCLEVBQ3ZCLHVCQUF1QixFQUN2QixtQkFBbUIsRUFDbkIsbUJBQW1CLEVBQ25CLHNCQUFzQixFQUN0QiwwQkFBMEIsRUFDMUIsa0NBQWtDLEVBQ2xDLG9CQUFvQixFQUNwQixpQkFBaUIsR0FDbEIsTUFBTSxvQ0FBb0MsQ0FBQztBQUM1QyxPQUFPLEVBQ0wsS0FBSyxFQUNMLE9BQU8sRUFDUCxTQUFTLEVBQ1QsS0FBSyxFQUNMLFFBQVEsRUFDUixLQUFLLEVBQ0wsUUFBUSxFQUNSLEtBQUssRUFDTCxNQUFNLEVBQ04sV0FBVyxFQUNYLE1BQU0sR0FFUCxNQUFNLGtCQUFrQixDQUFDO0FBQzFCLE9BQU8sRUFBRSxPQUFPLEVBQUUsTUFBTSxrQkFBa0IsQ0FBQztBQUMzQyxPQUFPLEVBQUUsTUFBTSxFQUFFLGNBQWMsRUFBRSxlQUFlLEVBQUUsTUFBTSwwQkFBMEIsQ0FBQztBQUNuRixPQUFPLEVBQWMsT0FBTyxFQUFlLE1BQU0sNkJBQTZCLENBQUM7QUFDL0UsT0FBTyxFQUFFLFdBQVcsRUFBRSxNQUFNLGlDQUFpQyxDQUFDO0FBRTlELE9BQU8sRUFBRSxVQUFVLEVBQUUsTUFBTSx3QkFBd0IsQ0FBQztBQUlwRCxPQUFPLEVBR0wsZUFBZSxHQUVoQixNQUFNLDhCQUE4QixDQUFDO0FBQ3RDLE9BQU8sRUFBa0IsYUFBYSxFQUFrQixnQkFBZ0IsRUFBRSxNQUFNLGNBQWMsQ0FBQztBQUMvRixPQUFPLEVBRUwsZUFBZSxFQUNmLGtC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QUFHLENBQUMsV0FBVyxDQUFDO1FBQ2pDLElBQUksQ0FBQyxVQUFVLENBQUMsQ0FBQyxHQUFHLENBQUMsV0FBVyxDQUFDO1FBQ2pDLElBQUksQ0FBQyxVQUFVLENBQUMsQ0FBQyxHQUFHLENBQUMsV0FBVyxDQUFDO1FBQ2pDLElBQUksQ0FBQyxVQUFVLENBQUMsQ0FBQyxHQUFHLENBQUMsV0FBVyxDQUFDO1FBQ2pDLEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxJQUFJLENBQUMsT0FBTyxFQUFFLENBQUMsRUFBRSxFQUFFO1lBQ3JDLE1BQU0sQ0FBQyxHQUFHLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUN0QixNQUFNLEVBQUUsR0FBRyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7WUFDdkIsNkJBQTZCO1lBQzdCLE1BQU0sSUFBSSxHQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLEVBQUUsR0FBRyxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBQ2xDLE1BQU0sSUFBSSxHQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLEVBQUUsR0FBRyxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBQ2xDLDJEQUEyRDtZQUMzRCxJQUFJLENBQUMsVUFBVSxDQUFDLENBQUMsR0FBRyxLQUFLLENBQUMsSUFBSSxDQUFDLFVBQVUsQ0FBQyxDQUFDLEVBQUUsS0FBSyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLENBQUMsQ0FBQztZQUNoRSxJQUFJLENBQUMsVUFBVSxDQUFDLENBQUMsR0FBRyxLQUFLLENBQUMsSUFBSSxDQUFDLFVBQVUsQ0FBQyxDQUFDLEVBQUUsS0FBSyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLENBQUMsQ0FBQztZQUNoRSwyREFBMkQ7WUFDM0QsSUFBSSxDQUFDLFVBQVUsQ0FBQyxDQUFDLEdBQUcsS0FBSyxDQUFDLElBQUksQ0FBQyxVQUFVLENBQUMsQ0FBQyxFQUFFLEtBQUssQ0FBQyxFQUFFLENBQUMsQ0FBQyxF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eXJpZ2h0IChjKSAyMDEzIEdvb2dsZSwgSW5jLlxyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQgeyBiMl9saW5lYXJTbG9wLCBiMl9tYXhGbG9hdCwgYjJBc3NlcnQsIGIyTWFrZUFycmF5LCBiMk1heWJlIH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQge1xyXG4gIGIyX2JhcnJpZXJDb2xsaXNpb25UaW1lLFxyXG4gIGIyX2ludmFsaWRQYXJ0aWNsZUluZGV4LFxyXG4gIGIyX21heFBhcnRpY2xlRm9yY2UsXHJcbiAgYjJfbWF4UGFydGljbGVJbmRleCxcclxuICBiMl9tYXhQYXJ0aWNsZVByZXNzdXJlLFxyXG4gIGIyX21heFRyaWFkRGlzdGFuY2VTcXVhcmVkLFxyXG4gIGIyX21pblBhcnRpY2xlU3lzdGVtQnVmZmVyQ2FwYWNpdHksXHJcbiAgYjJfbWluUGFydGljbGVXZWlnaHQsXHJcbiAgYjJfcGFydGljbGVTdHJpZGUsXHJcbn0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3NQYXJ0aWNsZVN5c3RlbSc7XHJcbmltcG9ydCB7XHJcbiAgYjJBYnMsXHJcbiAgYjJDbGFtcCxcclxuICBiMkludlNxcnQsXHJcbiAgYjJNYXgsXHJcbiAgYjJNYXhJbnQsXHJcbiAgYjJNaW4sXHJcbiAgYjJNaW5JbnQsXHJcbiAgYjJSb3QsXHJcbiAgYjJTcXJ0LFxyXG4gIGIyVHJhbnNmb3JtLFxyXG4gIGIyVmVjMixcclxuICBYWSxcclxufSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJDb2xvciB9IGZyb20gJy4uL2NvbW1vbi9iMkRyYXcnO1xyXG5pbXBvcnQgeyBiMkFBQkIsIGIyUmF5Q2FzdElucHV0LCBiMlJheUNhc3RPdXRwdXQgfSBmcm9tICcuLi9jb2xsaXNpb24vYjJDb2xsaXNpb24nO1xyXG5pbXBvcnQgeyBiMk1hc3NEYXRhLCBiMlNoYXBlLCBiMlNoYXBlVHlwZSB9IGZyb20gJy4uL2NvbGxpc2lvbi9zaGFwZXMvYjJTaGFwZSc7XHJcbmltcG9ydCB7IGIyRWRnZVNoYXBlIH0gZnJvbSAnLi4vY29sbGlzaW9uL3NoYXBlcy9iMkVkZ2VTaGFwZSc7XHJcbmltcG9ydCB7IGIyQ2hhaW5TaGFwZSB9IGZyb20gJy4uL2NvbGxpc2lvbi9zaGFwZXMvYjJDaGFpblNoYXBlJztcclxuaW1wb3J0IHsgYjJUaW1lU3RlcCB9IGZyb20gJy4uL2R5bmFtaWNzL2IyVGltZVN0ZXAnO1xyXG5pbXBvcnQgeyBiMkZpeHR1cmUgfSBmcm9tICcuLi9keW5hbWljcy9iMkZpeHR1cmUnO1xyXG5pbXBvcnQgeyBiMkJvZHkgfSBmcm9tICcuLi9keW5hbWljcy9iMkJvZHknO1xyXG5pbXBvcnQgeyBiMldvcmxkIH0gZnJvbSAnLi4vZHluYW1pY3MvYjJXb3JsZCc7XHJcbmltcG9ydCB7XHJcbiAgYjJDb250YWN0RmlsdGVyLFxyXG4gIGIyQ29udGFjdExpc3RlbmVyLFxyXG4gIGIyUXVlcnlDYWxsYmFjayxcclxuICBiMlJheUNhc3RDYWxsYmFjayxcclxufSBmcm9tICcuLi9keW5hbWljcy9iMldvcmxkQ2FsbGJhY2tzJztcclxuaW1wb3J0IHsgYjJJUGFydGljbGVEZWYsIGIyUGFydGljbGVEZWYsIGIyUGFydGljbGVGbGFnLCBiMlBhcnRpY2xlSGFuZGxlIH0gZnJvbSAnLi9iMlBhcnRpY2xlJztcclxuaW1wb3J0IHtcclxuICBiMklQYXJ0aWNsZUdyb3VwRGVmLFxyXG4gIGIyUGFydGljbGVHcm91cCxcclxuICBiMlBhcnRpY2xlR3JvdXBEZWYsXHJcbiAgYjJQYXJ0aWNsZUdyb3VwRmxhZyxcclxufSBmcm9tICcuL2IyUGFydGljbGVHcm91cCc7XHJcbmltcG9ydCB7IGIyVm9yb25vaURpYWdyYW0gfSBmcm9tICcuL2IyVm9yb25vaURpYWdyYW0nO1xyXG5pbXBvcnQgeyBiMkRpc3RhbmNlUHJveHkgfSBmcm9tICcuLi9jb2xsaXNpb24vYjJEaXN0YW5jZSc7XHJcblxyXG5mdW5jdGlvbiBzdGRfaXRlcl9zd2FwPFQ+KGFycmF5OiBUW10sIGE6IG51bWJlciwgYjogbnVtYmVyKTogdm9pZCB7XHJcbiAgY29uc3QgdG1wOiBUID0gYXJyYXlbYV07XHJcbiAgYXJyYXlbYV0gPSBhcnJheVtiXTtcclxuICBhcnJheVtiXSA9IHRtcDtcclxufVxyXG5cclxuZnVuY3Rpb24gZGVmYXVsdF9jb21wYXJlPFQ+KGE6IFQsIGI6IFQpOiBib29sZWFuIHtcclxuICByZXR1cm4gYSA8IGI7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIHN0ZF9zb3J0PFQ+KFxyXG4gIGFycmF5OiBUW10sXHJcbiAgZmlyc3QgPSAwLFxyXG4gIGxlbjogbnVtYmVyID0gYXJyYXkubGVuZ3RoIC0gZmlyc3QsXHJcbiAgY21wOiAoYTogVCwgYjogVCkgPT4gYm9vbGVhbiA9IGRlZmF1bHRfY29tcGFyZSxcclxuKTogVFtdIHtcclxuICBsZXQgbGVmdCA9IGZpcnN0O1xyXG4gIGNvbnN0IHN0YWNrOiBudW1iZXJbXSA9IFtdO1xyXG4gIGxldCBwb3MgPSAwO1xyXG5cclxuICBmb3IgKDs7KSB7XHJcbiAgICAvKiBvdXRlciBsb29wICovXHJcbiAgICBmb3IgKDsgbGVmdCArIDEgPCBsZW47IGxlbisrKSB7XHJcbiAgICAgIC8qIHNvcnQgbGVmdCB0byBsZW4tMSAqL1xyXG4gICAgICBjb25zdCBwaXZvdCA9IGFycmF5W2xlZnQgKyBNYXRoLmZsb29yKE1hdGgucmFuZG9tKCkgKiAobGVuIC0gbGVmdCkpXTsgLyogcGljayByYW5kb20gcGl2b3QgKi9cclxuICAgICAgc3RhY2tbcG9zKytdID0gbGVuOyAvKiBzb3J0IHJpZ2h0IHBhcnQgbGF0ZXIgKi9cclxuICAgICAgZm9yIChsZXQgcmlnaHQgPSBsZWZ0IC0gMTsgOyApIHtcclxuICAgICAgICAvKiBpbm5lciBsb29wOiBwYXJ0aXRpb25pbmcgKi9cclxuICAgICAgICAvLyBlc2xpbnQtZGlzYWJsZS1uZXh0LWxpbmUgbm8tZW1wdHlcclxuICAgICAgICB3aGlsZSAoY21wKGFycmF5WysrcmlnaHRdLCBwaXZvdCkpIHt9IC8qIGxvb2sgZm9yIGdyZWF0ZXIgZWxlbWVudCAqL1xyXG4gICAgICAgIC8vIGVzbGludC1kaXNhYmxlLW5leHQtbGluZSBuby1lbXB0eVxyXG4gICAgICAgIHdoaWxlIChjbXAocGl2b3QsIGFycmF5Wy0tbGVuXSkpIHt9IC8qIGxvb2sgZm9yIHNtYWxsZXIgZWxlbWVudCAqL1xyXG5cclxuICAgICAgICBpZiAocmlnaHQgPj0gbGVuKSB7XHJcbiAgICAgICAgICBicmVhaztcclxuICAgICAgICB9IC8qIHBhcnRpdGlvbiBwb2ludCBmb3VuZD8gKi9cclxuICAgICAgICBzdGRfaXRlcl9zd2FwKGFycmF5LCByaWdodCwgbGVuKTsgLyogdGhlIG9ubHkgc3dhcCAqL1xyXG4gICAgICB9IC8qIHBhcnRpdGlvbmVkLCBjb250aW51ZSBsZWZ0IHBhcnQgKi9cclxuICAgIH1cclxuICAgIGlmIChwb3MgPT09IDApIHtcclxuICAgICAgYnJlYWs7XHJcbiAgICB9IC8qIHN0YWNrIGVtcHR5PyAqL1xyXG4gICAgbGVmdCA9IGxlbjsgLyogbGVmdCB0byByaWdodCBpcyBzb3J0ZWQgKi9cclxuICAgIGxlbiA9IHN0YWNrWy0tcG9zXTsgLyogZ2V0IG5leHQgcmFuZ2UgdG8gc29ydCAqL1xyXG4gIH1cclxuXHJcbiAgcmV0dXJuIGFycmF5O1xyXG59XHJcblxyXG5mdW5jdGlvbiBzdGRfc3RhYmxlX3NvcnQ8VD4oXHJcbiAgYXJyYXk6IFRbXSxcclxuICBmaXJzdCA9IDAsXHJcbiAgbGVuOiBudW1iZXIgPSBhcnJheS5sZW5ndGggLSBmaXJzdCxcclxuICBjbXA6IChhOiBULCBiOiBUKSA9PiBib29sZWFuID0gZGVmYXVsdF9jb21wYXJlLFxyXG4pOiBUW10ge1xyXG4gIHJldHVybiBzdGRfc29ydChhcnJheSwgZmlyc3QsIGxlbiwgY21wKTtcclxufVxyXG5cclxuZnVuY3Rpb24gc3RkX3JlbW92ZV9pZjxUPihcclxuICBhcnJheTogVFtdLFxyXG4gIHByZWRpY2F0ZTogKHZhbHVlOiBUKSA9PiBib29sZWFuLFxyXG4gIGxlbmd0aDogbnVtYmVyID0gYXJyYXkubGVuZ3RoLFxyXG4pIHtcclxuICBsZXQgbCA9IDA7XHJcblxyXG4gIGZvciAobGV0IGMgPSAwOyBjIDwgbGVuZ3RoOyArK2MpIHtcclxuICAgIC8vIGlmIHdlIGNhbiBiZSBjb2xsYXBzZWQsIGtlZXAgbCB3aGVyZSBpdCBpcy5cclxuICAgIGlmIChwcmVkaWNhdGUoYXJyYXlbY10pKSB7XHJcbiAgICAgIGNvbnRpbnVlO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIHRoaXMgbm9kZSBjYW4ndCBiZSBjb2xsYXBzZWQ7IHB1c2ggaXQgYmFjayBhcyBmYXIgYXMgd2UgY2FuLlxyXG4gICAgaWYgKGMgPT09IGwpIHtcclxuICAgICAgKytsO1xyXG4gICAgICBjb250aW51ZTsgLy8gcXVpY2sgZXhpdCBpZiB3ZSdyZSBhbHJlYWR5IGluIHRoZSByaWdodCBzcG90XHJcbiAgICB9XHJcblxyXG4gICAgLy8gYXJyYXlbbCsrXSA9IGFycmF5W2NdO1xyXG4gICAgc3RkX2l0ZXJfc3dhcChhcnJheSwgbCsrLCBjKTtcclxuICB9XHJcblxyXG4gIHJldHVybiBsO1xyXG59XHJcblxyXG5mdW5jdGlvbiBzdGRfbG93ZXJfYm91bmQ8QSwgQj4oXHJcbiAgYXJyYXk6IEFbXSxcclxuICBmaXJzdDogbnVtYmVyLFxyXG4gIGxhc3Q6IG51bWJlcixcclxuICB2YWw6IEIsXHJcbiAgY21wOiAoYTogQSwgYjogQikgPT4gYm9vbGVhbixcclxuKTogbnVtYmVyIHtcclxuICBsZXQgY291bnQgPSBsYXN0IC0gZmlyc3Q7XHJcbiAgd2hpbGUgKGNvdW50ID4gMCkge1xyXG4gICAgY29uc3Qgc3RlcCA9IE1hdGguZmxvb3IoY291bnQgLyAyKTtcclxuICAgIGxldCBpdCA9IGZpcnN0ICsgc3RlcDtcclxuXHJcbiAgICBpZiAoY21wKGFycmF5W2l0XSwgdmFsKSkge1xyXG4gICAgICBmaXJzdCA9ICsraXQ7XHJcbiAgICAgIGNvdW50IC09IHN0ZXAgKyAxO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgY291bnQgPSBzdGVwO1xyXG4gICAgfVxyXG4gIH1cclxuICByZXR1cm4gZmlyc3Q7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIHN0ZF91cHBlcl9ib3VuZDxBLCBCPihcclxuICBhcnJheTogQltdLFxyXG4gIGZpcnN0OiBudW1iZXIsXHJcbiAgbGFzdDogbnVtYmVyLFxyXG4gIHZhbDogQSxcclxuICBjbXA6IChhOiBBLCBiOiBCKSA9PiBib29sZWFuLFxyXG4pOiBudW1iZXIge1xyXG4gIGxldCBjb3VudCA9IGxhc3QgLSBmaXJzdDtcclxuICB3aGlsZSAoY291bnQgPiAwKSB7XHJcbiAgICBjb25zdCBzdGVwID0gTWF0aC5mbG9vcihjb3VudCAvIDIpO1xyXG4gICAgbGV0IGl0ID0gZmlyc3QgKyBzdGVwO1xyXG5cclxuICAgIGlmICghY21wKHZhbCwgYXJyYXlbaXRdKSkge1xyXG4gICAgICBmaXJzdCA9ICsraXQ7XHJcbiAgICAgIGNvdW50IC09IHN0ZXAgKyAxO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgY291bnQgPSBzdGVwO1xyXG4gICAgfVxyXG4gIH1cclxuICByZXR1cm4gZmlyc3Q7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIHN0ZF9yb3RhdGU8VD4oYXJyYXk6IFRbXSwgZmlyc3Q6IG51bWJlciwgbl9maXJzdDogbnVtYmVyLCBsYXN0OiBudW1iZXIpOiB2b2lkIHtcclxuICBsZXQgbmV4dCA9IG5fZmlyc3Q7XHJcbiAgd2hpbGUgKGZpcnN0ICE9PSBuZXh0KSB7XHJcbiAgICBzdGRfaXRlcl9zd2FwKGFycmF5LCBmaXJzdCsrLCBuZXh0KyspO1xyXG4gICAgaWYgKG5leHQgPT09IGxhc3QpIHtcclxuICAgICAgbmV4dCA9IG5fZmlyc3Q7XHJcbiAgICB9IGVsc2UgaWYgKGZpcnN0ID09PSBuX2ZpcnN0KSB7XHJcbiAgICAgIG5fZmlyc3QgPSBuZXh0O1xyXG4gICAgfVxyXG4gIH1cclxufVxyXG5cclxuZnVuY3Rpb24gc3RkX3VuaXF1ZTxUPihcclxuICBhcnJheTogVFtdLFxyXG4gIGZpcnN0OiBudW1iZXIsXHJcbiAgbGFzdDogbnVtYmVyLFxyXG4gIGNtcDogKGE6IFQsIGI6IFQpID0+IGJvb2xlYW4sXHJcbik6IG51bWJlciB7XHJcbiAgaWYgKGZpcnN0ID09PSBsYXN0KSB7XHJcbiAgICByZXR1cm4gbGFzdDtcclxuICB9XHJcbiAgbGV0IHJlc3VsdCA9IGZpcnN0O1xyXG4gIHdoaWxlICgrK2ZpcnN0ICE9PSBsYXN0KSB7XHJcbiAgICBpZiAoIWNtcChhcnJheVtyZXN1bHRdLCBhcnJheVtmaXJzdF0pKSB7XHJcbiAgICAgIC8vL2FycmF5WysrcmVzdWx0XSA9IGFycmF5W2ZpcnN0XTtcclxuICAgICAgc3RkX2l0ZXJfc3dhcChhcnJheSwgKytyZXN1bHQsIGZpcnN0KTtcclxuICAgIH1cclxuICB9XHJcbiAgcmV0dXJuICsrcmVzdWx0O1xyXG59XHJcblxyXG5jb25zdCBuZXdJbmRpY2VzID0gKGk6IG51bWJlciwgc3RhcnQ6IG51bWJlciwgbWlkOiBudW1iZXIsIGVuZDogbnVtYmVyKTogbnVtYmVyID0+IHtcclxuICBpZiAoaSA8IHN0YXJ0KSB7XHJcbiAgICByZXR1cm4gaTtcclxuICB9IGVsc2UgaWYgKGkgPCBtaWQpIHtcclxuICAgIHJldHVybiBpICsgZW5kIC0gbWlkO1xyXG4gIH0gZWxzZSBpZiAoaSA8IGVuZCkge1xyXG4gICAgcmV0dXJuIGkgKyBzdGFydCAtIG1pZDtcclxuICB9IGVsc2Uge1xyXG4gICAgcmV0dXJuIGk7XHJcbiAgfVxyXG59O1xyXG5cclxuZXhwb3J0IGNsYXNzIGIyR3Jvd2FibGVCdWZmZXI8VD4ge1xyXG4gIGRhdGE6IFRbXSA9IFtdO1xyXG4gIGNvdW50ID0gMDtcclxuICBjYXBhY2l0eSA9IDA7XHJcbiAgYWxsb2NhdG9yOiAoKSA9PiBUO1xyXG5cclxuICBjb25zdHJ1Y3RvcihhbGxvY2F0b3I6ICgpID0+IFQpIHtcclxuICAgIHRoaXMuYWxsb2NhdG9yID0gYWxsb2NhdG9yO1xyXG4gIH1cclxuXHJcbiAgQXBwZW5kKCk6IG51bWJlciB7XHJcbiAgICBpZiAodGhpcy5jb3VudCA+PSB0aGlzLmNhcGFjaXR5KSB7XHJcbiAgICAgIHRoaXMuR3JvdygpO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRoaXMuY291bnQrKztcclxuICB9XHJcblxyXG4gIFJlc2VydmUobmV3Q2FwYWNpdHk6IG51bWJlcik6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMuY2FwYWNpdHkgPj0gbmV3Q2FwYWNpdHkpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5jYXBhY2l0eSA9PT0gdGhpcy5kYXRhLmxlbmd0aCk7XHJcbiAgICBmb3IgKGxldCBpID0gdGhpcy5jYXBhY2l0eTsgaSA8IG5ld0NhcGFjaXR5OyArK2kpIHtcclxuICAgICAgdGhpcy5kYXRhW2ldID0gdGhpcy5hbGxvY2F0b3IoKTtcclxuICAgIH1cclxuICAgIHRoaXMuY2FwYWNpdHkgPSBuZXdDYXBhY2l0eTtcclxuICB9XHJcblxyXG4gIEdyb3coKTogdm9pZCB7XHJcbiAgICAvLyBEb3VibGUgdGhlIGNhcGFjaXR5LlxyXG4gICAgY29uc3QgbmV3Q2FwYWNpdHkgPSB0aGlzLmNhcGFjaXR5ID8gMiAqIHRoaXMuY2FwYWNpdHkgOiBiMl9taW5QYXJ0aWNsZVN5c3RlbUJ1ZmZlckNhcGFjaXR5O1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChuZXdDYXBhY2l0eSA+IHRoaXMuY2FwYWNpdHkpO1xyXG4gICAgdGhpcy5SZXNlcnZlKG5ld0NhcGFjaXR5KTtcclxuICB9XHJcblxyXG4gIEZyZWUoKTogdm9pZCB7XHJcbiAgICBpZiAodGhpcy5kYXRhLmxlbmd0aCA9PT0gMCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5kYXRhID0gW107XHJcbiAgICB0aGlzLmNhcGFjaXR5ID0gMDtcclxuICAgIHRoaXMuY291bnQgPSAwO1xyXG4gIH1cclxuXHJcbiAgU2hvcnRlbihuZXdFbmQ6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgfVxyXG5cclxuICBEYXRhKCk6IFRbXSB7XHJcbiAgICByZXR1cm4gdGhpcy5kYXRhO1xyXG4gIH1cclxuXHJcbiAgR2V0Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLmNvdW50O1xyXG4gIH1cclxuXHJcbiAgU2V0Q291bnQobmV3Q291bnQ6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCgwIDw9IG5ld0NvdW50ICYmIG5ld0NvdW50IDw9IHRoaXMuY2FwYWNpdHkpO1xyXG4gICAgdGhpcy5jb3VudCA9IG5ld0NvdW50O1xyXG4gIH1cclxuXHJcbiAgR2V0Q2FwYWNpdHkoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLmNhcGFjaXR5O1xyXG4gIH1cclxuXHJcbiAgUmVtb3ZlSWYocHJlZDogKHQ6IFQpID0+IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGlmIChCMl9ERUJVRykge1xyXG4gICAgICBsZXQgY291bnQgPSAwO1xyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMuY291bnQ7ICsraSkge1xyXG4gICAgICAgIGlmICghcHJlZCh0aGlzLmRhdGFbaV0pKSB7XHJcbiAgICAgICAgICBjb3VudCsrO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgdGhpcy5jb3VudCA9IHN0ZF9yZW1vdmVfaWYodGhpcy5kYXRhLCBwcmVkLCB0aGlzLmNvdW50KTtcclxuXHJcbiAgICAgIGIyQXNzZXJ0KGNvdW50ID09PSB0aGlzLmNvdW50KTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMuY291bnQgPSBzdGRfcmVtb3ZlX2lmKHRoaXMuZGF0YSwgcHJlZCwgdGhpcy5jb3VudCk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBVbmlxdWUocHJlZDogKGE6IFQsIGI6IFQpID0+IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIHRoaXMuY291bnQgPSBzdGRfdW5pcXVlKHRoaXMuZGF0YSwgMCwgdGhpcy5jb3VudCwgcHJlZCk7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgdHlwZSBiMlBhcnRpY2xlSW5kZXggPSBudW1iZXI7XHJcblxyXG5leHBvcnQgY2xhc3MgYjJGaXh0dXJlUGFydGljbGVRdWVyeUNhbGxiYWNrIGV4dGVuZHMgYjJRdWVyeUNhbGxiYWNrIHtcclxuICBtX3N5c3RlbTogYjJQYXJ0aWNsZVN5c3RlbTtcclxuXHJcbiAgY29uc3RydWN0b3Ioc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtKSB7XHJcbiAgICBzdXBlcigpO1xyXG4gICAgdGhpcy5tX3N5c3RlbSA9IHN5c3RlbTtcclxuICB9XHJcblxyXG4gIFNob3VsZFF1ZXJ5UGFydGljbGVTeXN0ZW0oc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtKTogYm9vbGVhbiB7XHJcbiAgICAvLyBTa2lwIHJlcG9ydGluZyBwYXJ0aWNsZXMuXHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbiAgfVxyXG5cclxuICBSZXBvcnRGaXh0dXJlKGZpeHR1cmU6IGIyRml4dHVyZSk6IGJvb2xlYW4ge1xyXG4gICAgaWYgKGZpeHR1cmUuSXNTZW5zb3IoKSkge1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGNvbnN0IHNoYXBlID0gZml4dHVyZS5HZXRTaGFwZSgpO1xyXG4gICAgY29uc3QgY2hpbGRDb3VudCA9IHNoYXBlLkdldENoaWxkQ291bnQoKTtcclxuICAgIGZvciAobGV0IGNoaWxkSW5kZXggPSAwOyBjaGlsZEluZGV4IDwgY2hpbGRDb3VudDsgY2hpbGRJbmRleCsrKSB7XHJcbiAgICAgIGNvbnN0IGFhYmIgPSBmaXh0dXJlLkdldEFBQkIoY2hpbGRJbmRleCk7XHJcbiAgICAgIGNvbnN0IGVudW1lcmF0b3IgPSB0aGlzLm1fc3lzdGVtLkdldEluc2lkZUJvdW5kc0VudW1lcmF0b3IoYWFiYik7XHJcbiAgICAgIGxldCBpbmRleDogbnVtYmVyO1xyXG4gICAgICB3aGlsZSAoKGluZGV4ID0gZW51bWVyYXRvci5HZXROZXh0KCkpID49IDApIHtcclxuICAgICAgICB0aGlzLlJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZShmaXh0dXJlLCBjaGlsZEluZGV4LCBpbmRleCk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgUmVwb3J0UGFydGljbGUoc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtLCBpbmRleDogbnVtYmVyKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbiAgfVxyXG5cclxuICBSZXBvcnRGaXh0dXJlQW5kUGFydGljbGUoZml4dHVyZTogYjJGaXh0dXJlLCBjaGlsZEluZGV4OiBudW1iZXIsIGluZGV4OiBudW1iZXIpOiB2b2lkIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpOyAvLyBwdXJlIHZpcnR1YWxcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlQ29udGFjdCB7XHJcbiAgaW5kZXhBID0gMDtcclxuICBpbmRleEIgPSAwO1xyXG4gIHdlaWdodCA9IE5hTjtcclxuICBub3JtYWwgPSBuZXcgYjJWZWMyKCk7XHJcbiAgZmxhZ3MgPSBiMlBhcnRpY2xlRmxhZy5ub25lO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMud2VpZ2h0ID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgU2V0SW5kaWNlcyhhOiBudW1iZXIsIGI6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChhIDw9IGIyX21heFBhcnRpY2xlSW5kZXggJiYgYiA8PSBiMl9tYXhQYXJ0aWNsZUluZGV4KTtcclxuICAgIHRoaXMuaW5kZXhBID0gYTtcclxuICAgIHRoaXMuaW5kZXhCID0gYjtcclxuICB9XHJcblxyXG4gIFNldFdlaWdodCh3OiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMud2VpZ2h0ID0gdztcclxuICB9XHJcblxyXG4gIFNldE5vcm1hbChuOiBiMlZlYzIpOiB2b2lkIHtcclxuICAgIHRoaXMubm9ybWFsLkNvcHkobik7XHJcbiAgfVxyXG5cclxuICBTZXRGbGFncyhmOiBiMlBhcnRpY2xlRmxhZyk6IHZvaWQge1xyXG4gICAgdGhpcy5mbGFncyA9IGY7XHJcbiAgfVxyXG5cclxuICBHZXRJbmRleEEoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLmluZGV4QTtcclxuICB9XHJcblxyXG4gIEdldEluZGV4QigpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMuaW5kZXhCO1xyXG4gIH1cclxuXHJcbiAgR2V0V2VpZ2h0KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy53ZWlnaHQ7XHJcbiAgfVxyXG5cclxuICBHZXROb3JtYWwoKTogYjJWZWMyIHtcclxuICAgIHJldHVybiB0aGlzLm5vcm1hbDtcclxuICB9XHJcblxyXG4gIEdldEZsYWdzKCk6IGIyUGFydGljbGVGbGFnIHtcclxuICAgIHJldHVybiB0aGlzLmZsYWdzO1xyXG4gIH1cclxuXHJcbiAgSXNFcXVhbChyaHM6IGIyUGFydGljbGVDb250YWN0KTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gKFxyXG4gICAgICB0aGlzLmluZGV4QSA9PT0gcmhzLmluZGV4QSAmJlxyXG4gICAgICB0aGlzLmluZGV4QiA9PT0gcmhzLmluZGV4QiAmJlxyXG4gICAgICB0aGlzLmZsYWdzID09PSByaHMuZmxhZ3MgJiZcclxuICAgICAgdGhpcy53ZWlnaHQgPT09IHJocy53ZWlnaHQgJiZcclxuICAgICAgdGhpcy5ub3JtYWwueCA9PT0gcmhzLm5vcm1hbC54ICYmXHJcbiAgICAgIHRoaXMubm9ybWFsLnkgPT09IHJocy5ub3JtYWwueVxyXG4gICAgKTtcclxuICB9XHJcblxyXG4gIElzTm90RXF1YWwocmhzOiBiMlBhcnRpY2xlQ29udGFjdCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuICF0aGlzLklzRXF1YWwocmhzKTtcclxuICB9XHJcblxyXG4gIEFwcHJveGltYXRlbHlFcXVhbChyaHM6IGIyUGFydGljbGVDb250YWN0KTogYm9vbGVhbiB7XHJcbiAgICBjb25zdCBNQVhfV0VJR0hUX0RJRkYgPSAwLjAxOyAvLyBXZWlnaHQgMCB+IDEsIHNvIGFib3V0IDElXHJcbiAgICBjb25zdCBNQVhfTk9STUFMX0RJRkZfU1EgPSAwLjAxICogMC4wMTsgLy8gTm9ybWFsIGxlbmd0aCA9IDEsIHNvIDElXHJcbiAgICByZXR1cm4gKFxyXG4gICAgICB0aGlzLmluZGV4QSA9PT0gcmhzLmluZGV4QSAmJlxyXG4gICAgICB0aGlzLmluZGV4QiA9PT0gcmhzLmluZGV4QiAmJlxyXG4gICAgICB0aGlzLmZsYWdzID09PSByaHMuZmxhZ3MgJiZcclxuICAgICAgYjJBYnModGhpcy53ZWlnaHQgLSByaHMud2VpZ2h0KSA8IE1BWF9XRUlHSFRfRElGRiAmJlxyXG4gICAgICBiMlZlYzIuRGlzdGFuY2VTcXVhcmVkVlYodGhpcy5ub3JtYWwsIHJocy5ub3JtYWwpIDwgTUFYX05PUk1BTF9ESUZGX1NRXHJcbiAgICApO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVCb2R5Q29udGFjdCB7XHJcbiAgaW5kZXggPSAwOyAvLyBJbmRleCBvZiB0aGUgcGFydGljbGUgbWFraW5nIGNvbnRhY3QuXHJcbiAgYm9keSE6IGIyQm9keTsgLy8gVGhlIGJvZHkgbWFraW5nIGNvbnRhY3QuXHJcbiAgZml4dHVyZSE6IGIyRml4dHVyZTsgLy8gVGhlIHNwZWNpZmljIGZpeHR1cmUgbWFraW5nIGNvbnRhY3RcclxuICB3ZWlnaHQgPSBOYU47IC8vIFdlaWdodCBvZiB0aGUgY29udGFjdC4gQSB2YWx1ZSBiZXR3ZWVuIDAuMGYgYW5kIDEuMGYuXHJcbiAgbm9ybWFsID0gbmV3IGIyVmVjMigpOyAvLyBUaGUgbm9ybWFsaXplZCBkaXJlY3Rpb24gZnJvbSB0aGUgcGFydGljbGUgdG8gdGhlIGJvZHkuXHJcbiAgbWFzcyA9IE5hTjsgLy8gVGhlIGVmZmVjdGl2ZSBtYXNzIHVzZWQgaW4gY2FsY3VsYXRpbmcgZm9yY2UuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLndlaWdodCA9IDAuMDtcclxuICAgIHRoaXMubWFzcyA9IDAuMDtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlUGFpciB7XHJcbiAgaW5kZXhBID0gMDsgLy8gSW5kaWNlcyBvZiB0aGUgcmVzcGVjdGl2ZSBwYXJ0aWNsZXMgbWFraW5nIHBhaXIuXHJcbiAgaW5kZXhCID0gMDtcclxuICBmbGFncyA9IGIyUGFydGljbGVGbGFnLm5vbmU7IC8vIFRoZSBsb2dpY2FsIHN1bSBvZiB0aGUgcGFydGljbGUgZmxhZ3MuIFNlZSB0aGUgYjJQYXJ0aWNsZUZsYWcgZW51bS5cclxuICBzdHJlbmd0aCA9IE5hTjsgLy8gVGhlIHN0cmVuZ3RoIG9mIGNvaGVzaW9uIGFtb25nIHRoZSBwYXJ0aWNsZXMuXHJcbiAgZGlzdGFuY2UgPSBOYU47IC8vIFRoZSBpbml0aWFsIGRpc3RhbmNlIG9mIHRoZSBwYXJ0aWNsZXMuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLnN0cmVuZ3RoID0gMC4wO1xyXG4gICAgdGhpcy5kaXN0YW5jZSA9IDAuMDtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlVHJpYWQge1xyXG4gIGluZGV4QSA9IDA7IC8vIEluZGljZXMgb2YgdGhlIHJlc3BlY3RpdmUgcGFydGljbGVzIG1ha2luZyB0cmlhZC5cclxuICBpbmRleEIgPSAwO1xyXG4gIGluZGV4QyA9IDA7XHJcbiAgZmxhZ3MgPSBiMlBhcnRpY2xlRmxhZy5ub25lOyAvLyBUaGUgbG9naWNhbCBzdW0gb2YgdGhlIHBhcnRpY2xlIGZsYWdzLiBTZWUgdGhlIGIyUGFydGljbGVGbGFnIGVudW0uXHJcbiAgc3RyZW5ndGggPSBOYU47IC8vIFRoZSBzdHJlbmd0aCBvZiBjb2hlc2lvbiBhbW9uZyB0aGUgcGFydGljbGVzLlxyXG4gIHBhID0gbmV3IGIyVmVjMigpOyAvLyBWYWx1ZXMgdXNlZCBmb3IgY2FsY3VsYXRpb24uXHJcbiAgcGIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcGMgPSBuZXcgYjJWZWMyKCk7XHJcbiAga2EgPSBOYU47XHJcbiAga2IgPSBOYU47XHJcbiAga2MgPSBOYU47XHJcbiAgcyA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLnN0cmVuZ3RoID0gMC4wO1xyXG4gICAgdGhpcy5rYSA9IDAuMDtcclxuICAgIHRoaXMua2IgPSAwLjA7XHJcbiAgICB0aGlzLmtjID0gMC4wO1xyXG4gICAgdGhpcy5zID0gMC4wO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1EZWYge1xyXG4gIC8vIEluaXRpYWxpemUgcGh5c2ljYWwgY29lZmZpY2llbnRzIHRvIHRoZSBtYXhpbXVtIHZhbHVlcyB0aGF0XHJcbiAgLy8gbWFpbnRhaW4gbnVtZXJpY2FsIHN0YWJpbGl0eS5cclxuXHJcbiAgLyoqXHJcbiAgICogRW5hYmxlIHN0cmljdCBQYXJ0aWNsZS9Cb2R5IGNvbnRhY3QgY2hlY2suXHJcbiAgICogU2VlIFNldFN0cmljdENvbnRhY3RDaGVjayBmb3IgZGV0YWlscy5cclxuICAgKi9cclxuICBzdHJpY3RDb250YWN0Q2hlY2sgPSBmYWxzZTtcclxuXHJcbiAgLyoqXHJcbiAgICogU2V0IHRoZSBwYXJ0aWNsZSBkZW5zaXR5LlxyXG4gICAqIFNlZSBTZXREZW5zaXR5IGZvciBkZXRhaWxzLlxyXG4gICAqL1xyXG4gIGRlbnNpdHkgPSBOYU47XHJcblxyXG4gIC8qKlxyXG4gICAqIENoYW5nZSB0aGUgcGFydGljbGUgZ3Jhdml0eSBzY2FsZS4gQWRqdXN0cyB0aGUgZWZmZWN0IG9mIHRoZVxyXG4gICAqIGdsb2JhbCBncmF2aXR5IHZlY3RvciBvbiBwYXJ0aWNsZXMuIERlZmF1bHQgdmFsdWUgaXMgMS4wZi5cclxuICAgKi9cclxuICBncmF2aXR5U2NhbGUgPSBOYU47XHJcblxyXG4gIC8qKlxyXG4gICAqIFBhcnRpY2xlcyBiZWhhdmUgYXMgY2lyY2xlcyB3aXRoIHRoaXMgcmFkaXVzLiBJbiBCb3gyRCB1bml0cy5cclxuICAgKi9cclxuICByYWRpdXMgPSBOYU47XHJcblxyXG4gIC8qKlxyXG4gICAqIFNldCB0aGUgbWF4aW11bSBudW1iZXIgb2YgcGFydGljbGVzLlxyXG4gICAqIEJ5IGRlZmF1bHQsIHRoZXJlIGlzIG5vIG1heGltdW0uIFRoZSBwYXJ0aWNsZSBidWZmZXJzIGNhblxyXG4gICAqIGNvbnRpbnVlIHRvIGdyb3cgd2hpbGUgYjJXb3JsZCdzIGJsb2NrIGFsbG9jYXRvciBzdGlsbCBoYXNcclxuICAgKiBtZW1vcnkuXHJcbiAgICogU2VlIFNldE1heFBhcnRpY2xlQ291bnQgZm9yIGRldGFpbHMuXHJcbiAgICovXHJcbiAgbWF4Q291bnQgPSAwO1xyXG5cclxuICAvKipcclxuICAgKiBJbmNyZWFzZXMgcHJlc3N1cmUgaW4gcmVzcG9uc2UgdG8gY29tcHJlc3Npb25cclxuICAgKiBTbWFsbGVyIHZhbHVlcyBhbGxvdyBtb3JlIGNvbXByZXNzaW9uXHJcbiAgICovXHJcbiAgcHJlc3N1cmVTdHJlbmd0aCA9IDAuMDA1O1xyXG5cclxuICAvKipcclxuICAgKiBSZWR1Y2VzIHZlbG9jaXR5IGFsb25nIHRoZSBjb2xsaXNpb24gbm9ybWFsXHJcbiAgICogU21hbGxlciB2YWx1ZSByZWR1Y2VzIGxlc3NcclxuICAgKi9cclxuICBkYW1waW5nU3RyZW5ndGggPSBOYU47XHJcblxyXG4gIC8qKlxyXG4gICAqIFJlc3RvcmVzIHNoYXBlIG9mIGVsYXN0aWMgcGFydGljbGUgZ3JvdXBzXHJcbiAgICogTGFyZ2VyIHZhbHVlcyBpbmNyZWFzZSBlbGFzdGljIHBhcnRpY2xlIHZlbG9jaXR5XHJcbiAgICovXHJcbiAgZWxhc3RpY1N0cmVuZ3RoID0gMC4yNTtcclxuXHJcbiAgLyoqXHJcbiAgICogUmVzdG9yZXMgbGVuZ3RoIG9mIHNwcmluZyBwYXJ0aWNsZSBncm91cHNcclxuICAgKiBMYXJnZXIgdmFsdWVzIGluY3JlYXNlIHNwcmluZyBwYXJ0aWNsZSB2ZWxvY2l0eVxyXG4gICAqL1xyXG4gIHNwcmluZ1N0cmVuZ3RoID0gMC4yNTtcclxuXHJcbiAgLyoqXHJcbiAgICogUmVkdWNlcyByZWxhdGl2ZSB2ZWxvY2l0eSBvZiB2aXNjb3VzIHBhcnRpY2xlc1xyXG4gICAqIExhcmdlciB2YWx1ZXMgc2xvdyBkb3duIHZpc2NvdXMgcGFydGljbGVzIG1vcmVcclxuICAgKi9cclxuICB2aXNjb3VzU3RyZW5ndGggPSAwLjI1O1xyXG5cclxuICAvKipcclxuICAgKiBQcm9kdWNlcyBwcmVzc3VyZSBvbiB0ZW5zaWxlIHBhcnRpY2xlc1xyXG4gICAqIDB+MC4yLiBMYXJnZXIgdmFsdWVzIGluY3JlYXNlIHRoZSBhbW91bnQgb2Ygc3VyZmFjZSB0ZW5zaW9uLlxyXG4gICAqL1xyXG4gIHN1cmZhY2VUZW5zaW9uUHJlc3N1cmVTdHJlbmd0aCA9IDAuMjtcclxuXHJcbiAgLyoqXHJcbiAgICogU21vb3RoZXMgb3V0bGluZSBvZiB0ZW5zaWxlIHBhcnRpY2xlc1xyXG4gICAqIDB+MC4yLiBMYXJnZXIgdmFsdWVzIHJlc3VsdCBpbiByb3VuZGVyLCBzbW9vdGhlcixcclxuICAgKiB3YXRlci1kcm9wLWxpa2UgY2x1c3RlcnMgb2YgcGFydGljbGVzLlxyXG4gICAqL1xyXG4gIHN1cmZhY2VUZW5zaW9uTm9ybWFsU3RyZW5ndGggPSAwLjI7XHJcblxyXG4gIC8qKlxyXG4gICAqIFByb2R1Y2VzIGFkZGl0aW9uYWwgcHJlc3N1cmUgb24gcmVwdWxzaXZlIHBhcnRpY2xlc1xyXG4gICAqIExhcmdlciB2YWx1ZXMgcmVwdWxzZSBtb3JlXHJcbiAgICogTmVnYXRpdmUgdmFsdWVzIG1lYW4gYXR0cmFjdGlvbi4gVGhlIHJhbmdlIHdoZXJlIHBhcnRpY2xlc1xyXG4gICAqIGJlaGF2ZSBzdGFibHkgaXMgYWJvdXQgLTAuMiB0byAyLjAuXHJcbiAgICovXHJcbiAgcmVwdWxzaXZlU3RyZW5ndGggPSBOYU47XHJcblxyXG4gIC8qKlxyXG4gICAqIFByb2R1Y2VzIHJlcHVsc2lvbiBiZXR3ZWVuIHBvd2RlciBwYXJ0aWNsZXNcclxuICAgKiBMYXJnZXIgdmFsdWVzIHJlcHVsc2UgbW9yZVxyXG4gICAqL1xyXG4gIHBvd2RlclN0cmVuZ3RoID0gMC41O1xyXG5cclxuICAvKipcclxuICAgKiBQdXNoZXMgcGFydGljbGVzIG91dCBvZiBzb2xpZCBwYXJ0aWNsZSBncm91cFxyXG4gICAqIExhcmdlciB2YWx1ZXMgcmVwdWxzZSBtb3JlXHJcbiAgICovXHJcbiAgZWplY3Rpb25TdHJlbmd0aCA9IDAuNTtcclxuXHJcbiAgLyoqXHJcbiAgICogUHJvZHVjZXMgc3RhdGljIHByZXNzdXJlXHJcbiAgICogTGFyZ2VyIHZhbHVlcyBpbmNyZWFzZSB0aGUgcHJlc3N1cmUgb24gbmVpZ2hib3JpbmcgcGFydGlsY2VzXHJcbiAgICogRm9yIGEgZGVzY3JpcHRpb24gb2Ygc3RhdGljIHByZXNzdXJlLCBzZWVcclxuICAgKiBodHRwOi8vZW4ud2lraXBlZGlhLm9yZy93aWtpL1N0YXRpY19wcmVzc3VyZSNTdGF0aWNfcHJlc3N1cmVfaW5fZmx1aWRfZHluYW1pY3NcclxuICAgKi9cclxuICBzdGF0aWNQcmVzc3VyZVN0cmVuZ3RoID0gMC4yO1xyXG5cclxuICAvKipcclxuICAgKiBSZWR1Y2VzIGluc3RhYmlsaXR5IGluIHN0YXRpYyBwcmVzc3VyZSBjYWxjdWxhdGlvblxyXG4gICAqIExhcmdlciB2YWx1ZXMgbWFrZSBzdGFiaWxpemUgc3RhdGljIHByZXNzdXJlIHdpdGggZmV3ZXJcclxuICAgKiBpdGVyYXRpb25zXHJcbiAgICovXHJcbiAgc3RhdGljUHJlc3N1cmVSZWxheGF0aW9uID0gMC4yO1xyXG5cclxuICAvKipcclxuICAgKiBDb21wdXRlcyBzdGF0aWMgcHJlc3N1cmUgbW9yZSBwcmVjaXNlbHlcclxuICAgKiBTZWUgU2V0U3RhdGljUHJlc3N1cmVJdGVyYXRpb25zIGZvciBkZXRhaWxzXHJcbiAgICovXHJcbiAgc3RhdGljUHJlc3N1cmVJdGVyYXRpb25zID0gODtcclxuXHJcbiAgLyoqXHJcbiAgICogRGV0ZXJtaW5lcyBob3cgZmFzdCBjb2xvcnMgYXJlIG1peGVkXHJcbiAgICogMS4wZiA9PT4gbWl4ZWQgaW1tZWRpYXRlbHlcclxuICAgKiAwLjVmID09PiBtaXhlZCBoYWxmIHdheSBlYWNoIHNpbXVsYXRpb24gc3RlcCAoc2VlXHJcbiAgICogYjJXb3JsZDo6U3RlcCgpKVxyXG4gICAqL1xyXG4gIGNvbG9yTWl4aW5nU3RyZW5ndGggPSAwLjU7XHJcblxyXG4gIC8qKlxyXG4gICAqIFdoZXRoZXIgdG8gZGVzdHJveSBwYXJ0aWNsZXMgYnkgYWdlIHdoZW4gbm8gbW9yZSBwYXJ0aWNsZXNcclxuICAgKiBjYW4gYmUgY3JlYXRlZC4gIFNlZSAjYjJQYXJ0aWNsZVN5c3RlbTo6U2V0RGVzdHJ1Y3Rpb25CeUFnZSgpXHJcbiAgICogZm9yIG1vcmUgaW5mb3JtYXRpb24uXHJcbiAgICovXHJcbiAgZGVzdHJveUJ5QWdlID0gdHJ1ZTtcclxuXHJcbiAgLyoqXHJcbiAgICogR3JhbnVsYXJpdHkgb2YgcGFydGljbGUgbGlmZXRpbWVzIGluIHNlY29uZHMuICBCeSBkZWZhdWx0XHJcbiAgICogdGhpcyBpcyBzZXQgdG8gKDEuMGYgLyA2MC4wZikgc2Vjb25kcy4gIGIyUGFydGljbGVTeXN0ZW0gdXNlc1xyXG4gICAqIGEgMzItYml0IHNpZ25lZCB2YWx1ZSB0byB0cmFjayBwYXJ0aWNsZSBsaWZldGltZXMgc28gdGhlXHJcbiAgICogbWF4aW11bSBsaWZldGltZSBvZiBhIHBhcnRpY2xlIGlzICgyXjMyIC0gMSkgLyAoMS4wZiAvXHJcbiAgICogbGlmZXRpbWVHcmFudWxhcml0eSkgc2Vjb25kcy4gV2l0aCB0aGUgdmFsdWUgc2V0IHRvIDEvNjAgdGhlXHJcbiAgICogbWF4aW11bSBsaWZldGltZSBvciBhZ2Ugb2YgYSBwYXJ0aWNsZSBpcyAyLjI3IHllYXJzLlxyXG4gICAqL1xyXG4gIGxpZmV0aW1lR3JhbnVsYXJpdHkgPSAxLjAgLyA2MC4wO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHRoaXMuZGVuc2l0eSA9IDEuMDtcclxuICAgIHRoaXMuZ3Jhdml0eVNjYWxlID0gMS4wO1xyXG4gICAgdGhpcy5yYWRpdXMgPSAxLjA7XHJcbiAgICB0aGlzLmRhbXBpbmdTdHJlbmd0aCA9IDEuMDtcclxuICAgIHRoaXMucmVwdWxzaXZlU3RyZW5ndGggPSAxLjA7XHJcbiAgfVxyXG5cclxuICBDb3B5KGRlZjogYjJQYXJ0aWNsZVN5c3RlbURlZik6IGIyUGFydGljbGVTeXN0ZW1EZWYge1xyXG4gICAgdGhpcy5zdHJpY3RDb250YWN0Q2hlY2sgPSBkZWYuc3RyaWN0Q29udGFjdENoZWNrO1xyXG4gICAgdGhpcy5kZW5zaXR5ID0gZGVmLmRlbnNpdHk7XHJcbiAgICB0aGlzLmdyYXZpdHlTY2FsZSA9IGRlZi5ncmF2aXR5U2NhbGU7XHJcbiAgICB0aGlzLnJhZGl1cyA9IGRlZi5yYWRpdXM7XHJcbiAgICB0aGlzLm1heENvdW50ID0gZGVmLm1heENvdW50O1xyXG4gICAgdGhpcy5wcmVzc3VyZVN0cmVuZ3RoID0gZGVmLnByZXNzdXJlU3RyZW5ndGg7XHJcbiAgICB0aGlzLmRhbXBpbmdTdHJlbmd0aCA9IGRlZi5kYW1waW5nU3RyZW5ndGg7XHJcbiAgICB0aGlzLmVsYXN0aWNTdHJlbmd0aCA9IGRlZi5lbGFzdGljU3RyZW5ndGg7XHJcbiAgICB0aGlzLnNwcmluZ1N0cmVuZ3RoID0gZGVmLnNwcmluZ1N0cmVuZ3RoO1xyXG4gICAgdGhpcy52aXNjb3VzU3RyZW5ndGggPSBkZWYudmlzY291c1N0cmVuZ3RoO1xyXG4gICAgdGhpcy5zdXJmYWNlVGVuc2lvblByZXNzdXJlU3RyZW5ndGggPSBkZWYuc3VyZmFjZVRlbnNpb25QcmVzc3VyZVN0cmVuZ3RoO1xyXG4gICAgdGhpcy5zdXJmYWNlVGVuc2lvbk5vcm1hbFN0cmVuZ3RoID0gZGVmLnN1cmZhY2VUZW5zaW9uTm9ybWFsU3RyZW5ndGg7XHJcbiAgICB0aGlzLnJlcHVsc2l2ZVN0cmVuZ3RoID0gZGVmLnJlcHVsc2l2ZVN0cmVuZ3RoO1xyXG4gICAgdGhpcy5wb3dkZXJTdHJlbmd0aCA9IGRlZi5wb3dkZXJTdHJlbmd0aDtcclxuICAgIHRoaXMuZWplY3Rpb25TdHJlbmd0aCA9IGRlZi5lamVjdGlvblN0cmVuZ3RoO1xyXG4gICAgdGhpcy5zdGF0aWNQcmVzc3VyZVN0cmVuZ3RoID0gZGVmLnN0YXRpY1ByZXNzdXJlU3RyZW5ndGg7XHJcbiAgICB0aGlzLnN0YXRpY1ByZXNzdXJlUmVsYXhhdGlvbiA9IGRlZi5zdGF0aWNQcmVzc3VyZVJlbGF4YXRpb247XHJcbiAgICB0aGlzLnN0YXRpY1ByZXNzdXJlSXRlcmF0aW9ucyA9IGRlZi5zdGF0aWNQcmVzc3VyZUl0ZXJhdGlvbnM7XHJcbiAgICB0aGlzLmNvbG9yTWl4aW5nU3RyZW5ndGggPSBkZWYuY29sb3JNaXhpbmdTdHJlbmd0aDtcclxuICAgIHRoaXMuZGVzdHJveUJ5QWdlID0gZGVmLmRlc3Ryb3lCeUFnZTtcclxuICAgIHRoaXMubGlmZXRpbWVHcmFudWxhcml0eSA9IGRlZi5saWZldGltZUdyYW51bGFyaXR5O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBDbG9uZSgpOiBiMlBhcnRpY2xlU3lzdGVtRGVmIHtcclxuICAgIHJldHVybiBuZXcgYjJQYXJ0aWNsZVN5c3RlbURlZigpLkNvcHkodGhpcyk7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYXJ0aWNsZVN5c3RlbSB7XHJcbiAgbV9wYXVzZWQgPSBmYWxzZTtcclxuICBtX3RpbWVzdGFtcCA9IDA7XHJcbiAgbV9hbGxQYXJ0aWNsZUZsYWdzID0gYjJQYXJ0aWNsZUZsYWcubm9uZTtcclxuICBtX25lZWRzVXBkYXRlQWxsUGFydGljbGVGbGFncyA9IGZhbHNlO1xyXG4gIG1fYWxsR3JvdXBGbGFncyA9IGIyUGFydGljbGVHcm91cEZsYWcubm9uZTtcclxuICBtX25lZWRzVXBkYXRlQWxsR3JvdXBGbGFncyA9IGZhbHNlO1xyXG4gIG1faGFzRm9yY2UgPSBmYWxzZTtcclxuICBtX2l0ZXJhdGlvbkluZGV4ID0gMDtcclxuICBtX2ludmVyc2VEZW5zaXR5ID0gTmFOO1xyXG4gIG1fcGFydGljbGVEaWFtZXRlciA9IE5hTjtcclxuICBtX2ludmVyc2VEaWFtZXRlciA9IE5hTjtcclxuICBtX3NxdWFyZWREaWFtZXRlciA9IE5hTjtcclxuICBtX2NvdW50ID0gMDtcclxuICBtX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkgPSAwO1xyXG4gIC8qKlxyXG4gICAqIEFsbG9jYXRvciBmb3IgYjJQYXJ0aWNsZUhhbmRsZSBpbnN0YW5jZXMuXHJcbiAgICovXHJcbiAgLy8vbV9oYW5kbGVBbGxvY2F0b3I6IGFueSA9IG51bGw7XHJcbiAgLyoqXHJcbiAgICogTWFwcyBwYXJ0aWNsZSBpbmRpY2llcyB0byBoYW5kbGVzLlxyXG4gICAqL1xyXG4gIG1faGFuZGxlSW5kZXhCdWZmZXIgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8YjJQYXJ0aWNsZUhhbmRsZSB8IG51bGw+KCk7XHJcbiAgbV9mbGFnc0J1ZmZlciA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtX1VzZXJPdmVycmlkYWJsZUJ1ZmZlcjxiMlBhcnRpY2xlRmxhZz4oKTtcclxuICBtX3Bvc2l0aW9uQnVmZmVyID0gbmV3IGIyUGFydGljbGVTeXN0ZW1fVXNlck92ZXJyaWRhYmxlQnVmZmVyPGIyVmVjMj4oKTtcclxuICBtX3ZlbG9jaXR5QnVmZmVyID0gbmV3IGIyUGFydGljbGVTeXN0ZW1fVXNlck92ZXJyaWRhYmxlQnVmZmVyPGIyVmVjMj4oKTtcclxuICBtX2ZvcmNlQnVmZmVyOiBiMlZlYzJbXSA9IFtdO1xyXG4gIC8qKlxyXG4gICAqIHRoaXMubV93ZWlnaHRCdWZmZXIgaXMgcG9wdWxhdGVkIGluIENvbXB1dGVXZWlnaHQgYW5kIHVzZWQgaW5cclxuICAgKiBDb21wdXRlRGVwdGgoKSwgU29sdmVTdGF0aWNQcmVzc3VyZSgpIGFuZCBTb2x2ZVByZXNzdXJlKCkuXHJcbiAgICovXHJcbiAgbV93ZWlnaHRCdWZmZXI6IG51bWJlcltdID0gW107XHJcbiAgLyoqXHJcbiAgICogV2hlbiBhbnkgcGFydGljbGVzIGhhdmUgdGhlIGZsYWcgYjJfc3RhdGljUHJlc3N1cmVQYXJ0aWNsZSxcclxuICAgKiB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXIgaXMgZmlyc3QgYWxsb2NhdGVkIGFuZCB1c2VkIGluXHJcbiAgICogU29sdmVTdGF0aWNQcmVzc3VyZSgpIGFuZCBTb2x2ZVByZXNzdXJlKCkuICBJdCB3aWxsIGJlXHJcbiAgICogcmVhbGxvY2F0ZWQgb24gc3Vic2VxdWVudCBDcmVhdGVQYXJ0aWNsZSgpIGNhbGxzLlxyXG4gICAqL1xyXG4gIG1fc3RhdGljUHJlc3N1cmVCdWZmZXI6IG51bWJlcltdID0gW107XHJcbiAgLyoqXHJcbiAgICogdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlciBpcyB1c2VkIGluIG1hbnkgZnVuY3Rpb25zIGFzIGEgdGVtcG9yYXJ5XHJcbiAgICogYnVmZmVyIGZvciBzY2FsYXIgdmFsdWVzLlxyXG4gICAqL1xyXG4gIG1fYWNjdW11bGF0aW9uQnVmZmVyOiBudW1iZXJbXSA9IFtdO1xyXG4gIC8qKlxyXG4gICAqIFdoZW4gYW55IHBhcnRpY2xlcyBoYXZlIHRoZSBmbGFnIGIyX3RlbnNpbGVQYXJ0aWNsZSxcclxuICAgKiB0aGlzLm1fYWNjdW11bGF0aW9uMkJ1ZmZlciBpcyBmaXJzdCBhbGxvY2F0ZWQgYW5kIHVzZWQgaW5cclxuICAgKiBTb2x2ZVRlbnNpbGUoKSBhcyBhIHRlbXBvcmFyeSBidWZmZXIgZm9yIHZlY3RvciB2YWx1ZXMuICBJdFxyXG4gICAqIHdpbGwgYmUgcmVhbGxvY2F0ZWQgb24gc3Vic2VxdWVudCBDcmVhdGVQYXJ0aWNsZSgpIGNhbGxzLlxyXG4gICAqL1xyXG4gIG1fYWNjdW11bGF0aW9uMkJ1ZmZlcjogYjJWZWMyW10gPSBbXTtcclxuICAvKipcclxuICAgKiBXaGVuIGFueSBwYXJ0aWNsZSBncm91cHMgaGF2ZSB0aGUgZmxhZyBiMl9zb2xpZFBhcnRpY2xlR3JvdXAsXHJcbiAgICogdGhpcy5tX2RlcHRoQnVmZmVyIGlzIGZpcnN0IGFsbG9jYXRlZCBhbmQgcG9wdWxhdGVkIGluXHJcbiAgICogQ29tcHV0ZURlcHRoKCkgYW5kIHVzZWQgaW4gU29sdmVTb2xpZCgpLiBJdCB3aWxsIGJlXHJcbiAgICogcmVhbGxvY2F0ZWQgb24gc3Vic2VxdWVudCBDcmVhdGVQYXJ0aWNsZSgpIGNhbGxzLlxyXG4gICAqL1xyXG4gIG1fZGVwdGhCdWZmZXI6IG51bWJlcltdID0gW107XHJcbiAgbV9jb2xvckJ1ZmZlciA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtX1VzZXJPdmVycmlkYWJsZUJ1ZmZlcjxiMkNvbG9yPigpO1xyXG4gIG1fZ3JvdXBCdWZmZXI6IEFycmF5PGIyUGFydGljbGVHcm91cCB8IG51bGw+ID0gW107XHJcbiAgbV91c2VyRGF0YUJ1ZmZlciA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtX1VzZXJPdmVycmlkYWJsZUJ1ZmZlcjxhbnk+KCk7XHJcbiAgLyoqXHJcbiAgICogU3R1Y2sgcGFydGljbGUgZGV0ZWN0aW9uIHBhcmFtZXRlcnMgYW5kIHJlY29yZCBrZWVwaW5nXHJcbiAgICovXHJcbiAgbV9zdHVja1RocmVzaG9sZCA9IDA7XHJcbiAgbV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyID0gbmV3IGIyUGFydGljbGVTeXN0ZW1fVXNlck92ZXJyaWRhYmxlQnVmZmVyPG51bWJlcj4oKTtcclxuICBtX2JvZHlDb250YWN0Q291bnRCdWZmZXIgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8bnVtYmVyPigpO1xyXG4gIG1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8bnVtYmVyPigpO1xyXG4gIG1fc3R1Y2tQYXJ0aWNsZUJ1ZmZlciA9IG5ldyBiMkdyb3dhYmxlQnVmZmVyPG51bWJlcj4oKCkgPT4gMCk7XHJcbiAgbV9wcm94eUJ1ZmZlciA9IG5ldyBiMkdyb3dhYmxlQnVmZmVyPGIyUGFydGljbGVTeXN0ZW1fUHJveHk+KCgpID0+IG5ldyBiMlBhcnRpY2xlU3lzdGVtX1Byb3h5KCkpO1xyXG4gIG1fY29udGFjdEJ1ZmZlciA9IG5ldyBiMkdyb3dhYmxlQnVmZmVyPGIyUGFydGljbGVDb250YWN0PigoKSA9PiBuZXcgYjJQYXJ0aWNsZUNvbnRhY3QoKSk7XHJcbiAgbV9ib2R5Q29udGFjdEJ1ZmZlciA9IG5ldyBiMkdyb3dhYmxlQnVmZmVyPGIyUGFydGljbGVCb2R5Q29udGFjdD4oXHJcbiAgICAoKSA9PiBuZXcgYjJQYXJ0aWNsZUJvZHlDb250YWN0KCksXHJcbiAgKTtcclxuICBtX3BhaXJCdWZmZXIgPSBuZXcgYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlUGFpcj4oKCkgPT4gbmV3IGIyUGFydGljbGVQYWlyKCkpO1xyXG4gIG1fdHJpYWRCdWZmZXIgPSBuZXcgYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlVHJpYWQ+KCgpID0+IG5ldyBiMlBhcnRpY2xlVHJpYWQoKSk7XHJcbiAgLyoqXHJcbiAgICogVGltZSBlYWNoIHBhcnRpY2xlIHNob3VsZCBiZSBkZXN0cm95ZWQgcmVsYXRpdmUgdG8gdGhlIGxhc3RcclxuICAgKiB0aW1lIHRoaXMubV90aW1lRWxhcHNlZCB3YXMgaW5pdGlhbGl6ZWQuICBFYWNoIHVuaXQgb2YgdGltZVxyXG4gICAqIGNvcnJlc3BvbmRzIHRvIGIyUGFydGljbGVTeXN0ZW1EZWY6OmxpZmV0aW1lR3JhbnVsYXJpdHlcclxuICAgKiBzZWNvbmRzLlxyXG4gICAqL1xyXG4gIG1fZXhwaXJhdGlvblRpbWVCdWZmZXIgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8bnVtYmVyPigpO1xyXG4gIC8qKlxyXG4gICAqIExpc3Qgb2YgcGFydGljbGUgaW5kaWNlcyBzb3J0ZWQgYnkgZXhwaXJhdGlvbiB0aW1lLlxyXG4gICAqL1xyXG4gIG1faW5kZXhCeUV4cGlyYXRpb25UaW1lQnVmZmVyID0gbmV3IGIyUGFydGljbGVTeXN0ZW1fVXNlck92ZXJyaWRhYmxlQnVmZmVyPG51bWJlcj4oKTtcclxuICAvKipcclxuICAgKiBUaW1lIGVsYXBzZWQgaW4gMzI6MzIgZml4ZWQgcG9pbnQuICBFYWNoIG5vbi1mcmFjdGlvbmFsIHVuaXRcclxuICAgKiBvZiB0aW1lIGNvcnJlc3BvbmRzIHRvXHJcbiAgICogYjJQYXJ0aWNsZVN5c3RlbURlZjo6bGlmZXRpbWVHcmFudWxhcml0eSBzZWNvbmRzLlxyXG4gICAqL1xyXG4gIC8vIFRPRE86IGNoZWNrIGFuZCBpbXBsZW1lbnQgb3B0aW1pemVkIFNNSSBzdG9yYWdlP1xyXG4gIG1fdGltZUVsYXBzZWQgPSAwO1xyXG4gIC8qKlxyXG4gICAqIFdoZXRoZXIgdGhlIGV4cGlyYXRpb24gdGltZSBidWZmZXIgaGFzIGJlZW4gbW9kaWZpZWQgYW5kXHJcbiAgICogbmVlZHMgdG8gYmUgcmVzb3J0ZWQuXHJcbiAgICovXHJcbiAgbV9leHBpcmF0aW9uVGltZUJ1ZmZlclJlcXVpcmVzU29ydGluZyA9IGZhbHNlO1xyXG4gIG1fZ3JvdXBDb3VudCA9IDA7XHJcbiAgbV9ncm91cExpc3Q6IGIyUGFydGljbGVHcm91cCB8IG51bGwgPSBudWxsO1xyXG4gIG1fZGVmID0gbmV3IGIyUGFydGljbGVTeXN0ZW1EZWYoKTtcclxuICBtX3dvcmxkOiBiMldvcmxkO1xyXG4gIG1fcHJldjogYjJQYXJ0aWNsZVN5c3RlbSB8IG51bGwgPSBudWxsO1xyXG4gIG1fbmV4dDogYjJQYXJ0aWNsZVN5c3RlbSB8IG51bGwgPSBudWxsO1xyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgeFRydW5jQml0cyA9IDEyO1xyXG4gIHN0YXRpYyByZWFkb25seSB5VHJ1bmNCaXRzID0gMTI7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHRhZ0JpdHMgPSA4ICogNDsgLy8gOHUgKiBzaXplb2YodWludDMyKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgeU9mZnNldCA9IDEgPDwgKGIyUGFydGljbGVTeXN0ZW0ueVRydW5jQml0cyAtIDEpO1xyXG4gIHN0YXRpYyByZWFkb25seSB5U2hpZnQgPSBiMlBhcnRpY2xlU3lzdGVtLnRhZ0JpdHMgLSBiMlBhcnRpY2xlU3lzdGVtLnlUcnVuY0JpdHM7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHhTaGlmdCA9XHJcbiAgICBiMlBhcnRpY2xlU3lzdGVtLnRhZ0JpdHMgLSBiMlBhcnRpY2xlU3lzdGVtLnlUcnVuY0JpdHMgLSBiMlBhcnRpY2xlU3lzdGVtLnhUcnVuY0JpdHM7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHhTY2FsZSA9IDEgPDwgYjJQYXJ0aWNsZVN5c3RlbS54U2hpZnQ7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHhPZmZzZXQgPSBiMlBhcnRpY2xlU3lzdGVtLnhTY2FsZSAqICgxIDw8IChiMlBhcnRpY2xlU3lzdGVtLnhUcnVuY0JpdHMgLSAxKSk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHlNYXNrID0gKCgxIDw8IGIyUGFydGljbGVTeXN0ZW0ueVRydW5jQml0cykgLSAxKSA8PCBiMlBhcnRpY2xlU3lzdGVtLnlTaGlmdDtcclxuICBzdGF0aWMgcmVhZG9ubHkgeE1hc2sgPSB+YjJQYXJ0aWNsZVN5c3RlbS55TWFzaztcclxuXHJcbiAgc3RhdGljIGNvbXB1dGVUYWcoeDogbnVtYmVyLCB5OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgLy8vcmV0dXJuICgodWludDMyKSh5ICsgeU9mZnNldCkgPDwgeVNoaWZ0KSArICh1aW50MzIpKHhTY2FsZSAqIHggKyB4T2Zmc2V0KTtcclxuICAgIHJldHVybiAoXHJcbiAgICAgICgoKCh5ICsgYjJQYXJ0aWNsZVN5c3RlbS55T2Zmc2V0KSA+Pj4gMCkgPDwgYjJQYXJ0aWNsZVN5c3RlbS55U2hpZnQpICtcclxuICAgICAgICAoKGIyUGFydGljbGVTeXN0ZW0ueFNjYWxlICogeCArIGIyUGFydGljbGVTeXN0ZW0ueE9mZnNldCkgPj4+IDApKSA+Pj5cclxuICAgICAgMFxyXG4gICAgKTtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBjb21wdXRlUmVsYXRpdmVUYWcodGFnOiBudW1iZXIsIHg6IG51bWJlciwgeTogbnVtYmVyKTogbnVtYmVyIHtcclxuICAgIC8vL3JldHVybiB0YWcgKyAoeSA8PCB5U2hpZnQpICsgKHggPDwgeFNoaWZ0KTtcclxuICAgIHJldHVybiAodGFnICsgKHkgPDwgYjJQYXJ0aWNsZVN5c3RlbS55U2hpZnQpICsgKHggPDwgYjJQYXJ0aWNsZVN5c3RlbS54U2hpZnQpKSA+Pj4gMDtcclxuICB9XHJcblxyXG4gIGNvbnN0cnVjdG9yKGRlZjogYjJQYXJ0aWNsZVN5c3RlbURlZiwgd29ybGQ6IGIyV29ybGQpIHtcclxuICAgIHRoaXMubV9pbnZlcnNlRGVuc2l0eSA9IDAuMDtcclxuICAgIHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyID0gMC4wO1xyXG4gICAgdGhpcy5tX2ludmVyc2VEaWFtZXRlciA9IDAuMDtcclxuICAgIHRoaXMubV9zcXVhcmVkRGlhbWV0ZXIgPSAwLjA7XHJcblxyXG4gICAgdGhpcy5TZXRTdHJpY3RDb250YWN0Q2hlY2soZGVmLnN0cmljdENvbnRhY3RDaGVjayk7XHJcbiAgICB0aGlzLlNldERlbnNpdHkoZGVmLmRlbnNpdHkpO1xyXG4gICAgdGhpcy5TZXRHcmF2aXR5U2NhbGUoZGVmLmdyYXZpdHlTY2FsZSk7XHJcbiAgICB0aGlzLlNldFJhZGl1cyhkZWYucmFkaXVzKTtcclxuICAgIHRoaXMuU2V0TWF4UGFydGljbGVDb3VudChkZWYubWF4Q291bnQpO1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChkZWYubGlmZXRpbWVHcmFudWxhcml0eSA+IDAuMCk7XHJcbiAgICB0aGlzLm1fZGVmID0gZGVmLkNsb25lKCk7XHJcbiAgICB0aGlzLm1fd29ybGQgPSB3b3JsZDtcclxuICAgIHRoaXMuU2V0RGVzdHJ1Y3Rpb25CeUFnZSh0aGlzLm1fZGVmLmRlc3Ryb3lCeUFnZSk7XHJcbiAgfVxyXG5cclxuICBEcm9wKCk6IHZvaWQge1xyXG4gICAgd2hpbGUgKHRoaXMubV9ncm91cExpc3QpIHtcclxuICAgICAgdGhpcy5EZXN0cm95UGFydGljbGVHcm91cCh0aGlzLm1fZ3JvdXBMaXN0KTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLkZyZWVVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyKTtcclxuICAgIHRoaXMuRnJlZVVzZXJPdmVycmlkYWJsZUJ1ZmZlcih0aGlzLm1fZmxhZ3NCdWZmZXIpO1xyXG4gICAgdGhpcy5GcmVlVXNlck92ZXJyaWRhYmxlQnVmZmVyKHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyKTtcclxuICAgIHRoaXMuRnJlZVVzZXJPdmVycmlkYWJsZUJ1ZmZlcih0aGlzLm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlcik7XHJcbiAgICB0aGlzLkZyZWVVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX2NvbnNlY3V0aXZlQ29udGFjdFN0ZXBzQnVmZmVyKTtcclxuICAgIHRoaXMuRnJlZVVzZXJPdmVycmlkYWJsZUJ1ZmZlcih0aGlzLm1fcG9zaXRpb25CdWZmZXIpO1xyXG4gICAgdGhpcy5GcmVlVXNlck92ZXJyaWRhYmxlQnVmZmVyKHRoaXMubV92ZWxvY2l0eUJ1ZmZlcik7XHJcbiAgICB0aGlzLkZyZWVVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX2NvbG9yQnVmZmVyKTtcclxuICAgIHRoaXMuRnJlZVVzZXJPdmVycmlkYWJsZUJ1ZmZlcih0aGlzLm1fdXNlckRhdGFCdWZmZXIpO1xyXG4gICAgdGhpcy5GcmVlVXNlck92ZXJyaWRhYmxlQnVmZmVyKHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlcik7XHJcbiAgICB0aGlzLkZyZWVVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX2luZGV4QnlFeHBpcmF0aW9uVGltZUJ1ZmZlcik7XHJcbiAgICB0aGlzLkZyZWVCdWZmZXIodGhpcy5tX2ZvcmNlQnVmZmVyLCB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSk7XHJcbiAgICB0aGlzLkZyZWVCdWZmZXIodGhpcy5tX3dlaWdodEJ1ZmZlciwgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gICAgdGhpcy5GcmVlQnVmZmVyKHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlciwgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gICAgdGhpcy5GcmVlQnVmZmVyKHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXIsIHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5KTtcclxuICAgIHRoaXMuRnJlZUJ1ZmZlcih0aGlzLm1fYWNjdW11bGF0aW9uMkJ1ZmZlciwgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gICAgdGhpcy5GcmVlQnVmZmVyKHRoaXMubV9kZXB0aEJ1ZmZlciwgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gICAgdGhpcy5GcmVlQnVmZmVyKHRoaXMubV9ncm91cEJ1ZmZlciwgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQ3JlYXRlIGEgcGFydGljbGUgd2hvc2UgcHJvcGVydGllcyBoYXZlIGJlZW4gZGVmaW5lZC5cclxuICAgKlxyXG4gICAqIE5vIHJlZmVyZW5jZSB0byB0aGUgZGVmaW5pdGlvbiBpcyByZXRhaW5lZC5cclxuICAgKlxyXG4gICAqIEEgc2ltdWxhdGlvbiBzdGVwIG11c3Qgb2NjdXIgYmVmb3JlIGl0J3MgcG9zc2libGUgdG8gaW50ZXJhY3RcclxuICAgKiB3aXRoIGEgbmV3bHkgY3JlYXRlZCBwYXJ0aWNsZS4gIEZvciBleGFtcGxlLFxyXG4gICAqIERlc3Ryb3lQYXJ0aWNsZUluU2hhcGUoKSB3aWxsIG5vdCBkZXN0cm95IGEgcGFydGljbGUgdW50aWxcclxuICAgKiBiMldvcmxkOjpTdGVwKCkgaGFzIGJlZW4gY2FsbGVkLlxyXG4gICAqXHJcbiAgICogd2FybmluZzogVGhpcyBmdW5jdGlvbiBpcyBsb2NrZWQgZHVyaW5nIGNhbGxiYWNrcy5cclxuICAgKi9cclxuICBDcmVhdGVQYXJ0aWNsZShkZWY6IGIySVBhcnRpY2xlRGVmKTogbnVtYmVyIHtcclxuICAgIGlmICh0aGlzLm1fd29ybGQuSXNMb2NrZWQoKSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuXHJcbiAgICBpZiAodGhpcy5tX2NvdW50ID49IHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5KSB7XHJcbiAgICAgIC8vIERvdWJsZSB0aGUgcGFydGljbGUgY2FwYWNpdHkuXHJcbiAgICAgIGNvbnN0IGNhcGFjaXR5ID0gdGhpcy5tX2NvdW50ID8gMiAqIHRoaXMubV9jb3VudCA6IGIyX21pblBhcnRpY2xlU3lzdGVtQnVmZmVyQ2FwYWNpdHk7XHJcbiAgICAgIHRoaXMuUmVhbGxvY2F0ZUludGVybmFsQWxsb2NhdGVkQnVmZmVycyhjYXBhY2l0eSk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX2NvdW50ID49IHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5KSB7XHJcbiAgICAgIC8vIElmIHRoZSBvbGRlc3QgcGFydGljbGUgc2hvdWxkIGJlIGRlc3Ryb3llZC4uLlxyXG4gICAgICBpZiAodGhpcy5tX2RlZi5kZXN0cm95QnlBZ2UpIHtcclxuICAgICAgICB0aGlzLkRlc3Ryb3lPbGRlc3RQYXJ0aWNsZSgwLCBmYWxzZSk7XHJcbiAgICAgICAgLy8gTmVlZCB0byBkZXN0cm95IHRoaXMgcGFydGljbGUgKm5vdyogc28gdGhhdCBpdCdzIHBvc3NpYmxlIHRvXHJcbiAgICAgICAgLy8gY3JlYXRlIGEgbmV3IHBhcnRpY2xlLlxyXG4gICAgICAgIHRoaXMuU29sdmVab21iaWUoKTtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICByZXR1cm4gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIGNvbnN0IGluZGV4ID0gdGhpcy5tX2NvdW50Kys7XHJcbiAgICB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpbmRleF0gPSAwO1xyXG4gICAgaWYgKHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGEpIHtcclxuICAgICAgdGhpcy5tX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YVtpbmRleF0gPSAwO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGEpIHtcclxuICAgICAgdGhpcy5tX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YVtpbmRleF0gPSAwO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhKSB7XHJcbiAgICAgIHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhW2luZGV4XSA9IDA7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YVtpbmRleF0gPSAodGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGFbaW5kZXhdIHx8IG5ldyBiMlZlYzIoKSkuQ29weShcclxuICAgICAgYjJNYXliZShkZWYucG9zaXRpb24sIGIyVmVjMi5aRVJPKSxcclxuICAgICk7XHJcbiAgICB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YVtpbmRleF0gPSAodGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbaW5kZXhdIHx8IG5ldyBiMlZlYzIoKSkuQ29weShcclxuICAgICAgYjJNYXliZShkZWYudmVsb2NpdHksIGIyVmVjMi5aRVJPKSxcclxuICAgICk7XHJcbiAgICB0aGlzLm1fd2VpZ2h0QnVmZmVyW2luZGV4XSA9IDA7XHJcbiAgICB0aGlzLm1fZm9yY2VCdWZmZXJbaW5kZXhdID0gKHRoaXMubV9mb3JjZUJ1ZmZlcltpbmRleF0gfHwgbmV3IGIyVmVjMigpKS5TZXRaZXJvKCk7XHJcbiAgICBpZiAodGhpcy5tX3N0YXRpY1ByZXNzdXJlQnVmZmVyKSB7XHJcbiAgICAgIHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlcltpbmRleF0gPSAwO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9kZXB0aEJ1ZmZlcikge1xyXG4gICAgICB0aGlzLm1fZGVwdGhCdWZmZXJbaW5kZXhdID0gMDtcclxuICAgIH1cclxuICAgIGNvbnN0IGNvbG9yOiBiMkNvbG9yID0gbmV3IGIyQ29sb3IoKS5Db3B5KGIyTWF5YmUoZGVmLmNvbG9yLCBiMkNvbG9yLlpFUk8pKTtcclxuICAgIGlmICh0aGlzLm1fY29sb3JCdWZmZXIuZGF0YSB8fCAhY29sb3IuSXNaZXJvKCkpIHtcclxuICAgICAgdGhpcy5tX2NvbG9yQnVmZmVyLmRhdGEgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX2NvbG9yQnVmZmVyLmRhdGEpO1xyXG4gICAgICB0aGlzLm1fY29sb3JCdWZmZXIuZGF0YVtpbmRleF0gPSAodGhpcy5tX2NvbG9yQnVmZmVyLmRhdGFbaW5kZXhdIHx8IG5ldyBiMkNvbG9yKCkpLkNvcHkoXHJcbiAgICAgICAgY29sb3IsXHJcbiAgICAgICk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX3VzZXJEYXRhQnVmZmVyLmRhdGEgfHwgZGVmLnVzZXJEYXRhKSB7XHJcbiAgICAgIHRoaXMubV91c2VyRGF0YUJ1ZmZlci5kYXRhID0gdGhpcy5SZXF1ZXN0QnVmZmVyKHRoaXMubV91c2VyRGF0YUJ1ZmZlci5kYXRhKTtcclxuICAgICAgdGhpcy5tX3VzZXJEYXRhQnVmZmVyLmRhdGFbaW5kZXhdID0gZGVmLnVzZXJEYXRhO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhKSB7XHJcbiAgICAgIHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW2luZGV4XSA9IG51bGw7XHJcbiAgICB9XHJcbiAgICAvLy9Qcm94eSYgcHJveHkgPSBtX3Byb3h5QnVmZmVyLkFwcGVuZCgpO1xyXG4gICAgY29uc3QgcHJveHkgPSB0aGlzLm1fcHJveHlCdWZmZXIuZGF0YVt0aGlzLm1fcHJveHlCdWZmZXIuQXBwZW5kKCldO1xyXG5cclxuICAgIC8vIElmIHBhcnRpY2xlIGxpZmV0aW1lcyBhcmUgZW5hYmxlZCBvciB0aGUgbGlmZXRpbWUgaXMgc2V0IGluIHRoZSBwYXJ0aWNsZVxyXG4gICAgLy8gZGVmaW5pdGlvbiwgaW5pdGlhbGl6ZSB0aGUgbGlmZXRpbWUuXHJcbiAgICBjb25zdCBsaWZldGltZSA9IGIyTWF5YmUoZGVmLmxpZmV0aW1lLCAwLjApO1xyXG4gICAgY29uc3QgZmluaXRlTGlmZXRpbWUgPSBsaWZldGltZSA+IDAuMDtcclxuICAgIGlmICh0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSB8fCBmaW5pdGVMaWZldGltZSkge1xyXG4gICAgICB0aGlzLlNldFBhcnRpY2xlTGlmZXRpbWUoXHJcbiAgICAgICAgaW5kZXgsXHJcbiAgICAgICAgZmluaXRlTGlmZXRpbWUgPyBsaWZldGltZSA6IHRoaXMuRXhwaXJhdGlvblRpbWVUb0xpZmV0aW1lKC10aGlzLkdldFF1YW50aXplZFRpbWVFbGFwc2VkKCkpLFxyXG4gICAgICApO1xyXG4gICAgICAvLyBBZGQgYSByZWZlcmVuY2UgdG8gdGhlIG5ld2x5IGFkZGVkIHBhcnRpY2xlIHRvIHRoZSBlbmQgb2YgdGhlXHJcbiAgICAgIC8vIHF1ZXVlLlxyXG4gICAgICB0aGlzLm1faW5kZXhCeUV4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGFbaW5kZXhdID0gaW5kZXg7XHJcbiAgICB9XHJcblxyXG4gICAgcHJveHkuaW5kZXggPSBpbmRleDtcclxuICAgIGNvbnN0IGdyb3VwID0gYjJNYXliZShkZWYuZ3JvdXAsIG51bGwpO1xyXG4gICAgdGhpcy5tX2dyb3VwQnVmZmVyW2luZGV4XSA9IGdyb3VwO1xyXG4gICAgaWYgKGdyb3VwKSB7XHJcbiAgICAgIGlmIChncm91cC5tX2ZpcnN0SW5kZXggPCBncm91cC5tX2xhc3RJbmRleCkge1xyXG4gICAgICAgIC8vIE1vdmUgcGFydGljbGVzIGluIHRoZSBncm91cCBqdXN0IGJlZm9yZSB0aGUgbmV3IHBhcnRpY2xlLlxyXG4gICAgICAgIHRoaXMuUm90YXRlQnVmZmVyKGdyb3VwLm1fZmlyc3RJbmRleCwgZ3JvdXAubV9sYXN0SW5kZXgsIGluZGV4KTtcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGdyb3VwLm1fbGFzdEluZGV4ID09PSBpbmRleCk7XHJcbiAgICAgICAgLy8gVXBkYXRlIHRoZSBpbmRleCByYW5nZSBvZiB0aGUgZ3JvdXAgdG8gY29udGFpbiB0aGUgbmV3IHBhcnRpY2xlLlxyXG4gICAgICAgIGdyb3VwLm1fbGFzdEluZGV4ID0gaW5kZXggKyAxO1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIC8vIElmIHRoZSBncm91cCBpcyBlbXB0eSwgcmVzZXQgdGhlIGluZGV4IHJhbmdlIHRvIGNvbnRhaW4gb25seSB0aGVcclxuICAgICAgICAvLyBuZXcgcGFydGljbGUuXHJcbiAgICAgICAgZ3JvdXAubV9maXJzdEluZGV4ID0gaW5kZXg7XHJcbiAgICAgICAgZ3JvdXAubV9sYXN0SW5kZXggPSBpbmRleCArIDE7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIHRoaXMuU2V0UGFydGljbGVGbGFncyhpbmRleCwgYjJNYXliZShkZWYuZmxhZ3MsIDApKTtcclxuICAgIHJldHVybiBpbmRleDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFJldHJpZXZlIGEgaGFuZGxlIHRvIHRoZSBwYXJ0aWNsZSBhdCB0aGUgc3BlY2lmaWVkIGluZGV4LlxyXG4gICAqXHJcbiAgICogUGxlYXNlIHNlZSAjYjJQYXJ0aWNsZUhhbmRsZSBmb3Igd2h5IHlvdSBtaWdodCB3YW50IGEgaGFuZGxlLlxyXG4gICAqL1xyXG4gIEdldFBhcnRpY2xlSGFuZGxlRnJvbUluZGV4KGluZGV4OiBudW1iZXIpOiBiMlBhcnRpY2xlSGFuZGxlIHtcclxuICAgICEhQjJfREVCVUcgJiZcclxuICAgICAgYjJBc3NlcnQoaW5kZXggPj0gMCAmJiBpbmRleCA8IHRoaXMuR2V0UGFydGljbGVDb3VudCgpICYmIGluZGV4ICE9PSBiMl9pbnZhbGlkUGFydGljbGVJbmRleCk7XHJcbiAgICB0aGlzLm1faGFuZGxlSW5kZXhCdWZmZXIuZGF0YSA9IHRoaXMuUmVxdWVzdEJ1ZmZlcih0aGlzLm1faGFuZGxlSW5kZXhCdWZmZXIuZGF0YSk7XHJcbiAgICBsZXQgaGFuZGxlID0gdGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyLmRhdGFbaW5kZXhdO1xyXG4gICAgaWYgKGhhbmRsZSkge1xyXG4gICAgICByZXR1cm4gaGFuZGxlO1xyXG4gICAgfVxyXG4gICAgLy8gQ3JlYXRlIGEgaGFuZGxlLlxyXG4gICAgLy8vaGFuZGxlID0gbV9oYW5kbGVBbGxvY2F0b3IuQWxsb2NhdGUoKTtcclxuICAgIGhhbmRsZSA9IG5ldyBiMlBhcnRpY2xlSGFuZGxlKCk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGhhbmRsZSAhPT0gbnVsbCk7XHJcbiAgICBoYW5kbGUuaW5kZXggPSBpbmRleDtcclxuICAgIHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW2luZGV4XSA9IGhhbmRsZTtcclxuICAgIHJldHVybiBoYW5kbGU7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBEZXN0cm95IGEgcGFydGljbGUuXHJcbiAgICpcclxuICAgKiBUaGUgcGFydGljbGUgaXMgcmVtb3ZlZCBhZnRlciB0aGUgbmV4dCBzaW11bGF0aW9uIHN0ZXAgKHNlZVxyXG4gICAqIGIyV29ybGQ6OlN0ZXAoKSkuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gaW5kZXggSW5kZXggb2YgdGhlIHBhcnRpY2xlIHRvIGRlc3Ryb3kuXHJcbiAgICogQHBhcmFtIGNhbGxEZXN0cnVjdGlvbkxpc3RlbmVyIFdoZXRoZXIgdG8gY2FsbCB0aGVcclxuICAgKiAgICAgIGRlc3RydWN0aW9uIGxpc3RlbmVyIGp1c3QgYmVmb3JlIHRoZSBwYXJ0aWNsZSBpc1xyXG4gICAqICAgICAgZGVzdHJveWVkLlxyXG4gICAqL1xyXG4gIERlc3Ryb3lQYXJ0aWNsZShpbmRleDogbnVtYmVyLCBjYWxsRGVzdHJ1Y3Rpb25MaXN0ZW5lciA9IGZhbHNlKTogdm9pZCB7XHJcbiAgICBsZXQgZmxhZ3MgPSBiMlBhcnRpY2xlRmxhZy5iMl96b21iaWVQYXJ0aWNsZTtcclxuICAgIGlmIChjYWxsRGVzdHJ1Y3Rpb25MaXN0ZW5lcikge1xyXG4gICAgICBmbGFncyB8PSBiMlBhcnRpY2xlRmxhZy5iMl9kZXN0cnVjdGlvbkxpc3RlbmVyUGFydGljbGU7XHJcbiAgICB9XHJcbiAgICB0aGlzLlNldFBhcnRpY2xlRmxhZ3MoaW5kZXgsIHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2luZGV4XSB8IGZsYWdzKTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIERlc3Ryb3kgdGhlIE50aCBvbGRlc3QgcGFydGljbGUgaW4gdGhlIHN5c3RlbS5cclxuICAgKlxyXG4gICAqIFRoZSBwYXJ0aWNsZSBpcyByZW1vdmVkIGFmdGVyIHRoZSBuZXh0IGIyV29ybGQ6OlN0ZXAoKS5cclxuICAgKlxyXG4gICAqIEBwYXJhbSBpbmRleCBJbmRleCBvZiB0aGUgTnRoIG9sZGVzdCBwYXJ0aWNsZSB0b1xyXG4gICAqICAgICAgZGVzdHJveSwgMCB3aWxsIGRlc3Ryb3kgdGhlIG9sZGVzdCBwYXJ0aWNsZSBpbiB0aGVcclxuICAgKiAgICAgIHN5c3RlbSwgMSB3aWxsIGRlc3Ryb3kgdGhlIG5leHQgb2xkZXN0IHBhcnRpY2xlIGV0Yy5cclxuICAgKiBAcGFyYW0gY2FsbERlc3RydWN0aW9uTGlzdGVuZXIgV2hldGhlciB0byBjYWxsIHRoZVxyXG4gICAqICAgICAgZGVzdHJ1Y3Rpb24gbGlzdGVuZXIganVzdCBiZWZvcmUgdGhlIHBhcnRpY2xlIGlzXHJcbiAgICogICAgICBkZXN0cm95ZWQuXHJcbiAgICovXHJcbiAgRGVzdHJveU9sZGVzdFBhcnRpY2xlKGluZGV4OiBudW1iZXIsIGNhbGxEZXN0cnVjdGlvbkxpc3RlbmVyID0gZmFsc2UpOiB2b2lkIHtcclxuICAgIGNvbnN0IHBhcnRpY2xlQ291bnQgPSB0aGlzLkdldFBhcnRpY2xlQ291bnQoKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoaW5kZXggPj0gMCAmJiBpbmRleCA8IHBhcnRpY2xlQ291bnQpO1xyXG4gICAgLy8gTWFrZSBzdXJlIHBhcnRpY2xlIGxpZmV0aW1lIHRyYWNraW5nIGlzIGVuYWJsZWQuXHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSAhPT0gbnVsbCk7XHJcbiAgICAvLyBEZXN0cm95IHRoZSBvbGRlc3QgcGFydGljbGUgKHByZWZlcnJpbmcgdG8gZGVzdHJveSBmaW5pdGVcclxuICAgIC8vIGxpZmV0aW1lIHBhcnRpY2xlcyBmaXJzdCkgdG8gZnJlZSBhIHNsb3QgaW4gdGhlIGJ1ZmZlci5cclxuICAgIGNvbnN0IG9sZGVzdEZpbml0ZUxpZmV0aW1lUGFydGljbGUgPSB0aGlzLm1faW5kZXhCeUV4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGFbXHJcbiAgICAgIHBhcnRpY2xlQ291bnQgLSAoaW5kZXggKyAxKVxyXG4gICAgXTtcclxuICAgIGNvbnN0IG9sZGVzdEluZmluaXRlTGlmZXRpbWVQYXJ0aWNsZSA9IHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtpbmRleF07XHJcbiAgICB0aGlzLkRlc3Ryb3lQYXJ0aWNsZShcclxuICAgICAgdGhpcy5tX2V4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGFbb2xkZXN0RmluaXRlTGlmZXRpbWVQYXJ0aWNsZV0gPiAwLjBcclxuICAgICAgICA/IG9sZGVzdEZpbml0ZUxpZmV0aW1lUGFydGljbGVcclxuICAgICAgICA6IG9sZGVzdEluZmluaXRlTGlmZXRpbWVQYXJ0aWNsZSxcclxuICAgICAgY2FsbERlc3RydWN0aW9uTGlzdGVuZXIsXHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogRGVzdHJveSBwYXJ0aWNsZXMgaW5zaWRlIGEgc2hhcGUuXHJcbiAgICpcclxuICAgKiB3YXJuaW5nOiBUaGlzIGZ1bmN0aW9uIGlzIGxvY2tlZCBkdXJpbmcgY2FsbGJhY2tzLlxyXG4gICAqXHJcbiAgICogSW4gYWRkaXRpb24sIHRoaXMgZnVuY3Rpb24gaW1tZWRpYXRlbHkgZGVzdHJveXMgcGFydGljbGVzIGluXHJcbiAgICogdGhlIHNoYXBlIGluIGNvbnN0cmFzdCB0byBEZXN0cm95UGFydGljbGUoKSB3aGljaCBkZWZlcnMgdGhlXHJcbiAgICogZGVzdHJ1Y3Rpb24gdW50aWwgdGhlIG5leHQgc2ltdWxhdGlvbiBzdGVwLlxyXG4gICAqXHJcbiAgICogQHJldHVybiBOdW1iZXIgb2YgcGFydGljbGVzIGRlc3Ryb3llZC5cclxuICAgKiBAcGFyYW0gc2hhcGUgU2hhcGUgd2hpY2ggZW5jbG9zZXMgcGFydGljbGVzXHJcbiAgICogICAgICB0aGF0IHNob3VsZCBiZSBkZXN0cm95ZWQuXHJcbiAgICogQHBhcmFtIHhmIFRyYW5zZm9ybSBhcHBsaWVkIHRvIHRoZSBzaGFwZS5cclxuICAgKiBAcGFyYW0gY2FsbERlc3RydWN0aW9uTGlzdGVuZXIgV2hldGhlciB0byBjYWxsIHRoZVxyXG4gICAqICAgICAgd29ybGQgYjJEZXN0cnVjdGlvbkxpc3RlbmVyIGZvciBlYWNoIHBhcnRpY2xlXHJcbiAgICogICAgICBkZXN0cm95ZWQuXHJcbiAgICovXHJcbiAgRGVzdHJveVBhcnRpY2xlc0luU2hhcGUoXHJcbiAgICBzaGFwZTogYjJTaGFwZSxcclxuICAgIHhmOiBiMlRyYW5zZm9ybSxcclxuICAgIGNhbGxEZXN0cnVjdGlvbkxpc3RlbmVyID0gZmFsc2UsXHJcbiAgKTogbnVtYmVyIHtcclxuICAgIGNvbnN0IHNfYWFiYiA9IGIyUGFydGljbGVTeXN0ZW0uRGVzdHJveVBhcnRpY2xlc0luU2hhcGVfc19hYWJiO1xyXG4gICAgaWYgKHRoaXMubV93b3JsZC5Jc0xvY2tlZCgpKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IGNhbGxiYWNrID0gbmV3IGIyUGFydGljbGVTeXN0ZW1fRGVzdHJveVBhcnRpY2xlc0luU2hhcGVDYWxsYmFjayhcclxuICAgICAgdGhpcyxcclxuICAgICAgc2hhcGUsXHJcbiAgICAgIHhmLFxyXG4gICAgICBjYWxsRGVzdHJ1Y3Rpb25MaXN0ZW5lcixcclxuICAgICk7XHJcblxyXG4gICAgY29uc3QgYWFiYiA9IHNfYWFiYjtcclxuICAgIHNoYXBlLkNvbXB1dGVBQUJCKGFhYmIsIHhmLCAwKTtcclxuICAgIHRoaXMubV93b3JsZC5RdWVyeUFBQkIoY2FsbGJhY2ssIGFhYmIpO1xyXG4gICAgcmV0dXJuIGNhbGxiYWNrLkRlc3Ryb3llZCgpO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IERlc3Ryb3lQYXJ0aWNsZXNJblNoYXBlX3NfYWFiYiA9IG5ldyBiMkFBQkIoKTtcclxuXHJcbiAgLyoqXHJcbiAgICogQ3JlYXRlIGEgcGFydGljbGUgZ3JvdXAgd2hvc2UgcHJvcGVydGllcyBoYXZlIGJlZW4gZGVmaW5lZC5cclxuICAgKlxyXG4gICAqIE5vIHJlZmVyZW5jZSB0byB0aGUgZGVmaW5pdGlvbiBpcyByZXRhaW5lZC5cclxuICAgKlxyXG4gICAqIHdhcm5pbmc6IFRoaXMgZnVuY3Rpb24gaXMgbG9ja2VkIGR1cmluZyBjYWxsYmFja3MuXHJcbiAgICovXHJcbiAgQ3JlYXRlUGFydGljbGVHcm91cChncm91cERlZjogYjJJUGFydGljbGVHcm91cERlZik6IGIyUGFydGljbGVHcm91cCB7XHJcbiAgICBjb25zdCBzX3RyYW5zZm9ybSA9IGIyUGFydGljbGVTeXN0ZW0uQ3JlYXRlUGFydGljbGVHcm91cF9zX3RyYW5zZm9ybTtcclxuXHJcbiAgICBpZiAodGhpcy5tX3dvcmxkLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgdHJhbnNmb3JtID0gc190cmFuc2Zvcm07XHJcbiAgICB0cmFuc2Zvcm0uU2V0UG9zaXRpb25BbmdsZShiMk1heWJlKGdyb3VwRGVmLnBvc2l0aW9uLCBiMlZlYzIuWkVSTyksIGIyTWF5YmUoZ3JvdXBEZWYuYW5nbGUsIDApKTtcclxuICAgIGNvbnN0IGZpcnN0SW5kZXggPSB0aGlzLm1fY291bnQ7XHJcbiAgICBpZiAoZ3JvdXBEZWYuc2hhcGUpIHtcclxuICAgICAgdGhpcy5DcmVhdGVQYXJ0aWNsZXNXaXRoU2hhcGVGb3JHcm91cChncm91cERlZi5zaGFwZSwgZ3JvdXBEZWYsIHRyYW5zZm9ybSk7XHJcbiAgICB9XHJcbiAgICBpZiAoZ3JvdXBEZWYuc2hhcGVzKSB7XHJcbiAgICAgIHRoaXMuQ3JlYXRlUGFydGljbGVzV2l0aFNoYXBlc0Zvckdyb3VwKFxyXG4gICAgICAgIGdyb3VwRGVmLnNoYXBlcyxcclxuICAgICAgICBiMk1heWJlKGdyb3VwRGVmLnNoYXBlQ291bnQsIGdyb3VwRGVmLnNoYXBlcy5sZW5ndGgpLFxyXG4gICAgICAgIGdyb3VwRGVmLFxyXG4gICAgICAgIHRyYW5zZm9ybSxcclxuICAgICAgKTtcclxuICAgIH1cclxuICAgIGlmIChncm91cERlZi5wb3NpdGlvbkRhdGEpIHtcclxuICAgICAgY29uc3QgY291bnQgPSBiMk1heWJlKGdyb3VwRGVmLnBhcnRpY2xlQ291bnQsIGdyb3VwRGVmLnBvc2l0aW9uRGF0YS5sZW5ndGgpO1xyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IGNvdW50OyBpKyspIHtcclxuICAgICAgICBjb25zdCBwID0gZ3JvdXBEZWYucG9zaXRpb25EYXRhW2ldO1xyXG4gICAgICAgIHRoaXMuQ3JlYXRlUGFydGljbGVGb3JHcm91cChncm91cERlZiwgdHJhbnNmb3JtLCBwKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgY29uc3QgbGFzdEluZGV4ID0gdGhpcy5tX2NvdW50O1xyXG5cclxuICAgIGxldCBncm91cCA9IG5ldyBiMlBhcnRpY2xlR3JvdXAodGhpcyk7XHJcbiAgICBncm91cC5tX2ZpcnN0SW5kZXggPSBmaXJzdEluZGV4O1xyXG4gICAgZ3JvdXAubV9sYXN0SW5kZXggPSBsYXN0SW5kZXg7XHJcbiAgICBncm91cC5tX3N0cmVuZ3RoID0gYjJNYXliZShncm91cERlZi5zdHJlbmd0aCwgMSk7XHJcbiAgICBncm91cC5tX3VzZXJEYXRhID0gZ3JvdXBEZWYudXNlckRhdGE7XHJcbiAgICBncm91cC5tX3RyYW5zZm9ybS5Db3B5KHRyYW5zZm9ybSk7XHJcbiAgICBncm91cC5tX3ByZXYgPSBudWxsO1xyXG4gICAgZ3JvdXAubV9uZXh0ID0gdGhpcy5tX2dyb3VwTGlzdDtcclxuICAgIGlmICh0aGlzLm1fZ3JvdXBMaXN0KSB7XHJcbiAgICAgIHRoaXMubV9ncm91cExpc3QubV9wcmV2ID0gZ3JvdXA7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fZ3JvdXBMaXN0ID0gZ3JvdXA7XHJcbiAgICArK3RoaXMubV9ncm91cENvdW50O1xyXG4gICAgZm9yIChsZXQgaSA9IGZpcnN0SW5kZXg7IGkgPCBsYXN0SW5kZXg7IGkrKykge1xyXG4gICAgICB0aGlzLm1fZ3JvdXBCdWZmZXJbaV0gPSBncm91cDtcclxuICAgIH1cclxuICAgIHRoaXMuU2V0R3JvdXBGbGFncyhncm91cCwgYjJNYXliZShncm91cERlZi5ncm91cEZsYWdzLCAwKSk7XHJcblxyXG4gICAgLy8gQ3JlYXRlIHBhaXJzIGFuZCB0cmlhZHMgYmV0d2VlbiBwYXJ0aWNsZXMgaW4gdGhlIGdyb3VwLlxyXG4gICAgY29uc3QgZmlsdGVyID0gbmV3IGIyUGFydGljbGVTeXN0ZW1fQ29ubmVjdGlvbkZpbHRlcigpO1xyXG4gICAgdGhpcy5VcGRhdGVDb250YWN0cyh0cnVlKTtcclxuICAgIHRoaXMuVXBkYXRlUGFpcnNBbmRUcmlhZHMoZmlyc3RJbmRleCwgbGFzdEluZGV4LCBmaWx0ZXIpO1xyXG5cclxuICAgIGlmIChncm91cERlZi5ncm91cCkge1xyXG4gICAgICB0aGlzLkpvaW5QYXJ0aWNsZUdyb3Vwcyhncm91cERlZi5ncm91cCwgZ3JvdXApO1xyXG4gICAgICBncm91cCA9IGdyb3VwRGVmLmdyb3VwO1xyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBncm91cDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBDcmVhdGVQYXJ0aWNsZUdyb3VwX3NfdHJhbnNmb3JtID0gbmV3IGIyVHJhbnNmb3JtKCk7XHJcblxyXG4gIC8qKlxyXG4gICAqIEpvaW4gdHdvIHBhcnRpY2xlIGdyb3Vwcy5cclxuICAgKlxyXG4gICAqIHdhcm5pbmc6IFRoaXMgZnVuY3Rpb24gaXMgbG9ja2VkIGR1cmluZyBjYWxsYmFja3MuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gZ3JvdXBBIHRoZSBmaXJzdCBncm91cC4gRXhwYW5kcyB0byBlbmNvbXBhc3MgdGhlIHNlY29uZCBncm91cC5cclxuICAgKiBAcGFyYW0gZ3JvdXBCIHRoZSBzZWNvbmQgZ3JvdXAuIEl0IGlzIGRlc3Ryb3llZC5cclxuICAgKi9cclxuICBKb2luUGFydGljbGVHcm91cHMoZ3JvdXBBOiBiMlBhcnRpY2xlR3JvdXAsIGdyb3VwQjogYjJQYXJ0aWNsZUdyb3VwKTogdm9pZCB7XHJcbiAgICBpZiAodGhpcy5tX3dvcmxkLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChncm91cEEgIT09IGdyb3VwQik7XHJcbiAgICB0aGlzLlJvdGF0ZUJ1ZmZlcihncm91cEIubV9maXJzdEluZGV4LCBncm91cEIubV9sYXN0SW5kZXgsIHRoaXMubV9jb3VudCk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGdyb3VwQi5tX2xhc3RJbmRleCA9PT0gdGhpcy5tX2NvdW50KTtcclxuICAgIHRoaXMuUm90YXRlQnVmZmVyKGdyb3VwQS5tX2ZpcnN0SW5kZXgsIGdyb3VwQS5tX2xhc3RJbmRleCwgZ3JvdXBCLm1fZmlyc3RJbmRleCk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGdyb3VwQS5tX2xhc3RJbmRleCA9PT0gZ3JvdXBCLm1fZmlyc3RJbmRleCk7XHJcblxyXG4gICAgLy8gQ3JlYXRlIHBhaXJzIGFuZCB0cmlhZHMgY29ubmVjdGluZyBncm91cEEgYW5kIGdyb3VwQi5cclxuICAgIGNvbnN0IGZpbHRlciA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtX0pvaW5QYXJ0aWNsZUdyb3Vwc0ZpbHRlcihncm91cEIubV9maXJzdEluZGV4KTtcclxuICAgIHRoaXMuVXBkYXRlQ29udGFjdHModHJ1ZSk7XHJcbiAgICB0aGlzLlVwZGF0ZVBhaXJzQW5kVHJpYWRzKGdyb3VwQS5tX2ZpcnN0SW5kZXgsIGdyb3VwQi5tX2xhc3RJbmRleCwgZmlsdGVyKTtcclxuXHJcbiAgICBmb3IgKGxldCBpID0gZ3JvdXBCLm1fZmlyc3RJbmRleDsgaSA8IGdyb3VwQi5tX2xhc3RJbmRleDsgaSsrKSB7XHJcbiAgICAgIHRoaXMubV9ncm91cEJ1ZmZlcltpXSA9IGdyb3VwQTtcclxuICAgIH1cclxuICAgIGNvbnN0IGdyb3VwRmxhZ3MgPSBncm91cEEubV9ncm91cEZsYWdzIHwgZ3JvdXBCLm1fZ3JvdXBGbGFncztcclxuICAgIHRoaXMuU2V0R3JvdXBGbGFncyhncm91cEEsIGdyb3VwRmxhZ3MpO1xyXG4gICAgZ3JvdXBBLm1fbGFzdEluZGV4ID0gZ3JvdXBCLm1fbGFzdEluZGV4O1xyXG4gICAgZ3JvdXBCLm1fZmlyc3RJbmRleCA9IGdyb3VwQi5tX2xhc3RJbmRleDtcclxuICAgIHRoaXMuRGVzdHJveVBhcnRpY2xlR3JvdXAoZ3JvdXBCKTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFNwbGl0IHBhcnRpY2xlIGdyb3VwIGludG8gbXVsdGlwbGUgZGlzY29ubmVjdGVkIGdyb3Vwcy5cclxuICAgKlxyXG4gICAqIHdhcm5pbmc6IFRoaXMgZnVuY3Rpb24gaXMgbG9ja2VkIGR1cmluZyBjYWxsYmFja3MuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gZ3JvdXAgdGhlIGdyb3VwIHRvIGJlIHNwbGl0LlxyXG4gICAqL1xyXG4gIFNwbGl0UGFydGljbGVHcm91cChncm91cDogYjJQYXJ0aWNsZUdyb3VwKTogdm9pZCB7XHJcbiAgICB0aGlzLlVwZGF0ZUNvbnRhY3RzKHRydWUpO1xyXG4gICAgY29uc3QgcGFydGljbGVDb3VudCA9IGdyb3VwLkdldFBhcnRpY2xlQ291bnQoKTtcclxuICAgIC8vIFdlIGNyZWF0ZSBzZXZlcmFsIGxpbmtlZCBsaXN0cy4gRWFjaCBsaXN0IHJlcHJlc2VudHMgYSBzZXQgb2YgY29ubmVjdGVkIHBhcnRpY2xlcy5cclxuICAgIGNvbnN0IG5vZGVCdWZmZXI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZVtdID0gYjJNYWtlQXJyYXkoXHJcbiAgICAgIHBhcnRpY2xlQ291bnQsXHJcbiAgICAgIChpbmRleDogbnVtYmVyKSA9PiBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlKCksXHJcbiAgICApO1xyXG4gICAgYjJQYXJ0aWNsZVN5c3RlbS5Jbml0aWFsaXplUGFydGljbGVMaXN0cyhncm91cCwgbm9kZUJ1ZmZlcik7XHJcbiAgICB0aGlzLk1lcmdlUGFydGljbGVMaXN0c0luQ29udGFjdChncm91cCwgbm9kZUJ1ZmZlcik7XHJcbiAgICBjb25zdCBzdXJ2aXZpbmdMaXN0ID0gYjJQYXJ0aWNsZVN5c3RlbS5GaW5kTG9uZ2VzdFBhcnRpY2xlTGlzdChncm91cCwgbm9kZUJ1ZmZlcik7XHJcbiAgICB0aGlzLk1lcmdlWm9tYmllUGFydGljbGVMaXN0Tm9kZXMoZ3JvdXAsIG5vZGVCdWZmZXIsIHN1cnZpdmluZ0xpc3QpO1xyXG4gICAgdGhpcy5DcmVhdGVQYXJ0aWNsZUdyb3Vwc0Zyb21QYXJ0aWNsZUxpc3QoZ3JvdXAsIG5vZGVCdWZmZXIsIHN1cnZpdmluZ0xpc3QpO1xyXG4gICAgdGhpcy5VcGRhdGVQYWlyc0FuZFRyaWFkc1dpdGhQYXJ0aWNsZUxpc3QoZ3JvdXAsIG5vZGVCdWZmZXIpO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSB3b3JsZCBwYXJ0aWNsZSBncm91cCBsaXN0LiBXaXRoIHRoZSByZXR1cm5lZCBncm91cCxcclxuICAgKiB1c2UgYjJQYXJ0aWNsZUdyb3VwOjpHZXROZXh0IHRvIGdldCB0aGUgbmV4dCBncm91cCBpbiB0aGVcclxuICAgKiB3b3JsZCBsaXN0LlxyXG4gICAqXHJcbiAgICogQSBudWxsIGdyb3VwIGluZGljYXRlcyB0aGUgZW5kIG9mIHRoZSBsaXN0LlxyXG4gICAqXHJcbiAgICogQHJldHVybiB0aGUgaGVhZCBvZiB0aGUgd29ybGQgcGFydGljbGUgZ3JvdXAgbGlzdC5cclxuICAgKi9cclxuICBHZXRQYXJ0aWNsZUdyb3VwTGlzdCgpOiBiMlBhcnRpY2xlR3JvdXAgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fZ3JvdXBMaXN0O1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBudW1iZXIgb2YgcGFydGljbGUgZ3JvdXBzLlxyXG4gICAqL1xyXG4gIEdldFBhcnRpY2xlR3JvdXBDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ncm91cENvdW50O1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBudW1iZXIgb2YgcGFydGljbGVzLlxyXG4gICAqL1xyXG4gIEdldFBhcnRpY2xlQ291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY291bnQ7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIG1heGltdW0gbnVtYmVyIG9mIHBhcnRpY2xlcy5cclxuICAgKi9cclxuICBHZXRNYXhQYXJ0aWNsZUNvdW50KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2RlZi5tYXhDb3VudDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFNldCB0aGUgbWF4aW11bSBudW1iZXIgb2YgcGFydGljbGVzLlxyXG4gICAqXHJcbiAgICogQSB2YWx1ZSBvZiAwIG1lYW5zIHRoZXJlIGlzIG5vIG1heGltdW0uIFRoZSBwYXJ0aWNsZSBidWZmZXJzXHJcbiAgICogY2FuIGNvbnRpbnVlIHRvIGdyb3cgd2hpbGUgYjJXb3JsZCdzIGJsb2NrIGFsbG9jYXRvciBzdGlsbFxyXG4gICAqIGhhcyBtZW1vcnkuXHJcbiAgICpcclxuICAgKiBOb3RlOiBJZiB5b3UgdHJ5IHRvIENyZWF0ZVBhcnRpY2xlKCkgd2l0aCBtb3JlIHRoYW4gdGhpc1xyXG4gICAqIGNvdW50LCBiMl9pbnZhbGlkUGFydGljbGVJbmRleCBpcyByZXR1cm5lZCB1bmxlc3NcclxuICAgKiBTZXREZXN0cnVjdGlvbkJ5QWdlKCkgaXMgdXNlZCB0byBlbmFibGUgdGhlIGRlc3RydWN0aW9uIG9mXHJcbiAgICogdGhlIG9sZGVzdCBwYXJ0aWNsZXMgaW4gdGhlIHN5c3RlbS5cclxuICAgKi9cclxuICBTZXRNYXhQYXJ0aWNsZUNvdW50KGNvdW50OiBudW1iZXIpOiB2b2lkIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2NvdW50IDw9IGNvdW50KTtcclxuICAgIHRoaXMubV9kZWYubWF4Q291bnQgPSBjb3VudDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCBhbGwgZXhpc3RpbmcgcGFydGljbGUgZmxhZ3MuXHJcbiAgICovXHJcbiAgR2V0QWxsUGFydGljbGVGbGFncygpOiBiMlBhcnRpY2xlRmxhZyB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3M7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgYWxsIGV4aXN0aW5nIHBhcnRpY2xlIGdyb3VwIGZsYWdzLlxyXG4gICAqL1xyXG4gIEdldEFsbEdyb3VwRmxhZ3MoKTogYjJQYXJ0aWNsZUdyb3VwRmxhZyB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FsbEdyb3VwRmxhZ3M7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBQYXVzZSBvciB1bnBhdXNlIHRoZSBwYXJ0aWNsZSBzeXN0ZW0uIFdoZW4gcGF1c2VkLFxyXG4gICAqIGIyV29ybGQ6OlN0ZXAoKSBza2lwcyBvdmVyIHRoaXMgcGFydGljbGUgc3lzdGVtLiBBbGxcclxuICAgKiBiMlBhcnRpY2xlU3lzdGVtIGZ1bmN0aW9uIGNhbGxzIHN0aWxsIHdvcmsuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gcGF1c2VkIHBhdXNlZCBpcyB0cnVlIHRvIHBhdXNlLCBmYWxzZSB0byB1bi1wYXVzZS5cclxuICAgKi9cclxuICBTZXRQYXVzZWQocGF1c2VkOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fcGF1c2VkID0gcGF1c2VkO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogSW5pdGlhbGx5LCB0cnVlLCB0aGVuLCB0aGUgbGFzdCB2YWx1ZSBwYXNzZWQgaW50b1xyXG4gICAqIFNldFBhdXNlZCgpLlxyXG4gICAqXHJcbiAgICogQHJldHVybiB0cnVlIGlmIHRoZSBwYXJ0aWNsZSBzeXN0ZW0gaXMgYmVpbmcgdXBkYXRlZCBpbiBiMldvcmxkOjpTdGVwKCkuXHJcbiAgICovXHJcbiAgR2V0UGF1c2VkKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9wYXVzZWQ7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBDaGFuZ2UgdGhlIHBhcnRpY2xlIGRlbnNpdHkuXHJcbiAgICpcclxuICAgKiBQYXJ0aWNsZSBkZW5zaXR5IGFmZmVjdHMgdGhlIG1hc3Mgb2YgdGhlIHBhcnRpY2xlcywgd2hpY2ggaW5cclxuICAgKiB0dXJuIGFmZmVjdHMgaG93IHRoZSBwYXJ0aWNsZXMgaW50ZXJhY3Qgd2l0aCBiMkJvZGllcy4gTm90ZVxyXG4gICAqIHRoYXQgdGhlIGRlbnNpdHkgZG9lcyBub3QgYWZmZWN0IGhvdyB0aGUgcGFydGljbGVzIGludGVyYWN0XHJcbiAgICogd2l0aCBlYWNoIG90aGVyLlxyXG4gICAqL1xyXG4gIFNldERlbnNpdHkoZGVuc2l0eTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZGVmLmRlbnNpdHkgPSBkZW5zaXR5O1xyXG4gICAgdGhpcy5tX2ludmVyc2VEZW5zaXR5ID0gMSAvIHRoaXMubV9kZWYuZGVuc2l0eTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgcGFydGljbGUgZGVuc2l0eS5cclxuICAgKi9cclxuICBHZXREZW5zaXR5KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2RlZi5kZW5zaXR5O1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQ2hhbmdlIHRoZSBwYXJ0aWNsZSBncmF2aXR5IHNjYWxlLiBBZGp1c3RzIHRoZSBlZmZlY3Qgb2YgdGhlXHJcbiAgICogZ2xvYmFsIGdyYXZpdHkgdmVjdG9yIG9uIHBhcnRpY2xlcy5cclxuICAgKi9cclxuICBTZXRHcmF2aXR5U2NhbGUoZ3Jhdml0eVNjYWxlOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMubV9kZWYuZ3Jhdml0eVNjYWxlID0gZ3Jhdml0eVNjYWxlO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBwYXJ0aWNsZSBncmF2aXR5IHNjYWxlLlxyXG4gICAqL1xyXG4gIEdldEdyYXZpdHlTY2FsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9kZWYuZ3Jhdml0eVNjYWxlO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogRGFtcGluZyBpcyB1c2VkIHRvIHJlZHVjZSB0aGUgdmVsb2NpdHkgb2YgcGFydGljbGVzLiBUaGVcclxuICAgKiBkYW1waW5nIHBhcmFtZXRlciBjYW4gYmUgbGFyZ2VyIHRoYW4gMS4wZiBidXQgdGhlIGRhbXBpbmdcclxuICAgKiBlZmZlY3QgYmVjb21lcyBzZW5zaXRpdmUgdG8gdGhlIHRpbWUgc3RlcCB3aGVuIHRoZSBkYW1waW5nXHJcbiAgICogcGFyYW1ldGVyIGlzIGxhcmdlLlxyXG4gICAqL1xyXG4gIFNldERhbXBpbmcoZGFtcGluZzogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZGVmLmRhbXBpbmdTdHJlbmd0aCA9IGRhbXBpbmc7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgZGFtcGluZyBmb3IgcGFydGljbGVzXHJcbiAgICovXHJcbiAgR2V0RGFtcGluZygpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9kZWYuZGFtcGluZ1N0cmVuZ3RoO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQ2hhbmdlIHRoZSBudW1iZXIgb2YgaXRlcmF0aW9ucyB3aGVuIGNhbGN1bGF0aW5nIHRoZSBzdGF0aWNcclxuICAgKiBwcmVzc3VyZSBvZiBwYXJ0aWNsZXMuIEJ5IGRlZmF1bHQsIDggaXRlcmF0aW9ucy4gWW91IGNhblxyXG4gICAqIHJlZHVjZSB0aGUgbnVtYmVyIG9mIGl0ZXJhdGlvbnMgZG93biB0byAxIGluIHNvbWUgc2l0dWF0aW9ucyxcclxuICAgKiBidXQgdGhpcyBtYXkgY2F1c2UgaW5zdGFiaWxpdGllcyB3aGVuIG1hbnkgcGFydGljbGVzIGNvbWVcclxuICAgKiB0b2dldGhlci4gSWYgeW91IHNlZSBwYXJ0aWNsZXMgcG9wcGluZyBhd2F5IGZyb20gZWFjaCBvdGhlclxyXG4gICAqIGxpa2UgcG9wY29ybiwgeW91IG1heSBoYXZlIHRvIGluY3JlYXNlIHRoZSBudW1iZXIgb2ZcclxuICAgKiBpdGVyYXRpb25zLlxyXG4gICAqXHJcbiAgICogRm9yIGEgZGVzY3JpcHRpb24gb2Ygc3RhdGljIHByZXNzdXJlLCBzZWVcclxuICAgKiBodHRwOi8vZW4ud2lraXBlZGlhLm9yZy93aWtpL1N0YXRpY19wcmVzc3VyZSNTdGF0aWNfcHJlc3N1cmVfaW5fZmx1aWRfZHluYW1pY3NcclxuICAgKi9cclxuICBTZXRTdGF0aWNQcmVzc3VyZUl0ZXJhdGlvbnMoaXRlcmF0aW9uczogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZGVmLnN0YXRpY1ByZXNzdXJlSXRlcmF0aW9ucyA9IGl0ZXJhdGlvbnM7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIG51bWJlciBvZiBpdGVyYXRpb25zIGZvciBzdGF0aWMgcHJlc3N1cmUgb2ZcclxuICAgKiBwYXJ0aWNsZXMuXHJcbiAgICovXHJcbiAgR2V0U3RhdGljUHJlc3N1cmVJdGVyYXRpb25zKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2RlZi5zdGF0aWNQcmVzc3VyZUl0ZXJhdGlvbnM7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBDaGFuZ2UgdGhlIHBhcnRpY2xlIHJhZGl1cy5cclxuICAgKlxyXG4gICAqIFlvdSBzaG91bGQgc2V0IHRoaXMgb25seSBvbmNlLCBvbiB3b3JsZCBzdGFydC5cclxuICAgKiBJZiB5b3UgY2hhbmdlIHRoZSByYWRpdXMgZHVyaW5nIGV4ZWN1dGlvbiwgZXhpc3RpbmcgcGFydGljbGVzXHJcbiAgICogbWF5IGV4cGxvZGUsIHNocmluaywgb3IgYmVoYXZlIHVuZXhwZWN0ZWRseS5cclxuICAgKi9cclxuICBTZXRSYWRpdXMocmFkaXVzOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyID0gMiAqIHJhZGl1cztcclxuICAgIHRoaXMubV9zcXVhcmVkRGlhbWV0ZXIgPSB0aGlzLm1fcGFydGljbGVEaWFtZXRlciAqIHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyO1xyXG4gICAgdGhpcy5tX2ludmVyc2VEaWFtZXRlciA9IDEgLyB0aGlzLm1fcGFydGljbGVEaWFtZXRlcjtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgcGFydGljbGUgcmFkaXVzLlxyXG4gICAqL1xyXG4gIEdldFJhZGl1cygpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyIC8gMjtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgcG9zaXRpb24gb2YgZWFjaCBwYXJ0aWNsZVxyXG4gICAqXHJcbiAgICogQXJyYXkgaXMgbGVuZ3RoIEdldFBhcnRpY2xlQ291bnQoKVxyXG4gICAqXHJcbiAgICogQHJldHVybiB0aGUgcG9pbnRlciB0byB0aGUgaGVhZCBvZiB0aGUgcGFydGljbGUgcG9zaXRpb25zIGFycmF5LlxyXG4gICAqL1xyXG4gIEdldFBvc2l0aW9uQnVmZmVyKCk6IGIyVmVjMltdIHtcclxuICAgIHJldHVybiB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgdmVsb2NpdHkgb2YgZWFjaCBwYXJ0aWNsZVxyXG4gICAqXHJcbiAgICogQXJyYXkgaXMgbGVuZ3RoIEdldFBhcnRpY2xlQ291bnQoKVxyXG4gICAqXHJcbiAgICogQHJldHVybiB0aGUgcG9pbnRlciB0byB0aGUgaGVhZCBvZiB0aGUgcGFydGljbGUgdmVsb2NpdGllcyBhcnJheS5cclxuICAgKi9cclxuICBHZXRWZWxvY2l0eUJ1ZmZlcigpOiBiMlZlYzJbXSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIGNvbG9yIG9mIGVhY2ggcGFydGljbGVcclxuICAgKlxyXG4gICAqIEFycmF5IGlzIGxlbmd0aCBHZXRQYXJ0aWNsZUNvdW50KClcclxuICAgKlxyXG4gICAqIEByZXR1cm4gdGhlIHBvaW50ZXIgdG8gdGhlIGhlYWQgb2YgdGhlIHBhcnRpY2xlIGNvbG9ycyBhcnJheS5cclxuICAgKi9cclxuICBHZXRDb2xvckJ1ZmZlcigpOiBiMkNvbG9yW10ge1xyXG4gICAgdGhpcy5tX2NvbG9yQnVmZmVyLmRhdGEgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX2NvbG9yQnVmZmVyLmRhdGEpO1xyXG4gICAgcmV0dXJuIHRoaXMubV9jb2xvckJ1ZmZlci5kYXRhO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBwYXJ0aWNsZS1ncm91cCBvZiBlYWNoIHBhcnRpY2xlLlxyXG4gICAqXHJcbiAgICogQXJyYXkgaXMgbGVuZ3RoIEdldFBhcnRpY2xlQ291bnQoKVxyXG4gICAqXHJcbiAgICogQHJldHVybiB0aGUgcG9pbnRlciB0byB0aGUgaGVhZCBvZiB0aGUgcGFydGljbGUgZ3JvdXAgYXJyYXkuXHJcbiAgICovXHJcbiAgR2V0R3JvdXBCdWZmZXIoKTogQXJyYXk8YjJQYXJ0aWNsZUdyb3VwIHwgbnVsbD4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ncm91cEJ1ZmZlcjtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgd2VpZ2h0IG9mIGVhY2ggcGFydGljbGVcclxuICAgKlxyXG4gICAqIEFycmF5IGlzIGxlbmd0aCBHZXRQYXJ0aWNsZUNvdW50KClcclxuICAgKlxyXG4gICAqIEByZXR1cm4gdGhlIHBvaW50ZXIgdG8gdGhlIGhlYWQgb2YgdGhlIHBhcnRpY2xlIHBvc2l0aW9ucyBhcnJheS5cclxuICAgKi9cclxuICBHZXRXZWlnaHRCdWZmZXIoKTogbnVtYmVyW10ge1xyXG4gICAgcmV0dXJuIHRoaXMubV93ZWlnaHRCdWZmZXI7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIHVzZXItc3BlY2lmaWVkIGRhdGEgb2YgZWFjaCBwYXJ0aWNsZS5cclxuICAgKlxyXG4gICAqIEFycmF5IGlzIGxlbmd0aCBHZXRQYXJ0aWNsZUNvdW50KClcclxuICAgKlxyXG4gICAqIEByZXR1cm4gdGhlIHBvaW50ZXIgdG8gdGhlIGhlYWQgb2YgdGhlIHBhcnRpY2xlIHVzZXItZGF0YSBhcnJheS5cclxuICAgKi9cclxuICBHZXRVc2VyRGF0YUJ1ZmZlcjxUPigpOiBUW10ge1xyXG4gICAgdGhpcy5tX3VzZXJEYXRhQnVmZmVyLmRhdGEgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX3VzZXJEYXRhQnVmZmVyLmRhdGEpO1xyXG4gICAgcmV0dXJuIHRoaXMubV91c2VyRGF0YUJ1ZmZlci5kYXRhO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBmbGFncyBmb3IgZWFjaCBwYXJ0aWNsZS4gU2VlIHRoZSBiMlBhcnRpY2xlRmxhZyBlbnVtLlxyXG4gICAqXHJcbiAgICogQXJyYXkgaXMgbGVuZ3RoIEdldFBhcnRpY2xlQ291bnQoKVxyXG4gICAqXHJcbiAgICogQHJldHVybiB0aGUgcG9pbnRlciB0byB0aGUgaGVhZCBvZiB0aGUgcGFydGljbGUtZmxhZ3MgYXJyYXkuXHJcbiAgICovXHJcbiAgR2V0RmxhZ3NCdWZmZXIoKTogYjJQYXJ0aWNsZUZsYWdbXSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGE7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBTZXQgZmxhZ3MgZm9yIGEgcGFydGljbGUuIFNlZSB0aGUgYjJQYXJ0aWNsZUZsYWcgZW51bS5cclxuICAgKi9cclxuICBTZXRQYXJ0aWNsZUZsYWdzKGluZGV4OiBudW1iZXIsIG5ld0ZsYWdzOiBiMlBhcnRpY2xlRmxhZyk6IHZvaWQge1xyXG4gICAgY29uc3Qgb2xkRmxhZ3MgPSB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpbmRleF07XHJcbiAgICBpZiAob2xkRmxhZ3MgJiB+bmV3RmxhZ3MpIHtcclxuICAgICAgLy8gSWYgYW55IGZsYWdzIG1pZ2h0IGJlIHJlbW92ZWRcclxuICAgICAgdGhpcy5tX25lZWRzVXBkYXRlQWxsUGFydGljbGVGbGFncyA9IHRydWU7XHJcbiAgICB9XHJcbiAgICBpZiAofnRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgbmV3RmxhZ3MpIHtcclxuICAgICAgLy8gSWYgYW55IGZsYWdzIHdlcmUgYWRkZWRcclxuICAgICAgaWYgKG5ld0ZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfdGVuc2lsZVBhcnRpY2xlKSB7XHJcbiAgICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbjJCdWZmZXIgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX2FjY3VtdWxhdGlvbjJCdWZmZXIpO1xyXG4gICAgICB9XHJcbiAgICAgIGlmIChuZXdGbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX2NvbG9yTWl4aW5nUGFydGljbGUpIHtcclxuICAgICAgICB0aGlzLm1fY29sb3JCdWZmZXIuZGF0YSA9IHRoaXMuUmVxdWVzdEJ1ZmZlcih0aGlzLm1fY29sb3JCdWZmZXIuZGF0YSk7XHJcbiAgICAgIH1cclxuICAgICAgdGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgfD0gbmV3RmxhZ3M7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpbmRleF0gPSBuZXdGbGFncztcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCBmbGFncyBmb3IgYSBwYXJ0aWNsZS4gU2VlIHRoZSBiMlBhcnRpY2xlRmxhZyBlbnVtLlxyXG4gICAqL1xyXG4gIEdldFBhcnRpY2xlRmxhZ3MoaW5kZXg6IG51bWJlcik6IGIyUGFydGljbGVGbGFnIHtcclxuICAgIHJldHVybiB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpbmRleF07XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBTZXQgYW4gZXh0ZXJuYWwgYnVmZmVyIGZvciBwYXJ0aWNsZSBkYXRhLlxyXG4gICAqXHJcbiAgICogTm9ybWFsbHksIHRoZSBiMldvcmxkJ3MgYmxvY2sgYWxsb2NhdG9yIGlzIHVzZWQgZm9yIHBhcnRpY2xlXHJcbiAgICogZGF0YS4gSG93ZXZlciwgc29tZXRpbWVzIHlvdSBtYXkgaGF2ZSBhbiBPcGVuR0wgb3IgSmF2YVxyXG4gICAqIGJ1ZmZlciBmb3IgcGFydGljbGUgZGF0YS4gVG8gYXZvaWQgZGF0YSBkdXBsaWNhdGlvbiwgeW91IG1heVxyXG4gICAqIHN1cHBseSB0aGlzIGV4dGVybmFsIGJ1ZmZlci5cclxuICAgKlxyXG4gICAqIE5vdGUgdGhhdCwgd2hlbiBiMldvcmxkJ3MgYmxvY2sgYWxsb2NhdG9yIGlzIHVzZWQsIHRoZVxyXG4gICAqIHBhcnRpY2xlIGRhdGEgYnVmZmVycyBjYW4gZ3JvdyBhcyByZXF1aXJlZC4gSG93ZXZlciwgd2hlblxyXG4gICAqIGV4dGVybmFsIGJ1ZmZlcnMgYXJlIHVzZWQsIHRoZSBtYXhpbXVtIG51bWJlciBvZiBwYXJ0aWNsZXMgaXNcclxuICAgKiBjbGFtcGVkIHRvIHRoZSBzaXplIG9mIHRoZSBzbWFsbGVzdCBleHRlcm5hbCBidWZmZXIuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gYnVmZmVyIGEgcG9pbnRlciB0byBhIGJsb2NrIG9mIG1lbW9yeS5cclxuICAgKiBAcGFyYW0gY2FwYWNpdHkgdGhlIG51bWJlciBvZiB2YWx1ZXMgaW4gdGhlIGJsb2NrLlxyXG4gICAqL1xyXG4gIFNldEZsYWdzQnVmZmVyKGJ1ZmZlcjogYjJQYXJ0aWNsZUZsYWdbXSk6IHZvaWQge1xyXG4gICAgdGhpcy5TZXRVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX2ZsYWdzQnVmZmVyLCBidWZmZXIpO1xyXG4gIH1cclxuXHJcbiAgU2V0UG9zaXRpb25CdWZmZXIoYnVmZmVyOiBiMlZlYzJbXSB8IEZsb2F0MzJBcnJheSk6IHZvaWQge1xyXG4gICAgaWYgKGJ1ZmZlciBpbnN0YW5jZW9mIEZsb2F0MzJBcnJheSkge1xyXG4gICAgICBpZiAoKGJ1ZmZlci5sZW5ndGggJiAxKSAhPT0gMCkge1xyXG4gICAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgICB9XHJcbiAgICAgIGNvbnN0IGNvdW50OiBudW1iZXIgPSBidWZmZXIubGVuZ3RoIC8gMjtcclxuICAgICAgY29uc3QgYXJyYXk6IGIyVmVjMltdID0gW107XHJcbiAgICAgIGxldCBwdHIgPSAwO1xyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IGNvdW50OyArK2kpIHtcclxuICAgICAgICBhcnJheS5wdXNoKG5ldyBiMlZlYzIoYnVmZmVyW3B0cisrXSwgYnVmZmVyW3B0cisrXSkpO1xyXG4gICAgICB9XHJcbiAgICAgIGJ1ZmZlciA9IGFycmF5O1xyXG4gICAgfVxyXG4gICAgdGhpcy5TZXRVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX3Bvc2l0aW9uQnVmZmVyLCBidWZmZXIpO1xyXG4gIH1cclxuXHJcbiAgU2V0VmVsb2NpdHlCdWZmZXIoYnVmZmVyOiBiMlZlYzJbXSB8IEZsb2F0MzJBcnJheSk6IHZvaWQge1xyXG4gICAgaWYgKGJ1ZmZlciBpbnN0YW5jZW9mIEZsb2F0MzJBcnJheSkge1xyXG4gICAgICBpZiAoKGJ1ZmZlci5sZW5ndGggJiAxKSAhPT0gMCkge1xyXG4gICAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgICB9XHJcbiAgICAgIGNvbnN0IGNvdW50OiBudW1iZXIgPSBidWZmZXIubGVuZ3RoIC8gMjtcclxuICAgICAgY29uc3QgYXJyYXk6IGIyVmVjMltdID0gW107XHJcbiAgICAgIGxldCBwdHIgPSAwO1xyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IGNvdW50OyArK2kpIHtcclxuICAgICAgICBhcnJheS5wdXNoKG5ldyBiMlZlYzIoYnVmZmVyW3B0cisrXSwgYnVmZmVyW3B0cisrXSkpO1xyXG4gICAgICB9XHJcbiAgICAgIGJ1ZmZlciA9IGFycmF5O1xyXG4gICAgfVxyXG4gICAgdGhpcy5TZXRVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX3ZlbG9jaXR5QnVmZmVyLCBidWZmZXIpO1xyXG4gIH1cclxuXHJcbiAgU2V0Q29sb3JCdWZmZXIoYnVmZmVyOiBiMkNvbG9yW10gfCBGbG9hdDMyQXJyYXkpOiB2b2lkIHtcclxuICAgIGlmIChidWZmZXIgaW5zdGFuY2VvZiBGbG9hdDMyQXJyYXkpIHtcclxuICAgICAgaWYgKChidWZmZXIubGVuZ3RoICYgMykgIT09IDApIHtcclxuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgICAgfVxyXG4gICAgICBjb25zdCBjb3VudDogbnVtYmVyID0gYnVmZmVyLmxlbmd0aCAvIDQ7XHJcbiAgICAgIGNvbnN0IGFycmF5OiBiMkNvbG9yW10gPSBbXTtcclxuICAgICAgbGV0IHB0ciA9IDA7XHJcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgY291bnQ7ICsraSkge1xyXG4gICAgICAgIGFycmF5LnB1c2gobmV3IGIyQ29sb3IoYnVmZmVyW3B0cisrXSwgYnVmZmVyW3B0cisrXSwgYnVmZmVyW3B0cisrXSwgYnVmZmVyW3B0cisrXSkpO1xyXG4gICAgICB9XHJcbiAgICAgIGJ1ZmZlciA9IGFycmF5O1xyXG4gICAgfVxyXG4gICAgdGhpcy5TZXRVc2VyT3ZlcnJpZGFibGVCdWZmZXIodGhpcy5tX2NvbG9yQnVmZmVyLCBidWZmZXIpO1xyXG4gIH1cclxuXHJcbiAgU2V0VXNlckRhdGFCdWZmZXI8VD4oYnVmZmVyOiBUW10pOiB2b2lkIHtcclxuICAgIHRoaXMuU2V0VXNlck92ZXJyaWRhYmxlQnVmZmVyKHRoaXMubV91c2VyRGF0YUJ1ZmZlciwgYnVmZmVyKTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCBjb250YWN0cyBiZXR3ZWVuIHBhcnRpY2xlc1xyXG4gICAqIENvbnRhY3QgZGF0YSBjYW4gYmUgdXNlZCBmb3IgbWFueSByZWFzb25zLCBmb3IgZXhhbXBsZSB0b1xyXG4gICAqIHRyaWdnZXIgcmVuZGVyaW5nIG9yIGF1ZGlvIGVmZmVjdHMuXHJcbiAgICovXHJcbiAgR2V0Q29udGFjdHMoKTogYjJQYXJ0aWNsZUNvbnRhY3RbXSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YTtcclxuICB9XHJcblxyXG4gIEdldENvbnRhY3RDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50O1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IGNvbnRhY3RzIGJldHdlZW4gcGFydGljbGVzIGFuZCBib2RpZXNcclxuICAgKlxyXG4gICAqIENvbnRhY3QgZGF0YSBjYW4gYmUgdXNlZCBmb3IgbWFueSByZWFzb25zLCBmb3IgZXhhbXBsZSB0b1xyXG4gICAqIHRyaWdnZXIgcmVuZGVyaW5nIG9yIGF1ZGlvIGVmZmVjdHMuXHJcbiAgICovXHJcbiAgR2V0Qm9keUNvbnRhY3RzKCk6IGIyUGFydGljbGVCb2R5Q29udGFjdFtdIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuZGF0YTtcclxuICB9XHJcblxyXG4gIEdldEJvZHlDb250YWN0Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQ7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgYXJyYXkgb2YgcGFydGljbGUgcGFpcnMuIFRoZSBwYXJ0aWNsZXMgaW4gYSBwYWlyOlxyXG4gICAqICAgKDEpIGFyZSBjb250YWN0aW5nLFxyXG4gICAqICAgKDIpIGFyZSBpbiB0aGUgc2FtZSBwYXJ0aWNsZSBncm91cCxcclxuICAgKiAgICgzKSBhcmUgcGFydCBvZiBhIHJpZ2lkIHBhcnRpY2xlIGdyb3VwLCBvciBhcmUgc3ByaW5nLCBlbGFzdGljLFxyXG4gICAqICAgICAgIG9yIHdhbGwgcGFydGljbGVzLlxyXG4gICAqICAgKDQpIGhhdmUgYXQgbGVhc3Qgb25lIHBhcnRpY2xlIHRoYXQgaXMgYSBzcHJpbmcgb3IgYmFycmllclxyXG4gICAqICAgICAgIHBhcnRpY2xlIChpLmUuIG9uZSBvZiB0aGUgdHlwZXMgaW4ga19wYWlyRmxhZ3MpLFxyXG4gICAqICAgKDUpIGhhdmUgYXQgbGVhc3Qgb25lIHBhcnRpY2xlIHRoYXQgcmV0dXJucyB0cnVlIGZvclxyXG4gICAqICAgICAgIENvbm5lY3Rpb25GaWx0ZXI6OklzTmVjZXNzYXJ5LFxyXG4gICAqICAgKDYpIGFyZSBub3Qgem9tYmllIHBhcnRpY2xlcy5cclxuICAgKlxyXG4gICAqIEVzc2VudGlhbGx5LCB0aGlzIGlzIGFuIGFycmF5IG9mIHNwcmluZyBvciBiYXJyaWVyIHBhcnRpY2xlc1xyXG4gICAqIHRoYXQgYXJlIGludGVyYWN0aW5nLiBUaGUgYXJyYXkgaXMgc29ydGVkIGJ5IGIyUGFydGljbGVQYWlyJ3NcclxuICAgKiBpbmRleEEsIGFuZCB0aGVuIGluZGV4Qi4gVGhlcmUgYXJlIG5vIGR1cGxpY2F0ZSBlbnRyaWVzLlxyXG4gICAqL1xyXG4gIEdldFBhaXJzKCk6IGIyUGFydGljbGVQYWlyW10ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9wYWlyQnVmZmVyLmRhdGE7XHJcbiAgfVxyXG5cclxuICBHZXRQYWlyQ291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fcGFpckJ1ZmZlci5jb3VudDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCBhcnJheSBvZiBwYXJ0aWNsZSB0cmlhZHMuIFRoZSBwYXJ0aWNsZXMgaW4gYSB0cmlhZDpcclxuICAgKiAgICgxKSBhcmUgaW4gdGhlIHNhbWUgcGFydGljbGUgZ3JvdXAsXHJcbiAgICogICAoMikgYXJlIGluIGEgVm9yb25vaSB0cmlhbmdsZSB0b2dldGhlcixcclxuICAgKiAgICgzKSBhcmUgd2l0aGluIGIyX21heFRyaWFkRGlzdGFuY2UgcGFydGljbGUgZGlhbWV0ZXJzIG9mIGVhY2hcclxuICAgKiAgICAgICBvdGhlcixcclxuICAgKiAgICg0KSByZXR1cm4gdHJ1ZSBmb3IgQ29ubmVjdGlvbkZpbHRlcjo6U2hvdWxkQ3JlYXRlVHJpYWRcclxuICAgKiAgICg1KSBoYXZlIGF0IGxlYXN0IG9uZSBwYXJ0aWNsZSBvZiB0eXBlIGVsYXN0aWMgKGkuZS4gb25lIG9mIHRoZVxyXG4gICAqICAgICAgIHR5cGVzIGluIGtfdHJpYWRGbGFncyksXHJcbiAgICogICAoNikgYXJlIHBhcnQgb2YgYSByaWdpZCBwYXJ0aWNsZSBncm91cCwgb3IgYXJlIHNwcmluZywgZWxhc3RpYyxcclxuICAgKiAgICAgICBvciB3YWxsIHBhcnRpY2xlcy5cclxuICAgKiAgICg3KSBhcmUgbm90IHpvbWJpZSBwYXJ0aWNsZXMuXHJcbiAgICpcclxuICAgKiBFc3NlbnRpYWxseSwgdGhpcyBpcyBhbiBhcnJheSBvZiBlbGFzdGljIHBhcnRpY2xlcyB0aGF0IGFyZVxyXG4gICAqIGludGVyYWN0aW5nLiBUaGUgYXJyYXkgaXMgc29ydGVkIGJ5IGIyUGFydGljbGVUcmlhZCdzIGluZGV4QSxcclxuICAgKiB0aGVuIGluZGV4QiwgdGhlbiBpbmRleEMuIFRoZXJlIGFyZSBubyBkdXBsaWNhdGUgZW50cmllcy5cclxuICAgKi9cclxuICBHZXRUcmlhZHMoKTogYjJQYXJ0aWNsZVRyaWFkW10ge1xyXG4gICAgcmV0dXJuIHRoaXMubV90cmlhZEJ1ZmZlci5kYXRhO1xyXG4gIH1cclxuXHJcbiAgR2V0VHJpYWRDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV90cmlhZEJ1ZmZlci5jb3VudDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFNldCBhbiBvcHRpb25hbCB0aHJlc2hvbGQgZm9yIHRoZSBtYXhpbXVtIG51bWJlciBvZlxyXG4gICAqIGNvbnNlY3V0aXZlIHBhcnRpY2xlIGl0ZXJhdGlvbnMgdGhhdCBhIHBhcnRpY2xlIG1heSBjb250YWN0XHJcbiAgICogbXVsdGlwbGUgYm9kaWVzIGJlZm9yZSBpdCBpcyBjb25zaWRlcmVkIGEgY2FuZGlkYXRlIGZvciBiZWluZ1xyXG4gICAqIFwic3R1Y2tcIi4gU2V0dGluZyB0byB6ZXJvIG9yIGxlc3MgZGlzYWJsZXMuXHJcbiAgICovXHJcbiAgU2V0U3R1Y2tUaHJlc2hvbGQoc3RlcHM6IG51bWJlcik6IHZvaWQge1xyXG4gICAgdGhpcy5tX3N0dWNrVGhyZXNob2xkID0gc3RlcHM7XHJcblxyXG4gICAgaWYgKHN0ZXBzID4gMCkge1xyXG4gICAgICB0aGlzLm1fbGFzdEJvZHlDb250YWN0U3RlcEJ1ZmZlci5kYXRhID0gdGhpcy5SZXF1ZXN0QnVmZmVyKFxyXG4gICAgICAgIHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGEsXHJcbiAgICAgICk7XHJcbiAgICAgIHRoaXMubV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGEgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YSk7XHJcbiAgICAgIHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhID0gdGhpcy5SZXF1ZXN0QnVmZmVyKFxyXG4gICAgICAgIHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhLFxyXG4gICAgICApO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHBvdGVudGlhbGx5IHN0dWNrIHBhcnRpY2xlcyBmcm9tIHRoZSBsYXN0IHN0ZXA7IHRoZSB1c2VyXHJcbiAgICogbXVzdCBkZWNpZGUgaWYgdGhleSBhcmUgc3R1Y2sgb3Igbm90LCBhbmQgaWYgc28sIGRlbGV0ZSBvclxyXG4gICAqIG1vdmUgdGhlbVxyXG4gICAqL1xyXG4gIEdldFN0dWNrQ2FuZGlkYXRlcygpOiBudW1iZXJbXSB7XHJcbiAgICAvLy9yZXR1cm4gbV9zdHVja1BhcnRpY2xlQnVmZmVyLkRhdGEoKTtcclxuICAgIHJldHVybiB0aGlzLm1fc3R1Y2tQYXJ0aWNsZUJ1ZmZlci5EYXRhKCk7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIG51bWJlciBvZiBzdHVjayBwYXJ0aWNsZSBjYW5kaWRhdGVzIGZyb20gdGhlIGxhc3RcclxuICAgKiBzdGVwLlxyXG4gICAqL1xyXG4gIEdldFN0dWNrQ2FuZGlkYXRlQ291bnQoKTogbnVtYmVyIHtcclxuICAgIC8vL3JldHVybiBtX3N0dWNrUGFydGljbGVCdWZmZXIuR2V0Q291bnQoKTtcclxuICAgIHJldHVybiB0aGlzLm1fc3R1Y2tQYXJ0aWNsZUJ1ZmZlci5HZXRDb3VudCgpO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQ29tcHV0ZSB0aGUga2luZXRpYyBlbmVyZ3kgdGhhdCBjYW4gYmUgbG9zdCBieSBkYW1waW5nIGZvcmNlXHJcbiAgICovXHJcbiAgQ29tcHV0ZUNvbGxpc2lvbkVuZXJneSgpOiBudW1iZXIge1xyXG4gICAgY29uc3Qgc192ID0gYjJQYXJ0aWNsZVN5c3RlbS5Db21wdXRlQ29sbGlzaW9uRW5lcmd5X3NfdjtcclxuICAgIGNvbnN0IHZlbF9kYXRhID0gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgICBsZXQgc3VtX3YyID0gMDtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICBjb25zdCBiID0gY29udGFjdC5pbmRleEI7XHJcbiAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgLy8vYjJWZWMyIHYgPSBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYl0gLSBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYV07XHJcbiAgICAgIGNvbnN0IHYgPSBiMlZlYzIuU3ViVlYodmVsX2RhdGFbYl0sIHZlbF9kYXRhW2FdLCBzX3YpO1xyXG4gICAgICBjb25zdCB2biA9IGIyVmVjMi5Eb3RWVih2LCBuKTtcclxuICAgICAgaWYgKHZuIDwgMCkge1xyXG4gICAgICAgIHN1bV92MiArPSB2biAqIHZuO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gMC41ICogdGhpcy5HZXRQYXJ0aWNsZU1hc3MoKSAqIHN1bV92MjtcclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBDb21wdXRlQ29sbGlzaW9uRW5lcmd5X3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgLyoqXHJcbiAgICogU2V0IHN0cmljdCBQYXJ0aWNsZS9Cb2R5IGNvbnRhY3QgY2hlY2suXHJcbiAgICpcclxuICAgKiBUaGlzIGlzIGFuIG9wdGlvbiB0aGF0IHdpbGwgaGVscCBlbnN1cmUgY29ycmVjdCBiZWhhdmlvciBpZlxyXG4gICAqIHRoZXJlIGFyZSBjb3JuZXJzIGluIHRoZSB3b3JsZCBtb2RlbCB3aGVyZSBQYXJ0aWNsZS9Cb2R5XHJcbiAgICogY29udGFjdCBpcyBhbWJpZ3VvdXMuIFRoaXMgb3B0aW9uIHNjYWxlcyBhdCBuKmxvZyhuKSBvZiB0aGVcclxuICAgKiBudW1iZXIgb2YgUGFydGljbGUvQm9keSBjb250YWN0cywgc28gaXQgaXMgYmVzdCB0byBvbmx5XHJcbiAgICogZW5hYmxlIGlmIGl0IGlzIG5lY2Vzc2FyeSBmb3IgeW91ciBnZW9tZXRyeS4gRW5hYmxlIGlmIHlvdVxyXG4gICAqIHNlZSBzdHJhbmdlIHBhcnRpY2xlIGJlaGF2aW9yIGFyb3VuZCBiMkJvZHkgaW50ZXJzZWN0aW9ucy5cclxuICAgKi9cclxuICBTZXRTdHJpY3RDb250YWN0Q2hlY2soZW5hYmxlZDogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgdGhpcy5tX2RlZi5zdHJpY3RDb250YWN0Q2hlY2sgPSBlbmFibGVkO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBzdGF0dXMgb2YgdGhlIHN0cmljdCBjb250YWN0IGNoZWNrLlxyXG4gICAqL1xyXG4gIEdldFN0cmljdENvbnRhY3RDaGVjaygpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fZGVmLnN0cmljdENvbnRhY3RDaGVjaztcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFNldCB0aGUgbGlmZXRpbWUgKGluIHNlY29uZHMpIG9mIGEgcGFydGljbGUgcmVsYXRpdmUgdG8gdGhlXHJcbiAgICogY3VycmVudCB0aW1lLiAgQSBsaWZldGltZSBvZiBsZXNzIHRoYW4gb3IgZXF1YWwgdG8gMC4wZlxyXG4gICAqIHJlc3VsdHMgaW4gdGhlIHBhcnRpY2xlIGxpdmluZyBmb3JldmVyIHVudGlsIGl0J3MgbWFudWFsbHlcclxuICAgKiBkZXN0cm95ZWQgYnkgdGhlIGFwcGxpY2F0aW9uLlxyXG4gICAqL1xyXG4gIFNldFBhcnRpY2xlTGlmZXRpbWUoaW5kZXg6IG51bWJlciwgbGlmZXRpbWU6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzLlZhbGlkYXRlUGFydGljbGVJbmRleChpbmRleCkpO1xyXG4gICAgY29uc3QgaW5pdGlhbGl6ZUV4cGlyYXRpb25UaW1lcyA9IHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSA9PT0gbnVsbDtcclxuICAgIHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhID0gdGhpcy5SZXF1ZXN0QnVmZmVyKHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhKTtcclxuICAgIHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSA9IHRoaXMuUmVxdWVzdEJ1ZmZlcihcclxuICAgICAgdGhpcy5tX2luZGV4QnlFeHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhLFxyXG4gICAgKTtcclxuXHJcbiAgICAvLyBJbml0aWFsaXplIHRoZSBpbnZlcnNlIG1hcHBpbmcgYnVmZmVyLlxyXG4gICAgaWYgKGluaXRpYWxpemVFeHBpcmF0aW9uVGltZXMpIHtcclxuICAgICAgY29uc3QgcGFydGljbGVDb3VudCA9IHRoaXMuR2V0UGFydGljbGVDb3VudCgpO1xyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IHBhcnRpY2xlQ291bnQ7ICsraSkge1xyXG4gICAgICAgIHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtpXSA9IGk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIC8vL2NvbnN0IGludDMyIHF1YW50aXplZExpZmV0aW1lID0gKGludDMyKShsaWZldGltZSAvIG1fZGVmLmxpZmV0aW1lR3JhbnVsYXJpdHkpO1xyXG4gICAgY29uc3QgcXVhbnRpemVkTGlmZXRpbWUgPSBsaWZldGltZSAvIHRoaXMubV9kZWYubGlmZXRpbWVHcmFudWxhcml0eTtcclxuICAgIC8vIFVzZSBhIG5lZ2F0aXZlIGxpZmV0aW1lIHNvIHRoYXQgaXQncyBwb3NzaWJsZSB0byB0cmFjayB3aGljaFxyXG4gICAgLy8gb2YgdGhlIGluZmluaXRlIGxpZmV0aW1lIHBhcnRpY2xlcyBhcmUgb2xkZXIuXHJcbiAgICBjb25zdCBuZXdFeHBpcmF0aW9uVGltZSA9XHJcbiAgICAgIHF1YW50aXplZExpZmV0aW1lID4gMC4wXHJcbiAgICAgICAgPyB0aGlzLkdldFF1YW50aXplZFRpbWVFbGFwc2VkKCkgKyBxdWFudGl6ZWRMaWZldGltZVxyXG4gICAgICAgIDogcXVhbnRpemVkTGlmZXRpbWU7XHJcbiAgICBpZiAobmV3RXhwaXJhdGlvblRpbWUgIT09IHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhW2luZGV4XSkge1xyXG4gICAgICB0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtpbmRleF0gPSBuZXdFeHBpcmF0aW9uVGltZTtcclxuICAgICAgdGhpcy5tX2V4cGlyYXRpb25UaW1lQnVmZmVyUmVxdWlyZXNTb3J0aW5nID0gdHJ1ZTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgbGlmZXRpbWUgKGluIHNlY29uZHMpIG9mIGEgcGFydGljbGUgcmVsYXRpdmUgdG8gdGhlXHJcbiAgICogY3VycmVudCB0aW1lLiAgQSB2YWx1ZSA+IDAuMGYgaXMgcmV0dXJuZWQgaWYgdGhlIHBhcnRpY2xlIGlzXHJcbiAgICogc2NoZWR1bGVkIHRvIGJlIGRlc3Ryb3llZCBpbiB0aGUgZnV0dXJlLCB2YWx1ZXMgPD0gMC4wZlxyXG4gICAqIGluZGljYXRlIHRoZSBwYXJ0aWNsZSBoYXMgYW4gaW5maW5pdGUgbGlmZXRpbWUuXHJcbiAgICovXHJcbiAgR2V0UGFydGljbGVMaWZldGltZShpbmRleDogbnVtYmVyKTogbnVtYmVyIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5WYWxpZGF0ZVBhcnRpY2xlSW5kZXgoaW5kZXgpKTtcclxuICAgIHJldHVybiB0aGlzLkV4cGlyYXRpb25UaW1lVG9MaWZldGltZSh0aGlzLkdldEV4cGlyYXRpb25UaW1lQnVmZmVyKClbaW5kZXhdKTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEVuYWJsZSAvIGRpc2FibGUgZGVzdHJ1Y3Rpb24gb2YgcGFydGljbGVzIGluIENyZWF0ZVBhcnRpY2xlKClcclxuICAgKiB3aGVuIG5vIG1vcmUgcGFydGljbGVzIGNhbiBiZSBjcmVhdGVkIGR1ZSB0byBhIHByaW9yIGNhbGwgdG9cclxuICAgKiBTZXRNYXhQYXJ0aWNsZUNvdW50KCkuICBXaGVuIHRoaXMgaXMgZW5hYmxlZCwgdGhlIG9sZGVzdFxyXG4gICAqIHBhcnRpY2xlIGlzIGRlc3Ryb3llZCBpbiBDcmVhdGVQYXJ0aWNsZSgpIGZhdm9yaW5nIHRoZVxyXG4gICAqIGRlc3RydWN0aW9uIG9mIHBhcnRpY2xlcyB3aXRoIGEgZmluaXRlIGxpZmV0aW1lIG92ZXJcclxuICAgKiBwYXJ0aWNsZXMgd2l0aCBpbmZpbml0ZSBsaWZldGltZXMuIFRoaXMgZmVhdHVyZSBpcyBlbmFibGVkIGJ5XHJcbiAgICogZGVmYXVsdCB3aGVuIHBhcnRpY2xlIGxpZmV0aW1lcyBhcmUgdHJhY2tlZC4gIEV4cGxpY2l0bHlcclxuICAgKiBlbmFibGluZyB0aGlzIGZlYXR1cmUgdXNpbmcgdGhpcyBmdW5jdGlvbiBlbmFibGVzIHBhcnRpY2xlXHJcbiAgICogbGlmZXRpbWUgdHJhY2tpbmcuXHJcbiAgICovXHJcbiAgU2V0RGVzdHJ1Y3Rpb25CeUFnZShlbmFibGU6IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGlmIChlbmFibGUpIHtcclxuICAgICAgdGhpcy5HZXRFeHBpcmF0aW9uVGltZUJ1ZmZlcigpO1xyXG4gICAgfVxyXG4gICAgdGhpcy5tX2RlZi5kZXN0cm95QnlBZ2UgPSBlbmFibGU7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgd2hldGhlciB0aGUgb2xkZXN0IHBhcnRpY2xlIHdpbGwgYmUgZGVzdHJveWVkIGluXHJcbiAgICogQ3JlYXRlUGFydGljbGUoKSB3aGVuIHRoZSBtYXhpbXVtIG51bWJlciBvZiBwYXJ0aWNsZXMgYXJlXHJcbiAgICogcHJlc2VudCBpbiB0aGUgc3lzdGVtLlxyXG4gICAqL1xyXG4gIEdldERlc3RydWN0aW9uQnlBZ2UoKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2RlZi5kZXN0cm95QnlBZ2U7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIGFycmF5IG9mIHBhcnRpY2xlIGV4cGlyYXRpb24gdGltZXMgaW5kZXhlZCBieVxyXG4gICAqIHBhcnRpY2xlIGluZGV4LlxyXG4gICAqXHJcbiAgICogR2V0UGFydGljbGVDb3VudCgpIGl0ZW1zIGFyZSBpbiB0aGUgcmV0dXJuZWQgYXJyYXkuXHJcbiAgICovXHJcbiAgR2V0RXhwaXJhdGlvblRpbWVCdWZmZXIoKTogbnVtYmVyW10ge1xyXG4gICAgdGhpcy5tX2V4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX2V4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEpO1xyXG4gICAgcmV0dXJuIHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQ29udmVydCBhIGV4cGlyYXRpb24gdGltZSB2YWx1ZSBpbiByZXR1cm5lZCBieVxyXG4gICAqIEdldEV4cGlyYXRpb25UaW1lQnVmZmVyKCkgdG8gYSB0aW1lIGluIHNlY29uZHMgcmVsYXRpdmUgdG9cclxuICAgKiB0aGUgY3VycmVudCBzaW11bGF0aW9uIHRpbWUuXHJcbiAgICovXHJcbiAgRXhwaXJhdGlvblRpbWVUb0xpZmV0aW1lKGV4cGlyYXRpb25UaW1lOiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIChcclxuICAgICAgKGV4cGlyYXRpb25UaW1lID4gMCA/IGV4cGlyYXRpb25UaW1lIC0gdGhpcy5HZXRRdWFudGl6ZWRUaW1lRWxhcHNlZCgpIDogZXhwaXJhdGlvblRpbWUpICpcclxuICAgICAgdGhpcy5tX2RlZi5saWZldGltZUdyYW51bGFyaXR5XHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSBhcnJheSBvZiBwYXJ0aWNsZSBpbmRpY2VzIG9yZGVyZWQgYnkgcmV2ZXJzZVxyXG4gICAqIGxpZmV0aW1lLiBUaGUgb2xkZXN0IHBhcnRpY2xlIGluZGV4ZXMgYXJlIGF0IHRoZSBlbmQgb2YgdGhlXHJcbiAgICogYXJyYXkgd2l0aCB0aGUgbmV3ZXN0IGF0IHRoZSBzdGFydC4gIFBhcnRpY2xlcyB3aXRoIGluZmluaXRlXHJcbiAgICogbGlmZXRpbWVzIChpLmUgZXhwaXJhdGlvbiB0aW1lcyBsZXNzIHRoYW4gb3IgZXF1YWwgdG8gMCkgYXJlXHJcbiAgICogcGxhY2VkIGF0IHRoZSBzdGFydCBvZiB0aGUgYXJyYXkuXHJcbiAgICogRXhwaXJhdGlvblRpbWVUb0xpZmV0aW1lKEdldEV4cGlyYXRpb25UaW1lQnVmZmVyKClbaW5kZXhdKSBpc1xyXG4gICAqIGVxdWl2YWxlbnQgdG8gR2V0UGFydGljbGVMaWZldGltZShpbmRleCkuXHJcbiAgICpcclxuICAgKiBHZXRQYXJ0aWNsZUNvdW50KCkgaXRlbXMgYXJlIGluIHRoZSByZXR1cm5lZCBhcnJheS5cclxuICAgKi9cclxuICBHZXRJbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIoKTogbnVtYmVyW10ge1xyXG4gICAgLy8gSWYgcGFydGljbGVzIGFyZSBwcmVzZW50LCBpbml0aWFsaXplIC8gcmVpbml0aWFsaXplIHRoZSBsaWZldGltZSBidWZmZXIuXHJcbiAgICBpZiAodGhpcy5HZXRQYXJ0aWNsZUNvdW50KCkpIHtcclxuICAgICAgdGhpcy5TZXRQYXJ0aWNsZUxpZmV0aW1lKDAsIHRoaXMuR2V0UGFydGljbGVMaWZldGltZSgwKSk7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1faW5kZXhCeUV4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEgPSB0aGlzLlJlcXVlc3RCdWZmZXIoXHJcbiAgICAgICAgdGhpcy5tX2luZGV4QnlFeHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhLFxyXG4gICAgICApO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEFwcGx5IGFuIGltcHVsc2UgdG8gb25lIHBhcnRpY2xlLiBUaGlzIGltbWVkaWF0ZWx5IG1vZGlmaWVzXHJcbiAgICogdGhlIHZlbG9jaXR5LiBTaW1pbGFyIHRvIGIyQm9keTo6QXBwbHlMaW5lYXJJbXB1bHNlLlxyXG4gICAqXHJcbiAgICogQHBhcmFtIGluZGV4IHRoZSBwYXJ0aWNsZSB0aGF0IHdpbGwgYmUgbW9kaWZpZWQuXHJcbiAgICogQHBhcmFtIGltcHVsc2UgaW1wdWxzZSB0aGUgd29ybGQgaW1wdWxzZSB2ZWN0b3IsIHVzdWFsbHkgaW4gTi1zZWNvbmRzIG9yIGtnLW0vcy5cclxuICAgKi9cclxuICBQYXJ0aWNsZUFwcGx5TGluZWFySW1wdWxzZShpbmRleDogbnVtYmVyLCBpbXB1bHNlOiBYWSk6IHZvaWQge1xyXG4gICAgdGhpcy5BcHBseUxpbmVhckltcHVsc2UoaW5kZXgsIGluZGV4ICsgMSwgaW1wdWxzZSk7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBBcHBseSBhbiBpbXB1bHNlIHRvIGFsbCBwYXJ0aWNsZXMgYmV0d2VlbiAnZmlyc3RJbmRleCcgYW5kXHJcbiAgICogJ2xhc3RJbmRleCcuIFRoaXMgaW1tZWRpYXRlbHkgbW9kaWZpZXMgdGhlIHZlbG9jaXR5LiBOb3RlXHJcbiAgICogdGhhdCB0aGUgaW1wdWxzZSBpcyBhcHBsaWVkIHRvIHRoZSB0b3RhbCBtYXNzIG9mIGFsbFxyXG4gICAqIHBhcnRpY2xlcy4gU28sIGNhbGxpbmcgUGFydGljbGVBcHBseUxpbmVhckltcHVsc2UoMCwgaW1wdWxzZSlcclxuICAgKiBhbmQgUGFydGljbGVBcHBseUxpbmVhckltcHVsc2UoMSwgaW1wdWxzZSkgd2lsbCBpbXBhcnQgdHdpY2VcclxuICAgKiBhcyBtdWNoIHZlbG9jaXR5IGFzIGNhbGxpbmcganVzdCBBcHBseUxpbmVhckltcHVsc2UoMCwgMSxcclxuICAgKiBpbXB1bHNlKS5cclxuICAgKlxyXG4gICAqIEBwYXJhbSBmaXJzdEluZGV4IHRoZSBmaXJzdCBwYXJ0aWNsZSB0byBiZSBtb2RpZmllZC5cclxuICAgKiBAcGFyYW0gbGFzdEluZGV4IHRoZSBsYXN0IHBhcnRpY2xlIHRvIGJlIG1vZGlmaWVkLlxyXG4gICAqIEBwYXJhbSBpbXB1bHNlIHRoZSB3b3JsZCBpbXB1bHNlIHZlY3RvciwgdXN1YWxseSBpbiBOLXNlY29uZHMgb3Iga2ctbS9zLlxyXG4gICAqL1xyXG4gIEFwcGx5TGluZWFySW1wdWxzZShmaXJzdEluZGV4OiBudW1iZXIsIGxhc3RJbmRleDogbnVtYmVyLCBpbXB1bHNlOiBYWSk6IHZvaWQge1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IG51bVBhcnRpY2xlcyA9IGxhc3RJbmRleCAtIGZpcnN0SW5kZXg7XHJcbiAgICBjb25zdCB0b3RhbE1hc3MgPSBudW1QYXJ0aWNsZXMgKiB0aGlzLkdldFBhcnRpY2xlTWFzcygpO1xyXG4gICAgLy8vY29uc3QgYjJWZWMyIHZlbG9jaXR5RGVsdGEgPSBpbXB1bHNlIC8gdG90YWxNYXNzO1xyXG4gICAgY29uc3QgdmVsb2NpdHlEZWx0YSA9IG5ldyBiMlZlYzIoKS5Db3B5KGltcHVsc2UpLlNlbGZNdWwoMSAvIHRvdGFsTWFzcyk7XHJcbiAgICBmb3IgKGxldCBpID0gZmlyc3RJbmRleDsgaSA8IGxhc3RJbmRleDsgaSsrKSB7XHJcbiAgICAgIC8vL21fdmVsb2NpdHlCdWZmZXIuZGF0YVtpXSArPSB2ZWxvY2l0eURlbHRhO1xyXG4gICAgICB2ZWxfZGF0YVtpXS5TZWxmQWRkKHZlbG9jaXR5RGVsdGEpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIElzU2lnbmlmaWNhbnRGb3JjZShmb3JjZTogWFkpOiBib29sZWFuIHtcclxuICAgIHJldHVybiBmb3JjZS54ICE9PSAwIHx8IGZvcmNlLnkgIT09IDA7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBBcHBseSBhIGZvcmNlIHRvIHRoZSBjZW50ZXIgb2YgYSBwYXJ0aWNsZS5cclxuICAgKlxyXG4gICAqIEBwYXJhbSBpbmRleCB0aGUgcGFydGljbGUgdGhhdCB3aWxsIGJlIG1vZGlmaWVkLlxyXG4gICAqIEBwYXJhbSBmb3JjZSB0aGUgd29ybGQgZm9yY2UgdmVjdG9yLCB1c3VhbGx5IGluIE5ld3RvbnMgKE4pLlxyXG4gICAqL1xyXG4gIFBhcnRpY2xlQXBwbHlGb3JjZShpbmRleDogbnVtYmVyLCBmb3JjZTogWFkpOiB2b2lkIHtcclxuICAgIGlmIChcclxuICAgICAgYjJQYXJ0aWNsZVN5c3RlbS5Jc1NpZ25pZmljYW50Rm9yY2UoZm9yY2UpICYmXHJcbiAgICAgIHRoaXMuRm9yY2VDYW5CZUFwcGxpZWQodGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbaW5kZXhdKVxyXG4gICAgKSB7XHJcbiAgICAgIHRoaXMuUHJlcGFyZUZvcmNlQnVmZmVyKCk7XHJcbiAgICAgIC8vL21fZm9yY2VCdWZmZXJbaW5kZXhdICs9IGZvcmNlO1xyXG4gICAgICB0aGlzLm1fZm9yY2VCdWZmZXJbaW5kZXhdLlNlbGZBZGQoZm9yY2UpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogRGlzdHJpYnV0ZSBhIGZvcmNlIGFjcm9zcyBzZXZlcmFsIHBhcnRpY2xlcy4gVGhlIHBhcnRpY2xlc1xyXG4gICAqIG11c3Qgbm90IGJlIHdhbGwgcGFydGljbGVzLiBOb3RlIHRoYXQgdGhlIGZvcmNlIGlzXHJcbiAgICogZGlzdHJpYnV0ZWQgYWNyb3NzIGFsbCB0aGUgcGFydGljbGVzLCBzbyBjYWxsaW5nIHRoaXNcclxuICAgKiBmdW5jdGlvbiBmb3IgaW5kaWNlcyAwLi5OIGlzIG5vdCB0aGUgc2FtZSBhcyBjYWxsaW5nXHJcbiAgICogUGFydGljbGVBcHBseUZvcmNlKGksIGZvcmNlKSBmb3IgaSBpbiAwLi5OLlxyXG4gICAqXHJcbiAgICogQHBhcmFtIGZpcnN0SW5kZXggdGhlIGZpcnN0IHBhcnRpY2xlIHRvIGJlIG1vZGlmaWVkLlxyXG4gICAqIEBwYXJhbSBsYXN0SW5kZXggdGhlIGxhc3QgcGFydGljbGUgdG8gYmUgbW9kaWZpZWQuXHJcbiAgICogQHBhcmFtIGZvcmNlIHRoZSB3b3JsZCBmb3JjZSB2ZWN0b3IsIHVzdWFsbHkgaW4gTmV3dG9ucyAoTikuXHJcbiAgICovXHJcbiAgQXBwbHlGb3JjZShmaXJzdEluZGV4OiBudW1iZXIsIGxhc3RJbmRleDogbnVtYmVyLCBmb3JjZTogWFkpOiB2b2lkIHtcclxuICAgIC8vIEVuc3VyZSB3ZSdyZSBub3QgdHJ5aW5nIHRvIGFwcGx5IGZvcmNlIHRvIHBhcnRpY2xlcyB0aGF0IGNhbid0IG1vdmUsXHJcbiAgICAvLyBzdWNoIGFzIHdhbGwgcGFydGljbGVzLlxyXG4gICAgaWYgKEIyX0RFQlVHKSB7XHJcbiAgICAgIGxldCBmbGFncyA9IDA7XHJcbiAgICAgIGZvciAobGV0IGkgPSBmaXJzdEluZGV4OyBpIDwgbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgICBmbGFncyB8PSB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpXTtcclxuICAgICAgfVxyXG4gICAgICBiMkFzc2VydCh0aGlzLkZvcmNlQ2FuQmVBcHBsaWVkKGZsYWdzKSk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gRWFybHkgb3V0IGlmIGZvcmNlIGRvZXMgbm90aGluZyAob3B0aW1pemF0aW9uKS5cclxuICAgIC8vL2NvbnN0IGIyVmVjMiBkaXN0cmlidXRlZEZvcmNlID0gZm9yY2UgLyAoZmxvYXQzMikobGFzdEluZGV4IC0gZmlyc3RJbmRleCk7XHJcbiAgICBjb25zdCBkaXN0cmlidXRlZEZvcmNlID0gbmV3IGIyVmVjMigpLkNvcHkoZm9yY2UpLlNlbGZNdWwoMSAvIChsYXN0SW5kZXggLSBmaXJzdEluZGV4KSk7XHJcbiAgICBpZiAoYjJQYXJ0aWNsZVN5c3RlbS5Jc1NpZ25pZmljYW50Rm9yY2UoZGlzdHJpYnV0ZWRGb3JjZSkpIHtcclxuICAgICAgdGhpcy5QcmVwYXJlRm9yY2VCdWZmZXIoKTtcclxuXHJcbiAgICAgIC8vIERpc3RyaWJ1dGUgdGhlIGZvcmNlIG92ZXIgYWxsIHRoZSBwYXJ0aWNsZXMuXHJcbiAgICAgIGZvciAobGV0IGkgPSBmaXJzdEluZGV4OyBpIDwgbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgICAvLy9tX2ZvcmNlQnVmZmVyW2ldICs9IGRpc3RyaWJ1dGVkRm9yY2U7XHJcbiAgICAgICAgdGhpcy5tX2ZvcmNlQnVmZmVyW2ldLlNlbGZBZGQoZGlzdHJpYnV0ZWRGb3JjZSk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgbmV4dCBwYXJ0aWNsZS1zeXN0ZW0gaW4gdGhlIHdvcmxkJ3MgcGFydGljbGUtc3lzdGVtXHJcbiAgICogbGlzdC5cclxuICAgKi9cclxuICBHZXROZXh0KCk6IGIyUGFydGljbGVTeXN0ZW0gfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fbmV4dDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFF1ZXJ5IHRoZSBwYXJ0aWNsZSBzeXN0ZW0gZm9yIGFsbCBwYXJ0aWNsZXMgdGhhdCBwb3RlbnRpYWxseVxyXG4gICAqIG92ZXJsYXAgdGhlIHByb3ZpZGVkIEFBQkIuXHJcbiAgICogYjJRdWVyeUNhbGxiYWNrOjpTaG91bGRRdWVyeVBhcnRpY2xlU3lzdGVtIGlzIGlnbm9yZWQuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gY2FsbGJhY2sgYSB1c2VyIGltcGxlbWVudGVkIGNhbGxiYWNrIGNsYXNzLlxyXG4gICAqIEBwYXJhbSBhYWJiIHRoZSBxdWVyeSBib3guXHJcbiAgICovXHJcbiAgUXVlcnlBQUJCKGNhbGxiYWNrOiBiMlF1ZXJ5Q2FsbGJhY2ssIGFhYmI6IGIyQUFCQik6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV9wcm94eUJ1ZmZlci5jb3VudCA9PT0gMCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcbiAgICBjb25zdCBiZWdpblByb3h5ID0gMDtcclxuICAgIGNvbnN0IGVuZFByb3h5ID0gdGhpcy5tX3Byb3h5QnVmZmVyLmNvdW50O1xyXG4gICAgY29uc3QgZmlyc3RQcm94eSA9IHN0ZF9sb3dlcl9ib3VuZChcclxuICAgICAgdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGEsXHJcbiAgICAgIGJlZ2luUHJveHksXHJcbiAgICAgIGVuZFByb3h5LFxyXG4gICAgICBiMlBhcnRpY2xlU3lzdGVtLmNvbXB1dGVUYWcoXHJcbiAgICAgICAgdGhpcy5tX2ludmVyc2VEaWFtZXRlciAqIGFhYmIubG93ZXJCb3VuZC54LFxyXG4gICAgICAgIHRoaXMubV9pbnZlcnNlRGlhbWV0ZXIgKiBhYWJiLmxvd2VyQm91bmQueSxcclxuICAgICAgKSxcclxuICAgICAgYjJQYXJ0aWNsZVN5c3RlbV9Qcm94eS5Db21wYXJlUHJveHlUYWcsXHJcbiAgICApO1xyXG4gICAgY29uc3QgbGFzdFByb3h5ID0gc3RkX3VwcGVyX2JvdW5kKFxyXG4gICAgICB0aGlzLm1fcHJveHlCdWZmZXIuZGF0YSxcclxuICAgICAgZmlyc3RQcm94eSxcclxuICAgICAgZW5kUHJveHksXHJcbiAgICAgIGIyUGFydGljbGVTeXN0ZW0uY29tcHV0ZVRhZyhcclxuICAgICAgICB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogYWFiYi51cHBlckJvdW5kLngsXHJcbiAgICAgICAgdGhpcy5tX2ludmVyc2VEaWFtZXRlciAqIGFhYmIudXBwZXJCb3VuZC55LFxyXG4gICAgICApLFxyXG4gICAgICBiMlBhcnRpY2xlU3lzdGVtX1Byb3h5LkNvbXBhcmVUYWdQcm94eSxcclxuICAgICk7XHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgZm9yIChsZXQgayA9IGZpcnN0UHJveHk7IGsgPCBsYXN0UHJveHk7ICsraykge1xyXG4gICAgICBjb25zdCBwcm94eSA9IHRoaXMubV9wcm94eUJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb25zdCBpID0gcHJveHkuaW5kZXg7XHJcbiAgICAgIGNvbnN0IHAgPSBwb3NfZGF0YVtpXTtcclxuICAgICAgaWYgKFxyXG4gICAgICAgIGFhYmIubG93ZXJCb3VuZC54IDwgcC54ICYmXHJcbiAgICAgICAgcC54IDwgYWFiYi51cHBlckJvdW5kLnggJiZcclxuICAgICAgICBhYWJiLmxvd2VyQm91bmQueSA8IHAueSAmJlxyXG4gICAgICAgIHAueSA8IGFhYmIudXBwZXJCb3VuZC55XHJcbiAgICAgICkge1xyXG4gICAgICAgIGlmICghY2FsbGJhY2suUmVwb3J0UGFydGljbGUodGhpcywgaSkpIHtcclxuICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogUXVlcnkgdGhlIHBhcnRpY2xlIHN5c3RlbSBmb3IgYWxsIHBhcnRpY2xlcyB0aGF0IHBvdGVudGlhbGx5XHJcbiAgICogb3ZlcmxhcCB0aGUgcHJvdmlkZWQgc2hhcGUncyBBQUJCLiBDYWxscyBRdWVyeUFBQkJcclxuICAgKiBpbnRlcm5hbGx5LiBiMlF1ZXJ5Q2FsbGJhY2s6OlNob3VsZFF1ZXJ5UGFydGljbGVTeXN0ZW0gaXNcclxuICAgKiBpZ25vcmVkLlxyXG4gICAqXHJcbiAgICogQHBhcmFtIGNhbGxiYWNrIGEgdXNlciBpbXBsZW1lbnRlZCBjYWxsYmFjayBjbGFzcy5cclxuICAgKiBAcGFyYW0gc2hhcGUgdGhlIHF1ZXJ5IHNoYXBlXHJcbiAgICogQHBhcmFtIHhmIHRoZSB0cmFuc2Zvcm0gb2YgdGhlIEFBQkJcclxuICAgKiBAcGFyYW0gY2hpbGRJbmRleFxyXG4gICAqL1xyXG4gIFF1ZXJ5U2hhcGVBQUJCKGNhbGxiYWNrOiBiMlF1ZXJ5Q2FsbGJhY2ssIHNoYXBlOiBiMlNoYXBlLCB4ZjogYjJUcmFuc2Zvcm0sIGNoaWxkSW5kZXggPSAwKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX2FhYmIgPSBiMlBhcnRpY2xlU3lzdGVtLlF1ZXJ5U2hhcGVBQUJCX3NfYWFiYjtcclxuICAgIGNvbnN0IGFhYmIgPSBzX2FhYmI7XHJcbiAgICBzaGFwZS5Db21wdXRlQUFCQihhYWJiLCB4ZiwgY2hpbGRJbmRleCk7XHJcbiAgICB0aGlzLlF1ZXJ5QUFCQihjYWxsYmFjaywgYWFiYik7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgUXVlcnlTaGFwZUFBQkJfc19hYWJiID0gbmV3IGIyQUFCQigpO1xyXG5cclxuICBRdWVyeVBvaW50QUFCQihjYWxsYmFjazogYjJRdWVyeUNhbGxiYWNrLCBwb2ludDogWFksIHNsb3A6IG51bWJlciA9IGIyX2xpbmVhclNsb3ApOiB2b2lkIHtcclxuICAgIGNvbnN0IHNfYWFiYiA9IGIyUGFydGljbGVTeXN0ZW0uUXVlcnlQb2ludEFBQkJfc19hYWJiO1xyXG4gICAgY29uc3QgYWFiYiA9IHNfYWFiYjtcclxuICAgIGFhYmIubG93ZXJCb3VuZC5TZXQocG9pbnQueCAtIHNsb3AsIHBvaW50LnkgLSBzbG9wKTtcclxuICAgIGFhYmIudXBwZXJCb3VuZC5TZXQocG9pbnQueCArIHNsb3AsIHBvaW50LnkgKyBzbG9wKTtcclxuICAgIHRoaXMuUXVlcnlBQUJCKGNhbGxiYWNrLCBhYWJiKTtcclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBRdWVyeVBvaW50QUFCQl9zX2FhYmIgPSBuZXcgYjJBQUJCKCk7XHJcblxyXG4gIC8qKlxyXG4gICAqIFJheS1jYXN0IHRoZSBwYXJ0aWNsZSBzeXN0ZW0gZm9yIGFsbCBwYXJ0aWNsZXMgaW4gdGhlIHBhdGggb2ZcclxuICAgKiB0aGUgcmF5LiBZb3VyIGNhbGxiYWNrIGNvbnRyb2xzIHdoZXRoZXIgeW91IGdldCB0aGUgY2xvc2VzdFxyXG4gICAqIHBvaW50LCBhbnkgcG9pbnQsIG9yIG4tcG9pbnRzLiBUaGUgcmF5LWNhc3QgaWdub3JlcyBwYXJ0aWNsZXNcclxuICAgKiB0aGF0IGNvbnRhaW4gdGhlIHN0YXJ0aW5nIHBvaW50LlxyXG4gICAqIGIyUmF5Q2FzdENhbGxiYWNrOjpTaG91bGRRdWVyeVBhcnRpY2xlU3lzdGVtIGlzIGlnbm9yZWQuXHJcbiAgICpcclxuICAgKiBAcGFyYW0gY2FsbGJhY2sgYSB1c2VyIGltcGxlbWVudGVkIGNhbGxiYWNrIGNsYXNzLlxyXG4gICAqIEBwYXJhbSBwb2ludDEgdGhlIHJheSBzdGFydGluZyBwb2ludFxyXG4gICAqIEBwYXJhbSBwb2ludDIgdGhlIHJheSBlbmRpbmcgcG9pbnRcclxuICAgKi9cclxuICBSYXlDYXN0KGNhbGxiYWNrOiBiMlJheUNhc3RDYWxsYmFjaywgcG9pbnQxOiBYWSwgcG9pbnQyOiBYWSk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19hYWJiID0gYjJQYXJ0aWNsZVN5c3RlbS5SYXlDYXN0X3NfYWFiYjtcclxuICAgIGNvbnN0IHNfcCA9IGIyUGFydGljbGVTeXN0ZW0uUmF5Q2FzdF9zX3A7XHJcbiAgICBjb25zdCBzX3YgPSBiMlBhcnRpY2xlU3lzdGVtLlJheUNhc3Rfc192O1xyXG4gICAgY29uc3Qgc19uID0gYjJQYXJ0aWNsZVN5c3RlbS5SYXlDYXN0X3NfbjtcclxuICAgIGNvbnN0IHNfcG9pbnQgPSBiMlBhcnRpY2xlU3lzdGVtLlJheUNhc3Rfc19wb2ludDtcclxuICAgIGlmICh0aGlzLm1fcHJveHlCdWZmZXIuY291bnQgPT09IDApIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IGFhYmIgPSBzX2FhYmI7XHJcbiAgICBiMlZlYzIuTWluVihwb2ludDEsIHBvaW50MiwgYWFiYi5sb3dlckJvdW5kKTtcclxuICAgIGIyVmVjMi5NYXhWKHBvaW50MSwgcG9pbnQyLCBhYWJiLnVwcGVyQm91bmQpO1xyXG4gICAgbGV0IGZyYWN0aW9uID0gMTtcclxuICAgIC8vIHNvbHZpbmcgdGhlIGZvbGxvd2luZyBlcXVhdGlvbjpcclxuICAgIC8vICgoMS10KSpwb2ludDErdCpwb2ludDItcG9zaXRpb24pXjI9ZGlhbWV0ZXJeMlxyXG4gICAgLy8gd2hlcmUgdCBpcyBhIHBvdGVudGlhbCBmcmFjdGlvblxyXG4gICAgLy8vYjJWZWMyIHYgPSBwb2ludDIgLSBwb2ludDE7XHJcbiAgICBjb25zdCB2ID0gYjJWZWMyLlN1YlZWKHBvaW50MiwgcG9pbnQxLCBzX3YpO1xyXG4gICAgY29uc3QgdjIgPSBiMlZlYzIuRG90VlYodiwgdik7XHJcbiAgICBjb25zdCBlbnVtZXJhdG9yID0gdGhpcy5HZXRJbnNpZGVCb3VuZHNFbnVtZXJhdG9yKGFhYmIpO1xyXG5cclxuICAgIGxldCBpOiBudW1iZXI7XHJcbiAgICB3aGlsZSAoKGkgPSBlbnVtZXJhdG9yLkdldE5leHQoKSkgPj0gMCkge1xyXG4gICAgICAvLy9iMlZlYzIgcCA9IHBvaW50MSAtIG1fcG9zaXRpb25CdWZmZXIuZGF0YVtpXTtcclxuICAgICAgY29uc3QgcCA9IGIyVmVjMi5TdWJWVihwb2ludDEsIHBvc19kYXRhW2ldLCBzX3ApO1xyXG4gICAgICBjb25zdCBwdiA9IGIyVmVjMi5Eb3RWVihwLCB2KTtcclxuICAgICAgY29uc3QgcDIgPSBiMlZlYzIuRG90VlYocCwgcCk7XHJcbiAgICAgIGNvbnN0IGRldGVybWluYW50ID0gcHYgKiBwdiAtIHYyICogKHAyIC0gdGhpcy5tX3NxdWFyZWREaWFtZXRlcik7XHJcbiAgICAgIGlmIChkZXRlcm1pbmFudCA+PSAwKSB7XHJcbiAgICAgICAgY29uc3Qgc3FydERldGVybWluYW50ID0gYjJTcXJ0KGRldGVybWluYW50KTtcclxuICAgICAgICAvLyBmaW5kIGEgc29sdXRpb24gYmV0d2VlbiAwIGFuZCBmcmFjdGlvblxyXG4gICAgICAgIGxldCB0ID0gKC1wdiAtIHNxcnREZXRlcm1pbmFudCkgLyB2MjtcclxuICAgICAgICBpZiAodCA+IGZyYWN0aW9uKSB7XHJcbiAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICB9XHJcbiAgICAgICAgaWYgKHQgPCAwKSB7XHJcbiAgICAgICAgICB0ID0gKC1wdiArIHNxcnREZXRlcm1pbmFudCkgLyB2MjtcclxuICAgICAgICAgIGlmICh0IDwgMCB8fCB0ID4gZnJhY3Rpb24pIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIC8vL2IyVmVjMiBuID0gcCArIHQgKiB2O1xyXG4gICAgICAgIGNvbnN0IG4gPSBiMlZlYzIuQWRkVk11bFNWKHAsIHQsIHYsIHNfbik7XHJcbiAgICAgICAgbi5Ob3JtYWxpemUoKTtcclxuICAgICAgICAvLy9mbG9hdDMyIGYgPSBjYWxsYmFjay5SZXBvcnRQYXJ0aWNsZSh0aGlzLCBpLCBwb2ludDEgKyB0ICogdiwgbiwgdCk7XHJcbiAgICAgICAgY29uc3QgZiA9IGNhbGxiYWNrLlJlcG9ydFBhcnRpY2xlKHRoaXMsIGksIGIyVmVjMi5BZGRWTXVsU1YocG9pbnQxLCB0LCB2LCBzX3BvaW50KSwgbiwgdCk7XHJcbiAgICAgICAgZnJhY3Rpb24gPSBiMk1pbihmcmFjdGlvbiwgZik7XHJcbiAgICAgICAgaWYgKGZyYWN0aW9uIDw9IDApIHtcclxuICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFJheUNhc3Rfc19hYWJiID0gbmV3IGIyQUFCQigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBSYXlDYXN0X3NfcCA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgUmF5Q2FzdF9zX3YgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFJheUNhc3Rfc19uID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBSYXlDYXN0X3NfcG9pbnQgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIC8qKlxyXG4gICAqIENvbXB1dGUgdGhlIGF4aXMtYWxpZ25lZCBib3VuZGluZyBib3ggZm9yIGFsbCBwYXJ0aWNsZXNcclxuICAgKiBjb250YWluZWQgd2l0aGluIHRoaXMgcGFydGljbGUgc3lzdGVtLlxyXG4gICAqIEBwYXJhbSBhYWJiIFJldHVybnMgdGhlIGF4aXMtYWxpZ25lZCBib3VuZGluZyBib3ggb2YgdGhlIHN5c3RlbS5cclxuICAgKi9cclxuICBDb21wdXRlQUFCQihhYWJiOiBiMkFBQkIpOiB2b2lkIHtcclxuICAgIGNvbnN0IHBhcnRpY2xlQ291bnQgPSB0aGlzLkdldFBhcnRpY2xlQ291bnQoKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYWFiYiAhPT0gbnVsbCk7XHJcbiAgICBhYWJiLmxvd2VyQm91bmQueCA9ICtiMl9tYXhGbG9hdDtcclxuICAgIGFhYmIubG93ZXJCb3VuZC55ID0gK2IyX21heEZsb2F0O1xyXG4gICAgYWFiYi51cHBlckJvdW5kLnggPSAtYjJfbWF4RmxvYXQ7XHJcbiAgICBhYWJiLnVwcGVyQm91bmQueSA9IC1iMl9tYXhGbG9hdDtcclxuXHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBwYXJ0aWNsZUNvdW50OyBpKyspIHtcclxuICAgICAgY29uc3QgcCA9IHBvc19kYXRhW2ldO1xyXG4gICAgICBiMlZlYzIuTWluVihhYWJiLmxvd2VyQm91bmQsIHAsIGFhYmIubG93ZXJCb3VuZCk7XHJcbiAgICAgIGIyVmVjMi5NYXhWKGFhYmIudXBwZXJCb3VuZCwgcCwgYWFiYi51cHBlckJvdW5kKTtcclxuICAgIH1cclxuICAgIGFhYmIubG93ZXJCb3VuZC54IC09IHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyO1xyXG4gICAgYWFiYi5sb3dlckJvdW5kLnkgLT0gdGhpcy5tX3BhcnRpY2xlRGlhbWV0ZXI7XHJcbiAgICBhYWJiLnVwcGVyQm91bmQueCArPSB0aGlzLm1fcGFydGljbGVEaWFtZXRlcjtcclxuICAgIGFhYmIudXBwZXJCb3VuZC55ICs9IHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQWxsIHBhcnRpY2xlIHR5cGVzIHRoYXQgcmVxdWlyZSBjcmVhdGluZyBwYWlyc1xyXG4gICAqL1xyXG4gIHN0YXRpYyByZWFkb25seSBrX3BhaXJGbGFnczogbnVtYmVyID0gYjJQYXJ0aWNsZUZsYWcuYjJfc3ByaW5nUGFydGljbGU7XHJcblxyXG4gIC8qKlxyXG4gICAqIEFsbCBwYXJ0aWNsZSB0eXBlcyB0aGF0IHJlcXVpcmUgY3JlYXRpbmcgdHJpYWRzXHJcbiAgICovXHJcbiAgc3RhdGljIHJlYWRvbmx5IGtfdHJpYWRGbGFncyA9IGIyUGFydGljbGVGbGFnLmIyX2VsYXN0aWNQYXJ0aWNsZTtcclxuXHJcbiAgLyoqXHJcbiAgICogQWxsIHBhcnRpY2xlIHR5cGVzIHRoYXQgZG8gbm90IHByb2R1Y2UgZHluYW1pYyBwcmVzc3VyZVxyXG4gICAqL1xyXG4gIHN0YXRpYyByZWFkb25seSBrX25vUHJlc3N1cmVGbGFncyA9XHJcbiAgICBiMlBhcnRpY2xlRmxhZy5iMl9wb3dkZXJQYXJ0aWNsZSB8IGIyUGFydGljbGVGbGFnLmIyX3RlbnNpbGVQYXJ0aWNsZTtcclxuXHJcbiAgLyoqXHJcbiAgICogQWxsIHBhcnRpY2xlIHR5cGVzIHRoYXQgYXBwbHkgZXh0cmEgZGFtcGluZyBmb3JjZSB3aXRoIGJvZGllc1xyXG4gICAqL1xyXG4gIHN0YXRpYyByZWFkb25seSBrX2V4dHJhRGFtcGluZ0ZsYWdzID0gYjJQYXJ0aWNsZUZsYWcuYjJfc3RhdGljUHJlc3N1cmVQYXJ0aWNsZTtcclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IGtfYmFycmllcldhbGxGbGFncyA9XHJcbiAgICBiMlBhcnRpY2xlRmxhZy5iMl9iYXJyaWVyUGFydGljbGUgfCBiMlBhcnRpY2xlRmxhZy5iMl93YWxsUGFydGljbGU7XHJcblxyXG4gIEZyZWVCdWZmZXI8VD4oYjogVFtdIHwgbnVsbCwgY2FwYWNpdHk6IG51bWJlcik6IHZvaWQge1xyXG4gICAgaWYgKGIgPT09IG51bGwpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG4gICAgYi5sZW5ndGggPSAwO1xyXG4gIH1cclxuXHJcbiAgRnJlZVVzZXJPdmVycmlkYWJsZUJ1ZmZlcjxUPihiOiBiMlBhcnRpY2xlU3lzdGVtX1VzZXJPdmVycmlkYWJsZUJ1ZmZlcjxUPik6IHZvaWQge1xyXG4gICAgaWYgKGIudXNlclN1cHBsaWVkQ2FwYWNpdHkgPT09IDApIHtcclxuICAgICAgdGhpcy5GcmVlQnVmZmVyKGIuZGF0YSwgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogUmVhbGxvY2F0ZSBhIGJ1ZmZlclxyXG4gICAqL1xyXG4gIFJlYWxsb2NhdGVCdWZmZXIzPFQ+KG9sZEJ1ZmZlcjogVFtdIHwgbnVsbCwgb2xkQ2FwYWNpdHk6IG51bWJlciwgbmV3Q2FwYWNpdHk6IG51bWJlcik6IFRbXSB7XHJcbiAgICAvLyBiMkFzc2VydChuZXdDYXBhY2l0eSA+IG9sZENhcGFjaXR5KTtcclxuICAgIGlmIChuZXdDYXBhY2l0eSA8PSBvbGRDYXBhY2l0eSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuICAgIGNvbnN0IG5ld0J1ZmZlciA9IG9sZEJ1ZmZlciA/IG9sZEJ1ZmZlci5zbGljZSgpIDogW107XHJcbiAgICBuZXdCdWZmZXIubGVuZ3RoID0gbmV3Q2FwYWNpdHk7XHJcbiAgICByZXR1cm4gbmV3QnVmZmVyO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogUmVhbGxvY2F0ZSBhIGJ1ZmZlclxyXG4gICAqL1xyXG4gIFJlYWxsb2NhdGVCdWZmZXI1PFQ+KFxyXG4gICAgYnVmZmVyOiBUW10gfCBudWxsLFxyXG4gICAgdXNlclN1cHBsaWVkQ2FwYWNpdHk6IG51bWJlcixcclxuICAgIG9sZENhcGFjaXR5OiBudW1iZXIsXHJcbiAgICBuZXdDYXBhY2l0eTogbnVtYmVyLFxyXG4gICAgZGVmZXJyZWQ6IGJvb2xlYW4sXHJcbiAgKTogVFtdIHtcclxuICAgIC8vIGIyQXNzZXJ0KG5ld0NhcGFjaXR5ID4gb2xkQ2FwYWNpdHkpO1xyXG4gICAgaWYgKG5ld0NhcGFjaXR5IDw9IG9sZENhcGFjaXR5KSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG4gICAgLy8gQSAnZGVmZXJyZWQnIGJ1ZmZlciBpcyByZWFsbG9jYXRlZCBvbmx5IGlmIGl0IGlzIG5vdCBOVUxMLlxyXG4gICAgLy8gSWYgJ3VzZXJTdXBwbGllZENhcGFjaXR5JyBpcyBub3QgemVybywgYnVmZmVyIGlzIHVzZXIgc3VwcGxpZWQgYW5kIG11c3RcclxuICAgIC8vIGJlIGtlcHQuXHJcbiAgICAvLyBiMkFzc2VydCghdXNlclN1cHBsaWVkQ2FwYWNpdHkgfHwgbmV3Q2FwYWNpdHkgPD0gdXNlclN1cHBsaWVkQ2FwYWNpdHkpO1xyXG4gICAgaWYgKCEoIXVzZXJTdXBwbGllZENhcGFjaXR5IHx8IG5ld0NhcGFjaXR5IDw9IHVzZXJTdXBwbGllZENhcGFjaXR5KSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuICAgIGlmICgoIWRlZmVycmVkIHx8IGJ1ZmZlcikgJiYgIXVzZXJTdXBwbGllZENhcGFjaXR5KSB7XHJcbiAgICAgIGJ1ZmZlciA9IHRoaXMuUmVhbGxvY2F0ZUJ1ZmZlcjMoYnVmZmVyLCBvbGRDYXBhY2l0eSwgbmV3Q2FwYWNpdHkpO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGJ1ZmZlciBhcyBhbnk7IC8vIFRPRE86IGZpeCB0aGlzXHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBSZWFsbG9jYXRlIGEgYnVmZmVyXHJcbiAgICovXHJcbiAgUmVhbGxvY2F0ZUJ1ZmZlcjQ8VD4oXHJcbiAgICBidWZmZXI6IGIyUGFydGljbGVTeXN0ZW1fVXNlck92ZXJyaWRhYmxlQnVmZmVyPGFueT4sXHJcbiAgICBvbGRDYXBhY2l0eTogbnVtYmVyLFxyXG4gICAgbmV3Q2FwYWNpdHk6IG51bWJlcixcclxuICAgIGRlZmVycmVkOiBib29sZWFuLFxyXG4gICk6IFRbXSB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KG5ld0NhcGFjaXR5ID4gb2xkQ2FwYWNpdHkpO1xyXG4gICAgcmV0dXJuIHRoaXMuUmVhbGxvY2F0ZUJ1ZmZlcjUoXHJcbiAgICAgIGJ1ZmZlci5kYXRhLFxyXG4gICAgICBidWZmZXIudXNlclN1cHBsaWVkQ2FwYWNpdHksXHJcbiAgICAgIG9sZENhcGFjaXR5LFxyXG4gICAgICBuZXdDYXBhY2l0eSxcclxuICAgICAgZGVmZXJyZWQsXHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgUmVxdWVzdEJ1ZmZlcjxUPihidWZmZXI6IFRbXSB8IG51bGwpOiBUW10ge1xyXG4gICAgaWYgKCFidWZmZXIpIHtcclxuICAgICAgaWYgKHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5ID09PSAwKSB7XHJcbiAgICAgICAgdGhpcy5SZWFsbG9jYXRlSW50ZXJuYWxBbGxvY2F0ZWRCdWZmZXJzKGIyX21pblBhcnRpY2xlU3lzdGVtQnVmZmVyQ2FwYWNpdHkpO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBidWZmZXIgPSBbXTtcclxuICAgICAgYnVmZmVyLmxlbmd0aCA9IHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5O1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGJ1ZmZlcjtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIFJlYWxsb2NhdGUgdGhlIGhhbmRsZSAvIGluZGV4IG1hcCBhbmQgc2NoZWR1bGUgdGhlIGFsbG9jYXRpb25cclxuICAgKiBvZiBhIG5ldyBwb29sIGZvciBoYW5kbGUgYWxsb2NhdGlvbi5cclxuICAgKi9cclxuICBSZWFsbG9jYXRlSGFuZGxlQnVmZmVycyhuZXdDYXBhY2l0eTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KG5ld0NhcGFjaXR5ID4gdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHkpO1xyXG4gICAgLy8gUmVhbGxvY2F0ZSBhIG5ldyBoYW5kbGUgLyBpbmRleCBtYXAgYnVmZmVyLCBjb3B5aW5nIG9sZCBoYW5kbGUgcG9pbnRlcnNcclxuICAgIC8vIGlzIGZpbmUgc2luY2UgdGhleSdyZSBrZXB0IGFyb3VuZC5cclxuICAgIHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhID0gdGhpcy5SZWFsbG9jYXRlQnVmZmVyNChcclxuICAgICAgdGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyLFxyXG4gICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgbmV3Q2FwYWNpdHksXHJcbiAgICAgIHRydWUsXHJcbiAgICApO1xyXG4gICAgLy8gU2V0IHRoZSBzaXplIG9mIHRoZSBuZXh0IGhhbmRsZSBhbGxvY2F0aW9uLlxyXG4gICAgLy8vdGhpcy5tX2hhbmRsZUFsbG9jYXRvci5TZXRJdGVtc1BlclNsYWIobmV3Q2FwYWNpdHkgLSB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSk7XHJcbiAgfVxyXG5cclxuICBSZWFsbG9jYXRlSW50ZXJuYWxBbGxvY2F0ZWRCdWZmZXJzKGNhcGFjaXR5OiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGZ1bmN0aW9uIExpbWl0Q2FwYWNpdHkoY2FwYWNpdHk6IG51bWJlciwgbWF4Q291bnQ6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICAgIHJldHVybiBtYXhDb3VudCAmJiBjYXBhY2l0eSA+IG1heENvdW50ID8gbWF4Q291bnQgOiBjYXBhY2l0eTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBEb24ndCBpbmNyZWFzZSBjYXBhY2l0eSBiZXlvbmQgdGhlIHNtYWxsZXN0IHVzZXItc3VwcGxpZWQgYnVmZmVyIHNpemUuXHJcbiAgICBjYXBhY2l0eSA9IExpbWl0Q2FwYWNpdHkoY2FwYWNpdHksIHRoaXMubV9kZWYubWF4Q291bnQpO1xyXG4gICAgY2FwYWNpdHkgPSBMaW1pdENhcGFjaXR5KGNhcGFjaXR5LCB0aGlzLm1fZmxhZ3NCdWZmZXIudXNlclN1cHBsaWVkQ2FwYWNpdHkpO1xyXG4gICAgY2FwYWNpdHkgPSBMaW1pdENhcGFjaXR5KGNhcGFjaXR5LCB0aGlzLm1fcG9zaXRpb25CdWZmZXIudXNlclN1cHBsaWVkQ2FwYWNpdHkpO1xyXG4gICAgY2FwYWNpdHkgPSBMaW1pdENhcGFjaXR5KGNhcGFjaXR5LCB0aGlzLm1fdmVsb2NpdHlCdWZmZXIudXNlclN1cHBsaWVkQ2FwYWNpdHkpO1xyXG4gICAgY2FwYWNpdHkgPSBMaW1pdENhcGFjaXR5KGNhcGFjaXR5LCB0aGlzLm1fY29sb3JCdWZmZXIudXNlclN1cHBsaWVkQ2FwYWNpdHkpO1xyXG4gICAgY2FwYWNpdHkgPSBMaW1pdENhcGFjaXR5KGNhcGFjaXR5LCB0aGlzLm1fdXNlckRhdGFCdWZmZXIudXNlclN1cHBsaWVkQ2FwYWNpdHkpO1xyXG4gICAgaWYgKHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5IDwgY2FwYWNpdHkpIHtcclxuICAgICAgdGhpcy5SZWFsbG9jYXRlSGFuZGxlQnVmZmVycyhjYXBhY2l0eSk7XHJcbiAgICAgIHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhID0gdGhpcy5SZWFsbG9jYXRlQnVmZmVyNChcclxuICAgICAgICB0aGlzLm1fZmxhZ3NCdWZmZXIsXHJcbiAgICAgICAgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHksXHJcbiAgICAgICAgY2FwYWNpdHksXHJcbiAgICAgICAgZmFsc2UsXHJcbiAgICAgICk7XHJcblxyXG4gICAgICAvLyBDb25kaXRpb25hbGx5IGRlZmVyIHRoZXNlIGFzIHRoZXkgYXJlIG9wdGlvbmFsIGlmIHRoZSBmZWF0dXJlIGlzXHJcbiAgICAgIC8vIG5vdCBlbmFibGVkLlxyXG4gICAgICBjb25zdCBzdHVjayA9IHRoaXMubV9zdHVja1RocmVzaG9sZCA+IDA7XHJcbiAgICAgIHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGEgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI0KFxyXG4gICAgICAgIHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLFxyXG4gICAgICAgIHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5LFxyXG4gICAgICAgIGNhcGFjaXR5LFxyXG4gICAgICAgIHN0dWNrLFxyXG4gICAgICApO1xyXG4gICAgICB0aGlzLm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlci5kYXRhID0gdGhpcy5SZWFsbG9jYXRlQnVmZmVyNChcclxuICAgICAgICB0aGlzLm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlcixcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICBzdHVjayxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX2NvbnNlY3V0aXZlQ29udGFjdFN0ZXBzQnVmZmVyLmRhdGEgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI0KFxyXG4gICAgICAgIHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlcixcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICBzdHVjayxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGEgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI0KFxyXG4gICAgICAgIHRoaXMubV9wb3NpdGlvbkJ1ZmZlcixcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICBmYWxzZSxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGEgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI0KFxyXG4gICAgICAgIHRoaXMubV92ZWxvY2l0eUJ1ZmZlcixcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICBmYWxzZSxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX2ZvcmNlQnVmZmVyID0gdGhpcy5SZWFsbG9jYXRlQnVmZmVyNShcclxuICAgICAgICB0aGlzLm1fZm9yY2VCdWZmZXIsXHJcbiAgICAgICAgMCxcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICBmYWxzZSxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX3dlaWdodEJ1ZmZlciA9IHRoaXMuUmVhbGxvY2F0ZUJ1ZmZlcjUoXHJcbiAgICAgICAgdGhpcy5tX3dlaWdodEJ1ZmZlcixcclxuICAgICAgICAwLFxyXG4gICAgICAgIHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5LFxyXG4gICAgICAgIGNhcGFjaXR5LFxyXG4gICAgICAgIGZhbHNlLFxyXG4gICAgICApO1xyXG4gICAgICB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXIgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI1KFxyXG4gICAgICAgIHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlcixcclxuICAgICAgICAwLFxyXG4gICAgICAgIHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5LFxyXG4gICAgICAgIGNhcGFjaXR5LFxyXG4gICAgICAgIHRydWUsXHJcbiAgICAgICk7XHJcbiAgICAgIHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXIgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI1KFxyXG4gICAgICAgIHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXIsXHJcbiAgICAgICAgMCxcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICBmYWxzZSxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbjJCdWZmZXIgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI1KFxyXG4gICAgICAgIHRoaXMubV9hY2N1bXVsYXRpb24yQnVmZmVyLFxyXG4gICAgICAgIDAsXHJcbiAgICAgICAgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHksXHJcbiAgICAgICAgY2FwYWNpdHksXHJcbiAgICAgICAgdHJ1ZSxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX2RlcHRoQnVmZmVyID0gdGhpcy5SZWFsbG9jYXRlQnVmZmVyNShcclxuICAgICAgICB0aGlzLm1fZGVwdGhCdWZmZXIsXHJcbiAgICAgICAgMCxcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICB0cnVlLFxyXG4gICAgICApO1xyXG4gICAgICB0aGlzLm1fY29sb3JCdWZmZXIuZGF0YSA9IHRoaXMuUmVhbGxvY2F0ZUJ1ZmZlcjQoXHJcbiAgICAgICAgdGhpcy5tX2NvbG9yQnVmZmVyLFxyXG4gICAgICAgIHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5LFxyXG4gICAgICAgIGNhcGFjaXR5LFxyXG4gICAgICAgIHRydWUsXHJcbiAgICAgICk7XHJcbiAgICAgIHRoaXMubV9ncm91cEJ1ZmZlciA9IHRoaXMuUmVhbGxvY2F0ZUJ1ZmZlcjUoXHJcbiAgICAgICAgdGhpcy5tX2dyb3VwQnVmZmVyLFxyXG4gICAgICAgIDAsXHJcbiAgICAgICAgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHksXHJcbiAgICAgICAgY2FwYWNpdHksXHJcbiAgICAgICAgZmFsc2UsXHJcbiAgICAgICk7XHJcbiAgICAgIHRoaXMubV91c2VyRGF0YUJ1ZmZlci5kYXRhID0gdGhpcy5SZWFsbG9jYXRlQnVmZmVyNChcclxuICAgICAgICB0aGlzLm1fdXNlckRhdGFCdWZmZXIsXHJcbiAgICAgICAgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHksXHJcbiAgICAgICAgY2FwYWNpdHksXHJcbiAgICAgICAgdHJ1ZSxcclxuICAgICAgKTtcclxuICAgICAgdGhpcy5tX2V4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI0KFxyXG4gICAgICAgIHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlcixcclxuICAgICAgICB0aGlzLm1faW50ZXJuYWxBbGxvY2F0ZWRDYXBhY2l0eSxcclxuICAgICAgICBjYXBhY2l0eSxcclxuICAgICAgICB0cnVlLFxyXG4gICAgICApO1xyXG4gICAgICB0aGlzLm1faW5kZXhCeUV4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEgPSB0aGlzLlJlYWxsb2NhdGVCdWZmZXI0KFxyXG4gICAgICAgIHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIsXHJcbiAgICAgICAgdGhpcy5tX2ludGVybmFsQWxsb2NhdGVkQ2FwYWNpdHksXHJcbiAgICAgICAgY2FwYWNpdHksXHJcbiAgICAgICAgZmFsc2UsXHJcbiAgICAgICk7XHJcbiAgICAgIHRoaXMubV9pbnRlcm5hbEFsbG9jYXRlZENhcGFjaXR5ID0gY2FwYWNpdHk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBDcmVhdGVQYXJ0aWNsZUZvckdyb3VwKGdyb3VwRGVmOiBiMklQYXJ0aWNsZUdyb3VwRGVmLCB4ZjogYjJUcmFuc2Zvcm0sIHA6IFhZKTogdm9pZCB7XHJcbiAgICBjb25zdCBwYXJ0aWNsZURlZiA9IG5ldyBiMlBhcnRpY2xlRGVmKCk7XHJcbiAgICBwYXJ0aWNsZURlZi5mbGFncyA9IGIyTWF5YmUoZ3JvdXBEZWYuZmxhZ3MsIDApO1xyXG4gICAgLy8vcGFydGljbGVEZWYucG9zaXRpb24gPSBiMk11bCh4ZiwgcCk7XHJcbiAgICBiMlRyYW5zZm9ybS5NdWxYVih4ZiwgcCwgcGFydGljbGVEZWYucG9zaXRpb24pO1xyXG4gICAgLy8vcGFydGljbGVEZWYudmVsb2NpdHkgPVxyXG4gICAgLy8vICBncm91cERlZi5saW5lYXJWZWxvY2l0eSArXHJcbiAgICAvLy8gIGIyQ3Jvc3MoZ3JvdXBEZWYuYW5ndWxhclZlbG9jaXR5LFxyXG4gICAgLy8vICAgICAgcGFydGljbGVEZWYucG9zaXRpb24gLSBncm91cERlZi5wb3NpdGlvbik7XHJcbiAgICBiMlZlYzIuQWRkVlYoXHJcbiAgICAgIGIyTWF5YmUoZ3JvdXBEZWYubGluZWFyVmVsb2NpdHksIGIyVmVjMi5aRVJPKSxcclxuICAgICAgYjJWZWMyLkNyb3NzU1YoXHJcbiAgICAgICAgYjJNYXliZShncm91cERlZi5hbmd1bGFyVmVsb2NpdHksIDApLFxyXG4gICAgICAgIGIyVmVjMi5TdWJWVihwYXJ0aWNsZURlZi5wb3NpdGlvbiwgYjJNYXliZShncm91cERlZi5wb3NpdGlvbiwgYjJWZWMyLlpFUk8pLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgYjJWZWMyLnNfdDAsXHJcbiAgICAgICksXHJcbiAgICAgIHBhcnRpY2xlRGVmLnZlbG9jaXR5LFxyXG4gICAgKTtcclxuICAgIHBhcnRpY2xlRGVmLmNvbG9yLkNvcHkoYjJNYXliZShncm91cERlZi5jb2xvciwgYjJDb2xvci5aRVJPKSk7XHJcbiAgICBwYXJ0aWNsZURlZi5saWZldGltZSA9IGIyTWF5YmUoZ3JvdXBEZWYubGlmZXRpbWUsIDApO1xyXG4gICAgcGFydGljbGVEZWYudXNlckRhdGEgPSBncm91cERlZi51c2VyRGF0YTtcclxuICAgIHRoaXMuQ3JlYXRlUGFydGljbGUocGFydGljbGVEZWYpO1xyXG4gIH1cclxuXHJcbiAgQ3JlYXRlUGFydGljbGVzU3Ryb2tlU2hhcGVGb3JHcm91cChcclxuICAgIHNoYXBlOiBiMlNoYXBlLFxyXG4gICAgZ3JvdXBEZWY6IGIySVBhcnRpY2xlR3JvdXBEZWYsXHJcbiAgICB4ZjogYjJUcmFuc2Zvcm0sXHJcbiAgKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX2VkZ2UgPSBiMlBhcnRpY2xlU3lzdGVtLkNyZWF0ZVBhcnRpY2xlc1N0cm9rZVNoYXBlRm9yR3JvdXBfc19lZGdlO1xyXG4gICAgY29uc3Qgc19kID0gYjJQYXJ0aWNsZVN5c3RlbS5DcmVhdGVQYXJ0aWNsZXNTdHJva2VTaGFwZUZvckdyb3VwX3NfZDtcclxuICAgIGNvbnN0IHNfcCA9IGIyUGFydGljbGVTeXN0ZW0uQ3JlYXRlUGFydGljbGVzU3Ryb2tlU2hhcGVGb3JHcm91cF9zX3A7XHJcbiAgICBsZXQgc3RyaWRlID0gYjJNYXliZShncm91cERlZi5zdHJpZGUsIDApO1xyXG4gICAgaWYgKHN0cmlkZSA9PT0gMCkge1xyXG4gICAgICBzdHJpZGUgPSB0aGlzLkdldFBhcnRpY2xlU3RyaWRlKCk7XHJcbiAgICB9XHJcbiAgICBsZXQgcG9zaXRpb25PbkVkZ2UgPSAwO1xyXG4gICAgY29uc3QgY2hpbGRDb3VudCA9IHNoYXBlLkdldENoaWxkQ291bnQoKTtcclxuICAgIGZvciAobGV0IGNoaWxkSW5kZXggPSAwOyBjaGlsZEluZGV4IDwgY2hpbGRDb3VudDsgY2hpbGRJbmRleCsrKSB7XHJcbiAgICAgIGxldCBlZGdlOiBiMkVkZ2VTaGFwZSB8IG51bGwgPSBudWxsO1xyXG4gICAgICBpZiAoc2hhcGUuR2V0VHlwZSgpID09PSBiMlNoYXBlVHlwZS5lX2VkZ2VTaGFwZSkge1xyXG4gICAgICAgIGVkZ2UgPSBzaGFwZSBhcyBiMkVkZ2VTaGFwZTtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHNoYXBlLkdldFR5cGUoKSA9PT0gYjJTaGFwZVR5cGUuZV9jaGFpblNoYXBlKTtcclxuICAgICAgICBlZGdlID0gc19lZGdlO1xyXG4gICAgICAgIChzaGFwZSBhcyBiMkNoYWluU2hhcGUpLkdldENoaWxkRWRnZShlZGdlLCBjaGlsZEluZGV4KTtcclxuICAgICAgfVxyXG4gICAgICBjb25zdCBkID0gYjJWZWMyLlN1YlZWKGVkZ2UubV92ZXJ0ZXgyLCBlZGdlLm1fdmVydGV4MSwgc19kKTtcclxuICAgICAgY29uc3QgZWRnZUxlbmd0aCA9IGQuTGVuZ3RoKCk7XHJcblxyXG4gICAgICB3aGlsZSAocG9zaXRpb25PbkVkZ2UgPCBlZGdlTGVuZ3RoKSB7XHJcbiAgICAgICAgLy8vYjJWZWMyIHAgPSBlZGdlLm1fdmVydGV4MSArIHBvc2l0aW9uT25FZGdlIC8gZWRnZUxlbmd0aCAqIGQ7XHJcbiAgICAgICAgY29uc3QgcCA9IGIyVmVjMi5BZGRWTXVsU1YoZWRnZS5tX3ZlcnRleDEsIHBvc2l0aW9uT25FZGdlIC8gZWRnZUxlbmd0aCwgZCwgc19wKTtcclxuICAgICAgICB0aGlzLkNyZWF0ZVBhcnRpY2xlRm9yR3JvdXAoZ3JvdXBEZWYsIHhmLCBwKTtcclxuICAgICAgICBwb3NpdGlvbk9uRWRnZSArPSBzdHJpZGU7XHJcbiAgICAgIH1cclxuICAgICAgcG9zaXRpb25PbkVkZ2UgLT0gZWRnZUxlbmd0aDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBDcmVhdGVQYXJ0aWNsZXNTdHJva2VTaGFwZUZvckdyb3VwX3NfZWRnZSA9IG5ldyBiMkVkZ2VTaGFwZSgpO1xyXG4gIHN0YXRpYyByZWFkb25seSBDcmVhdGVQYXJ0aWNsZXNTdHJva2VTaGFwZUZvckdyb3VwX3NfZCA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgQ3JlYXRlUGFydGljbGVzU3Ryb2tlU2hhcGVGb3JHcm91cF9zX3AgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIENyZWF0ZVBhcnRpY2xlc0ZpbGxTaGFwZUZvckdyb3VwKFxyXG4gICAgc2hhcGU6IGIyU2hhcGUsXHJcbiAgICBncm91cERlZjogYjJJUGFydGljbGVHcm91cERlZixcclxuICAgIHhmOiBiMlRyYW5zZm9ybSxcclxuICApOiB2b2lkIHtcclxuICAgIGNvbnN0IHNfYWFiYiA9IGIyUGFydGljbGVTeXN0ZW0uQ3JlYXRlUGFydGljbGVzRmlsbFNoYXBlRm9yR3JvdXBfc19hYWJiO1xyXG4gICAgY29uc3Qgc19wID0gYjJQYXJ0aWNsZVN5c3RlbS5DcmVhdGVQYXJ0aWNsZXNGaWxsU2hhcGVGb3JHcm91cF9zX3A7XHJcbiAgICBsZXQgc3RyaWRlID0gYjJNYXliZShncm91cERlZi5zdHJpZGUsIDApO1xyXG4gICAgaWYgKHN0cmlkZSA9PT0gMCkge1xyXG4gICAgICBzdHJpZGUgPSB0aGlzLkdldFBhcnRpY2xlU3RyaWRlKCk7XHJcbiAgICB9XHJcbiAgICAvLy9iMlRyYW5zZm9ybSBpZGVudGl0eTtcclxuICAgIC8vLyBpZGVudGl0eS5TZXRJZGVudGl0eSgpO1xyXG4gICAgY29uc3QgaWRlbnRpdHkgPSBiMlRyYW5zZm9ybS5JREVOVElUWTtcclxuICAgIGNvbnN0IGFhYmIgPSBzX2FhYmI7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHNoYXBlLkdldENoaWxkQ291bnQoKSA9PT0gMSk7XHJcbiAgICBzaGFwZS5Db21wdXRlQUFCQihhYWJiLCBpZGVudGl0eSwgMCk7XHJcbiAgICBmb3IgKFxyXG4gICAgICBsZXQgeSA9IE1hdGguZmxvb3IoYWFiYi5sb3dlckJvdW5kLnkgLyBzdHJpZGUpICogc3RyaWRlO1xyXG4gICAgICB5IDwgYWFiYi51cHBlckJvdW5kLnk7XHJcbiAgICAgIHkgKz0gc3RyaWRlXHJcbiAgICApIHtcclxuICAgICAgZm9yIChcclxuICAgICAgICBsZXQgeCA9IE1hdGguZmxvb3IoYWFiYi5sb3dlckJvdW5kLnggLyBzdHJpZGUpICogc3RyaWRlO1xyXG4gICAgICAgIHggPCBhYWJiLnVwcGVyQm91bmQueDtcclxuICAgICAgICB4ICs9IHN0cmlkZVxyXG4gICAgICApIHtcclxuICAgICAgICBjb25zdCBwID0gc19wLlNldCh4LCB5KTtcclxuICAgICAgICBpZiAoc2hhcGUuVGVzdFBvaW50KGlkZW50aXR5LCBwKSkge1xyXG4gICAgICAgICAgdGhpcy5DcmVhdGVQYXJ0aWNsZUZvckdyb3VwKGdyb3VwRGVmLCB4ZiwgcCk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgQ3JlYXRlUGFydGljbGVzRmlsbFNoYXBlRm9yR3JvdXBfc19hYWJiID0gbmV3IGIyQUFCQigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBDcmVhdGVQYXJ0aWNsZXNGaWxsU2hhcGVGb3JHcm91cF9zX3AgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIENyZWF0ZVBhcnRpY2xlc1dpdGhTaGFwZUZvckdyb3VwKFxyXG4gICAgc2hhcGU6IGIyU2hhcGUsXHJcbiAgICBncm91cERlZjogYjJJUGFydGljbGVHcm91cERlZixcclxuICAgIHhmOiBiMlRyYW5zZm9ybSxcclxuICApOiB2b2lkIHtcclxuICAgIHN3aXRjaCAoc2hhcGUuR2V0VHlwZSgpKSB7XHJcbiAgICAgIGNhc2UgYjJTaGFwZVR5cGUuZV9lZGdlU2hhcGU6XHJcbiAgICAgIGNhc2UgYjJTaGFwZVR5cGUuZV9jaGFpblNoYXBlOlxyXG4gICAgICAgIHRoaXMuQ3JlYXRlUGFydGljbGVzU3Ryb2tlU2hhcGVGb3JHcm91cChzaGFwZSwgZ3JvdXBEZWYsIHhmKTtcclxuICAgICAgICBicmVhaztcclxuICAgICAgY2FzZSBiMlNoYXBlVHlwZS5lX3BvbHlnb25TaGFwZTpcclxuICAgICAgY2FzZSBiMlNoYXBlVHlwZS5lX2NpcmNsZVNoYXBlOlxyXG4gICAgICAgIHRoaXMuQ3JlYXRlUGFydGljbGVzRmlsbFNoYXBlRm9yR3JvdXAoc2hhcGUsIGdyb3VwRGVmLCB4Zik7XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICAgIGRlZmF1bHQ6XHJcbiAgICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBDcmVhdGVQYXJ0aWNsZXNXaXRoU2hhcGVzRm9yR3JvdXAoXHJcbiAgICBzaGFwZXM6IGIyU2hhcGVbXSxcclxuICAgIHNoYXBlQ291bnQ6IG51bWJlcixcclxuICAgIGdyb3VwRGVmOiBiMklQYXJ0aWNsZUdyb3VwRGVmLFxyXG4gICAgeGY6IGIyVHJhbnNmb3JtLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3QgY29tcG9zaXRlU2hhcGUgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9Db21wb3NpdGVTaGFwZShzaGFwZXMsIHNoYXBlQ291bnQpO1xyXG4gICAgdGhpcy5DcmVhdGVQYXJ0aWNsZXNGaWxsU2hhcGVGb3JHcm91cChjb21wb3NpdGVTaGFwZSwgZ3JvdXBEZWYsIHhmKTtcclxuICB9XHJcblxyXG4gIENsb25lUGFydGljbGUob2xkSW5kZXg6IG51bWJlciwgZ3JvdXA6IGIyUGFydGljbGVHcm91cCk6IG51bWJlciB7XHJcbiAgICBjb25zdCBkZWYgPSBuZXcgYjJQYXJ0aWNsZURlZigpO1xyXG4gICAgZGVmLmZsYWdzID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbb2xkSW5kZXhdO1xyXG4gICAgZGVmLnBvc2l0aW9uLkNvcHkodGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGFbb2xkSW5kZXhdKTtcclxuICAgIGRlZi52ZWxvY2l0eS5Db3B5KHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW29sZEluZGV4XSk7XHJcbiAgICBpZiAodGhpcy5tX2NvbG9yQnVmZmVyLmRhdGEpIHtcclxuICAgICAgZGVmLmNvbG9yLkNvcHkodGhpcy5tX2NvbG9yQnVmZmVyLmRhdGFbb2xkSW5kZXhdKTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fdXNlckRhdGFCdWZmZXIuZGF0YSkge1xyXG4gICAgICBkZWYudXNlckRhdGEgPSB0aGlzLm1fdXNlckRhdGFCdWZmZXIuZGF0YVtvbGRJbmRleF07XHJcbiAgICB9XHJcbiAgICBkZWYuZ3JvdXAgPSBncm91cDtcclxuICAgIGNvbnN0IG5ld0luZGV4ID0gdGhpcy5DcmVhdGVQYXJ0aWNsZShkZWYpO1xyXG4gICAgaWYgKHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhKSB7XHJcbiAgICAgIGNvbnN0IGhhbmRsZSA9IHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW29sZEluZGV4XTtcclxuICAgICAgaWYgKGhhbmRsZSkge1xyXG4gICAgICAgIGhhbmRsZS5pbmRleCA9IG5ld0luZGV4O1xyXG4gICAgICB9XHJcbiAgICAgIHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW25ld0luZGV4XSA9IGhhbmRsZTtcclxuICAgICAgdGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyLmRhdGFbb2xkSW5kZXhdID0gbnVsbDtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fbGFzdEJvZHlDb250YWN0U3RlcEJ1ZmZlci5kYXRhKSB7XHJcbiAgICAgIHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGFbbmV3SW5kZXhdID0gdGhpcy5tX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YVtcclxuICAgICAgICBvbGRJbmRleFxyXG4gICAgICBdO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGEpIHtcclxuICAgICAgdGhpcy5tX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YVtuZXdJbmRleF0gPSB0aGlzLm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlci5kYXRhW29sZEluZGV4XTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIuZGF0YSkge1xyXG4gICAgICB0aGlzLm1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIuZGF0YVtcclxuICAgICAgICBuZXdJbmRleFxyXG4gICAgICBdID0gdGhpcy5tX2NvbnNlY3V0aXZlQ29udGFjdFN0ZXBzQnVmZmVyLmRhdGFbb2xkSW5kZXhdO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9oYXNGb3JjZSkge1xyXG4gICAgICB0aGlzLm1fZm9yY2VCdWZmZXJbbmV3SW5kZXhdLkNvcHkodGhpcy5tX2ZvcmNlQnVmZmVyW29sZEluZGV4XSk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX3N0YXRpY1ByZXNzdXJlQnVmZmVyKSB7XHJcbiAgICAgIHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlcltuZXdJbmRleF0gPSB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXJbb2xkSW5kZXhdO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9kZXB0aEJ1ZmZlcikge1xyXG4gICAgICB0aGlzLm1fZGVwdGhCdWZmZXJbbmV3SW5kZXhdID0gdGhpcy5tX2RlcHRoQnVmZmVyW29sZEluZGV4XTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSkge1xyXG4gICAgICB0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtuZXdJbmRleF0gPSB0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtvbGRJbmRleF07XHJcbiAgICB9XHJcbiAgICByZXR1cm4gbmV3SW5kZXg7XHJcbiAgfVxyXG5cclxuICBEZXN0cm95UGFydGljbGVzSW5Hcm91cChncm91cDogYjJQYXJ0aWNsZUdyb3VwLCBjYWxsRGVzdHJ1Y3Rpb25MaXN0ZW5lciA9IGZhbHNlKTogdm9pZCB7XHJcbiAgICBmb3IgKGxldCBpID0gZ3JvdXAubV9maXJzdEluZGV4OyBpIDwgZ3JvdXAubV9sYXN0SW5kZXg7IGkrKykge1xyXG4gICAgICB0aGlzLkRlc3Ryb3lQYXJ0aWNsZShpLCBjYWxsRGVzdHJ1Y3Rpb25MaXN0ZW5lcik7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBEZXN0cm95UGFydGljbGVHcm91cChncm91cDogYjJQYXJ0aWNsZUdyb3VwKTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV9ncm91cENvdW50ID4gMCk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGdyb3VwICE9PSBudWxsKTtcclxuXHJcbiAgICBpZiAodGhpcy5tX3dvcmxkLm1fZGVzdHJ1Y3Rpb25MaXN0ZW5lcikge1xyXG4gICAgICB0aGlzLm1fd29ybGQubV9kZXN0cnVjdGlvbkxpc3RlbmVyLlNheUdvb2RieWVQYXJ0aWNsZUdyb3VwKGdyb3VwKTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLlNldEdyb3VwRmxhZ3MoZ3JvdXAsIDApO1xyXG4gICAgZm9yIChsZXQgaSA9IGdyb3VwLm1fZmlyc3RJbmRleDsgaSA8IGdyb3VwLm1fbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgdGhpcy5tX2dyb3VwQnVmZmVyW2ldID0gbnVsbDtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoZ3JvdXAubV9wcmV2KSB7XHJcbiAgICAgIGdyb3VwLm1fcHJldi5tX25leHQgPSBncm91cC5tX25leHQ7XHJcbiAgICB9XHJcbiAgICBpZiAoZ3JvdXAubV9uZXh0KSB7XHJcbiAgICAgIGdyb3VwLm1fbmV4dC5tX3ByZXYgPSBncm91cC5tX3ByZXY7XHJcbiAgICB9XHJcbiAgICBpZiAoZ3JvdXAgPT09IHRoaXMubV9ncm91cExpc3QpIHtcclxuICAgICAgdGhpcy5tX2dyb3VwTGlzdCA9IGdyb3VwLm1fbmV4dDtcclxuICAgIH1cclxuXHJcbiAgICAtLXRoaXMubV9ncm91cENvdW50O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIFBhcnRpY2xlQ2FuQmVDb25uZWN0ZWQoZmxhZ3M6IGIyUGFydGljbGVGbGFnLCBncm91cDogYjJQYXJ0aWNsZUdyb3VwIHwgbnVsbCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIChcclxuICAgICAgKGZsYWdzICZcclxuICAgICAgICAoYjJQYXJ0aWNsZUZsYWcuYjJfd2FsbFBhcnRpY2xlIHxcclxuICAgICAgICAgIGIyUGFydGljbGVGbGFnLmIyX3NwcmluZ1BhcnRpY2xlIHxcclxuICAgICAgICAgIGIyUGFydGljbGVGbGFnLmIyX2VsYXN0aWNQYXJ0aWNsZSkpICE9PVxyXG4gICAgICAgIDAgfHxcclxuICAgICAgKGdyb3VwICE9PSBudWxsICYmIChncm91cC5HZXRHcm91cEZsYWdzKCkgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3JpZ2lkUGFydGljbGVHcm91cCkgIT09IDApXHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgVXBkYXRlUGFpcnNBbmRUcmlhZHMoXHJcbiAgICBmaXJzdEluZGV4OiBudW1iZXIsXHJcbiAgICBsYXN0SW5kZXg6IG51bWJlcixcclxuICAgIGZpbHRlcjogYjJQYXJ0aWNsZVN5c3RlbV9Db25uZWN0aW9uRmlsdGVyLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19kYWIgPSBiMlBhcnRpY2xlU3lzdGVtLlVwZGF0ZVBhaXJzQW5kVHJpYWRzX3NfZGFiO1xyXG4gICAgY29uc3Qgc19kYmMgPSBiMlBhcnRpY2xlU3lzdGVtLlVwZGF0ZVBhaXJzQW5kVHJpYWRzX3NfZGJjO1xyXG4gICAgY29uc3Qgc19kY2EgPSBiMlBhcnRpY2xlU3lzdGVtLlVwZGF0ZVBhaXJzQW5kVHJpYWRzX3NfZGNhO1xyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIC8vIENyZWF0ZSBwYWlycyBvciB0cmlhZHMuXHJcbiAgICAvLyBBbGwgcGFydGljbGVzIGluIGVhY2ggcGFpci90cmlhZCBzaG91bGQgc2F0aXNmeSB0aGUgZm9sbG93aW5nOlxyXG4gICAgLy8gKiBmaXJzdEluZGV4IDw9IGluZGV4IDwgbGFzdEluZGV4XHJcbiAgICAvLyAqIGRvbid0IGhhdmUgYjJfem9tYmllUGFydGljbGVcclxuICAgIC8vICogUGFydGljbGVDYW5CZUNvbm5lY3RlZCByZXR1cm5zIHRydWVcclxuICAgIC8vICogU2hvdWxkQ3JlYXRlUGFpci9TaG91bGRDcmVhdGVUcmlhZCByZXR1cm5zIHRydWVcclxuICAgIC8vIEFueSBwYXJ0aWNsZXMgaW4gZWFjaCBwYWlyL3RyaWFkIHNob3VsZCBzYXRpc2Z5IHRoZSBmb2xsb3dpbmc6XHJcbiAgICAvLyAqIGZpbHRlci5Jc05lZWRlZCByZXR1cm5zIHRydWVcclxuICAgIC8vICogaGF2ZSBvbmUgb2Yga19wYWlyRmxhZ3Mva190cmlhZHNGbGFnc1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmaXJzdEluZGV4IDw9IGxhc3RJbmRleCk7XHJcbiAgICBsZXQgcGFydGljbGVGbGFncyA9IDA7XHJcbiAgICBmb3IgKGxldCBpID0gZmlyc3RJbmRleDsgaSA8IGxhc3RJbmRleDsgaSsrKSB7XHJcbiAgICAgIHBhcnRpY2xlRmxhZ3MgfD0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbaV07XHJcbiAgICB9XHJcbiAgICBpZiAocGFydGljbGVGbGFncyAmIGIyUGFydGljbGVTeXN0ZW0ua19wYWlyRmxhZ3MpIHtcclxuICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fY29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgICBjb25zdCBhZiA9IHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2FdO1xyXG4gICAgICAgIGNvbnN0IGJmID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbYl07XHJcbiAgICAgICAgY29uc3QgZ3JvdXBBID0gdGhpcy5tX2dyb3VwQnVmZmVyW2FdO1xyXG4gICAgICAgIGNvbnN0IGdyb3VwQiA9IHRoaXMubV9ncm91cEJ1ZmZlcltiXTtcclxuICAgICAgICBpZiAoXHJcbiAgICAgICAgICBhID49IGZpcnN0SW5kZXggJiZcclxuICAgICAgICAgIGEgPCBsYXN0SW5kZXggJiZcclxuICAgICAgICAgIGIgPj0gZmlyc3RJbmRleCAmJlxyXG4gICAgICAgICAgYiA8IGxhc3RJbmRleCAmJlxyXG4gICAgICAgICAgISgoYWYgfCBiZikgJiBiMlBhcnRpY2xlRmxhZy5iMl96b21iaWVQYXJ0aWNsZSkgJiZcclxuICAgICAgICAgIChhZiB8IGJmKSAmIGIyUGFydGljbGVTeXN0ZW0ua19wYWlyRmxhZ3MgJiZcclxuICAgICAgICAgIChmaWx0ZXIuSXNOZWNlc3NhcnkoYSkgfHwgZmlsdGVyLklzTmVjZXNzYXJ5KGIpKSAmJlxyXG4gICAgICAgICAgYjJQYXJ0aWNsZVN5c3RlbS5QYXJ0aWNsZUNhbkJlQ29ubmVjdGVkKGFmLCBncm91cEEpICYmXHJcbiAgICAgICAgICBiMlBhcnRpY2xlU3lzdGVtLlBhcnRpY2xlQ2FuQmVDb25uZWN0ZWQoYmYsIGdyb3VwQikgJiZcclxuICAgICAgICAgIGZpbHRlci5TaG91bGRDcmVhdGVQYWlyKGEsIGIpXHJcbiAgICAgICAgKSB7XHJcbiAgICAgICAgICAvLy9iMlBhcnRpY2xlUGFpciYgcGFpciA9IG1fcGFpckJ1ZmZlci5BcHBlbmQoKTtcclxuICAgICAgICAgIGNvbnN0IHBhaXIgPSB0aGlzLm1fcGFpckJ1ZmZlci5kYXRhW3RoaXMubV9wYWlyQnVmZmVyLkFwcGVuZCgpXTtcclxuICAgICAgICAgIHBhaXIuaW5kZXhBID0gYTtcclxuICAgICAgICAgIHBhaXIuaW5kZXhCID0gYjtcclxuICAgICAgICAgIHBhaXIuZmxhZ3MgPSBjb250YWN0LmZsYWdzO1xyXG4gICAgICAgICAgcGFpci5zdHJlbmd0aCA9IGIyTWluKGdyb3VwQSA/IGdyb3VwQS5tX3N0cmVuZ3RoIDogMSwgZ3JvdXBCID8gZ3JvdXBCLm1fc3RyZW5ndGggOiAxKTtcclxuICAgICAgICAgIC8vL3BhaXIuZGlzdGFuY2UgPSBiMkRpc3RhbmNlKHBvc19kYXRhW2FdLCBwb3NfZGF0YVtiXSk7IC8vIFRPRE86IHRoaXMgd2FzIHdyb25nIVxyXG4gICAgICAgICAgcGFpci5kaXN0YW5jZSA9IGIyVmVjMi5EaXN0YW5jZVZWKHBvc19kYXRhW2FdLCBwb3NfZGF0YVtiXSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIC8vL3N0ZDo6c3RhYmxlX3NvcnQobV9wYWlyQnVmZmVyLkJlZ2luKCksIG1fcGFpckJ1ZmZlci5FbmQoKSwgQ29tcGFyZVBhaXJJbmRpY2VzKTtcclxuICAgICAgICBzdGRfc3RhYmxlX3NvcnQoXHJcbiAgICAgICAgICB0aGlzLm1fcGFpckJ1ZmZlci5kYXRhLFxyXG4gICAgICAgICAgMCxcclxuICAgICAgICAgIHRoaXMubV9wYWlyQnVmZmVyLmNvdW50LFxyXG4gICAgICAgICAgYjJQYXJ0aWNsZVN5c3RlbS5Db21wYXJlUGFpckluZGljZXMsXHJcbiAgICAgICAgKTtcclxuICAgICAgICAvLy9tX3BhaXJCdWZmZXIuVW5pcXVlKE1hdGNoUGFpckluZGljZXMpO1xyXG4gICAgICAgIHRoaXMubV9wYWlyQnVmZmVyLlVuaXF1ZShiMlBhcnRpY2xlU3lzdGVtLk1hdGNoUGFpckluZGljZXMpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICBpZiAocGFydGljbGVGbGFncyAmIGIyUGFydGljbGVTeXN0ZW0ua190cmlhZEZsYWdzKSB7XHJcbiAgICAgIGNvbnN0IGRpYWdyYW0gPSBuZXcgYjJWb3Jvbm9pRGlhZ3JhbShsYXN0SW5kZXggLSBmaXJzdEluZGV4KTtcclxuICAgICAgLy8vbGV0IG5lY2Vzc2FyeV9jb3VudCA9IDA7XHJcbiAgICAgIGZvciAobGV0IGkgPSBmaXJzdEluZGV4OyBpIDwgbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgICBjb25zdCBmbGFncyA9IHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgIGNvbnN0IGdyb3VwID0gdGhpcy5tX2dyb3VwQnVmZmVyW2ldO1xyXG4gICAgICAgIGlmIChcclxuICAgICAgICAgICEoZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl96b21iaWVQYXJ0aWNsZSkgJiZcclxuICAgICAgICAgIGIyUGFydGljbGVTeXN0ZW0uUGFydGljbGVDYW5CZUNvbm5lY3RlZChmbGFncywgZ3JvdXApXHJcbiAgICAgICAgKSB7XHJcbiAgICAgICAgICAvLy9pZiAoZmlsdGVyLklzTmVjZXNzYXJ5KGkpKSB7XHJcbiAgICAgICAgICAvLy8rK25lY2Vzc2FyeV9jb3VudDtcclxuICAgICAgICAgIC8vL31cclxuICAgICAgICAgIGRpYWdyYW0uQWRkR2VuZXJhdG9yKHBvc19kYXRhW2ldLCBpLCBmaWx0ZXIuSXNOZWNlc3NhcnkoaSkpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgICAvLy9pZiAobmVjZXNzYXJ5X2NvdW50ID09PSAwKSB7XHJcbiAgICAgIC8vLy8vZGVidWdnZXI7XHJcbiAgICAgIC8vL2ZvciAobGV0IGkgPSBmaXJzdEluZGV4OyBpIDwgbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgLy8vICBmaWx0ZXIuSXNOZWNlc3NhcnkoaSk7XHJcbiAgICAgIC8vL31cclxuICAgICAgLy8vfVxyXG4gICAgICBjb25zdCBzdHJpZGUgPSB0aGlzLkdldFBhcnRpY2xlU3RyaWRlKCk7XHJcbiAgICAgIGRpYWdyYW0uR2VuZXJhdGUoc3RyaWRlIC8gMiwgc3RyaWRlICogMik7XHJcbiAgICAgIGNvbnN0IGNhbGxiYWNrID0gLypVcGRhdGVUcmlhZHNDYWxsYmFjayovIChhOiBudW1iZXIsIGI6IG51bWJlciwgYzogbnVtYmVyKTogdm9pZCA9PiB7XHJcbiAgICAgICAgY29uc3QgYWYgPSB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVthXTtcclxuICAgICAgICBjb25zdCBiZiA9IHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2JdO1xyXG4gICAgICAgIGNvbnN0IGNmID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbY107XHJcbiAgICAgICAgaWYgKChhZiB8IGJmIHwgY2YpICYgYjJQYXJ0aWNsZVN5c3RlbS5rX3RyaWFkRmxhZ3MgJiYgZmlsdGVyLlNob3VsZENyZWF0ZVRyaWFkKGEsIGIsIGMpKSB7XHJcbiAgICAgICAgICBjb25zdCBwYSA9IHBvc19kYXRhW2FdO1xyXG4gICAgICAgICAgY29uc3QgcGIgPSBwb3NfZGF0YVtiXTtcclxuICAgICAgICAgIGNvbnN0IHBjID0gcG9zX2RhdGFbY107XHJcbiAgICAgICAgICBjb25zdCBkYWIgPSBiMlZlYzIuU3ViVlYocGEsIHBiLCBzX2RhYik7XHJcbiAgICAgICAgICBjb25zdCBkYmMgPSBiMlZlYzIuU3ViVlYocGIsIHBjLCBzX2RiYyk7XHJcbiAgICAgICAgICBjb25zdCBkY2EgPSBiMlZlYzIuU3ViVlYocGMsIHBhLCBzX2RjYSk7XHJcbiAgICAgICAgICBjb25zdCBtYXhEaXN0YW5jZVNxdWFyZWQgPSBiMl9tYXhUcmlhZERpc3RhbmNlU3F1YXJlZCAqIHRoaXMubV9zcXVhcmVkRGlhbWV0ZXI7XHJcbiAgICAgICAgICBpZiAoXHJcbiAgICAgICAgICAgIGIyVmVjMi5Eb3RWVihkYWIsIGRhYikgPiBtYXhEaXN0YW5jZVNxdWFyZWQgfHxcclxuICAgICAgICAgICAgYjJWZWMyLkRvdFZWKGRiYywgZGJjKSA+IG1heERpc3RhbmNlU3F1YXJlZCB8fFxyXG4gICAgICAgICAgICBiMlZlYzIuRG90VlYoZGNhLCBkY2EpID4gbWF4RGlzdGFuY2VTcXVhcmVkXHJcbiAgICAgICAgICApIHtcclxuICAgICAgICAgICAgcmV0dXJuO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgY29uc3QgZ3JvdXBBID0gdGhpcy5tX2dyb3VwQnVmZmVyW2FdO1xyXG4gICAgICAgICAgY29uc3QgZ3JvdXBCID0gdGhpcy5tX2dyb3VwQnVmZmVyW2JdO1xyXG4gICAgICAgICAgY29uc3QgZ3JvdXBDID0gdGhpcy5tX2dyb3VwQnVmZmVyW2NdO1xyXG4gICAgICAgICAgLy8vYjJQYXJ0aWNsZVRyaWFkJiB0cmlhZCA9IG1fc3lzdGVtLm1fdHJpYWRCdWZmZXIuQXBwZW5kKCk7XHJcbiAgICAgICAgICBjb25zdCB0cmlhZCA9IHRoaXMubV90cmlhZEJ1ZmZlci5kYXRhW3RoaXMubV90cmlhZEJ1ZmZlci5BcHBlbmQoKV07XHJcbiAgICAgICAgICB0cmlhZC5pbmRleEEgPSBhO1xyXG4gICAgICAgICAgdHJpYWQuaW5kZXhCID0gYjtcclxuICAgICAgICAgIHRyaWFkLmluZGV4QyA9IGM7XHJcbiAgICAgICAgICB0cmlhZC5mbGFncyA9IGFmIHwgYmYgfCBjZjtcclxuICAgICAgICAgIHRyaWFkLnN0cmVuZ3RoID0gYjJNaW4oXHJcbiAgICAgICAgICAgIGIyTWluKGdyb3VwQSA/IGdyb3VwQS5tX3N0cmVuZ3RoIDogMSwgZ3JvdXBCID8gZ3JvdXBCLm1fc3RyZW5ndGggOiAxKSxcclxuICAgICAgICAgICAgZ3JvdXBDID8gZ3JvdXBDLm1fc3RyZW5ndGggOiAxLFxyXG4gICAgICAgICAgKTtcclxuICAgICAgICAgIC8vL2xldCBtaWRQb2ludCA9IGIyVmVjMi5NdWxTVigxLjAgLyAzLjAsIGIyVmVjMi5BZGRWVihwYSwgYjJWZWMyLkFkZFZWKHBiLCBwYywgbmV3IGIyVmVjMigpKSwgbmV3IGIyVmVjMigpKSwgbmV3IGIyVmVjMigpKTtcclxuICAgICAgICAgIGNvbnN0IG1pZFBvaW50X3ggPSAocGEueCArIHBiLnggKyBwYy54KSAvIDMuMDtcclxuICAgICAgICAgIGNvbnN0IG1pZFBvaW50X3kgPSAocGEueSArIHBiLnkgKyBwYy55KSAvIDMuMDtcclxuICAgICAgICAgIC8vL3RyaWFkLnBhID0gYjJWZWMyLlN1YlZWKHBhLCBtaWRQb2ludCwgbmV3IGIyVmVjMigpKTtcclxuICAgICAgICAgIHRyaWFkLnBhLnggPSBwYS54IC0gbWlkUG9pbnRfeDtcclxuICAgICAgICAgIHRyaWFkLnBhLnkgPSBwYS55IC0gbWlkUG9pbnRfeTtcclxuICAgICAgICAgIC8vL3RyaWFkLnBiID0gYjJWZWMyLlN1YlZWKHBiLCBtaWRQb2ludCwgbmV3IGIyVmVjMigpKTtcclxuICAgICAgICAgIHRyaWFkLnBiLnggPSBwYi54IC0gbWlkUG9pbnRfeDtcclxuICAgICAgICAgIHRyaWFkLnBiLnkgPSBwYi55IC0gbWlkUG9pbnRfeTtcclxuICAgICAgICAgIC8vL3RyaWFkLnBjID0gYjJWZWMyLlN1YlZWKHBjLCBtaWRQb2ludCwgbmV3IGIyVmVjMigpKTtcclxuICAgICAgICAgIHRyaWFkLnBjLnggPSBwYy54IC0gbWlkUG9pbnRfeDtcclxuICAgICAgICAgIHRyaWFkLnBjLnkgPSBwYy55IC0gbWlkUG9pbnRfeTtcclxuICAgICAgICAgIHRyaWFkLmthID0gLWIyVmVjMi5Eb3RWVihkY2EsIGRhYik7XHJcbiAgICAgICAgICB0cmlhZC5rYiA9IC1iMlZlYzIuRG90VlYoZGFiLCBkYmMpO1xyXG4gICAgICAgICAgdHJpYWQua2MgPSAtYjJWZWMyLkRvdFZWKGRiYywgZGNhKTtcclxuICAgICAgICAgIHRyaWFkLnMgPSBiMlZlYzIuQ3Jvc3NWVihwYSwgcGIpICsgYjJWZWMyLkNyb3NzVlYocGIsIHBjKSArIGIyVmVjMi5Dcm9zc1ZWKHBjLCBwYSk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9O1xyXG4gICAgICBkaWFncmFtLkdldE5vZGVzKGNhbGxiYWNrKTtcclxuICAgICAgLy8vc3RkOjpzdGFibGVfc29ydChtX3RyaWFkQnVmZmVyLkJlZ2luKCksIG1fdHJpYWRCdWZmZXIuRW5kKCksIENvbXBhcmVUcmlhZEluZGljZXMpO1xyXG4gICAgICBzdGRfc3RhYmxlX3NvcnQoXHJcbiAgICAgICAgdGhpcy5tX3RyaWFkQnVmZmVyLmRhdGEsXHJcbiAgICAgICAgMCxcclxuICAgICAgICB0aGlzLm1fdHJpYWRCdWZmZXIuY291bnQsXHJcbiAgICAgICAgYjJQYXJ0aWNsZVN5c3RlbS5Db21wYXJlVHJpYWRJbmRpY2VzLFxyXG4gICAgICApO1xyXG4gICAgICAvLy9tX3RyaWFkQnVmZmVyLlVuaXF1ZShNYXRjaFRyaWFkSW5kaWNlcyk7XHJcbiAgICAgIHRoaXMubV90cmlhZEJ1ZmZlci5VbmlxdWUoYjJQYXJ0aWNsZVN5c3RlbS5NYXRjaFRyaWFkSW5kaWNlcyk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBVcGRhdGVQYWlyc0FuZFRyaWFkc19zX2RhYiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBVcGRhdGVQYWlyc0FuZFRyaWFkc19zX2RiYyA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBVcGRhdGVQYWlyc0FuZFRyaWFkc19zX2RjYSA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgVXBkYXRlUGFpcnNBbmRUcmlhZHNXaXRoUmVhY3RpdmVQYXJ0aWNsZXMoKTogdm9pZCB7XHJcbiAgICBjb25zdCBmaWx0ZXIgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9SZWFjdGl2ZUZpbHRlcih0aGlzLm1fZmxhZ3NCdWZmZXIpO1xyXG4gICAgdGhpcy5VcGRhdGVQYWlyc0FuZFRyaWFkcygwLCB0aGlzLm1fY291bnQsIGZpbHRlcik7XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpXSAmPSB+YjJQYXJ0aWNsZUZsYWcuYjJfcmVhY3RpdmVQYXJ0aWNsZTtcclxuICAgIH1cclxuICAgIHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICY9IH5iMlBhcnRpY2xlRmxhZy5iMl9yZWFjdGl2ZVBhcnRpY2xlO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIENvbXBhcmVQYWlySW5kaWNlcyhhOiBiMlBhcnRpY2xlUGFpciwgYjogYjJQYXJ0aWNsZVBhaXIpOiBib29sZWFuIHtcclxuICAgIGNvbnN0IGRpZmZBID0gYS5pbmRleEEgLSBiLmluZGV4QTtcclxuICAgIGlmIChkaWZmQSAhPT0gMCkge1xyXG4gICAgICByZXR1cm4gZGlmZkEgPCAwO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGEuaW5kZXhCIDwgYi5pbmRleEI7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTWF0Y2hQYWlySW5kaWNlcyhhOiBiMlBhcnRpY2xlUGFpciwgYjogYjJQYXJ0aWNsZVBhaXIpOiBib29sZWFuIHtcclxuICAgIHJldHVybiBhLmluZGV4QSA9PT0gYi5pbmRleEEgJiYgYS5pbmRleEIgPT09IGIuaW5kZXhCO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIENvbXBhcmVUcmlhZEluZGljZXMoYTogYjJQYXJ0aWNsZVRyaWFkLCBiOiBiMlBhcnRpY2xlVHJpYWQpOiBib29sZWFuIHtcclxuICAgIGNvbnN0IGRpZmZBID0gYS5pbmRleEEgLSBiLmluZGV4QTtcclxuICAgIGlmIChkaWZmQSAhPT0gMCkge1xyXG4gICAgICByZXR1cm4gZGlmZkEgPCAwO1xyXG4gICAgfVxyXG4gICAgY29uc3QgZGlmZkIgPSBhLmluZGV4QiAtIGIuaW5kZXhCO1xyXG4gICAgaWYgKGRpZmZCICE9PSAwKSB7XHJcbiAgICAgIHJldHVybiBkaWZmQiA8IDA7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gYS5pbmRleEMgPCBiLmluZGV4QztcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNYXRjaFRyaWFkSW5kaWNlcyhhOiBiMlBhcnRpY2xlVHJpYWQsIGI6IGIyUGFydGljbGVUcmlhZCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIGEuaW5kZXhBID09PSBiLmluZGV4QSAmJiBhLmluZGV4QiA9PT0gYi5pbmRleEIgJiYgYS5pbmRleEMgPT09IGIuaW5kZXhDO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIEluaXRpYWxpemVQYXJ0aWNsZUxpc3RzKFxyXG4gICAgZ3JvdXA6IGIyUGFydGljbGVHcm91cCxcclxuICAgIG5vZGVCdWZmZXI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZVtdLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3QgYnVmZmVySW5kZXggPSBncm91cC5HZXRCdWZmZXJJbmRleCgpO1xyXG4gICAgY29uc3QgcGFydGljbGVDb3VudCA9IGdyb3VwLkdldFBhcnRpY2xlQ291bnQoKTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcGFydGljbGVDb3VudDsgaSsrKSB7XHJcbiAgICAgIGNvbnN0IG5vZGU6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSA9IG5vZGVCdWZmZXJbaV07XHJcbiAgICAgIG5vZGUubGlzdCA9IG5vZGU7XHJcbiAgICAgIG5vZGUubmV4dCA9IG51bGw7XHJcbiAgICAgIG5vZGUuY291bnQgPSAxO1xyXG4gICAgICBub2RlLmluZGV4ID0gaSArIGJ1ZmZlckluZGV4O1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgTWVyZ2VQYXJ0aWNsZUxpc3RzSW5Db250YWN0KFxyXG4gICAgZ3JvdXA6IGIyUGFydGljbGVHcm91cCxcclxuICAgIG5vZGVCdWZmZXI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZVtdLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3QgYnVmZmVySW5kZXggPSBncm91cC5HZXRCdWZmZXJJbmRleCgpO1xyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fY29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIC8qY29uc3QgYjJQYXJ0aWNsZUNvbnRhY3QmKi9cclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4QTtcclxuICAgICAgY29uc3QgYiA9IGNvbnRhY3QuaW5kZXhCO1xyXG4gICAgICBpZiAoIWdyb3VwLkNvbnRhaW5zUGFydGljbGUoYSkgfHwgIWdyb3VwLkNvbnRhaW5zUGFydGljbGUoYikpIHtcclxuICAgICAgICBjb250aW51ZTtcclxuICAgICAgfVxyXG4gICAgICBsZXQgbGlzdEE6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSA9IG5vZGVCdWZmZXJbYSAtIGJ1ZmZlckluZGV4XS5saXN0O1xyXG4gICAgICBsZXQgbGlzdEI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSA9IG5vZGVCdWZmZXJbYiAtIGJ1ZmZlckluZGV4XS5saXN0O1xyXG4gICAgICBpZiAobGlzdEEgPT09IGxpc3RCKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuICAgICAgLy8gVG8gbWluaW1pemUgdGhlIGNvc3Qgb2YgaW5zZXJ0aW9uLCBtYWtlIHN1cmUgbGlzdEEgaXMgbG9uZ2VyIHRoYW5cclxuICAgICAgLy8gbGlzdEIuXHJcbiAgICAgIGlmIChsaXN0QS5jb3VudCA8IGxpc3RCLmNvdW50KSB7XHJcbiAgICAgICAgY29uc3QgX3RtcCA9IGxpc3RBO1xyXG4gICAgICAgIGxpc3RBID0gbGlzdEI7XHJcbiAgICAgICAgbGlzdEIgPSBfdG1wOyAvLy9iMlN3YXAobGlzdEEsIGxpc3RCKTtcclxuICAgICAgfVxyXG4gICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGxpc3RBLmNvdW50ID49IGxpc3RCLmNvdW50KTtcclxuICAgICAgYjJQYXJ0aWNsZVN5c3RlbS5NZXJnZVBhcnRpY2xlTGlzdHMobGlzdEEsIGxpc3RCKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHN0YXRpYyBNZXJnZVBhcnRpY2xlTGlzdHMoXHJcbiAgICBsaXN0QTogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlLFxyXG4gICAgbGlzdEI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSxcclxuICApOiB2b2lkIHtcclxuICAgIC8vIEluc2VydCBsaXN0QiBiZXR3ZWVuIGluZGV4IDAgYW5kIDEgb2YgbGlzdEFcclxuICAgIC8vIEV4YW1wbGU6XHJcbiAgICAvLyAgICAgbGlzdEEgPT4gYTEgPT4gYTIgPT4gYTMgPT4gbnVsbFxyXG4gICAgLy8gICAgIGxpc3RCID0+IGIxID0+IGIyID0+IG51bGxcclxuICAgIC8vIHRvXHJcbiAgICAvLyAgICAgbGlzdEEgPT4gbGlzdEIgPT4gYjEgPT4gYjIgPT4gYTEgPT4gYTIgPT4gYTMgPT4gbnVsbFxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChsaXN0QSAhPT0gbGlzdEIpO1xyXG4gICAgZm9yIChsZXQgYjogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlID0gbGlzdEI7IDsgKSB7XHJcbiAgICAgIGIubGlzdCA9IGxpc3RBO1xyXG4gICAgICBjb25zdCBuZXh0QjogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlIHwgbnVsbCA9IGIubmV4dDtcclxuICAgICAgaWYgKG5leHRCKSB7XHJcbiAgICAgICAgYiA9IG5leHRCO1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIGIubmV4dCA9IGxpc3RBLm5leHQ7XHJcbiAgICAgICAgYnJlYWs7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIGxpc3RBLm5leHQgPSBsaXN0QjtcclxuICAgIGxpc3RBLmNvdW50ICs9IGxpc3RCLmNvdW50O1xyXG4gICAgbGlzdEIuY291bnQgPSAwO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIEZpbmRMb25nZXN0UGFydGljbGVMaXN0KFxyXG4gICAgZ3JvdXA6IGIyUGFydGljbGVHcm91cCxcclxuICAgIG5vZGVCdWZmZXI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZVtdLFxyXG4gICk6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSB7XHJcbiAgICBjb25zdCBwYXJ0aWNsZUNvdW50ID0gZ3JvdXAuR2V0UGFydGljbGVDb3VudCgpO1xyXG4gICAgbGV0IHJlc3VsdDogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlID0gbm9kZUJ1ZmZlclswXTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcGFydGljbGVDb3VudDsgaSsrKSB7XHJcbiAgICAgIGNvbnN0IG5vZGU6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSA9IG5vZGVCdWZmZXJbaV07XHJcbiAgICAgIGlmIChyZXN1bHQuY291bnQgPCBub2RlLmNvdW50KSB7XHJcbiAgICAgICAgcmVzdWx0ID0gbm9kZTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgcmV0dXJuIHJlc3VsdDtcclxuICB9XHJcblxyXG4gIE1lcmdlWm9tYmllUGFydGljbGVMaXN0Tm9kZXMoXHJcbiAgICBncm91cDogYjJQYXJ0aWNsZUdyb3VwLFxyXG4gICAgbm9kZUJ1ZmZlcjogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlW10sXHJcbiAgICBzdXJ2aXZpbmdMaXN0OiBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlTGlzdE5vZGUsXHJcbiAgKTogdm9pZCB7XHJcbiAgICBjb25zdCBwYXJ0aWNsZUNvdW50ID0gZ3JvdXAuR2V0UGFydGljbGVDb3VudCgpO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBwYXJ0aWNsZUNvdW50OyBpKyspIHtcclxuICAgICAgY29uc3Qgbm9kZTogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlID0gbm9kZUJ1ZmZlcltpXTtcclxuICAgICAgaWYgKFxyXG4gICAgICAgIG5vZGUgIT09IHN1cnZpdmluZ0xpc3QgJiZcclxuICAgICAgICB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtub2RlLmluZGV4XSAmIGIyUGFydGljbGVGbGFnLmIyX3pvbWJpZVBhcnRpY2xlXHJcbiAgICAgICkge1xyXG4gICAgICAgIGIyUGFydGljbGVTeXN0ZW0uTWVyZ2VQYXJ0aWNsZUxpc3RBbmROb2RlKHN1cnZpdmluZ0xpc3QsIG5vZGUpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTWVyZ2VQYXJ0aWNsZUxpc3RBbmROb2RlKFxyXG4gICAgbGlzdDogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlLFxyXG4gICAgbm9kZTogYjJQYXJ0aWNsZVN5c3RlbV9QYXJ0aWNsZUxpc3ROb2RlLFxyXG4gICk6IHZvaWQge1xyXG4gICAgLy8gSW5zZXJ0IG5vZGUgYmV0d2VlbiBpbmRleCAwIGFuZCAxIG9mIGxpc3RcclxuICAgIC8vIEV4YW1wbGU6XHJcbiAgICAvLyAgICAgbGlzdCA9PiBhMSA9PiBhMiA9PiBhMyA9PiBudWxsXHJcbiAgICAvLyAgICAgbm9kZSA9PiBudWxsXHJcbiAgICAvLyB0b1xyXG4gICAgLy8gICAgIGxpc3QgPT4gbm9kZSA9PiBhMSA9PiBhMiA9PiBhMyA9PiBudWxsXHJcbiAgICBpZiAoQjJfREVCVUcpIHtcclxuICAgICAgYjJBc3NlcnQobm9kZSAhPT0gbGlzdCk7XHJcbiAgICAgIGIyQXNzZXJ0KG5vZGUubGlzdCA9PT0gbm9kZSk7XHJcbiAgICAgIGIyQXNzZXJ0KG5vZGUuY291bnQgPT09IDEpO1xyXG4gICAgfVxyXG4gICAgbm9kZS5saXN0ID0gbGlzdDtcclxuICAgIG5vZGUubmV4dCA9IGxpc3QubmV4dDtcclxuICAgIGxpc3QubmV4dCA9IG5vZGU7XHJcbiAgICBsaXN0LmNvdW50Kys7XHJcbiAgICBub2RlLmNvdW50ID0gMDtcclxuICB9XHJcblxyXG4gIENyZWF0ZVBhcnRpY2xlR3JvdXBzRnJvbVBhcnRpY2xlTGlzdChcclxuICAgIGdyb3VwOiBiMlBhcnRpY2xlR3JvdXAsXHJcbiAgICBub2RlQnVmZmVyOiBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlTGlzdE5vZGVbXSxcclxuICAgIHN1cnZpdmluZ0xpc3Q6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSxcclxuICApOiB2b2lkIHtcclxuICAgIGNvbnN0IHBhcnRpY2xlQ291bnQgPSBncm91cC5HZXRQYXJ0aWNsZUNvdW50KCk7XHJcbiAgICBjb25zdCBkZWYgPSBuZXcgYjJQYXJ0aWNsZUdyb3VwRGVmKCk7XHJcbiAgICBkZWYuZ3JvdXBGbGFncyA9IGdyb3VwLkdldEdyb3VwRmxhZ3MoKTtcclxuICAgIGRlZi51c2VyRGF0YSA9IGdyb3VwLkdldFVzZXJEYXRhKCk7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHBhcnRpY2xlQ291bnQ7IGkrKykge1xyXG4gICAgICBjb25zdCBsaXN0OiBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlTGlzdE5vZGUgPSBub2RlQnVmZmVyW2ldO1xyXG4gICAgICBpZiAoIWxpc3QuY291bnQgfHwgbGlzdCA9PT0gc3Vydml2aW5nTGlzdCkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQobGlzdC5saXN0ID09PSBsaXN0KTtcclxuICAgICAgY29uc3QgbmV3R3JvdXA6IGIyUGFydGljbGVHcm91cCA9IHRoaXMuQ3JlYXRlUGFydGljbGVHcm91cChkZWYpO1xyXG4gICAgICBmb3IgKGxldCBub2RlOiBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlTGlzdE5vZGUgfCBudWxsID0gbGlzdDsgbm9kZTsgbm9kZSA9IG5vZGUubmV4dCkge1xyXG4gICAgICAgIGNvbnN0IG9sZEluZGV4ID0gbm9kZS5pbmRleDtcclxuICAgICAgICBpZiAoQjJfREVCVUcpIHtcclxuICAgICAgICAgIGNvbnN0IGZsYWdzID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbb2xkSW5kZXhdO1xyXG4gICAgICAgICAgYjJBc3NlcnQoIShmbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX3pvbWJpZVBhcnRpY2xlKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGNvbnN0IG5ld0luZGV4ID0gdGhpcy5DbG9uZVBhcnRpY2xlKG9sZEluZGV4LCBuZXdHcm91cCk7XHJcbiAgICAgICAgdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbb2xkSW5kZXhdIHw9IGIyUGFydGljbGVGbGFnLmIyX3pvbWJpZVBhcnRpY2xlO1xyXG4gICAgICAgIG5vZGUuaW5kZXggPSBuZXdJbmRleDtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgVXBkYXRlUGFpcnNBbmRUcmlhZHNXaXRoUGFydGljbGVMaXN0KFxyXG4gICAgZ3JvdXA6IGIyUGFydGljbGVHcm91cCxcclxuICAgIG5vZGVCdWZmZXI6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZVtdLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3QgYnVmZmVySW5kZXggPSBncm91cC5HZXRCdWZmZXJJbmRleCgpO1xyXG4gICAgLy8gVXBkYXRlIGluZGljZXMgaW4gcGFpcnMgYW5kIHRyaWFkcy4gSWYgYW4gaW5kZXggYmVsb25ncyB0byB0aGUgZ3JvdXAsXHJcbiAgICAvLyByZXBsYWNlIGl0IHdpdGggdGhlIGNvcnJlc3BvbmRpbmcgdmFsdWUgaW4gbm9kZUJ1ZmZlci5cclxuICAgIC8vIE5vdGUgdGhhdCBub2RlQnVmZmVyIGlzIGFsbG9jYXRlZCBvbmx5IGZvciB0aGUgZ3JvdXAgYW5kIHRoZSBpbmRleCBzaG91bGRcclxuICAgIC8vIGJlIHNoaWZ0ZWQgYnkgYnVmZmVySW5kZXguXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9wYWlyQnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgcGFpciA9IHRoaXMubV9wYWlyQnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBwYWlyLmluZGV4QTtcclxuICAgICAgY29uc3QgYiA9IHBhaXIuaW5kZXhCO1xyXG4gICAgICBpZiAoZ3JvdXAuQ29udGFpbnNQYXJ0aWNsZShhKSkge1xyXG4gICAgICAgIHBhaXIuaW5kZXhBID0gbm9kZUJ1ZmZlclthIC0gYnVmZmVySW5kZXhdLmluZGV4O1xyXG4gICAgICB9XHJcbiAgICAgIGlmIChncm91cC5Db250YWluc1BhcnRpY2xlKGIpKSB7XHJcbiAgICAgICAgcGFpci5pbmRleEIgPSBub2RlQnVmZmVyW2IgLSBidWZmZXJJbmRleF0uaW5kZXg7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3RyaWFkQnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgdHJpYWQgPSB0aGlzLm1fdHJpYWRCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29uc3QgYSA9IHRyaWFkLmluZGV4QTtcclxuICAgICAgY29uc3QgYiA9IHRyaWFkLmluZGV4QjtcclxuICAgICAgY29uc3QgYyA9IHRyaWFkLmluZGV4QztcclxuICAgICAgaWYgKGdyb3VwLkNvbnRhaW5zUGFydGljbGUoYSkpIHtcclxuICAgICAgICB0cmlhZC5pbmRleEEgPSBub2RlQnVmZmVyW2EgLSBidWZmZXJJbmRleF0uaW5kZXg7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKGdyb3VwLkNvbnRhaW5zUGFydGljbGUoYikpIHtcclxuICAgICAgICB0cmlhZC5pbmRleEIgPSBub2RlQnVmZmVyW2IgLSBidWZmZXJJbmRleF0uaW5kZXg7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKGdyb3VwLkNvbnRhaW5zUGFydGljbGUoYykpIHtcclxuICAgICAgICB0cmlhZC5pbmRleEMgPSBub2RlQnVmZmVyW2MgLSBidWZmZXJJbmRleF0uaW5kZXg7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIENvbXB1dGVEZXB0aCgpOiB2b2lkIHtcclxuICAgIGNvbnN0IGNvbnRhY3RHcm91cHM6IGIyUGFydGljbGVDb250YWN0W10gPSBbXTsgLy8gVE9ETzogc3RhdGljXHJcbiAgICBsZXQgY29udGFjdEdyb3Vwc0NvdW50ID0gMDtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICBjb25zdCBiID0gY29udGFjdC5pbmRleEI7XHJcbiAgICAgIGNvbnN0IGdyb3VwQSA9IHRoaXMubV9ncm91cEJ1ZmZlclthXTtcclxuICAgICAgY29uc3QgZ3JvdXBCID0gdGhpcy5tX2dyb3VwQnVmZmVyW2JdO1xyXG4gICAgICBpZiAoXHJcbiAgICAgICAgZ3JvdXBBICYmXHJcbiAgICAgICAgZ3JvdXBBID09PSBncm91cEIgJiZcclxuICAgICAgICBncm91cEEubV9ncm91cEZsYWdzICYgYjJQYXJ0aWNsZUdyb3VwRmxhZy5iMl9wYXJ0aWNsZUdyb3VwTmVlZHNVcGRhdGVEZXB0aFxyXG4gICAgICApIHtcclxuICAgICAgICBjb250YWN0R3JvdXBzW2NvbnRhY3RHcm91cHNDb3VudCsrXSA9IGNvbnRhY3Q7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIGNvbnN0IGdyb3Vwc1RvVXBkYXRlOiBiMlBhcnRpY2xlR3JvdXBbXSA9IFtdOyAvLyBUT0RPOiBzdGF0aWNcclxuICAgIGxldCBncm91cHNUb1VwZGF0ZUNvdW50ID0gMDtcclxuICAgIGZvciAobGV0IGdyb3VwID0gdGhpcy5tX2dyb3VwTGlzdDsgZ3JvdXA7IGdyb3VwID0gZ3JvdXAuR2V0TmV4dCgpKSB7XHJcbiAgICAgIGlmIChncm91cC5tX2dyb3VwRmxhZ3MgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3BhcnRpY2xlR3JvdXBOZWVkc1VwZGF0ZURlcHRoKSB7XHJcbiAgICAgICAgZ3JvdXBzVG9VcGRhdGVbZ3JvdXBzVG9VcGRhdGVDb3VudCsrXSA9IGdyb3VwO1xyXG4gICAgICAgIHRoaXMuU2V0R3JvdXBGbGFncyhcclxuICAgICAgICAgIGdyb3VwLFxyXG4gICAgICAgICAgZ3JvdXAubV9ncm91cEZsYWdzICYgfmIyUGFydGljbGVHcm91cEZsYWcuYjJfcGFydGljbGVHcm91cE5lZWRzVXBkYXRlRGVwdGgsXHJcbiAgICAgICAgKTtcclxuICAgICAgICBmb3IgKGxldCBpID0gZ3JvdXAubV9maXJzdEluZGV4OyBpIDwgZ3JvdXAubV9sYXN0SW5kZXg7IGkrKykge1xyXG4gICAgICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlcltpXSA9IDA7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICAvLyBDb21wdXRlIHN1bSBvZiB3ZWlnaHQgb2YgY29udGFjdHMgZXhjZXB0IGJldHdlZW4gZGlmZmVyZW50IGdyb3Vwcy5cclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgY29udGFjdEdyb3Vwc0NvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IGNvbnRhY3RHcm91cHNba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4QTtcclxuICAgICAgY29uc3QgYiA9IGNvbnRhY3QuaW5kZXhCO1xyXG4gICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgIHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXJbYV0gKz0gdztcclxuICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlcltiXSArPSB3O1xyXG4gICAgfVxyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2RlcHRoQnVmZmVyICE9PSBudWxsKTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgZ3JvdXBzVG9VcGRhdGVDb3VudDsgaSsrKSB7XHJcbiAgICAgIGNvbnN0IGdyb3VwID0gZ3JvdXBzVG9VcGRhdGVbaV07XHJcbiAgICAgIGZvciAobGV0IGkgPSBncm91cC5tX2ZpcnN0SW5kZXg7IGkgPCBncm91cC5tX2xhc3RJbmRleDsgaSsrKSB7XHJcbiAgICAgICAgY29uc3QgdyA9IHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXJbaV07XHJcbiAgICAgICAgdGhpcy5tX2RlcHRoQnVmZmVyW2ldID0gdyA8IDAuOCA/IDAgOiBiMl9tYXhGbG9hdDtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgLy8gVGhlIG51bWJlciBvZiBpdGVyYXRpb25zIGlzIGVxdWFsIHRvIHBhcnRpY2xlIG51bWJlciBmcm9tIHRoZSBkZWVwZXN0XHJcbiAgICAvLyBwYXJ0aWNsZSB0byB0aGUgbmVhcmVzdCBzdXJmYWNlIHBhcnRpY2xlLCBhbmQgaW4gZ2VuZXJhbCBpdCBpcyBzbWFsbGVyXHJcbiAgICAvLyB0aGFuIHNxcnQgb2YgdG90YWwgcGFydGljbGUgbnVtYmVyLlxyXG4gICAgLy8vaW50MzIgaXRlcmF0aW9uQ291bnQgPSAoaW50MzIpYjJTcXJ0KChmbG9hdCltX2NvdW50KTtcclxuICAgIGNvbnN0IGl0ZXJhdGlvbkNvdW50ID0gYjJTcXJ0KHRoaXMubV9jb3VudCkgPj4gMDtcclxuICAgIGZvciAobGV0IHQgPSAwOyB0IDwgaXRlcmF0aW9uQ291bnQ7IHQrKykge1xyXG4gICAgICBsZXQgdXBkYXRlZCA9IGZhbHNlO1xyXG4gICAgICBmb3IgKGxldCBrID0gMDsgayA8IGNvbnRhY3RHcm91cHNDb3VudDsgaysrKSB7XHJcbiAgICAgICAgY29uc3QgY29udGFjdCA9IGNvbnRhY3RHcm91cHNba107XHJcbiAgICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgICBjb25zdCByID0gMSAtIGNvbnRhY3Qud2VpZ2h0O1xyXG4gICAgICAgIC8vL2Zsb2F0MzImIGFwMCA9IG1fZGVwdGhCdWZmZXJbYV07XHJcbiAgICAgICAgY29uc3QgYXAwID0gdGhpcy5tX2RlcHRoQnVmZmVyW2FdO1xyXG4gICAgICAgIC8vL2Zsb2F0MzImIGJwMCA9IG1fZGVwdGhCdWZmZXJbYl07XHJcbiAgICAgICAgY29uc3QgYnAwID0gdGhpcy5tX2RlcHRoQnVmZmVyW2JdO1xyXG4gICAgICAgIGNvbnN0IGFwMSA9IGJwMCArIHI7XHJcbiAgICAgICAgY29uc3QgYnAxID0gYXAwICsgcjtcclxuICAgICAgICBpZiAoYXAwID4gYXAxKSB7XHJcbiAgICAgICAgICAvLy9hcDAgPSBhcDE7XHJcbiAgICAgICAgICB0aGlzLm1fZGVwdGhCdWZmZXJbYV0gPSBhcDE7XHJcbiAgICAgICAgICB1cGRhdGVkID0gdHJ1ZTtcclxuICAgICAgICB9XHJcbiAgICAgICAgaWYgKGJwMCA+IGJwMSkge1xyXG4gICAgICAgICAgLy8vYnAwID0gYnAxO1xyXG4gICAgICAgICAgdGhpcy5tX2RlcHRoQnVmZmVyW2JdID0gYnAxO1xyXG4gICAgICAgICAgdXBkYXRlZCA9IHRydWU7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICAgIGlmICghdXBkYXRlZCkge1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGdyb3Vwc1RvVXBkYXRlQ291bnQ7IGkrKykge1xyXG4gICAgICBjb25zdCBncm91cCA9IGdyb3Vwc1RvVXBkYXRlW2ldO1xyXG4gICAgICBmb3IgKGxldCBpID0gZ3JvdXAubV9maXJzdEluZGV4OyBpIDwgZ3JvdXAubV9sYXN0SW5kZXg7IGkrKykge1xyXG4gICAgICAgIGlmICh0aGlzLm1fZGVwdGhCdWZmZXJbaV0gPCBiMl9tYXhGbG9hdCkge1xyXG4gICAgICAgICAgdGhpcy5tX2RlcHRoQnVmZmVyW2ldICo9IHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICB0aGlzLm1fZGVwdGhCdWZmZXJbaV0gPSAwO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgR2V0SW5zaWRlQm91bmRzRW51bWVyYXRvcihhYWJiOiBSZWFkb25seTxiMkFBQkI+KTogYjJQYXJ0aWNsZVN5c3RlbV9JbnNpZGVCb3VuZHNFbnVtZXJhdG9yIHtcclxuICAgIGNvbnN0IGxvd2VyVGFnID0gYjJQYXJ0aWNsZVN5c3RlbS5jb21wdXRlVGFnKFxyXG4gICAgICB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogYWFiYi5sb3dlckJvdW5kLnggLSAxLFxyXG4gICAgICB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogYWFiYi5sb3dlckJvdW5kLnkgLSAxLFxyXG4gICAgKTtcclxuICAgIGNvbnN0IHVwcGVyVGFnID0gYjJQYXJ0aWNsZVN5c3RlbS5jb21wdXRlVGFnKFxyXG4gICAgICB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogYWFiYi51cHBlckJvdW5kLnggKyAxLFxyXG4gICAgICB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogYWFiYi51cHBlckJvdW5kLnkgKyAxLFxyXG4gICAgKTtcclxuICAgIC8vL2NvbnN0IFByb3h5KiBiZWdpblByb3h5ID0gbV9wcm94eUJ1ZmZlci5CZWdpbigpO1xyXG4gICAgY29uc3QgYmVnaW5Qcm94eSA9IDA7XHJcbiAgICAvLy9jb25zdCBQcm94eSogZW5kUHJveHkgPSBtX3Byb3h5QnVmZmVyLkVuZCgpO1xyXG4gICAgY29uc3QgZW5kUHJveHkgPSB0aGlzLm1fcHJveHlCdWZmZXIuY291bnQ7XHJcbiAgICAvLy9jb25zdCBQcm94eSogZmlyc3RQcm94eSA9IHN0ZDo6bG93ZXJfYm91bmQoYmVnaW5Qcm94eSwgZW5kUHJveHksIGxvd2VyVGFnKTtcclxuICAgIGNvbnN0IGZpcnN0UHJveHkgPSBzdGRfbG93ZXJfYm91bmQoXHJcbiAgICAgIHRoaXMubV9wcm94eUJ1ZmZlci5kYXRhLFxyXG4gICAgICBiZWdpblByb3h5LFxyXG4gICAgICBlbmRQcm94eSxcclxuICAgICAgbG93ZXJUYWcsXHJcbiAgICAgIGIyUGFydGljbGVTeXN0ZW1fUHJveHkuQ29tcGFyZVByb3h5VGFnLFxyXG4gICAgKTtcclxuICAgIC8vL2NvbnN0IFByb3h5KiBsYXN0UHJveHkgPSBzdGQ6OnVwcGVyX2JvdW5kKGZpcnN0UHJveHksIGVuZFByb3h5LCB1cHBlclRhZyk7XHJcbiAgICBjb25zdCBsYXN0UHJveHkgPSBzdGRfdXBwZXJfYm91bmQoXHJcbiAgICAgIHRoaXMubV9wcm94eUJ1ZmZlci5kYXRhLFxyXG4gICAgICBiZWdpblByb3h5LFxyXG4gICAgICBlbmRQcm94eSxcclxuICAgICAgdXBwZXJUYWcsXHJcbiAgICAgIGIyUGFydGljbGVTeXN0ZW1fUHJveHkuQ29tcGFyZVRhZ1Byb3h5LFxyXG4gICAgKTtcclxuXHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGJlZ2luUHJveHkgPD0gZmlyc3RQcm94eSk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZpcnN0UHJveHkgPD0gbGFzdFByb3h5KTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQobGFzdFByb3h5IDw9IGVuZFByb3h5KTtcclxuXHJcbiAgICByZXR1cm4gbmV3IGIyUGFydGljbGVTeXN0ZW1fSW5zaWRlQm91bmRzRW51bWVyYXRvcihcclxuICAgICAgdGhpcyxcclxuICAgICAgbG93ZXJUYWcsXHJcbiAgICAgIHVwcGVyVGFnLFxyXG4gICAgICBmaXJzdFByb3h5LFxyXG4gICAgICBsYXN0UHJveHksXHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgVXBkYXRlQWxsUGFydGljbGVGbGFncygpOiB2b2lkIHtcclxuICAgIHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzID0gMDtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyBpKyspIHtcclxuICAgICAgdGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgfD0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbaV07XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fbmVlZHNVcGRhdGVBbGxQYXJ0aWNsZUZsYWdzID0gZmFsc2U7XHJcbiAgfVxyXG5cclxuICBVcGRhdGVBbGxHcm91cEZsYWdzKCk6IHZvaWQge1xyXG4gICAgdGhpcy5tX2FsbEdyb3VwRmxhZ3MgPSAwO1xyXG4gICAgZm9yIChsZXQgZ3JvdXAgPSB0aGlzLm1fZ3JvdXBMaXN0OyBncm91cDsgZ3JvdXAgPSBncm91cC5HZXROZXh0KCkpIHtcclxuICAgICAgdGhpcy5tX2FsbEdyb3VwRmxhZ3MgfD0gZ3JvdXAubV9ncm91cEZsYWdzO1xyXG4gICAgfVxyXG4gICAgdGhpcy5tX25lZWRzVXBkYXRlQWxsR3JvdXBGbGFncyA9IGZhbHNlO1xyXG4gIH1cclxuXHJcbiAgQWRkQ29udGFjdChhOiBudW1iZXIsIGI6IG51bWJlciwgY29udGFjdHM6IGIyR3Jvd2FibGVCdWZmZXI8YjJQYXJ0aWNsZUNvbnRhY3Q+KTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGNvbnRhY3RzID09PSB0aGlzLm1fY29udGFjdEJ1ZmZlcik7XHJcbiAgICBjb25zdCBmbGFnc19kYXRhID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGE7XHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgLy8vYjJWZWMyIGQgPSBtX3Bvc2l0aW9uQnVmZmVyLmRhdGFbYl0gLSBtX3Bvc2l0aW9uQnVmZmVyLmRhdGFbYV07XHJcbiAgICBjb25zdCBkID0gYjJWZWMyLlN1YlZWKHBvc19kYXRhW2JdLCBwb3NfZGF0YVthXSwgYjJQYXJ0aWNsZVN5c3RlbS5BZGRDb250YWN0X3NfZCk7XHJcbiAgICBjb25zdCBkaXN0QnRQYXJ0aWNsZXNTcSA9IGIyVmVjMi5Eb3RWVihkLCBkKTtcclxuICAgIGlmICgwIDwgZGlzdEJ0UGFydGljbGVzU3EgJiYgZGlzdEJ0UGFydGljbGVzU3EgPCB0aGlzLm1fc3F1YXJlZERpYW1ldGVyKSB7XHJcbiAgICAgIGNvbnN0IGludkQgPSBiMkludlNxcnQoZGlzdEJ0UGFydGljbGVzU3EpO1xyXG4gICAgICAvLy9iMlBhcnRpY2xlQ29udGFjdCYgY29udGFjdCA9IGNvbnRhY3RzLkFwcGVuZCgpO1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVt0aGlzLm1fY29udGFjdEJ1ZmZlci5BcHBlbmQoKV07XHJcbiAgICAgIGNvbnRhY3QuaW5kZXhBID0gYTtcclxuICAgICAgY29udGFjdC5pbmRleEIgPSBiO1xyXG4gICAgICBjb250YWN0LmZsYWdzID0gZmxhZ3NfZGF0YVthXSB8IGZsYWdzX2RhdGFbYl07XHJcbiAgICAgIGNvbnRhY3Qud2VpZ2h0ID0gMSAtIGRpc3RCdFBhcnRpY2xlc1NxICogaW52RCAqIHRoaXMubV9pbnZlcnNlRGlhbWV0ZXI7XHJcbiAgICAgIGNvbnRhY3Qubm9ybWFsLnggPSBpbnZEICogZC54O1xyXG4gICAgICBjb250YWN0Lm5vcm1hbC55ID0gaW52RCAqIGQueTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBBZGRDb250YWN0X3NfZCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgRmluZENvbnRhY3RzX1JlZmVyZW5jZShjb250YWN0czogYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlQ29udGFjdD4pOiB2b2lkIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoY29udGFjdHMgPT09IHRoaXMubV9jb250YWN0QnVmZmVyKTtcclxuICAgIGNvbnN0IGJlZ2luUHJveHkgPSAwO1xyXG4gICAgY29uc3QgZW5kUHJveHkgPSB0aGlzLm1fcHJveHlCdWZmZXIuY291bnQ7XHJcblxyXG4gICAgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQgPSAwO1xyXG4gICAgZm9yIChsZXQgYSA9IGJlZ2luUHJveHksIGMgPSBiZWdpblByb3h5OyBhIDwgZW5kUHJveHk7IGErKykge1xyXG4gICAgICBjb25zdCByaWdodFRhZyA9IGIyUGFydGljbGVTeXN0ZW0uY29tcHV0ZVJlbGF0aXZlVGFnKHRoaXMubV9wcm94eUJ1ZmZlci5kYXRhW2FdLnRhZywgMSwgMCk7XHJcbiAgICAgIGZvciAobGV0IGIgPSBhICsgMTsgYiA8IGVuZFByb3h5OyBiKyspIHtcclxuICAgICAgICBpZiAocmlnaHRUYWcgPCB0aGlzLm1fcHJveHlCdWZmZXIuZGF0YVtiXS50YWcpIHtcclxuICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuICAgICAgICB0aGlzLkFkZENvbnRhY3QoXHJcbiAgICAgICAgICB0aGlzLm1fcHJveHlCdWZmZXIuZGF0YVthXS5pbmRleCxcclxuICAgICAgICAgIHRoaXMubV9wcm94eUJ1ZmZlci5kYXRhW2JdLmluZGV4LFxyXG4gICAgICAgICAgdGhpcy5tX2NvbnRhY3RCdWZmZXIsXHJcbiAgICAgICAgKTtcclxuICAgICAgfVxyXG4gICAgICBjb25zdCBib3R0b21MZWZ0VGFnID0gYjJQYXJ0aWNsZVN5c3RlbS5jb21wdXRlUmVsYXRpdmVUYWcoXHJcbiAgICAgICAgdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGFbYV0udGFnLFxyXG4gICAgICAgIC0xLFxyXG4gICAgICAgIDEsXHJcbiAgICAgICk7XHJcbiAgICAgIGZvciAoOyBjIDwgZW5kUHJveHk7IGMrKykge1xyXG4gICAgICAgIGlmIChib3R0b21MZWZ0VGFnIDw9IHRoaXMubV9wcm94eUJ1ZmZlci5kYXRhW2NdLnRhZykge1xyXG4gICAgICAgICAgYnJlYWs7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICAgIGNvbnN0IGJvdHRvbVJpZ2h0VGFnID0gYjJQYXJ0aWNsZVN5c3RlbS5jb21wdXRlUmVsYXRpdmVUYWcoXHJcbiAgICAgICAgdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGFbYV0udGFnLFxyXG4gICAgICAgIDEsXHJcbiAgICAgICAgMSxcclxuICAgICAgKTtcclxuICAgICAgZm9yIChsZXQgYiA9IGM7IGIgPCBlbmRQcm94eTsgYisrKSB7XHJcbiAgICAgICAgaWYgKGJvdHRvbVJpZ2h0VGFnIDwgdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGFbYl0udGFnKSB7XHJcbiAgICAgICAgICBicmVhaztcclxuICAgICAgICB9XHJcbiAgICAgICAgdGhpcy5BZGRDb250YWN0KFxyXG4gICAgICAgICAgdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGFbYV0uaW5kZXgsXHJcbiAgICAgICAgICB0aGlzLm1fcHJveHlCdWZmZXIuZGF0YVtiXS5pbmRleCxcclxuICAgICAgICAgIHRoaXMubV9jb250YWN0QnVmZmVyLFxyXG4gICAgICAgICk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vL3ZvaWQgUmVvcmRlckZvckZpbmRDb250YWN0KEZpbmRDb250YWN0SW5wdXQqIHJlb3JkZXJlZCwgaW50IGFsaWduZWRDb3VudCkgY29uc3Q7XHJcbiAgLy8vdm9pZCBHYXRoZXJDaGVja3NPbmVQYXJ0aWNsZShjb25zdCB1aW50MzIgYm91bmQsIGNvbnN0IGludCBzdGFydEluZGV4LCBjb25zdCBpbnQgcGFydGljbGVJbmRleCwgaW50KiBuZXh0VW5jaGVja2VkSW5kZXgsIGIyR3Jvd2FibGVCdWZmZXI8RmluZENvbnRhY3RDaGVjaz4mIGNoZWNrcykgY29uc3Q7XHJcbiAgLy8vdm9pZCBHYXRoZXJDaGVja3MoYjJHcm93YWJsZUJ1ZmZlcjxGaW5kQ29udGFjdENoZWNrPiYgY2hlY2tzKSBjb25zdDtcclxuICAvLy92b2lkIEZpbmRDb250YWN0c19TaW1kKGIyR3Jvd2FibGVCdWZmZXI8YjJQYXJ0aWNsZUNvbnRhY3Q+JiBjb250YWN0cykgY29uc3Q7XHJcblxyXG4gIEZpbmRDb250YWN0cyhjb250YWN0czogYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlQ29udGFjdD4pOiB2b2lkIHtcclxuICAgIHRoaXMuRmluZENvbnRhY3RzX1JlZmVyZW5jZShjb250YWN0cyk7XHJcbiAgfVxyXG5cclxuICAvLy9zdGF0aWMgdm9pZCBVcGRhdGVQcm94eVRhZ3MoY29uc3QgdWludDMyKiBjb25zdCB0YWdzLCBiMkdyb3dhYmxlQnVmZmVyPFByb3h5PiYgcHJveGllcyk7XHJcbiAgLy8vc3RhdGljIGJvb2wgUHJveHlCdWZmZXJIYXNJbmRleChpbnQzMiBpbmRleCwgY29uc3QgUHJveHkqIGNvbnN0IGEsIGludCBjb3VudCk7XHJcbiAgLy8vc3RhdGljIGludCBOdW1Qcm94aWVzV2l0aFNhbWVUYWcoY29uc3QgUHJveHkqIGNvbnN0IGEsIGNvbnN0IFByb3h5KiBjb25zdCBiLCBpbnQgY291bnQpO1xyXG4gIC8vL3N0YXRpYyBib29sIEFyZVByb3h5QnVmZmVyc1RoZVNhbWUoY29uc3QgYjJHcm93YWJsZUJ1ZmZlcjxQcm94eT4mIGEsIGNvbnN0IGIyR3Jvd2FibGVCdWZmZXI8UHJveHk+JiBiKTtcclxuXHJcbiAgVXBkYXRlUHJveGllc19SZWZlcmVuY2UocHJveGllczogYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlU3lzdGVtX1Byb3h5Pik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChwcm94aWVzID09PSB0aGlzLm1fcHJveHlCdWZmZXIpO1xyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IGludl9kaWFtID0gdGhpcy5tX2ludmVyc2VEaWFtZXRlcjtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3Byb3h5QnVmZmVyLmNvdW50OyArK2spIHtcclxuICAgICAgY29uc3QgcHJveHkgPSB0aGlzLm1fcHJveHlCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29uc3QgaSA9IHByb3h5LmluZGV4O1xyXG4gICAgICBjb25zdCBwID0gcG9zX2RhdGFbaV07XHJcbiAgICAgIHByb3h5LnRhZyA9IGIyUGFydGljbGVTeXN0ZW0uY29tcHV0ZVRhZyhpbnZfZGlhbSAqIHAueCwgaW52X2RpYW0gKiBwLnkpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vdm9pZCBVcGRhdGVQcm94aWVzX1NpbWQoYjJHcm93YWJsZUJ1ZmZlcjxQcm94eT4mIHByb3hpZXMpIGNvbnN0O1xyXG5cclxuICBVcGRhdGVQcm94aWVzKHByb3hpZXM6IGIyR3Jvd2FibGVCdWZmZXI8YjJQYXJ0aWNsZVN5c3RlbV9Qcm94eT4pOiB2b2lkIHtcclxuICAgIHRoaXMuVXBkYXRlUHJveGllc19SZWZlcmVuY2UocHJveGllcyk7XHJcbiAgfVxyXG5cclxuICBTb3J0UHJveGllcyhwcm94aWVzOiBiMkdyb3dhYmxlQnVmZmVyPGIyUGFydGljbGVTeXN0ZW1fUHJveHk+KTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHByb3hpZXMgPT09IHRoaXMubV9wcm94eUJ1ZmZlcik7XHJcblxyXG4gICAgLy8vc3RkOjpzb3J0KHByb3hpZXMuQmVnaW4oKSwgcHJveGllcy5FbmQoKSk7XHJcbiAgICBzdGRfc29ydChcclxuICAgICAgdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGEsXHJcbiAgICAgIDAsXHJcbiAgICAgIHRoaXMubV9wcm94eUJ1ZmZlci5jb3VudCxcclxuICAgICAgYjJQYXJ0aWNsZVN5c3RlbV9Qcm94eS5Db21wYXJlUHJveHlQcm94eSxcclxuICAgICk7XHJcbiAgfVxyXG5cclxuICBGaWx0ZXJDb250YWN0cyhjb250YWN0czogYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlQ29udGFjdD4pOiB2b2lkIHtcclxuICAgIC8vIE9wdGlvbmFsbHkgZmlsdGVyIHRoZSBjb250YWN0LlxyXG4gICAgY29uc3QgY29udGFjdEZpbHRlciA9IHRoaXMuR2V0UGFydGljbGVDb250YWN0RmlsdGVyKCk7XHJcbiAgICBpZiAoY29udGFjdEZpbHRlciA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgLy8vIGNvbnRhY3RzLlJlbW92ZUlmKGIyUGFydGljbGVDb250YWN0UmVtb3ZlUHJlZGljYXRlKHRoaXMsIGNvbnRhY3RGaWx0ZXIpKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoY29udGFjdHMgPT09IHRoaXMubV9jb250YWN0QnVmZmVyKTtcclxuICAgIGNvbnN0IHByZWRpY2F0ZSA9IChjb250YWN0OiBiMlBhcnRpY2xlQ29udGFjdCk6IGJvb2xlYW4gPT4ge1xyXG4gICAgICByZXR1cm4gKFxyXG4gICAgICAgIChjb250YWN0LmZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfcGFydGljbGVDb250YWN0RmlsdGVyUGFydGljbGUpICE9PSAwICYmXHJcbiAgICAgICAgIWNvbnRhY3RGaWx0ZXIuU2hvdWxkQ29sbGlkZVBhcnRpY2xlUGFydGljbGUodGhpcywgY29udGFjdC5pbmRleEEsIGNvbnRhY3QuaW5kZXhCKVxyXG4gICAgICApO1xyXG4gICAgfTtcclxuICAgIHRoaXMubV9jb250YWN0QnVmZmVyLlJlbW92ZUlmKHByZWRpY2F0ZSk7XHJcbiAgfVxyXG5cclxuICBOb3RpZnlDb250YWN0TGlzdGVuZXJQcmVDb250YWN0KHBhcnRpY2xlUGFpcnM6IGIyUGFydGljbGVQYWlyU2V0KTogdm9pZCB7XHJcbiAgICBjb25zdCBjb250YWN0TGlzdGVuZXIgPSB0aGlzLkdldFBhcnRpY2xlQ29udGFjdExpc3RlbmVyKCk7XHJcbiAgICBpZiAoY29udGFjdExpc3RlbmVyID09PSBudWxsKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICAvLy9wYXJ0aWNsZVBhaXJzLkluaXRpYWxpemUobV9jb250YWN0QnVmZmVyLkJlZ2luKCksIG1fY29udGFjdEJ1ZmZlci5HZXRDb3VudCgpLCBHZXRGbGFnc0J1ZmZlcigpKTtcclxuICAgIHBhcnRpY2xlUGFpcnMuSW5pdGlhbGl6ZSh0aGlzLm1fY29udGFjdEJ1ZmZlciwgdGhpcy5tX2ZsYWdzQnVmZmVyKTtcclxuXHJcbiAgICB0aHJvdyBuZXcgRXJyb3IoKTsgLy8gVE9ETzogbm90aWZ5XHJcbiAgfVxyXG5cclxuICBOb3RpZnlDb250YWN0TGlzdGVuZXJQb3N0Q29udGFjdChwYXJ0aWNsZVBhaXJzOiBiMlBhcnRpY2xlUGFpclNldCk6IHZvaWQge1xyXG4gICAgY29uc3QgY29udGFjdExpc3RlbmVyID0gdGhpcy5HZXRQYXJ0aWNsZUNvbnRhY3RMaXN0ZW5lcigpO1xyXG4gICAgaWYgKGNvbnRhY3RMaXN0ZW5lciA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgLy8gTG9vcCB0aHJvdWdoIGFsbCBuZXcgY29udGFjdHMsIHJlcG9ydGluZyBhbnkgbmV3IG9uZXMsIGFuZFxyXG4gICAgLy8gXCJpbnZhbGlkYXRpbmdcIiB0aGUgb25lcyB0aGF0IHN0aWxsIGV4aXN0LlxyXG4gICAgLy8vY29uc3QgYjJQYXJ0aWNsZUNvbnRhY3QqIGNvbnN0IGVuZENvbnRhY3QgPSBtX2NvbnRhY3RCdWZmZXIuRW5kKCk7XHJcbiAgICAvLy9mb3IgKGIyUGFydGljbGVDb250YWN0KiBjb250YWN0ID0gbV9jb250YWN0QnVmZmVyLkJlZ2luKCk7IGNvbnRhY3QgPCBlbmRDb250YWN0OyArK2NvbnRhY3QpXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50OyArK2spIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIC8vL1BhcnRpY2xlUGFpciBwYWlyO1xyXG4gICAgICAvLy9wYWlyLmZpcnN0ID0gY29udGFjdC5HZXRJbmRleEEoKTtcclxuICAgICAgLy8vcGFpci5zZWNvbmQgPSBjb250YWN0LkdldEluZGV4QigpO1xyXG4gICAgICAvLy9jb25zdCBpbnQzMiBpdGVtSW5kZXggPSBwYXJ0aWNsZVBhaXJzLkZpbmQocGFpcik7XHJcbiAgICAgIGNvbnN0IGl0ZW1JbmRleCA9IC0xOyAvLyBUT0RPXHJcbiAgICAgIGlmIChpdGVtSW5kZXggPj0gMCkge1xyXG4gICAgICAgIC8vIEFscmVhZHkgdG91Y2hpbmcsIGlnbm9yZSB0aGlzIGNvbnRhY3QuXHJcbiAgICAgICAgcGFydGljbGVQYWlycy5JbnZhbGlkYXRlKGl0ZW1JbmRleCk7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgLy8gSnVzdCBzdGFydGVkIHRvdWNoaW5nLCBpbmZvcm0gdGhlIGxpc3RlbmVyLlxyXG4gICAgICAgIGNvbnRhY3RMaXN0ZW5lci5CZWdpbkNvbnRhY3RQYXJ0aWNsZVBhcnRpY2xlKHRoaXMsIGNvbnRhY3QpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gUmVwb3J0IHBhcnRpY2xlcyB0aGF0IGFyZSBubyBsb25nZXIgdG91Y2hpbmcuXHJcbiAgICAvLyBUaGF0IGlzLCBhbnkgcGFpcnMgdGhhdCB3ZXJlIG5vdCBpbnZhbGlkYXRlZCBhYm92ZS5cclxuICAgIC8vL2NvbnN0IGludDMyIHBhaXJDb3VudCA9IHBhcnRpY2xlUGFpcnMuR2V0Q291bnQoKTtcclxuICAgIC8vL2NvbnN0IFBhcnRpY2xlUGFpciogY29uc3QgcGFpcnMgPSBwYXJ0aWNsZVBhaXJzLkdldEJ1ZmZlcigpO1xyXG4gICAgLy8vY29uc3QgaW50OCogY29uc3QgdmFsaWQgPSBwYXJ0aWNsZVBhaXJzLkdldFZhbGlkQnVmZmVyKCk7XHJcbiAgICAvLy9mb3IgKGludDMyIGkgPSAwOyBpIDwgcGFpckNvdW50OyArK2kpXHJcbiAgICAvLy97XHJcbiAgICAvLy8gIGlmICh2YWxpZFtpXSlcclxuICAgIC8vLyAge1xyXG4gICAgLy8vICAgIGNvbnRhY3RMaXN0ZW5lci5FbmRDb250YWN0UGFydGljbGVQYXJ0aWNsZSh0aGlzLCBwYWlyc1tpXS5maXJzdCwgcGFpcnNbaV0uc2Vjb25kKTtcclxuICAgIC8vLyAgfVxyXG4gICAgLy8vfVxyXG5cclxuICAgIHRocm93IG5ldyBFcnJvcigpOyAvLyBUT0RPOiBub3RpZnlcclxuICB9XHJcblxyXG4gIHN0YXRpYyBiMlBhcnRpY2xlQ29udGFjdElzWm9tYmllKGNvbnRhY3Q6IGIyUGFydGljbGVDb250YWN0KTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gKGNvbnRhY3QuZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl96b21iaWVQYXJ0aWNsZSkgPT09IGIyUGFydGljbGVGbGFnLmIyX3pvbWJpZVBhcnRpY2xlO1xyXG4gIH1cclxuXHJcbiAgVXBkYXRlQ29udGFjdHMoZXhjZXB0Wm9tYmllOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICB0aGlzLlVwZGF0ZVByb3hpZXModGhpcy5tX3Byb3h5QnVmZmVyKTtcclxuICAgIHRoaXMuU29ydFByb3hpZXModGhpcy5tX3Byb3h5QnVmZmVyKTtcclxuXHJcbiAgICBjb25zdCBwYXJ0aWNsZVBhaXJzID0gbmV3IGIyUGFydGljbGVQYWlyU2V0KCk7IC8vIFRPRE86IHN0YXRpY1xyXG4gICAgdGhpcy5Ob3RpZnlDb250YWN0TGlzdGVuZXJQcmVDb250YWN0KHBhcnRpY2xlUGFpcnMpO1xyXG5cclxuICAgIHRoaXMuRmluZENvbnRhY3RzKHRoaXMubV9jb250YWN0QnVmZmVyKTtcclxuICAgIHRoaXMuRmlsdGVyQ29udGFjdHModGhpcy5tX2NvbnRhY3RCdWZmZXIpO1xyXG5cclxuICAgIHRoaXMuTm90aWZ5Q29udGFjdExpc3RlbmVyUG9zdENvbnRhY3QocGFydGljbGVQYWlycyk7XHJcblxyXG4gICAgaWYgKGV4Y2VwdFpvbWJpZSkge1xyXG4gICAgICB0aGlzLm1fY29udGFjdEJ1ZmZlci5SZW1vdmVJZihiMlBhcnRpY2xlU3lzdGVtLmIyUGFydGljbGVDb250YWN0SXNab21iaWUpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgTm90aWZ5Qm9keUNvbnRhY3RMaXN0ZW5lclByZUNvbnRhY3QoZml4dHVyZVNldDogYjJQYXJ0aWNsZVN5c3RlbV9GaXh0dXJlUGFydGljbGVTZXQpOiB2b2lkIHtcclxuICAgIGNvbnN0IGNvbnRhY3RMaXN0ZW5lciA9IHRoaXMuR2V0Rml4dHVyZUNvbnRhY3RMaXN0ZW5lcigpO1xyXG4gICAgaWYgKGNvbnRhY3RMaXN0ZW5lciA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgLy8vZml4dHVyZVNldC5Jbml0aWFsaXplKG1fYm9keUNvbnRhY3RCdWZmZXIuQmVnaW4oKSwgbV9ib2R5Q29udGFjdEJ1ZmZlci5HZXRDb3VudCgpLCBHZXRGbGFnc0J1ZmZlcigpKTtcclxuICAgIGZpeHR1cmVTZXQuSW5pdGlhbGl6ZSh0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIsIHRoaXMubV9mbGFnc0J1ZmZlcik7XHJcblxyXG4gICAgdGhyb3cgbmV3IEVycm9yKCk7IC8vIFRPRE86IG5vdGlmeVxyXG4gIH1cclxuXHJcbiAgTm90aWZ5Qm9keUNvbnRhY3RMaXN0ZW5lclBvc3RDb250YWN0KGZpeHR1cmVTZXQ6IGIyUGFydGljbGVTeXN0ZW1fRml4dHVyZVBhcnRpY2xlU2V0KTogdm9pZCB7XHJcbiAgICBjb25zdCBjb250YWN0TGlzdGVuZXIgPSB0aGlzLkdldEZpeHR1cmVDb250YWN0TGlzdGVuZXIoKTtcclxuICAgIGlmIChjb250YWN0TGlzdGVuZXIgPT09IG51bGwpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIExvb3AgdGhyb3VnaCBhbGwgbmV3IGNvbnRhY3RzLCByZXBvcnRpbmcgYW55IG5ldyBvbmVzLCBhbmRcclxuICAgIC8vIFwiaW52YWxpZGF0aW5nXCIgdGhlIG9uZXMgdGhhdCBzdGlsbCBleGlzdC5cclxuICAgIC8vL2ZvciAoYjJQYXJ0aWNsZUJvZHlDb250YWN0KiBjb250YWN0ID0gbV9ib2R5Q29udGFjdEJ1ZmZlci5CZWdpbigpOyBjb250YWN0ICE9PSBtX2JvZHlDb250YWN0QnVmZmVyLkVuZCgpOyArK2NvbnRhY3QpXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IGNvbnRhY3QgPSB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChjb250YWN0ICE9PSBudWxsKTtcclxuICAgICAgLy8vRml4dHVyZVBhcnRpY2xlIGZpeHR1cmVQYXJ0aWNsZVRvRmluZDtcclxuICAgICAgLy8vZml4dHVyZVBhcnRpY2xlVG9GaW5kLmZpcnN0ID0gY29udGFjdC5maXh0dXJlO1xyXG4gICAgICAvLy9maXh0dXJlUGFydGljbGVUb0ZpbmQuc2Vjb25kID0gY29udGFjdC5pbmRleDtcclxuICAgICAgLy8vY29uc3QgaW50MzIgaW5kZXggPSBmaXh0dXJlU2V0LkZpbmQoZml4dHVyZVBhcnRpY2xlVG9GaW5kKTtcclxuICAgICAgY29uc3QgaW5kZXggPSAtMTsgLy8gVE9ET1xyXG4gICAgICBpZiAoaW5kZXggPj0gMCkge1xyXG4gICAgICAgIC8vIEFscmVhZHkgdG91Y2hpbmcgcmVtb3ZlIHRoaXMgZnJvbSB0aGUgc2V0LlxyXG4gICAgICAgIGZpeHR1cmVTZXQuSW52YWxpZGF0ZShpbmRleCk7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgLy8gSnVzdCBzdGFydGVkIHRvdWNoaW5nLCByZXBvcnQgaXQhXHJcbiAgICAgICAgY29udGFjdExpc3RlbmVyLkJlZ2luQ29udGFjdEZpeHR1cmVQYXJ0aWNsZSh0aGlzLCBjb250YWN0KTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIElmIHRoZSBjb250YWN0IGxpc3RlbmVyIGlzIGVuYWJsZWQsIHJlcG9ydCBhbGwgZml4dHVyZXMgdGhhdCBhcmUgbm9cclxuICAgIC8vIGxvbmdlciBpbiBjb250YWN0IHdpdGggcGFydGljbGVzLlxyXG4gICAgLy8vY29uc3QgRml4dHVyZVBhcnRpY2xlKiBjb25zdCBmaXh0dXJlUGFydGljbGVzID0gZml4dHVyZVNldC5HZXRCdWZmZXIoKTtcclxuICAgIC8vL2NvbnN0IGludDgqIGNvbnN0IGZpeHR1cmVQYXJ0aWNsZXNWYWxpZCA9IGZpeHR1cmVTZXQuR2V0VmFsaWRCdWZmZXIoKTtcclxuICAgIC8vL2NvbnN0IGludDMyIGZpeHR1cmVQYXJ0aWNsZUNvdW50ID0gZml4dHVyZVNldC5HZXRDb3VudCgpO1xyXG4gICAgLy8vZm9yIChpbnQzMiBpID0gMDsgaSA8IGZpeHR1cmVQYXJ0aWNsZUNvdW50OyArK2kpXHJcbiAgICAvLy97XHJcbiAgICAvLy8gIGlmIChmaXh0dXJlUGFydGljbGVzVmFsaWRbaV0pXHJcbiAgICAvLy8gIHtcclxuICAgIC8vLyAgICBjb25zdCBGaXh0dXJlUGFydGljbGUqIGNvbnN0IGZpeHR1cmVQYXJ0aWNsZSA9ICZmaXh0dXJlUGFydGljbGVzW2ldO1xyXG4gICAgLy8vICAgIGNvbnRhY3RMaXN0ZW5lci5FbmRDb250YWN0Rml4dHVyZVBhcnRpY2xlKGZpeHR1cmVQYXJ0aWNsZS5maXJzdCwgdGhpcywgZml4dHVyZVBhcnRpY2xlLnNlY29uZCk7XHJcbiAgICAvLy8gIH1cclxuICAgIC8vL31cclxuXHJcbiAgICB0aHJvdyBuZXcgRXJyb3IoKTsgLy8gVE9ETzogbm90aWZ5XHJcbiAgfVxyXG5cclxuICBVcGRhdGVCb2R5Q29udGFjdHMoKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX2FhYmIgPSBiMlBhcnRpY2xlU3lzdGVtLlVwZGF0ZUJvZHlDb250YWN0c19zX2FhYmI7XHJcblxyXG4gICAgLy8gSWYgdGhlIHBhcnRpY2xlIGNvbnRhY3QgbGlzdGVuZXIgaXMgZW5hYmxlZCwgZ2VuZXJhdGUgYSBzZXQgb2ZcclxuICAgIC8vIGZpeHR1cmUgLyBwYXJ0aWNsZSBjb250YWN0cy5cclxuICAgIGNvbnN0IGZpeHR1cmVTZXQgPSBuZXcgYjJQYXJ0aWNsZVN5c3RlbV9GaXh0dXJlUGFydGljbGVTZXQoKTsgLy8gVE9ETzogc3RhdGljXHJcbiAgICB0aGlzLk5vdGlmeUJvZHlDb250YWN0TGlzdGVuZXJQcmVDb250YWN0KGZpeHR1cmVTZXQpO1xyXG5cclxuICAgIGlmICh0aGlzLm1fc3R1Y2tUaHJlc2hvbGQgPiAwKSB7XHJcbiAgICAgIGNvbnN0IHBhcnRpY2xlQ291bnQgPSB0aGlzLkdldFBhcnRpY2xlQ291bnQoKTtcclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBwYXJ0aWNsZUNvdW50OyBpKyspIHtcclxuICAgICAgICAvLyBEZXRlY3Qgc3R1Y2sgcGFydGljbGVzLCBzZWUgY29tbWVudCBpblxyXG4gICAgICAgIC8vIGIyUGFydGljbGVTeXN0ZW06OkRldGVjdFN0dWNrUGFydGljbGUoKVxyXG4gICAgICAgIHRoaXMubV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGFbaV0gPSAwO1xyXG4gICAgICAgIGlmICh0aGlzLm1fdGltZXN0YW1wID4gdGhpcy5tX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YVtpXSArIDEpIHtcclxuICAgICAgICAgIHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhW2ldID0gMDtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5TZXRDb3VudCgwKTtcclxuICAgIHRoaXMubV9zdHVja1BhcnRpY2xlQnVmZmVyLlNldENvdW50KDApO1xyXG5cclxuICAgIGNvbnN0IGFhYmIgPSBzX2FhYmI7XHJcbiAgICB0aGlzLkNvbXB1dGVBQUJCKGFhYmIpO1xyXG5cclxuICAgIGlmICh0aGlzLlVwZGF0ZUJvZHlDb250YWN0c19jYWxsYmFjayA9PT0gbnVsbCkge1xyXG4gICAgICB0aGlzLlVwZGF0ZUJvZHlDb250YWN0c19jYWxsYmFjayA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtX1VwZGF0ZUJvZHlDb250YWN0c0NhbGxiYWNrKHRoaXMpO1xyXG4gICAgfVxyXG4gICAgY29uc3QgY2FsbGJhY2sgPSB0aGlzLlVwZGF0ZUJvZHlDb250YWN0c19jYWxsYmFjaztcclxuICAgIGNhbGxiYWNrLm1fY29udGFjdEZpbHRlciA9IHRoaXMuR2V0Rml4dHVyZUNvbnRhY3RGaWx0ZXIoKTtcclxuICAgIHRoaXMubV93b3JsZC5RdWVyeUFBQkIoY2FsbGJhY2ssIGFhYmIpO1xyXG5cclxuICAgIGlmICh0aGlzLm1fZGVmLnN0cmljdENvbnRhY3RDaGVjaykge1xyXG4gICAgICB0aGlzLlJlbW92ZVNwdXJpb3VzQm9keUNvbnRhY3RzKCk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5Ob3RpZnlCb2R5Q29udGFjdExpc3RlbmVyUG9zdENvbnRhY3QoZml4dHVyZVNldCk7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgVXBkYXRlQm9keUNvbnRhY3RzX3NfYWFiYiA9IG5ldyBiMkFBQkIoKTtcclxuICBVcGRhdGVCb2R5Q29udGFjdHNfY2FsbGJhY2s6IGIyUGFydGljbGVTeXN0ZW1fVXBkYXRlQm9keUNvbnRhY3RzQ2FsbGJhY2sgfCBudWxsID0gbnVsbDtcclxuXHJcbiAgU29sdmUoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19zdWJTdGVwID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZV9zX3N1YlN0ZXA7XHJcbiAgICBpZiAodGhpcy5tX2NvdW50ID09PSAwKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuICAgIC8vIElmIHBhcnRpY2xlIGxpZmV0aW1lcyBhcmUgZW5hYmxlZCwgZGVzdHJveSBwYXJ0aWNsZXMgdGhhdCBhcmUgdG9vIG9sZC5cclxuICAgIGlmICh0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSkge1xyXG4gICAgICB0aGlzLlNvbHZlTGlmZXRpbWVzKHN0ZXApO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfem9tYmllUGFydGljbGUpIHtcclxuICAgICAgdGhpcy5Tb2x2ZVpvbWJpZSgpO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9uZWVkc1VwZGF0ZUFsbFBhcnRpY2xlRmxhZ3MpIHtcclxuICAgICAgdGhpcy5VcGRhdGVBbGxQYXJ0aWNsZUZsYWdzKCk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX25lZWRzVXBkYXRlQWxsR3JvdXBGbGFncykge1xyXG4gICAgICB0aGlzLlVwZGF0ZUFsbEdyb3VwRmxhZ3MoKTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fcGF1c2VkKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuICAgIGZvciAoXHJcbiAgICAgIHRoaXMubV9pdGVyYXRpb25JbmRleCA9IDA7XHJcbiAgICAgIHRoaXMubV9pdGVyYXRpb25JbmRleCA8IHN0ZXAucGFydGljbGVJdGVyYXRpb25zO1xyXG4gICAgICB0aGlzLm1faXRlcmF0aW9uSW5kZXgrK1xyXG4gICAgKSB7XHJcbiAgICAgICsrdGhpcy5tX3RpbWVzdGFtcDtcclxuICAgICAgY29uc3Qgc3ViU3RlcCA9IHNfc3ViU3RlcC5Db3B5KHN0ZXApO1xyXG4gICAgICBzdWJTdGVwLmR0IC89IHN0ZXAucGFydGljbGVJdGVyYXRpb25zO1xyXG4gICAgICBzdWJTdGVwLmludl9kdCAqPSBzdGVwLnBhcnRpY2xlSXRlcmF0aW9ucztcclxuICAgICAgdGhpcy5VcGRhdGVDb250YWN0cyhmYWxzZSk7XHJcbiAgICAgIHRoaXMuVXBkYXRlQm9keUNvbnRhY3RzKCk7XHJcbiAgICAgIHRoaXMuQ29tcHV0ZVdlaWdodCgpO1xyXG4gICAgICBpZiAodGhpcy5tX2FsbEdyb3VwRmxhZ3MgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3BhcnRpY2xlR3JvdXBOZWVkc1VwZGF0ZURlcHRoKSB7XHJcbiAgICAgICAgdGhpcy5Db21wdXRlRGVwdGgoKTtcclxuICAgICAgfVxyXG4gICAgICBpZiAodGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9yZWFjdGl2ZVBhcnRpY2xlKSB7XHJcbiAgICAgICAgdGhpcy5VcGRhdGVQYWlyc0FuZFRyaWFkc1dpdGhSZWFjdGl2ZVBhcnRpY2xlcygpO1xyXG4gICAgICB9XHJcbiAgICAgIGlmICh0aGlzLm1faGFzRm9yY2UpIHtcclxuICAgICAgICB0aGlzLlNvbHZlRm9yY2Uoc3ViU3RlcCk7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfdmlzY291c1BhcnRpY2xlKSB7XHJcbiAgICAgICAgdGhpcy5Tb2x2ZVZpc2NvdXMoKTtcclxuICAgICAgfVxyXG4gICAgICBpZiAodGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9yZXB1bHNpdmVQYXJ0aWNsZSkge1xyXG4gICAgICAgIHRoaXMuU29sdmVSZXB1bHNpdmUoc3ViU3RlcCk7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfcG93ZGVyUGFydGljbGUpIHtcclxuICAgICAgICB0aGlzLlNvbHZlUG93ZGVyKHN1YlN0ZXApO1xyXG4gICAgICB9XHJcbiAgICAgIGlmICh0aGlzLm1fYWxsUGFydGljbGVGbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX3RlbnNpbGVQYXJ0aWNsZSkge1xyXG4gICAgICAgIHRoaXMuU29sdmVUZW5zaWxlKHN1YlN0ZXApO1xyXG4gICAgICB9XHJcbiAgICAgIGlmICh0aGlzLm1fYWxsR3JvdXBGbGFncyAmIGIyUGFydGljbGVHcm91cEZsYWcuYjJfc29saWRQYXJ0aWNsZUdyb3VwKSB7XHJcbiAgICAgICAgdGhpcy5Tb2x2ZVNvbGlkKHN1YlN0ZXApO1xyXG4gICAgICB9XHJcbiAgICAgIGlmICh0aGlzLm1fYWxsUGFydGljbGVGbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX2NvbG9yTWl4aW5nUGFydGljbGUpIHtcclxuICAgICAgICB0aGlzLlNvbHZlQ29sb3JNaXhpbmcoKTtcclxuICAgICAgfVxyXG4gICAgICB0aGlzLlNvbHZlR3Jhdml0eShzdWJTdGVwKTtcclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfc3RhdGljUHJlc3N1cmVQYXJ0aWNsZSkge1xyXG4gICAgICAgIHRoaXMuU29sdmVTdGF0aWNQcmVzc3VyZShzdWJTdGVwKTtcclxuICAgICAgfVxyXG4gICAgICB0aGlzLlNvbHZlUHJlc3N1cmUoc3ViU3RlcCk7XHJcbiAgICAgIHRoaXMuU29sdmVEYW1waW5nKHN1YlN0ZXApO1xyXG4gICAgICBpZiAodGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgJiBiMlBhcnRpY2xlU3lzdGVtLmtfZXh0cmFEYW1waW5nRmxhZ3MpIHtcclxuICAgICAgICB0aGlzLlNvbHZlRXh0cmFEYW1waW5nKCk7XHJcbiAgICAgIH1cclxuICAgICAgLy8gU29sdmVFbGFzdGljIGFuZCBTb2x2ZVNwcmluZyByZWZlciB0aGUgY3VycmVudCB2ZWxvY2l0aWVzIGZvclxyXG4gICAgICAvLyBudW1lcmljYWwgc3RhYmlsaXR5LCB0aGV5IHNob3VsZCBiZSBjYWxsZWQgYXMgbGF0ZSBhcyBwb3NzaWJsZS5cclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfZWxhc3RpY1BhcnRpY2xlKSB7XHJcbiAgICAgICAgdGhpcy5Tb2x2ZUVsYXN0aWMoc3ViU3RlcCk7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfc3ByaW5nUGFydGljbGUpIHtcclxuICAgICAgICB0aGlzLlNvbHZlU3ByaW5nKHN1YlN0ZXApO1xyXG4gICAgICB9XHJcbiAgICAgIHRoaXMuTGltaXRWZWxvY2l0eShzdWJTdGVwKTtcclxuICAgICAgaWYgKHRoaXMubV9hbGxHcm91cEZsYWdzICYgYjJQYXJ0aWNsZUdyb3VwRmxhZy5iMl9yaWdpZFBhcnRpY2xlR3JvdXApIHtcclxuICAgICAgICB0aGlzLlNvbHZlUmlnaWREYW1waW5nKCk7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfYmFycmllclBhcnRpY2xlKSB7XHJcbiAgICAgICAgdGhpcy5Tb2x2ZUJhcnJpZXIoc3ViU3RlcCk7XHJcbiAgICAgIH1cclxuICAgICAgLy8gU29sdmVDb2xsaXNpb24sIFNvbHZlUmlnaWQgYW5kIFNvbHZlV2FsbCBzaG91bGQgYmUgY2FsbGVkIGFmdGVyXHJcbiAgICAgIC8vIG90aGVyIGZvcmNlIGZ1bmN0aW9ucyBiZWNhdXNlIHRoZXkgbWF5IHJlcXVpcmUgcGFydGljbGVzIHRvIGhhdmVcclxuICAgICAgLy8gc3BlY2lmaWMgdmVsb2NpdGllcy5cclxuICAgICAgdGhpcy5Tb2x2ZUNvbGxpc2lvbihzdWJTdGVwKTtcclxuICAgICAgaWYgKHRoaXMubV9hbGxHcm91cEZsYWdzICYgYjJQYXJ0aWNsZUdyb3VwRmxhZy5iMl9yaWdpZFBhcnRpY2xlR3JvdXApIHtcclxuICAgICAgICB0aGlzLlNvbHZlUmlnaWQoc3ViU3RlcCk7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9hbGxQYXJ0aWNsZUZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfd2FsbFBhcnRpY2xlKSB7XHJcbiAgICAgICAgdGhpcy5Tb2x2ZVdhbGwoKTtcclxuICAgICAgfVxyXG4gICAgICAvLyBUaGUgcGFydGljbGUgcG9zaXRpb25zIGNhbiBiZSB1cGRhdGVkIG9ubHkgYXQgdGhlIGVuZCBvZiBzdWJzdGVwLlxyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgaSsrKSB7XHJcbiAgICAgICAgLy8vbV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2ldICs9IHN1YlN0ZXAuZHQgKiBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbaV07XHJcbiAgICAgICAgdGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGFbaV0uU2VsZk11bEFkZChzdWJTdGVwLmR0LCB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YVtpXSk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZV9zX3N1YlN0ZXAgPSBuZXcgYjJUaW1lU3RlcCgpO1xyXG5cclxuICBTb2x2ZUNvbGxpc2lvbihzdGVwOiBiMlRpbWVTdGVwKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX2FhYmIgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlQ29sbGlzaW9uX3NfYWFiYjtcclxuICAgIGNvbnN0IHBvc19kYXRhID0gdGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGE7XHJcbiAgICBjb25zdCB2ZWxfZGF0YSA9IHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhO1xyXG5cclxuICAgIC8vIFRoaXMgZnVuY3Rpb24gZGV0ZWN0cyBwYXJ0aWNsZXMgd2hpY2ggYXJlIGNyb3NzaW5nIGJvdW5kYXJ5IG9mIGJvZGllc1xyXG4gICAgLy8gYW5kIG1vZGlmaWVzIHZlbG9jaXRpZXMgb2YgdGhlbSBzbyB0aGF0IHRoZXkgd2lsbCBtb3ZlIGp1c3QgaW4gZnJvbnQgb2ZcclxuICAgIC8vIGJvdW5kYXJ5LiBUaGlzIGZ1bmN0aW9uIGZ1bmN0aW9uIGFsc28gYXBwbGllcyB0aGUgcmVhY3Rpb24gZm9yY2UgdG9cclxuICAgIC8vIGJvZGllcyBhcyBwcmVjaXNlbHkgYXMgdGhlIG51bWVyaWNhbCBzdGFiaWxpdHkgaXMga2VwdC5cclxuICAgIGNvbnN0IGFhYmIgPSBzX2FhYmI7XHJcbiAgICBhYWJiLmxvd2VyQm91bmQueCA9ICtiMl9tYXhGbG9hdDtcclxuICAgIGFhYmIubG93ZXJCb3VuZC55ID0gK2IyX21heEZsb2F0O1xyXG4gICAgYWFiYi51cHBlckJvdW5kLnggPSAtYjJfbWF4RmxvYXQ7XHJcbiAgICBhYWJiLnVwcGVyQm91bmQueSA9IC1iMl9tYXhGbG9hdDtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyBpKyspIHtcclxuICAgICAgY29uc3QgdiA9IHZlbF9kYXRhW2ldO1xyXG4gICAgICBjb25zdCBwMSA9IHBvc19kYXRhW2ldO1xyXG4gICAgICAvLy9sZXQgcDIgPSBwMSArIHN0ZXAuZHQgKiB2O1xyXG4gICAgICBjb25zdCBwMl94ID0gcDEueCArIHN0ZXAuZHQgKiB2Lng7XHJcbiAgICAgIGNvbnN0IHAyX3kgPSBwMS55ICsgc3RlcC5kdCAqIHYueTtcclxuICAgICAgLy8vYWFiYi5sb3dlckJvdW5kID0gYjJNaW4oYWFiYi5sb3dlckJvdW5kLCBiMk1pbihwMSwgcDIpKTtcclxuICAgICAgYWFiYi5sb3dlckJvdW5kLnggPSBiMk1pbihhYWJiLmxvd2VyQm91bmQueCwgYjJNaW4ocDEueCwgcDJfeCkpO1xyXG4gICAgICBhYWJiLmxvd2VyQm91bmQueSA9IGIyTWluKGFhYmIubG93ZXJCb3VuZC55LCBiMk1pbihwMS55LCBwMl95KSk7XHJcbiAgICAgIC8vL2FhYmIudXBwZXJCb3VuZCA9IGIyTWF4KGFhYmIudXBwZXJCb3VuZCwgYjJNYXgocDEsIHAyKSk7XHJcbiAgICAgIGFhYmIudXBwZXJCb3VuZC54ID0gYjJNYXgoYWFiYi51cHBlckJvdW5kLngsIGIyTWF4KHAxLngsIHAyX3gpKTtcclxuICAgICAgYWFiYi51cHBlckJvdW5kLnkgPSBiMk1heChhYWJiLnVwcGVyQm91bmQueSwgYjJNYXgocDEueSwgcDJfeSkpO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMuU29sdmVDb2xsaXNpb25fY2FsbGJhY2sgPT09IG51bGwpIHtcclxuICAgICAgdGhpcy5Tb2x2ZUNvbGxpc2lvbl9jYWxsYmFjayA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtX1NvbHZlQ29sbGlzaW9uQ2FsbGJhY2sodGhpcywgc3RlcCk7XHJcbiAgICB9XHJcbiAgICBjb25zdCBjYWxsYmFjayA9IHRoaXMuU29sdmVDb2xsaXNpb25fY2FsbGJhY2s7XHJcbiAgICBjYWxsYmFjay5tX3N0ZXAgPSBzdGVwO1xyXG4gICAgdGhpcy5tX3dvcmxkLlF1ZXJ5QUFCQihjYWxsYmFjaywgYWFiYik7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVDb2xsaXNpb25fc19hYWJiID0gbmV3IGIyQUFCQigpO1xyXG4gIFNvbHZlQ29sbGlzaW9uX2NhbGxiYWNrOiBiMlBhcnRpY2xlU3lzdGVtX1NvbHZlQ29sbGlzaW9uQ2FsbGJhY2sgfCBudWxsID0gbnVsbDtcclxuXHJcbiAgTGltaXRWZWxvY2l0eShzdGVwOiBiMlRpbWVTdGVwKTogdm9pZCB7XHJcbiAgICBjb25zdCB2ZWxfZGF0YSA9IHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgY3JpdGljYWxWZWxvY2l0eVNxdWFyZWQgPSB0aGlzLkdldENyaXRpY2FsVmVsb2NpdHlTcXVhcmVkKHN0ZXApO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICBjb25zdCB2ID0gdmVsX2RhdGFbaV07XHJcbiAgICAgIGNvbnN0IHYyID0gYjJWZWMyLkRvdFZWKHYsIHYpO1xyXG4gICAgICBpZiAodjIgPiBjcml0aWNhbFZlbG9jaXR5U3F1YXJlZCkge1xyXG4gICAgICAgIC8vL3YgKj0gYjJTcXJ0KGNyaXRpY2FsVmVsb2NpdHlTcXVhcmVkIC8gdjIpO1xyXG4gICAgICAgIHYuU2VsZk11bChiMlNxcnQoY3JpdGljYWxWZWxvY2l0eVNxdWFyZWQgLyB2MikpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTb2x2ZUdyYXZpdHkoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19ncmF2aXR5ID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUdyYXZpdHlfc19ncmF2aXR5O1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIC8vL2IyVmVjMiBncmF2aXR5ID0gc3RlcC5kdCAqIG1fZGVmLmdyYXZpdHlTY2FsZSAqIG1fd29ybGQuR2V0R3Jhdml0eSgpO1xyXG4gICAgY29uc3QgZ3Jhdml0eSA9IGIyVmVjMi5NdWxTVihcclxuICAgICAgc3RlcC5kdCAqIHRoaXMubV9kZWYuZ3Jhdml0eVNjYWxlLFxyXG4gICAgICB0aGlzLm1fd29ybGQuR2V0R3Jhdml0eSgpLFxyXG4gICAgICBzX2dyYXZpdHksXHJcbiAgICApO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICB2ZWxfZGF0YVtpXS5TZWxmQWRkKGdyYXZpdHkpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlR3Jhdml0eV9zX2dyYXZpdHkgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlQmFycmllcihzdGVwOiBiMlRpbWVTdGVwKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX2FhYmIgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlQmFycmllcl9zX2FhYmI7XHJcbiAgICBjb25zdCBzX3ZhID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUJhcnJpZXJfc192YTtcclxuICAgIGNvbnN0IHNfdmIgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlQmFycmllcl9zX3ZiO1xyXG4gICAgY29uc3Qgc19wYmEgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlQmFycmllcl9zX3BiYTtcclxuICAgIGNvbnN0IHNfdmJhID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUJhcnJpZXJfc192YmE7XHJcbiAgICBjb25zdCBzX3ZjID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUJhcnJpZXJfc192YztcclxuICAgIGNvbnN0IHNfcGNhID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUJhcnJpZXJfc19wY2E7XHJcbiAgICBjb25zdCBzX3ZjYSA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVCYXJyaWVyX3NfdmNhO1xyXG4gICAgY29uc3Qgc19xYmEgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlQmFycmllcl9zX3FiYTtcclxuICAgIGNvbnN0IHNfcWNhID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUJhcnJpZXJfc19xY2E7XHJcbiAgICBjb25zdCBzX2R2ID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUJhcnJpZXJfc19kdjtcclxuICAgIGNvbnN0IHNfZiA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVCYXJyaWVyX3NfZjtcclxuICAgIGNvbnN0IHBvc19kYXRhID0gdGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGE7XHJcbiAgICBjb25zdCB2ZWxfZGF0YSA9IHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhO1xyXG4gICAgLy8gSWYgYSBwYXJ0aWNsZSBpcyBwYXNzaW5nIGJldHdlZW4gcGFpcmVkIGJhcnJpZXIgcGFydGljbGVzLFxyXG4gICAgLy8gaXRzIHZlbG9jaXR5IHdpbGwgYmUgZGVjZWxlcmF0ZWQgdG8gYXZvaWQgcGFzc2luZy5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyBpKyspIHtcclxuICAgICAgY29uc3QgZmxhZ3MgPSB0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpXTtcclxuICAgICAgLy8vaWYgKChmbGFncyAmIGIyUGFydGljbGVTeXN0ZW0ua19iYXJyaWVyV2FsbEZsYWdzKSA9PT0gYjJQYXJ0aWNsZVN5c3RlbS5rX2JhcnJpZXJXYWxsRmxhZ3MpXHJcbiAgICAgIGlmICgoZmxhZ3MgJiBiMlBhcnRpY2xlU3lzdGVtLmtfYmFycmllcldhbGxGbGFncykgIT09IDApIHtcclxuICAgICAgICB2ZWxfZGF0YVtpXS5TZXRaZXJvKCk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIGNvbnN0IHRtYXggPSBiMl9iYXJyaWVyQ29sbGlzaW9uVGltZSAqIHN0ZXAuZHQ7XHJcbiAgICBjb25zdCBtYXNzID0gdGhpcy5HZXRQYXJ0aWNsZU1hc3MoKTtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3BhaXJCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBwYWlyID0gdGhpcy5tX3BhaXJCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgaWYgKHBhaXIuZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9iYXJyaWVyUGFydGljbGUpIHtcclxuICAgICAgICBjb25zdCBhID0gcGFpci5pbmRleEE7XHJcbiAgICAgICAgY29uc3QgYiA9IHBhaXIuaW5kZXhCO1xyXG4gICAgICAgIGNvbnN0IHBhID0gcG9zX2RhdGFbYV07XHJcbiAgICAgICAgY29uc3QgcGIgPSBwb3NfZGF0YVtiXTtcclxuICAgICAgICAvLy8gYjJBQUJCIGFhYmI7XHJcbiAgICAgICAgY29uc3QgYWFiYiA9IHNfYWFiYjtcclxuICAgICAgICAvLy9hYWJiLmxvd2VyQm91bmQgPSBiMk1pbihwYSwgcGIpO1xyXG4gICAgICAgIGIyVmVjMi5NaW5WKHBhLCBwYiwgYWFiYi5sb3dlckJvdW5kKTtcclxuICAgICAgICAvLy9hYWJiLnVwcGVyQm91bmQgPSBiMk1heChwYSwgcGIpO1xyXG4gICAgICAgIGIyVmVjMi5NYXhWKHBhLCBwYiwgYWFiYi51cHBlckJvdW5kKTtcclxuICAgICAgICBjb25zdCBhR3JvdXAgPSB0aGlzLm1fZ3JvdXBCdWZmZXJbYV07XHJcbiAgICAgICAgY29uc3QgYkdyb3VwID0gdGhpcy5tX2dyb3VwQnVmZmVyW2JdO1xyXG4gICAgICAgIC8vL2IyVmVjMiB2YSA9IEdldExpbmVhclZlbG9jaXR5KGFHcm91cCwgYSwgcGEpO1xyXG4gICAgICAgIGNvbnN0IHZhID0gdGhpcy5HZXRMaW5lYXJWZWxvY2l0eShhR3JvdXAsIGEsIHBhLCBzX3ZhKTtcclxuICAgICAgICAvLy9iMlZlYzIgdmIgPSBHZXRMaW5lYXJWZWxvY2l0eShiR3JvdXAsIGIsIHBiKTtcclxuICAgICAgICBjb25zdCB2YiA9IHRoaXMuR2V0TGluZWFyVmVsb2NpdHkoYkdyb3VwLCBiLCBwYiwgc192Yik7XHJcbiAgICAgICAgLy8vYjJWZWMyIHBiYSA9IHBiIC0gcGE7XHJcbiAgICAgICAgY29uc3QgcGJhID0gYjJWZWMyLlN1YlZWKHBiLCBwYSwgc19wYmEpO1xyXG4gICAgICAgIC8vL2IyVmVjMiB2YmEgPSB2YiAtIHZhO1xyXG4gICAgICAgIGNvbnN0IHZiYSA9IGIyVmVjMi5TdWJWVih2YiwgdmEsIHNfdmJhKTtcclxuICAgICAgICAvLy9JbnNpZGVCb3VuZHNFbnVtZXJhdG9yIGVudW1lcmF0b3IgPSBHZXRJbnNpZGVCb3VuZHNFbnVtZXJhdG9yKGFhYmIpO1xyXG4gICAgICAgIGNvbnN0IGVudW1lcmF0b3IgPSB0aGlzLkdldEluc2lkZUJvdW5kc0VudW1lcmF0b3IoYWFiYik7XHJcbiAgICAgICAgbGV0IGM6IG51bWJlcjtcclxuICAgICAgICB3aGlsZSAoKGMgPSBlbnVtZXJhdG9yLkdldE5leHQoKSkgPj0gMCkge1xyXG4gICAgICAgICAgY29uc3QgcGMgPSBwb3NfZGF0YVtjXTtcclxuICAgICAgICAgIGNvbnN0IGNHcm91cCA9IHRoaXMubV9ncm91cEJ1ZmZlcltjXTtcclxuICAgICAgICAgIGlmIChhR3JvdXAgIT09IGNHcm91cCAmJiBiR3JvdXAgIT09IGNHcm91cCkge1xyXG4gICAgICAgICAgICAvLy9iMlZlYzIgdmMgPSBHZXRMaW5lYXJWZWxvY2l0eShjR3JvdXAsIGMsIHBjKTtcclxuICAgICAgICAgICAgY29uc3QgdmMgPSB0aGlzLkdldExpbmVhclZlbG9jaXR5KGNHcm91cCwgYywgcGMsIHNfdmMpO1xyXG4gICAgICAgICAgICAvLyBTb2x2ZSB0aGUgZXF1YXRpb24gYmVsb3c6XHJcbiAgICAgICAgICAgIC8vICAgKDEtcykqKHBhK3QqdmEpK3MqKHBiK3QqdmIpID0gcGMrdCp2Y1xyXG4gICAgICAgICAgICAvLyB3aGljaCBleHByZXNzZXMgdGhhdCB0aGUgcGFydGljbGUgYyB3aWxsIHBhc3MgYSBsaW5lXHJcbiAgICAgICAgICAgIC8vIGNvbm5lY3RpbmcgdGhlIHBhcnRpY2xlcyBhIGFuZCBiIGF0IHRoZSB0aW1lIG9mIHQuXHJcbiAgICAgICAgICAgIC8vIGlmIHMgaXMgYmV0d2VlbiAwIGFuZCAxLCBjIHdpbGwgcGFzcyBiZXR3ZWVuIGEgYW5kIGIuXHJcbiAgICAgICAgICAgIC8vL2IyVmVjMiBwY2EgPSBwYyAtIHBhO1xyXG4gICAgICAgICAgICBjb25zdCBwY2EgPSBiMlZlYzIuU3ViVlYocGMsIHBhLCBzX3BjYSk7XHJcbiAgICAgICAgICAgIC8vL2IyVmVjMiB2Y2EgPSB2YyAtIHZhO1xyXG4gICAgICAgICAgICBjb25zdCB2Y2EgPSBiMlZlYzIuU3ViVlYodmMsIHZhLCBzX3ZjYSk7XHJcbiAgICAgICAgICAgIGNvbnN0IGUyID0gYjJWZWMyLkNyb3NzVlYodmJhLCB2Y2EpO1xyXG4gICAgICAgICAgICBjb25zdCBlMSA9IGIyVmVjMi5Dcm9zc1ZWKHBiYSwgdmNhKSAtIGIyVmVjMi5Dcm9zc1ZWKHBjYSwgdmJhKTtcclxuICAgICAgICAgICAgY29uc3QgZTAgPSBiMlZlYzIuQ3Jvc3NWVihwYmEsIHBjYSk7XHJcbiAgICAgICAgICAgIGxldCBzOiBudW1iZXIsIHQ6IG51bWJlcjtcclxuICAgICAgICAgICAgLy8vYjJWZWMyIHFiYSwgcWNhO1xyXG4gICAgICAgICAgICBjb25zdCBxYmEgPSBzX3FiYSxcclxuICAgICAgICAgICAgICBxY2EgPSBzX3FjYTtcclxuICAgICAgICAgICAgaWYgKGUyID09PSAwKSB7XHJcbiAgICAgICAgICAgICAgaWYgKGUxID09PSAwKSB7XHJcbiAgICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgdCA9IC1lMCAvIGUxO1xyXG4gICAgICAgICAgICAgIGlmICghKHQgPj0gMCAmJiB0IDwgdG1heCkpIHtcclxuICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAvLy9xYmEgPSBwYmEgKyB0ICogdmJhO1xyXG4gICAgICAgICAgICAgIGIyVmVjMi5BZGRWTXVsU1YocGJhLCB0LCB2YmEsIHFiYSk7XHJcbiAgICAgICAgICAgICAgLy8vcWNhID0gcGNhICsgdCAqIHZjYTtcclxuICAgICAgICAgICAgICBiMlZlYzIuQWRkVk11bFNWKHBjYSwgdCwgdmNhLCBxY2EpO1xyXG4gICAgICAgICAgICAgIHMgPSBiMlZlYzIuRG90VlYocWJhLCBxY2EpIC8gYjJWZWMyLkRvdFZWKHFiYSwgcWJhKTtcclxuICAgICAgICAgICAgICBpZiAoIShzID49IDAgJiYgcyA8PSAxKSkge1xyXG4gICAgICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICAgIGNvbnN0IGRldCA9IGUxICogZTEgLSA0ICogZTAgKiBlMjtcclxuICAgICAgICAgICAgICBpZiAoZGV0IDwgMCkge1xyXG4gICAgICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgIGNvbnN0IHNxcnREZXQgPSBiMlNxcnQoZGV0KTtcclxuICAgICAgICAgICAgICBsZXQgdDEgPSAoLWUxIC0gc3FydERldCkgLyAoMiAqIGUyKTtcclxuICAgICAgICAgICAgICBsZXQgdDIgPSAoLWUxICsgc3FydERldCkgLyAoMiAqIGUyKTtcclxuICAgICAgICAgICAgICAvLy9pZiAodDEgPiB0MikgYjJTd2FwKHQxLCB0Mik7XHJcbiAgICAgICAgICAgICAgaWYgKHQxID4gdDIpIHtcclxuICAgICAgICAgICAgICAgIGNvbnN0IHRtcCA9IHQxO1xyXG4gICAgICAgICAgICAgICAgdDEgPSB0MjtcclxuICAgICAgICAgICAgICAgIHQyID0gdG1wO1xyXG4gICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICB0ID0gdDE7XHJcbiAgICAgICAgICAgICAgLy8vcWJhID0gcGJhICsgdCAqIHZiYTtcclxuICAgICAgICAgICAgICBiMlZlYzIuQWRkVk11bFNWKHBiYSwgdCwgdmJhLCBxYmEpO1xyXG4gICAgICAgICAgICAgIC8vL3FjYSA9IHBjYSArIHQgKiB2Y2E7XHJcbiAgICAgICAgICAgICAgYjJWZWMyLkFkZFZNdWxTVihwY2EsIHQsIHZjYSwgcWNhKTtcclxuICAgICAgICAgICAgICAvLy9zID0gYjJEb3QocWJhLCBxY2EpIC8gYjJEb3QocWJhLCBxYmEpO1xyXG4gICAgICAgICAgICAgIHMgPSBiMlZlYzIuRG90VlYocWJhLCBxY2EpIC8gYjJWZWMyLkRvdFZWKHFiYSwgcWJhKTtcclxuICAgICAgICAgICAgICBpZiAoISh0ID49IDAgJiYgdCA8IHRtYXggJiYgcyA+PSAwICYmIHMgPD0gMSkpIHtcclxuICAgICAgICAgICAgICAgIHQgPSB0MjtcclxuICAgICAgICAgICAgICAgIGlmICghKHQgPj0gMCAmJiB0IDwgdG1heCkpIHtcclxuICAgICAgICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICAvLy9xYmEgPSBwYmEgKyB0ICogdmJhO1xyXG4gICAgICAgICAgICAgICAgYjJWZWMyLkFkZFZNdWxTVihwYmEsIHQsIHZiYSwgcWJhKTtcclxuICAgICAgICAgICAgICAgIC8vL3FjYSA9IHBjYSArIHQgKiB2Y2E7XHJcbiAgICAgICAgICAgICAgICBiMlZlYzIuQWRkVk11bFNWKHBjYSwgdCwgdmNhLCBxY2EpO1xyXG4gICAgICAgICAgICAgICAgLy8vcyA9IGIyRG90KHFiYSwgcWNhKSAvIGIyRG90KHFiYSwgcWJhKTtcclxuICAgICAgICAgICAgICAgIHMgPSBiMlZlYzIuRG90VlYocWJhLCBxY2EpIC8gYjJWZWMyLkRvdFZWKHFiYSwgcWJhKTtcclxuICAgICAgICAgICAgICAgIGlmICghKHMgPj0gMCAmJiBzIDw9IDEpKSB7XHJcbiAgICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAvLyBBcHBseSBhIGZvcmNlIHRvIHBhcnRpY2xlIGMgc28gdGhhdCBpdCB3aWxsIGhhdmUgdGhlXHJcbiAgICAgICAgICAgIC8vIGludGVycG9sYXRlZCB2ZWxvY2l0eSBhdCB0aGUgY29sbGlzaW9uIHBvaW50IG9uIGxpbmUgYWIuXHJcbiAgICAgICAgICAgIC8vL2IyVmVjMiBkdiA9IHZhICsgcyAqIHZiYSAtIHZjO1xyXG4gICAgICAgICAgICBjb25zdCBkdiA9IHNfZHY7XHJcbiAgICAgICAgICAgIGR2LnggPSB2YS54ICsgcyAqIHZiYS54IC0gdmMueDtcclxuICAgICAgICAgICAgZHYueSA9IHZhLnkgKyBzICogdmJhLnkgLSB2Yy55O1xyXG4gICAgICAgICAgICAvLy9iMlZlYzIgZiA9IEdldFBhcnRpY2xlTWFzcygpICogZHY7XHJcbiAgICAgICAgICAgIGNvbnN0IGYgPSBiMlZlYzIuTXVsU1YobWFzcywgZHYsIHNfZik7XHJcbiAgICAgICAgICAgIGlmIChjR3JvdXAgJiYgdGhpcy5Jc1JpZ2lkR3JvdXAoY0dyb3VwKSkge1xyXG4gICAgICAgICAgICAgIC8vIElmIGMgYmVsb25ncyB0byBhIHJpZ2lkIGdyb3VwLCB0aGUgZm9yY2Ugd2lsbCBiZVxyXG4gICAgICAgICAgICAgIC8vIGRpc3RyaWJ1dGVkIGluIHRoZSBncm91cC5cclxuICAgICAgICAgICAgICBjb25zdCBtYXNzID0gY0dyb3VwLkdldE1hc3MoKTtcclxuICAgICAgICAgICAgICBjb25zdCBpbmVydGlhID0gY0dyb3VwLkdldEluZXJ0aWEoKTtcclxuICAgICAgICAgICAgICBpZiAobWFzcyA+IDApIHtcclxuICAgICAgICAgICAgICAgIC8vL2NHcm91cC5tX2xpbmVhclZlbG9jaXR5ICs9IDEgLyBtYXNzICogZjtcclxuICAgICAgICAgICAgICAgIGNHcm91cC5tX2xpbmVhclZlbG9jaXR5LlNlbGZNdWxBZGQoMSAvIG1hc3MsIGYpO1xyXG4gICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICBpZiAoaW5lcnRpYSA+IDApIHtcclxuICAgICAgICAgICAgICAgIC8vL2NHcm91cC5tX2FuZ3VsYXJWZWxvY2l0eSArPSBiMkNyb3NzKHBjIC0gY0dyb3VwLkdldENlbnRlcigpLCBmKSAvIGluZXJ0aWE7XHJcbiAgICAgICAgICAgICAgICBjR3JvdXAubV9hbmd1bGFyVmVsb2NpdHkgKz1cclxuICAgICAgICAgICAgICAgICAgYjJWZWMyLkNyb3NzVlYoYjJWZWMyLlN1YlZWKHBjLCBjR3JvdXAuR2V0Q2VudGVyKCksIGIyVmVjMi5zX3QwKSwgZikgLyBpbmVydGlhO1xyXG4gICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgICAvLy9tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbY10gKz0gZHY7XHJcbiAgICAgICAgICAgICAgdmVsX2RhdGFbY10uU2VsZkFkZChkdik7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgLy8gQXBwbHkgYSByZXZlcnNlZCBmb3JjZSB0byBwYXJ0aWNsZSBjIGFmdGVyIHBhcnRpY2xlXHJcbiAgICAgICAgICAgIC8vIG1vdmVtZW50IHNvIHRoYXQgbW9tZW50dW0gd2lsbCBiZSBwcmVzZXJ2ZWQuXHJcbiAgICAgICAgICAgIC8vL1BhcnRpY2xlQXBwbHlGb3JjZShjLCAtc3RlcC5pbnZfZHQgKiBmKTtcclxuICAgICAgICAgICAgdGhpcy5QYXJ0aWNsZUFwcGx5Rm9yY2UoYywgZi5TZWxmTXVsKC1zdGVwLmludl9kdCkpO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlQmFycmllcl9zX2FhYmIgPSBuZXcgYjJBQUJCKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlQmFycmllcl9zX3ZhID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUJhcnJpZXJfc192YiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVCYXJyaWVyX3NfcGJhID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUJhcnJpZXJfc192YmEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlQmFycmllcl9zX3ZjID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUJhcnJpZXJfc19wY2EgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlQmFycmllcl9zX3ZjYSA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVCYXJyaWVyX3NfcWJhID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUJhcnJpZXJfc19xY2EgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlQmFycmllcl9zX2R2ID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUJhcnJpZXJfc19mID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVN0YXRpY1ByZXNzdXJlKHN0ZXA6IGIyVGltZVN0ZXApOiB2b2lkIHtcclxuICAgIHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlciA9IHRoaXMuUmVxdWVzdEJ1ZmZlcih0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXIpO1xyXG4gICAgY29uc3QgY3JpdGljYWxQcmVzc3VyZSA9IHRoaXMuR2V0Q3JpdGljYWxQcmVzc3VyZShzdGVwKTtcclxuICAgIGNvbnN0IHByZXNzdXJlUGVyV2VpZ2h0ID0gdGhpcy5tX2RlZi5zdGF0aWNQcmVzc3VyZVN0cmVuZ3RoICogY3JpdGljYWxQcmVzc3VyZTtcclxuICAgIGNvbnN0IG1heFByZXNzdXJlID0gYjJfbWF4UGFydGljbGVQcmVzc3VyZSAqIGNyaXRpY2FsUHJlc3N1cmU7XHJcbiAgICBjb25zdCByZWxheGF0aW9uID0gdGhpcy5tX2RlZi5zdGF0aWNQcmVzc3VyZVJlbGF4YXRpb247XHJcbiAgICAvLy8gQ29tcHV0ZSBwcmVzc3VyZSBzYXRpc2Z5aW5nIHRoZSBtb2RpZmllZCBQb2lzc29uIGVxdWF0aW9uOlxyXG4gICAgLy8vICAgU3VtX2Zvcl9qKChwX2kgLSBwX2opICogd19paikgKyByZWxheGF0aW9uICogcF9pID1cclxuICAgIC8vLyAgIHByZXNzdXJlUGVyV2VpZ2h0ICogKHdfaSAtIGIyX21pblBhcnRpY2xlV2VpZ2h0KVxyXG4gICAgLy8vIGJ5IGl0ZXJhdGluZyB0aGUgY2FsY3VsYXRpb246XHJcbiAgICAvLy8gICBwX2kgPSAoU3VtX2Zvcl9qKHBfaiAqIHdfaWopICsgcHJlc3N1cmVQZXJXZWlnaHQgKlxyXG4gICAgLy8vICAgICAgICAgKHdfaSAtIGIyX21pblBhcnRpY2xlV2VpZ2h0KSkgLyAod19pICsgcmVsYXhhdGlvbilcclxuICAgIC8vLyB3aGVyZVxyXG4gICAgLy8vICAgcF9pIGFuZCBwX2ogYXJlIHN0YXRpYyBwcmVzc3VyZSBvZiBwYXJ0aWNsZSBpIGFuZCBqXHJcbiAgICAvLy8gICB3X2lqIGlzIGNvbnRhY3Qgd2VpZ2h0IGJldHdlZW4gcGFydGljbGUgaSBhbmQgalxyXG4gICAgLy8vICAgd19pIGlzIHN1bSBvZiBjb250YWN0IHdlaWdodCBvZiBwYXJ0aWNsZSBpXHJcbiAgICBmb3IgKGxldCB0ID0gMDsgdCA8IHRoaXMubV9kZWYuc3RhdGljUHJlc3N1cmVJdGVyYXRpb25zOyB0KyspIHtcclxuICAgICAgLy8vbWVtc2V0KG1fYWNjdW11bGF0aW9uQnVmZmVyLCAwLCBzaXplb2YoKm1fYWNjdW11bGF0aW9uQnVmZmVyKSAqIG1fY291bnQpO1xyXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgaSsrKSB7XHJcbiAgICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlcltpXSA9IDA7XHJcbiAgICAgIH1cclxuICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fY29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgICAgaWYgKGNvbnRhY3QuZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9zdGF0aWNQcmVzc3VyZVBhcnRpY2xlKSB7XHJcbiAgICAgICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleEE7XHJcbiAgICAgICAgICBjb25zdCBiID0gY29udGFjdC5pbmRleEI7XHJcbiAgICAgICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgICAgICB0aGlzLm1fYWNjdW11bGF0aW9uQnVmZmVyW2FdICs9IHcgKiB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXJbYl07IC8vIGEgPC0gYlxyXG4gICAgICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlcltiXSArPSB3ICogdGhpcy5tX3N0YXRpY1ByZXNzdXJlQnVmZmVyW2FdOyAvLyBiIDwtIGFcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICAgIGNvbnN0IHcgPSB0aGlzLm1fd2VpZ2h0QnVmZmVyW2ldO1xyXG4gICAgICAgIGlmICh0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpXSAmIGIyUGFydGljbGVGbGFnLmIyX3N0YXRpY1ByZXNzdXJlUGFydGljbGUpIHtcclxuICAgICAgICAgIGNvbnN0IHdoID0gdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlcltpXTtcclxuICAgICAgICAgIGNvbnN0IGggPSAod2ggKyBwcmVzc3VyZVBlcldlaWdodCAqICh3IC0gYjJfbWluUGFydGljbGVXZWlnaHQpKSAvICh3ICsgcmVsYXhhdGlvbik7XHJcbiAgICAgICAgICB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXJbaV0gPSBiMkNsYW1wKGgsIDAuMCwgbWF4UHJlc3N1cmUpO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXJbaV0gPSAwO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgQ29tcHV0ZVdlaWdodCgpOiB2b2lkIHtcclxuICAgIC8vIGNhbGN1bGF0ZXMgdGhlIHN1bSBvZiBjb250YWN0LXdlaWdodHMgZm9yIGVhY2ggcGFydGljbGVcclxuICAgIC8vIHRoYXQgbWVhbnMgZGltZW5zaW9ubGVzcyBkZW5zaXR5XHJcbiAgICAvLy9tZW1zZXQobV93ZWlnaHRCdWZmZXIsIDAsIHNpemVvZigqbV93ZWlnaHRCdWZmZXIpICogbV9jb3VudCk7XHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb3VudDsgaysrKSB7XHJcbiAgICAgIHRoaXMubV93ZWlnaHRCdWZmZXJba10gPSAwO1xyXG4gICAgfVxyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4O1xyXG4gICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgIHRoaXMubV93ZWlnaHRCdWZmZXJbYV0gKz0gdztcclxuICAgIH1cclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICBjb25zdCBiID0gY29udGFjdC5pbmRleEI7XHJcbiAgICAgIGNvbnN0IHcgPSBjb250YWN0LndlaWdodDtcclxuICAgICAgdGhpcy5tX3dlaWdodEJ1ZmZlclthXSArPSB3O1xyXG4gICAgICB0aGlzLm1fd2VpZ2h0QnVmZmVyW2JdICs9IHc7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTb2x2ZVByZXNzdXJlKHN0ZXA6IGIyVGltZVN0ZXApOiB2b2lkIHtcclxuICAgIGNvbnN0IHNfZiA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVQcmVzc3VyZV9zX2Y7XHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIC8vIGNhbGN1bGF0ZXMgcHJlc3N1cmUgYXMgYSBsaW5lYXIgZnVuY3Rpb24gb2YgZGVuc2l0eVxyXG4gICAgY29uc3QgY3JpdGljYWxQcmVzc3VyZSA9IHRoaXMuR2V0Q3JpdGljYWxQcmVzc3VyZShzdGVwKTtcclxuICAgIGNvbnN0IHByZXNzdXJlUGVyV2VpZ2h0ID0gdGhpcy5tX2RlZi5wcmVzc3VyZVN0cmVuZ3RoICogY3JpdGljYWxQcmVzc3VyZTtcclxuICAgIGNvbnN0IG1heFByZXNzdXJlID0gYjJfbWF4UGFydGljbGVQcmVzc3VyZSAqIGNyaXRpY2FsUHJlc3N1cmU7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgaSsrKSB7XHJcbiAgICAgIGNvbnN0IHcgPSB0aGlzLm1fd2VpZ2h0QnVmZmVyW2ldO1xyXG4gICAgICBjb25zdCBoID0gcHJlc3N1cmVQZXJXZWlnaHQgKiBiMk1heCgwLjAsIHcgLSBiMl9taW5QYXJ0aWNsZVdlaWdodCk7XHJcbiAgICAgIHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXJbaV0gPSBiMk1pbihoLCBtYXhQcmVzc3VyZSk7XHJcbiAgICB9XHJcbiAgICAvLyBpZ25vcmVzIHBhcnRpY2xlcyB3aGljaCBoYXZlIHRoZWlyIG93biByZXB1bHNpdmUgZm9yY2VcclxuICAgIGlmICh0aGlzLm1fYWxsUGFydGljbGVGbGFncyAmIGIyUGFydGljbGVTeXN0ZW0ua19ub1ByZXNzdXJlRmxhZ3MpIHtcclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICAgIGlmICh0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpXSAmIGIyUGFydGljbGVTeXN0ZW0ua19ub1ByZXNzdXJlRmxhZ3MpIHtcclxuICAgICAgICAgIHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXJbaV0gPSAwO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgLy8gc3RhdGljIHByZXNzdXJlXHJcbiAgICBpZiAodGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9zdGF0aWNQcmVzc3VyZVBhcnRpY2xlKSB7XHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX3N0YXRpY1ByZXNzdXJlQnVmZmVyICE9PSBudWxsKTtcclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICAgIGlmICh0aGlzLm1fZmxhZ3NCdWZmZXIuZGF0YVtpXSAmIGIyUGFydGljbGVGbGFnLmIyX3N0YXRpY1ByZXNzdXJlUGFydGljbGUpIHtcclxuICAgICAgICAgIHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXJbaV0gKz0gdGhpcy5tX3N0YXRpY1ByZXNzdXJlQnVmZmVyW2ldO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgLy8gYXBwbGllcyBwcmVzc3VyZSBiZXR3ZWVuIGVhY2ggcGFydGljbGVzIGluIGNvbnRhY3RcclxuICAgIGNvbnN0IHZlbG9jaXR5UGVyUHJlc3N1cmUgPSBzdGVwLmR0IC8gKHRoaXMubV9kZWYuZGVuc2l0eSAqIHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyKTtcclxuICAgIGNvbnN0IGludl9tYXNzID0gdGhpcy5HZXRQYXJ0aWNsZUludk1hc3MoKTtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleDtcclxuICAgICAgY29uc3QgYiA9IGNvbnRhY3QuYm9keTtcclxuICAgICAgY29uc3QgdyA9IGNvbnRhY3Qud2VpZ2h0O1xyXG4gICAgICBjb25zdCBtID0gY29udGFjdC5tYXNzO1xyXG4gICAgICBjb25zdCBuID0gY29udGFjdC5ub3JtYWw7XHJcbiAgICAgIGNvbnN0IHAgPSBwb3NfZGF0YVthXTtcclxuICAgICAgY29uc3QgaCA9IHRoaXMubV9hY2N1bXVsYXRpb25CdWZmZXJbYV0gKyBwcmVzc3VyZVBlcldlaWdodCAqIHc7XHJcbiAgICAgIC8vL2IyVmVjMiBmID0gdmVsb2NpdHlQZXJQcmVzc3VyZSAqIHcgKiBtICogaCAqIG47XHJcbiAgICAgIGNvbnN0IGYgPSBiMlZlYzIuTXVsU1YodmVsb2NpdHlQZXJQcmVzc3VyZSAqIHcgKiBtICogaCwgbiwgc19mKTtcclxuICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdIC09IEdldFBhcnRpY2xlSW52TWFzcygpICogZjtcclxuICAgICAgdmVsX2RhdGFbYV0uU2VsZk11bFN1YihpbnZfbWFzcywgZik7XHJcbiAgICAgIGIuQXBwbHlMaW5lYXJJbXB1bHNlKGYsIHAsIHRydWUpO1xyXG4gICAgfVxyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fY29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IGNvbnRhY3QgPSB0aGlzLm1fY29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleEE7XHJcbiAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgY29uc3QgdyA9IGNvbnRhY3Qud2VpZ2h0O1xyXG4gICAgICBjb25zdCBuID0gY29udGFjdC5ub3JtYWw7XHJcbiAgICAgIGNvbnN0IGggPSB0aGlzLm1fYWNjdW11bGF0aW9uQnVmZmVyW2FdICsgdGhpcy5tX2FjY3VtdWxhdGlvbkJ1ZmZlcltiXTtcclxuICAgICAgLy8vYjJWZWMyIGYgPSB2ZWxvY2l0eVBlclByZXNzdXJlICogdyAqIGggKiBuO1xyXG4gICAgICBjb25zdCBmID0gYjJWZWMyLk11bFNWKHZlbG9jaXR5UGVyUHJlc3N1cmUgKiB3ICogaCwgbiwgc19mKTtcclxuICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdIC09IGY7XHJcbiAgICAgIHZlbF9kYXRhW2FdLlNlbGZTdWIoZik7XHJcbiAgICAgIC8vL21fdmVsb2NpdHlCdWZmZXIuZGF0YVtiXSArPSBmO1xyXG4gICAgICB2ZWxfZGF0YVtiXS5TZWxmQWRkKGYpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlUHJlc3N1cmVfc19mID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZURhbXBpbmcoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc192ID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZURhbXBpbmdfc192O1xyXG4gICAgY29uc3Qgc19mID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZURhbXBpbmdfc19mO1xyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IHZlbF9kYXRhID0gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgICAvLyByZWR1Y2VzIG5vcm1hbCB2ZWxvY2l0eSBvZiBlYWNoIGNvbnRhY3RcclxuICAgIGNvbnN0IGxpbmVhckRhbXBpbmcgPSB0aGlzLm1fZGVmLmRhbXBpbmdTdHJlbmd0aDtcclxuICAgIGNvbnN0IHF1YWRyYXRpY0RhbXBpbmcgPSAxIC8gdGhpcy5HZXRDcml0aWNhbFZlbG9jaXR5KHN0ZXApO1xyXG4gICAgY29uc3QgaW52X21hc3MgPSB0aGlzLkdldFBhcnRpY2xlSW52TWFzcygpO1xyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4O1xyXG4gICAgICBjb25zdCBiID0gY29udGFjdC5ib2R5O1xyXG4gICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgIGNvbnN0IG0gPSBjb250YWN0Lm1hc3M7XHJcbiAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgY29uc3QgcCA9IHBvc19kYXRhW2FdO1xyXG4gICAgICAvLy9iMlZlYzIgdiA9IGIuR2V0TGluZWFyVmVsb2NpdHlGcm9tV29ybGRQb2ludChwKSAtIG1fdmVsb2NpdHlCdWZmZXIuZGF0YVthXTtcclxuICAgICAgY29uc3QgdiA9IGIyVmVjMi5TdWJWVihiLkdldExpbmVhclZlbG9jaXR5RnJvbVdvcmxkUG9pbnQocCwgYjJWZWMyLnNfdDApLCB2ZWxfZGF0YVthXSwgc192KTtcclxuICAgICAgY29uc3Qgdm4gPSBiMlZlYzIuRG90VlYodiwgbik7XHJcbiAgICAgIGlmICh2biA8IDApIHtcclxuICAgICAgICBjb25zdCBkYW1waW5nID0gYjJNYXgobGluZWFyRGFtcGluZyAqIHcsIGIyTWluKC1xdWFkcmF0aWNEYW1waW5nICogdm4sIDAuNSkpO1xyXG4gICAgICAgIC8vL2IyVmVjMiBmID0gZGFtcGluZyAqIG0gKiB2biAqIG47XHJcbiAgICAgICAgY29uc3QgZiA9IGIyVmVjMi5NdWxTVihkYW1waW5nICogbSAqIHZuLCBuLCBzX2YpO1xyXG4gICAgICAgIC8vL21fdmVsb2NpdHlCdWZmZXIuZGF0YVthXSArPSBHZXRQYXJ0aWNsZUludk1hc3MoKSAqIGY7XHJcbiAgICAgICAgdmVsX2RhdGFbYV0uU2VsZk11bEFkZChpbnZfbWFzcywgZik7XHJcbiAgICAgICAgLy8vYi5BcHBseUxpbmVhckltcHVsc2UoLWYsIHAsIHRydWUpO1xyXG4gICAgICAgIGIuQXBwbHlMaW5lYXJJbXB1bHNlKGYuU2VsZk5lZygpLCBwLCB0cnVlKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fY29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IGNvbnRhY3QgPSB0aGlzLm1fY29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleEE7XHJcbiAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgY29uc3QgdyA9IGNvbnRhY3Qud2VpZ2h0O1xyXG4gICAgICBjb25zdCBuID0gY29udGFjdC5ub3JtYWw7XHJcbiAgICAgIC8vL2IyVmVjMiB2ID0gbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2JdIC0gbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdO1xyXG4gICAgICBjb25zdCB2ID0gYjJWZWMyLlN1YlZWKHZlbF9kYXRhW2JdLCB2ZWxfZGF0YVthXSwgc192KTtcclxuICAgICAgY29uc3Qgdm4gPSBiMlZlYzIuRG90VlYodiwgbik7XHJcbiAgICAgIGlmICh2biA8IDApIHtcclxuICAgICAgICAvLy9mbG9hdDMyIGRhbXBpbmcgPSBiMk1heChsaW5lYXJEYW1waW5nICogdywgYjJNaW4oLSBxdWFkcmF0aWNEYW1waW5nICogdm4sIDAuNWYpKTtcclxuICAgICAgICBjb25zdCBkYW1waW5nID0gYjJNYXgobGluZWFyRGFtcGluZyAqIHcsIGIyTWluKC1xdWFkcmF0aWNEYW1waW5nICogdm4sIDAuNSkpO1xyXG4gICAgICAgIC8vL2IyVmVjMiBmID0gZGFtcGluZyAqIHZuICogbjtcclxuICAgICAgICBjb25zdCBmID0gYjJWZWMyLk11bFNWKGRhbXBpbmcgKiB2biwgbiwgc19mKTtcclxuICAgICAgICAvLy90aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YVthXSArPSBmO1xyXG4gICAgICAgIHZlbF9kYXRhW2FdLlNlbGZBZGQoZik7XHJcbiAgICAgICAgLy8vdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYl0gLT0gZjtcclxuICAgICAgICB2ZWxfZGF0YVtiXS5TZWxmU3ViKGYpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVEYW1waW5nX3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVEYW1waW5nX3NfZiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVSaWdpZERhbXBpbmcoKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX3QwID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVJpZ2lkRGFtcGluZ19zX3QwO1xyXG4gICAgY29uc3Qgc190MSA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVSaWdpZERhbXBpbmdfc190MTtcclxuICAgIGNvbnN0IHNfcCA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVSaWdpZERhbXBpbmdfc19wO1xyXG4gICAgY29uc3Qgc192ID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVJpZ2lkRGFtcGluZ19zX3Y7XHJcbiAgICBjb25zdCBpbnZNYXNzQSA9IFswLjBdLFxyXG4gICAgICBpbnZJbmVydGlhQSA9IFswLjBdLFxyXG4gICAgICB0YW5nZW50RGlzdGFuY2VBID0gWzAuMF07IC8vIFRPRE86IHN0YXRpY1xyXG4gICAgY29uc3QgaW52TWFzc0IgPSBbMC4wXSxcclxuICAgICAgaW52SW5lcnRpYUIgPSBbMC4wXSxcclxuICAgICAgdGFuZ2VudERpc3RhbmNlQiA9IFswLjBdOyAvLyBUT0RPOiBzdGF0aWNcclxuICAgIC8vIEFwcGx5IGltcHVsc2UgdG8gcmlnaWQgcGFydGljbGUgZ3JvdXBzIGNvbGxpZGluZyB3aXRoIG90aGVyIG9iamVjdHNcclxuICAgIC8vIHRvIHJlZHVjZSByZWxhdGl2ZSB2ZWxvY2l0eSBhdCB0aGUgY29sbGlkaW5nIHBvaW50LlxyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IGRhbXBpbmcgPSB0aGlzLm1fZGVmLmRhbXBpbmdTdHJlbmd0aDtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleDtcclxuICAgICAgY29uc3QgYUdyb3VwID0gdGhpcy5tX2dyb3VwQnVmZmVyW2FdO1xyXG4gICAgICBpZiAoYUdyb3VwICYmIHRoaXMuSXNSaWdpZEdyb3VwKGFHcm91cCkpIHtcclxuICAgICAgICBjb25zdCBiID0gY29udGFjdC5ib2R5O1xyXG4gICAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgICAgY29uc3QgcCA9IHBvc19kYXRhW2FdO1xyXG4gICAgICAgIC8vL2IyVmVjMiB2ID0gYi5HZXRMaW5lYXJWZWxvY2l0eUZyb21Xb3JsZFBvaW50KHApIC0gYUdyb3VwLkdldExpbmVhclZlbG9jaXR5RnJvbVdvcmxkUG9pbnQocCk7XHJcbiAgICAgICAgY29uc3QgdiA9IGIyVmVjMi5TdWJWVihcclxuICAgICAgICAgIGIuR2V0TGluZWFyVmVsb2NpdHlGcm9tV29ybGRQb2ludChwLCBzX3QwKSxcclxuICAgICAgICAgIGFHcm91cC5HZXRMaW5lYXJWZWxvY2l0eUZyb21Xb3JsZFBvaW50KHAsIHNfdDEpLFxyXG4gICAgICAgICAgc192LFxyXG4gICAgICAgICk7XHJcbiAgICAgICAgY29uc3Qgdm4gPSBiMlZlYzIuRG90VlYodiwgbik7XHJcbiAgICAgICAgaWYgKHZuIDwgMCkge1xyXG4gICAgICAgICAgLy8gVGhlIGdyb3VwJ3MgYXZlcmFnZSB2ZWxvY2l0eSBhdCBwYXJ0aWNsZSBwb3NpdGlvbiAncCcgaXMgcHVzaGluZ1xyXG4gICAgICAgICAgLy8gdGhlIHBhcnRpY2xlIGludG8gdGhlIGJvZHkuXHJcbiAgICAgICAgICAvLy90aGlzLkluaXREYW1waW5nUGFyYW1ldGVyV2l0aFJpZ2lkR3JvdXBPclBhcnRpY2xlKCZpbnZNYXNzQSwgJmludkluZXJ0aWFBLCAmdGFuZ2VudERpc3RhbmNlQSwgdHJ1ZSwgYUdyb3VwLCBhLCBwLCBuKTtcclxuICAgICAgICAgIHRoaXMuSW5pdERhbXBpbmdQYXJhbWV0ZXJXaXRoUmlnaWRHcm91cE9yUGFydGljbGUoXHJcbiAgICAgICAgICAgIGludk1hc3NBLFxyXG4gICAgICAgICAgICBpbnZJbmVydGlhQSxcclxuICAgICAgICAgICAgdGFuZ2VudERpc3RhbmNlQSxcclxuICAgICAgICAgICAgdHJ1ZSxcclxuICAgICAgICAgICAgYUdyb3VwLFxyXG4gICAgICAgICAgICBhLFxyXG4gICAgICAgICAgICBwLFxyXG4gICAgICAgICAgICBuLFxyXG4gICAgICAgICAgKTtcclxuICAgICAgICAgIC8vIENhbGN1bGF0ZSBiLm1fSSBmcm9tIGZ1bmN0aW9ucyBvZiBiMkJvZHkuXHJcbiAgICAgICAgICAvLy90aGlzLkluaXREYW1waW5nUGFyYW1ldGVyKCZpbnZNYXNzQiwgJmludkluZXJ0aWFCLCAmdGFuZ2VudERpc3RhbmNlQiwgYi5HZXRNYXNzKCksIGIuR2V0SW5lcnRpYSgpIC0gYi5HZXRNYXNzKCkgKiBiLkdldExvY2FsQ2VudGVyKCkuTGVuZ3RoU3F1YXJlZCgpLCBiLkdldFdvcmxkQ2VudGVyKCksIHAsIG4pO1xyXG4gICAgICAgICAgdGhpcy5Jbml0RGFtcGluZ1BhcmFtZXRlcihcclxuICAgICAgICAgICAgaW52TWFzc0IsXHJcbiAgICAgICAgICAgIGludkluZXJ0aWFCLFxyXG4gICAgICAgICAgICB0YW5nZW50RGlzdGFuY2VCLFxyXG4gICAgICAgICAgICBiLkdldE1hc3MoKSxcclxuICAgICAgICAgICAgYi5HZXRJbmVydGlhKCkgLSBiLkdldE1hc3MoKSAqIGIuR2V0TG9jYWxDZW50ZXIoKS5MZW5ndGhTcXVhcmVkKCksXHJcbiAgICAgICAgICAgIGIuR2V0V29ybGRDZW50ZXIoKSxcclxuICAgICAgICAgICAgcCxcclxuICAgICAgICAgICAgbixcclxuICAgICAgICAgICk7XHJcbiAgICAgICAgICAvLy9mbG9hdDMyIGYgPSBkYW1waW5nICogYjJNaW4odywgMS4wKSAqIHRoaXMuQ29tcHV0ZURhbXBpbmdJbXB1bHNlKGludk1hc3NBLCBpbnZJbmVydGlhQSwgdGFuZ2VudERpc3RhbmNlQSwgaW52TWFzc0IsIGludkluZXJ0aWFCLCB0YW5nZW50RGlzdGFuY2VCLCB2bik7XHJcbiAgICAgICAgICBjb25zdCBmID1cclxuICAgICAgICAgICAgZGFtcGluZyAqXHJcbiAgICAgICAgICAgIGIyTWluKHcsIDEuMCkgKlxyXG4gICAgICAgICAgICB0aGlzLkNvbXB1dGVEYW1waW5nSW1wdWxzZShcclxuICAgICAgICAgICAgICBpbnZNYXNzQVswXSxcclxuICAgICAgICAgICAgICBpbnZJbmVydGlhQVswXSxcclxuICAgICAgICAgICAgICB0YW5nZW50RGlzdGFuY2VBWzBdLFxyXG4gICAgICAgICAgICAgIGludk1hc3NCWzBdLFxyXG4gICAgICAgICAgICAgIGludkluZXJ0aWFCWzBdLFxyXG4gICAgICAgICAgICAgIHRhbmdlbnREaXN0YW5jZUJbMF0sXHJcbiAgICAgICAgICAgICAgdm4sXHJcbiAgICAgICAgICAgICk7XHJcbiAgICAgICAgICAvLy90aGlzLkFwcGx5RGFtcGluZyhpbnZNYXNzQSwgaW52SW5lcnRpYUEsIHRhbmdlbnREaXN0YW5jZUEsIHRydWUsIGFHcm91cCwgYSwgZiwgbik7XHJcbiAgICAgICAgICB0aGlzLkFwcGx5RGFtcGluZyhcclxuICAgICAgICAgICAgaW52TWFzc0FbMF0sXHJcbiAgICAgICAgICAgIGludkluZXJ0aWFBWzBdLFxyXG4gICAgICAgICAgICB0YW5nZW50RGlzdGFuY2VBWzBdLFxyXG4gICAgICAgICAgICB0cnVlLFxyXG4gICAgICAgICAgICBhR3JvdXAsXHJcbiAgICAgICAgICAgIGEsXHJcbiAgICAgICAgICAgIGYsXHJcbiAgICAgICAgICAgIG4sXHJcbiAgICAgICAgICApO1xyXG4gICAgICAgICAgLy8vYi5BcHBseUxpbmVhckltcHVsc2UoLWYgKiBuLCBwLCB0cnVlKTtcclxuICAgICAgICAgIGIuQXBwbHlMaW5lYXJJbXB1bHNlKGIyVmVjMi5NdWxTVigtZiwgbiwgYjJWZWMyLnNfdDApLCBwLCB0cnVlKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICBjb25zdCBiID0gY29udGFjdC5pbmRleEI7XHJcbiAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgY29uc3QgdyA9IGNvbnRhY3Qud2VpZ2h0O1xyXG4gICAgICBjb25zdCBhR3JvdXAgPSB0aGlzLm1fZ3JvdXBCdWZmZXJbYV07XHJcbiAgICAgIGNvbnN0IGJHcm91cCA9IHRoaXMubV9ncm91cEJ1ZmZlcltiXTtcclxuICAgICAgY29uc3QgYVJpZ2lkID0gdGhpcy5Jc1JpZ2lkR3JvdXAoYUdyb3VwKTtcclxuICAgICAgY29uc3QgYlJpZ2lkID0gdGhpcy5Jc1JpZ2lkR3JvdXAoYkdyb3VwKTtcclxuICAgICAgaWYgKGFHcm91cCAhPT0gYkdyb3VwICYmIChhUmlnaWQgfHwgYlJpZ2lkKSkge1xyXG4gICAgICAgIC8vL2IyVmVjMiBwID0gMC41ZiAqICh0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YVthXSArIHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2JdKTtcclxuICAgICAgICBjb25zdCBwID0gYjJWZWMyLk1pZFZWKHBvc19kYXRhW2FdLCBwb3NfZGF0YVtiXSwgc19wKTtcclxuICAgICAgICAvLy9iMlZlYzIgdiA9IEdldExpbmVhclZlbG9jaXR5KGJHcm91cCwgYiwgcCkgLSBHZXRMaW5lYXJWZWxvY2l0eShhR3JvdXAsIGEsIHApO1xyXG4gICAgICAgIGNvbnN0IHYgPSBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgICB0aGlzLkdldExpbmVhclZlbG9jaXR5KGJHcm91cCwgYiwgcCwgc190MCksXHJcbiAgICAgICAgICB0aGlzLkdldExpbmVhclZlbG9jaXR5KGFHcm91cCwgYSwgcCwgc190MSksXHJcbiAgICAgICAgICBzX3YsXHJcbiAgICAgICAgKTtcclxuICAgICAgICBjb25zdCB2biA9IGIyVmVjMi5Eb3RWVih2LCBuKTtcclxuICAgICAgICBpZiAodm4gPCAwKSB7XHJcbiAgICAgICAgICAvLy90aGlzLkluaXREYW1waW5nUGFyYW1ldGVyV2l0aFJpZ2lkR3JvdXBPclBhcnRpY2xlKCZpbnZNYXNzQSwgJmludkluZXJ0aWFBLCAmdGFuZ2VudERpc3RhbmNlQSwgYVJpZ2lkLCBhR3JvdXAsIGEsIHAsIG4pO1xyXG4gICAgICAgICAgdGhpcy5Jbml0RGFtcGluZ1BhcmFtZXRlcldpdGhSaWdpZEdyb3VwT3JQYXJ0aWNsZShcclxuICAgICAgICAgICAgaW52TWFzc0EsXHJcbiAgICAgICAgICAgIGludkluZXJ0aWFBLFxyXG4gICAgICAgICAgICB0YW5nZW50RGlzdGFuY2VBLFxyXG4gICAgICAgICAgICBhUmlnaWQsXHJcbiAgICAgICAgICAgIGFHcm91cCxcclxuICAgICAgICAgICAgYSxcclxuICAgICAgICAgICAgcCxcclxuICAgICAgICAgICAgbixcclxuICAgICAgICAgICk7XHJcbiAgICAgICAgICAvLy90aGlzLkluaXREYW1waW5nUGFyYW1ldGVyV2l0aFJpZ2lkR3JvdXBPclBhcnRpY2xlKCZpbnZNYXNzQiwgJmludkluZXJ0aWFCLCAmdGFuZ2VudERpc3RhbmNlQiwgYlJpZ2lkLCBiR3JvdXAsIGIsIHAsIG4pO1xyXG4gICAgICAgICAgdGhpcy5Jbml0RGFtcGluZ1BhcmFtZXRlcldpdGhSaWdpZEdyb3VwT3JQYXJ0aWNsZShcclxuICAgICAgICAgICAgaW52TWFzc0IsXHJcbiAgICAgICAgICAgIGludkluZXJ0aWFCLFxyXG4gICAgICAgICAgICB0YW5nZW50RGlzdGFuY2VCLFxyXG4gICAgICAgICAgICBiUmlnaWQsXHJcbiAgICAgICAgICAgIGJHcm91cCxcclxuICAgICAgICAgICAgYixcclxuICAgICAgICAgICAgcCxcclxuICAgICAgICAgICAgbixcclxuICAgICAgICAgICk7XHJcbiAgICAgICAgICAvLy9mbG9hdDMyIGYgPSBkYW1waW5nICogdyAqIHRoaXMuQ29tcHV0ZURhbXBpbmdJbXB1bHNlKGludk1hc3NBLCBpbnZJbmVydGlhQSwgdGFuZ2VudERpc3RhbmNlQSwgaW52TWFzc0IsIGludkluZXJ0aWFCLCB0YW5nZW50RGlzdGFuY2VCLCB2bik7XHJcbiAgICAgICAgICBjb25zdCBmID1cclxuICAgICAgICAgICAgZGFtcGluZyAqXHJcbiAgICAgICAgICAgIHcgKlxyXG4gICAgICAgICAgICB0aGlzLkNvbXB1dGVEYW1waW5nSW1wdWxzZShcclxuICAgICAgICAgICAgICBpbnZNYXNzQVswXSxcclxuICAgICAgICAgICAgICBpbnZJbmVydGlhQVswXSxcclxuICAgICAgICAgICAgICB0YW5nZW50RGlzdGFuY2VBWzBdLFxyXG4gICAgICAgICAgICAgIGludk1hc3NCWzBdLFxyXG4gICAgICAgICAgICAgIGludkluZXJ0aWFCWzBdLFxyXG4gICAgICAgICAgICAgIHRhbmdlbnREaXN0YW5jZUJbMF0sXHJcbiAgICAgICAgICAgICAgdm4sXHJcbiAgICAgICAgICAgICk7XHJcbiAgICAgICAgICAvLy90aGlzLkFwcGx5RGFtcGluZyhpbnZNYXNzQSwgaW52SW5lcnRpYUEsIHRhbmdlbnREaXN0YW5jZUEsIGFSaWdpZCwgYUdyb3VwLCBhLCBmLCBuKTtcclxuICAgICAgICAgIHRoaXMuQXBwbHlEYW1waW5nKFxyXG4gICAgICAgICAgICBpbnZNYXNzQVswXSxcclxuICAgICAgICAgICAgaW52SW5lcnRpYUFbMF0sXHJcbiAgICAgICAgICAgIHRhbmdlbnREaXN0YW5jZUFbMF0sXHJcbiAgICAgICAgICAgIGFSaWdpZCxcclxuICAgICAgICAgICAgYUdyb3VwLFxyXG4gICAgICAgICAgICBhLFxyXG4gICAgICAgICAgICBmLFxyXG4gICAgICAgICAgICBuLFxyXG4gICAgICAgICAgKTtcclxuICAgICAgICAgIC8vL3RoaXMuQXBwbHlEYW1waW5nKGludk1hc3NCLCBpbnZJbmVydGlhQiwgdGFuZ2VudERpc3RhbmNlQiwgYlJpZ2lkLCBiR3JvdXAsIGIsIC1mLCBuKTtcclxuICAgICAgICAgIHRoaXMuQXBwbHlEYW1waW5nKFxyXG4gICAgICAgICAgICBpbnZNYXNzQlswXSxcclxuICAgICAgICAgICAgaW52SW5lcnRpYUJbMF0sXHJcbiAgICAgICAgICAgIHRhbmdlbnREaXN0YW5jZUJbMF0sXHJcbiAgICAgICAgICAgIGJSaWdpZCxcclxuICAgICAgICAgICAgYkdyb3VwLFxyXG4gICAgICAgICAgICBiLFxyXG4gICAgICAgICAgICAtZixcclxuICAgICAgICAgICAgbixcclxuICAgICAgICAgICk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVSaWdpZERhbXBpbmdfc190MCA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVSaWdpZERhbXBpbmdfc190MSA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVSaWdpZERhbXBpbmdfc19wID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZVJpZ2lkRGFtcGluZ19zX3YgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlRXh0cmFEYW1waW5nKCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc192ID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUV4dHJhRGFtcGluZ19zX3Y7XHJcbiAgICBjb25zdCBzX2YgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlRXh0cmFEYW1waW5nX3NfZjtcclxuICAgIGNvbnN0IHZlbF9kYXRhID0gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgICAvLyBBcHBsaWVzIGFkZGl0aW9uYWwgZGFtcGluZyBmb3JjZSBiZXR3ZWVuIGJvZGllcyBhbmQgcGFydGljbGVzIHdoaWNoIGNhblxyXG4gICAgLy8gcHJvZHVjZSBzdHJvbmcgcmVwdWxzaXZlIGZvcmNlLiBBcHBseWluZyBkYW1waW5nIGZvcmNlIG11bHRpcGxlIHRpbWVzXHJcbiAgICAvLyBpcyBlZmZlY3RpdmUgaW4gc3VwcHJlc3NpbmcgdmlicmF0aW9uLlxyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IGludl9tYXNzID0gdGhpcy5HZXRQYXJ0aWNsZUludk1hc3MoKTtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleDtcclxuICAgICAgaWYgKHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2FdICYgYjJQYXJ0aWNsZVN5c3RlbS5rX2V4dHJhRGFtcGluZ0ZsYWdzKSB7XHJcbiAgICAgICAgY29uc3QgYiA9IGNvbnRhY3QuYm9keTtcclxuICAgICAgICBjb25zdCBtID0gY29udGFjdC5tYXNzO1xyXG4gICAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgICBjb25zdCBwID0gcG9zX2RhdGFbYV07XHJcbiAgICAgICAgLy8vYjJWZWMyIHYgPSBiLkdldExpbmVhclZlbG9jaXR5RnJvbVdvcmxkUG9pbnQocCkgLSBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYV07XHJcbiAgICAgICAgY29uc3QgdiA9IGIyVmVjMi5TdWJWVihiLkdldExpbmVhclZlbG9jaXR5RnJvbVdvcmxkUG9pbnQocCwgYjJWZWMyLnNfdDApLCB2ZWxfZGF0YVthXSwgc192KTtcclxuICAgICAgICAvLy9mbG9hdDMyIHZuID0gYjJEb3Qodiwgbik7XHJcbiAgICAgICAgY29uc3Qgdm4gPSBiMlZlYzIuRG90VlYodiwgbik7XHJcbiAgICAgICAgaWYgKHZuIDwgMCkge1xyXG4gICAgICAgICAgLy8vYjJWZWMyIGYgPSAwLjVmICogbSAqIHZuICogbjtcclxuICAgICAgICAgIGNvbnN0IGYgPSBiMlZlYzIuTXVsU1YoMC41ICogbSAqIHZuLCBuLCBzX2YpO1xyXG4gICAgICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdICs9IEdldFBhcnRpY2xlSW52TWFzcygpICogZjtcclxuICAgICAgICAgIHZlbF9kYXRhW2FdLlNlbGZNdWxBZGQoaW52X21hc3MsIGYpO1xyXG4gICAgICAgICAgLy8vYi5BcHBseUxpbmVhckltcHVsc2UoLWYsIHAsIHRydWUpO1xyXG4gICAgICAgICAgYi5BcHBseUxpbmVhckltcHVsc2UoZi5TZWxmTmVnKCksIHAsIHRydWUpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlRXh0cmFEYW1waW5nX3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVFeHRyYURhbXBpbmdfc19mID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVdhbGwoKTogdm9pZCB7XHJcbiAgICBjb25zdCB2ZWxfZGF0YSA9IHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICBpZiAodGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbaV0gJiBiMlBhcnRpY2xlRmxhZy5iMl93YWxsUGFydGljbGUpIHtcclxuICAgICAgICB2ZWxfZGF0YVtpXS5TZXRaZXJvKCk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIFNvbHZlUmlnaWQoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19wb3NpdGlvbiA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVSaWdpZF9zX3Bvc2l0aW9uO1xyXG4gICAgY29uc3Qgc19yb3RhdGlvbiA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVSaWdpZF9zX3JvdGF0aW9uO1xyXG4gICAgY29uc3Qgc190cmFuc2Zvcm0gPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlUmlnaWRfc190cmFuc2Zvcm07XHJcbiAgICBjb25zdCBzX3ZlbG9jaXR5VHJhbnNmb3JtID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVJpZ2lkX3NfdmVsb2NpdHlUcmFuc2Zvcm07XHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIGZvciAobGV0IGdyb3VwID0gdGhpcy5tX2dyb3VwTGlzdDsgZ3JvdXA7IGdyb3VwID0gZ3JvdXAuR2V0TmV4dCgpKSB7XHJcbiAgICAgIGlmIChncm91cC5tX2dyb3VwRmxhZ3MgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3JpZ2lkUGFydGljbGVHcm91cCkge1xyXG4gICAgICAgIGdyb3VwLlVwZGF0ZVN0YXRpc3RpY3MoKTtcclxuICAgICAgICAvLy9iMlJvdCByb3RhdGlvbihzdGVwLmR0ICogZ3JvdXAubV9hbmd1bGFyVmVsb2NpdHkpO1xyXG4gICAgICAgIGNvbnN0IHJvdGF0aW9uID0gc19yb3RhdGlvbjtcclxuICAgICAgICByb3RhdGlvbi5TZXRBbmdsZShzdGVwLmR0ICogZ3JvdXAubV9hbmd1bGFyVmVsb2NpdHkpO1xyXG4gICAgICAgIC8vL2IyVHJhbnNmb3JtIHRyYW5zZm9ybShncm91cC5tX2NlbnRlciArIHN0ZXAuZHQgKiBncm91cC5tX2xpbmVhclZlbG9jaXR5IC0gYjJNdWwocm90YXRpb24sIGdyb3VwLm1fY2VudGVyKSwgcm90YXRpb24pO1xyXG4gICAgICAgIGNvbnN0IHBvc2l0aW9uID0gYjJWZWMyLkFkZFZWKFxyXG4gICAgICAgICAgZ3JvdXAubV9jZW50ZXIsXHJcbiAgICAgICAgICBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgICAgIGIyVmVjMi5NdWxTVihzdGVwLmR0LCBncm91cC5tX2xpbmVhclZlbG9jaXR5LCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgICAgIGIyUm90Lk11bFJWKHJvdGF0aW9uLCBncm91cC5tX2NlbnRlciwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICAgICAgICBiMlZlYzIuc190MCxcclxuICAgICAgICAgICksXHJcbiAgICAgICAgICBzX3Bvc2l0aW9uLFxyXG4gICAgICAgICk7XHJcbiAgICAgICAgY29uc3QgdHJhbnNmb3JtID0gc190cmFuc2Zvcm07XHJcbiAgICAgICAgdHJhbnNmb3JtLlNldFBvc2l0aW9uUm90YXRpb24ocG9zaXRpb24sIHJvdGF0aW9uKTtcclxuICAgICAgICAvLy9ncm91cC5tX3RyYW5zZm9ybSA9IGIyTXVsKHRyYW5zZm9ybSwgZ3JvdXAubV90cmFuc2Zvcm0pO1xyXG4gICAgICAgIGIyVHJhbnNmb3JtLk11bFhYKHRyYW5zZm9ybSwgZ3JvdXAubV90cmFuc2Zvcm0sIGdyb3VwLm1fdHJhbnNmb3JtKTtcclxuICAgICAgICBjb25zdCB2ZWxvY2l0eVRyYW5zZm9ybSA9IHNfdmVsb2NpdHlUcmFuc2Zvcm07XHJcbiAgICAgICAgdmVsb2NpdHlUcmFuc2Zvcm0ucC54ID0gc3RlcC5pbnZfZHQgKiB0cmFuc2Zvcm0ucC54O1xyXG4gICAgICAgIHZlbG9jaXR5VHJhbnNmb3JtLnAueSA9IHN0ZXAuaW52X2R0ICogdHJhbnNmb3JtLnAueTtcclxuICAgICAgICB2ZWxvY2l0eVRyYW5zZm9ybS5xLnMgPSBzdGVwLmludl9kdCAqIHRyYW5zZm9ybS5xLnM7XHJcbiAgICAgICAgdmVsb2NpdHlUcmFuc2Zvcm0ucS5jID0gc3RlcC5pbnZfZHQgKiAodHJhbnNmb3JtLnEuYyAtIDEpO1xyXG4gICAgICAgIGZvciAobGV0IGkgPSBncm91cC5tX2ZpcnN0SW5kZXg7IGkgPCBncm91cC5tX2xhc3RJbmRleDsgaSsrKSB7XHJcbiAgICAgICAgICAvLy9tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbaV0gPSBiMk11bCh2ZWxvY2l0eVRyYW5zZm9ybSwgbV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2ldKTtcclxuICAgICAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHZlbG9jaXR5VHJhbnNmb3JtLCBwb3NfZGF0YVtpXSwgdmVsX2RhdGFbaV0pO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlUmlnaWRfc19wb3NpdGlvbiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVSaWdpZF9zX3JvdGF0aW9uID0gbmV3IGIyUm90KCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlUmlnaWRfc190cmFuc2Zvcm0gPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVSaWdpZF9zX3ZlbG9jaXR5VHJhbnNmb3JtID0gbmV3IGIyVHJhbnNmb3JtKCk7XHJcblxyXG4gIFNvbHZlRWxhc3RpYyhzdGVwOiBiMlRpbWVTdGVwKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX3BhID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUVsYXN0aWNfc19wYTtcclxuICAgIGNvbnN0IHNfcGIgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlRWxhc3RpY19zX3BiO1xyXG4gICAgY29uc3Qgc19wYyA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVFbGFzdGljX3NfcGM7XHJcbiAgICBjb25zdCBzX3IgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlRWxhc3RpY19zX3I7XHJcbiAgICBjb25zdCBzX3QwID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZUVsYXN0aWNfc190MDtcclxuICAgIGNvbnN0IHBvc19kYXRhID0gdGhpcy5tX3Bvc2l0aW9uQnVmZmVyLmRhdGE7XHJcbiAgICBjb25zdCB2ZWxfZGF0YSA9IHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgZWxhc3RpY1N0cmVuZ3RoID0gc3RlcC5pbnZfZHQgKiB0aGlzLm1fZGVmLmVsYXN0aWNTdHJlbmd0aDtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3RyaWFkQnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgdHJpYWQgPSB0aGlzLm1fdHJpYWRCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgaWYgKHRyaWFkLmZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfZWxhc3RpY1BhcnRpY2xlKSB7XHJcbiAgICAgICAgY29uc3QgYSA9IHRyaWFkLmluZGV4QTtcclxuICAgICAgICBjb25zdCBiID0gdHJpYWQuaW5kZXhCO1xyXG4gICAgICAgIGNvbnN0IGMgPSB0cmlhZC5pbmRleEM7XHJcbiAgICAgICAgY29uc3Qgb2EgPSB0cmlhZC5wYTtcclxuICAgICAgICBjb25zdCBvYiA9IHRyaWFkLnBiO1xyXG4gICAgICAgIGNvbnN0IG9jID0gdHJpYWQucGM7XHJcbiAgICAgICAgLy8vYjJWZWMyIHBhID0gbV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2FdO1xyXG4gICAgICAgIGNvbnN0IHBhID0gc19wYS5Db3B5KHBvc19kYXRhW2FdKTtcclxuICAgICAgICAvLy9iMlZlYzIgcGIgPSBtX3Bvc2l0aW9uQnVmZmVyLmRhdGFbYl07XHJcbiAgICAgICAgY29uc3QgcGIgPSBzX3BiLkNvcHkocG9zX2RhdGFbYl0pO1xyXG4gICAgICAgIC8vL2IyVmVjMiBwYyA9IG1fcG9zaXRpb25CdWZmZXIuZGF0YVtjXTtcclxuICAgICAgICBjb25zdCBwYyA9IHNfcGMuQ29weShwb3NfZGF0YVtjXSk7XHJcbiAgICAgICAgY29uc3QgdmEgPSB2ZWxfZGF0YVthXTtcclxuICAgICAgICBjb25zdCB2YiA9IHZlbF9kYXRhW2JdO1xyXG4gICAgICAgIGNvbnN0IHZjID0gdmVsX2RhdGFbY107XHJcbiAgICAgICAgLy8vcGEgKz0gc3RlcC5kdCAqIHZhO1xyXG4gICAgICAgIHBhLlNlbGZNdWxBZGQoc3RlcC5kdCwgdmEpO1xyXG4gICAgICAgIC8vL3BiICs9IHN0ZXAuZHQgKiB2YjtcclxuICAgICAgICBwYi5TZWxmTXVsQWRkKHN0ZXAuZHQsIHZiKTtcclxuICAgICAgICAvLy9wYyArPSBzdGVwLmR0ICogdmM7XHJcbiAgICAgICAgcGMuU2VsZk11bEFkZChzdGVwLmR0LCB2Yyk7XHJcbiAgICAgICAgLy8vYjJWZWMyIG1pZFBvaW50ID0gKGZsb2F0MzIpIDEgLyAzICogKHBhICsgcGIgKyBwYyk7XHJcbiAgICAgICAgY29uc3QgbWlkUG9pbnRfeCA9IChwYS54ICsgcGIueCArIHBjLngpIC8gMy4wO1xyXG4gICAgICAgIGNvbnN0IG1pZFBvaW50X3kgPSAocGEueSArIHBiLnkgKyBwYy55KSAvIDMuMDtcclxuICAgICAgICAvLy9wYSAtPSBtaWRQb2ludDtcclxuICAgICAgICBwYS54IC09IG1pZFBvaW50X3g7XHJcbiAgICAgICAgcGEueSAtPSBtaWRQb2ludF95O1xyXG4gICAgICAgIC8vL3BiIC09IG1pZFBvaW50O1xyXG4gICAgICAgIHBiLnggLT0gbWlkUG9pbnRfeDtcclxuICAgICAgICBwYi55IC09IG1pZFBvaW50X3k7XHJcbiAgICAgICAgLy8vcGMgLT0gbWlkUG9pbnQ7XHJcbiAgICAgICAgcGMueCAtPSBtaWRQb2ludF94O1xyXG4gICAgICAgIHBjLnkgLT0gbWlkUG9pbnRfeTtcclxuICAgICAgICAvLy9iMlJvdCByO1xyXG4gICAgICAgIGNvbnN0IHIgPSBzX3I7XHJcbiAgICAgICAgci5zID0gYjJWZWMyLkNyb3NzVlYob2EsIHBhKSArIGIyVmVjMi5Dcm9zc1ZWKG9iLCBwYikgKyBiMlZlYzIuQ3Jvc3NWVihvYywgcGMpO1xyXG4gICAgICAgIHIuYyA9IGIyVmVjMi5Eb3RWVihvYSwgcGEpICsgYjJWZWMyLkRvdFZWKG9iLCBwYikgKyBiMlZlYzIuRG90VlYob2MsIHBjKTtcclxuICAgICAgICBjb25zdCByMiA9IHIucyAqIHIucyArIHIuYyAqIHIuYztcclxuICAgICAgICBsZXQgaW52UiA9IGIySW52U3FydChyMik7XHJcbiAgICAgICAgaWYgKCFpc0Zpbml0ZShpbnZSKSkge1xyXG4gICAgICAgICAgaW52UiA9IDEuOTgxNzc1MzdlMTk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHIucyAqPSBpbnZSO1xyXG4gICAgICAgIHIuYyAqPSBpbnZSO1xyXG4gICAgICAgIC8vL3IuYW5nbGUgPSBNYXRoLmF0YW4yKHIucywgci5jKTsgLy8gVE9ETzogb3B0aW1pemVcclxuICAgICAgICBjb25zdCBzdHJlbmd0aCA9IGVsYXN0aWNTdHJlbmd0aCAqIHRyaWFkLnN0cmVuZ3RoO1xyXG4gICAgICAgIC8vL3ZhICs9IHN0cmVuZ3RoICogKGIyTXVsKHIsIG9hKSAtIHBhKTtcclxuICAgICAgICBiMlJvdC5NdWxSVihyLCBvYSwgc190MCk7XHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHNfdDAsIHBhLCBzX3QwKTtcclxuICAgICAgICBiMlZlYzIuTXVsU1Yoc3RyZW5ndGgsIHNfdDAsIHNfdDApO1xyXG4gICAgICAgIHZhLlNlbGZBZGQoc190MCk7XHJcbiAgICAgICAgLy8vdmIgKz0gc3RyZW5ndGggKiAoYjJNdWwociwgb2IpIC0gcGIpO1xyXG4gICAgICAgIGIyUm90Lk11bFJWKHIsIG9iLCBzX3QwKTtcclxuICAgICAgICBiMlZlYzIuU3ViVlYoc190MCwgcGIsIHNfdDApO1xyXG4gICAgICAgIGIyVmVjMi5NdWxTVihzdHJlbmd0aCwgc190MCwgc190MCk7XHJcbiAgICAgICAgdmIuU2VsZkFkZChzX3QwKTtcclxuICAgICAgICAvLy92YyArPSBzdHJlbmd0aCAqIChiMk11bChyLCBvYykgLSBwYyk7XHJcbiAgICAgICAgYjJSb3QuTXVsUlYociwgb2MsIHNfdDApO1xyXG4gICAgICAgIGIyVmVjMi5TdWJWVihzX3QwLCBwYywgc190MCk7XHJcbiAgICAgICAgYjJWZWMyLk11bFNWKHN0cmVuZ3RoLCBzX3QwLCBzX3QwKTtcclxuICAgICAgICB2Yy5TZWxmQWRkKHNfdDApO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVFbGFzdGljX3NfcGEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlRWxhc3RpY19zX3BiID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUVsYXN0aWNfc19wYyA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVFbGFzdGljX3NfciA9IG5ldyBiMlJvdCgpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZUVsYXN0aWNfc190MCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVTcHJpbmcoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19wYSA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVTcHJpbmdfc19wYTtcclxuICAgIGNvbnN0IHNfcGIgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlU3ByaW5nX3NfcGI7XHJcbiAgICBjb25zdCBzX2QgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlU3ByaW5nX3NfZDtcclxuICAgIGNvbnN0IHNfZiA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVTcHJpbmdfc19mO1xyXG4gICAgY29uc3QgcG9zX2RhdGEgPSB0aGlzLm1fcG9zaXRpb25CdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IHZlbF9kYXRhID0gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgICBjb25zdCBzcHJpbmdTdHJlbmd0aCA9IHN0ZXAuaW52X2R0ICogdGhpcy5tX2RlZi5zcHJpbmdTdHJlbmd0aDtcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3BhaXJCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBwYWlyID0gdGhpcy5tX3BhaXJCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgaWYgKHBhaXIuZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9zcHJpbmdQYXJ0aWNsZSkge1xyXG4gICAgICAgIC8vL2ludDMyIGEgPSBwYWlyLmluZGV4QTtcclxuICAgICAgICBjb25zdCBhID0gcGFpci5pbmRleEE7XHJcbiAgICAgICAgLy8vaW50MzIgYiA9IHBhaXIuaW5kZXhCO1xyXG4gICAgICAgIGNvbnN0IGIgPSBwYWlyLmluZGV4QjtcclxuICAgICAgICAvLy9iMlZlYzIgcGEgPSBtX3Bvc2l0aW9uQnVmZmVyLmRhdGFbYV07XHJcbiAgICAgICAgY29uc3QgcGEgPSBzX3BhLkNvcHkocG9zX2RhdGFbYV0pO1xyXG4gICAgICAgIC8vL2IyVmVjMiBwYiA9IG1fcG9zaXRpb25CdWZmZXIuZGF0YVtiXTtcclxuICAgICAgICBjb25zdCBwYiA9IHNfcGIuQ29weShwb3NfZGF0YVtiXSk7XHJcbiAgICAgICAgLy8vYjJWZWMyJiB2YSA9IG1fdmVsb2NpdHlCdWZmZXIuZGF0YVthXTtcclxuICAgICAgICBjb25zdCB2YSA9IHZlbF9kYXRhW2FdO1xyXG4gICAgICAgIC8vL2IyVmVjMiYgdmIgPSBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYl07XHJcbiAgICAgICAgY29uc3QgdmIgPSB2ZWxfZGF0YVtiXTtcclxuICAgICAgICAvLy9wYSArPSBzdGVwLmR0ICogdmE7XHJcbiAgICAgICAgcGEuU2VsZk11bEFkZChzdGVwLmR0LCB2YSk7XHJcbiAgICAgICAgLy8vcGIgKz0gc3RlcC5kdCAqIHZiO1xyXG4gICAgICAgIHBiLlNlbGZNdWxBZGQoc3RlcC5kdCwgdmIpO1xyXG4gICAgICAgIC8vL2IyVmVjMiBkID0gcGIgLSBwYTtcclxuICAgICAgICBjb25zdCBkID0gYjJWZWMyLlN1YlZWKHBiLCBwYSwgc19kKTtcclxuICAgICAgICAvLy9mbG9hdDMyIHIwID0gcGFpci5kaXN0YW5jZTtcclxuICAgICAgICBjb25zdCByMCA9IHBhaXIuZGlzdGFuY2U7XHJcbiAgICAgICAgLy8vZmxvYXQzMiByMSA9IGQuTGVuZ3RoKCk7XHJcbiAgICAgICAgY29uc3QgcjEgPSBkLkxlbmd0aCgpO1xyXG4gICAgICAgIC8vL2Zsb2F0MzIgc3RyZW5ndGggPSBzcHJpbmdTdHJlbmd0aCAqIHBhaXIuc3RyZW5ndGg7XHJcbiAgICAgICAgY29uc3Qgc3RyZW5ndGggPSBzcHJpbmdTdHJlbmd0aCAqIHBhaXIuc3RyZW5ndGg7XHJcbiAgICAgICAgLy8vYjJWZWMyIGYgPSBzdHJlbmd0aCAqIChyMCAtIHIxKSAvIHIxICogZDtcclxuICAgICAgICBjb25zdCBmID0gYjJWZWMyLk11bFNWKChzdHJlbmd0aCAqIChyMCAtIHIxKSkgLyByMSwgZCwgc19mKTtcclxuICAgICAgICAvLy92YSAtPSBmO1xyXG4gICAgICAgIHZhLlNlbGZTdWIoZik7XHJcbiAgICAgICAgLy8vdmIgKz0gZjtcclxuICAgICAgICB2Yi5TZWxmQWRkKGYpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVTcHJpbmdfc19wYSA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVTcHJpbmdfc19wYiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVTcHJpbmdfc19kID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZVNwcmluZ19zX2YgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlVGVuc2lsZShzdGVwOiBiMlRpbWVTdGVwKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX3dlaWdodGVkTm9ybWFsID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVRlbnNpbGVfc193ZWlnaHRlZE5vcm1hbDtcclxuICAgIGNvbnN0IHNfcyA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVUZW5zaWxlX3NfcztcclxuICAgIGNvbnN0IHNfZiA9IGIyUGFydGljbGVTeXN0ZW0uU29sdmVUZW5zaWxlX3NfZjtcclxuICAgIGNvbnN0IHZlbF9kYXRhID0gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV9hY2N1bXVsYXRpb24yQnVmZmVyICE9PSBudWxsKTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyBpKyspIHtcclxuICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbjJCdWZmZXJbaV0gPSBuZXcgYjJWZWMyKCk7XHJcbiAgICAgIHRoaXMubV9hY2N1bXVsYXRpb24yQnVmZmVyW2ldLlNldFplcm8oKTtcclxuICAgIH1cclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgaWYgKGNvbnRhY3QuZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl90ZW5zaWxlUGFydGljbGUpIHtcclxuICAgICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleEE7XHJcbiAgICAgICAgY29uc3QgYiA9IGNvbnRhY3QuaW5kZXhCO1xyXG4gICAgICAgIGNvbnN0IHcgPSBjb250YWN0LndlaWdodDtcclxuICAgICAgICBjb25zdCBuID0gY29udGFjdC5ub3JtYWw7XHJcbiAgICAgICAgLy8vYjJWZWMyIHdlaWdodGVkTm9ybWFsID0gKDEgLSB3KSAqIHcgKiBuO1xyXG4gICAgICAgIGNvbnN0IHdlaWdodGVkTm9ybWFsID0gYjJWZWMyLk11bFNWKCgxIC0gdykgKiB3LCBuLCBzX3dlaWdodGVkTm9ybWFsKTtcclxuICAgICAgICAvLy9tX2FjY3VtdWxhdGlvbjJCdWZmZXJbYV0gLT0gd2VpZ2h0ZWROb3JtYWw7XHJcbiAgICAgICAgdGhpcy5tX2FjY3VtdWxhdGlvbjJCdWZmZXJbYV0uU2VsZlN1Yih3ZWlnaHRlZE5vcm1hbCk7XHJcbiAgICAgICAgLy8vbV9hY2N1bXVsYXRpb24yQnVmZmVyW2JdICs9IHdlaWdodGVkTm9ybWFsO1xyXG4gICAgICAgIHRoaXMubV9hY2N1bXVsYXRpb24yQnVmZmVyW2JdLlNlbGZBZGQod2VpZ2h0ZWROb3JtYWwpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICBjb25zdCBjcml0aWNhbFZlbG9jaXR5ID0gdGhpcy5HZXRDcml0aWNhbFZlbG9jaXR5KHN0ZXApO1xyXG4gICAgY29uc3QgcHJlc3N1cmVTdHJlbmd0aCA9IHRoaXMubV9kZWYuc3VyZmFjZVRlbnNpb25QcmVzc3VyZVN0cmVuZ3RoICogY3JpdGljYWxWZWxvY2l0eTtcclxuICAgIGNvbnN0IG5vcm1hbFN0cmVuZ3RoID0gdGhpcy5tX2RlZi5zdXJmYWNlVGVuc2lvbk5vcm1hbFN0cmVuZ3RoICogY3JpdGljYWxWZWxvY2l0eTtcclxuICAgIGNvbnN0IG1heFZlbG9jaXR5VmFyaWF0aW9uID0gYjJfbWF4UGFydGljbGVGb3JjZSAqIGNyaXRpY2FsVmVsb2NpdHk7XHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGlmIChjb250YWN0LmZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfdGVuc2lsZVBhcnRpY2xlKSB7XHJcbiAgICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgICAgY29uc3QgbiA9IGNvbnRhY3Qubm9ybWFsO1xyXG4gICAgICAgIGNvbnN0IGggPSB0aGlzLm1fd2VpZ2h0QnVmZmVyW2FdICsgdGhpcy5tX3dlaWdodEJ1ZmZlcltiXTtcclxuICAgICAgICAvLy9iMlZlYzIgcyA9IG1fYWNjdW11bGF0aW9uMkJ1ZmZlcltiXSAtIG1fYWNjdW11bGF0aW9uMkJ1ZmZlclthXTtcclxuICAgICAgICBjb25zdCBzID0gYjJWZWMyLlN1YlZWKHRoaXMubV9hY2N1bXVsYXRpb24yQnVmZmVyW2JdLCB0aGlzLm1fYWNjdW11bGF0aW9uMkJ1ZmZlclthXSwgc19zKTtcclxuICAgICAgICBjb25zdCBmbiA9XHJcbiAgICAgICAgICBiMk1pbihcclxuICAgICAgICAgICAgcHJlc3N1cmVTdHJlbmd0aCAqIChoIC0gMikgKyBub3JtYWxTdHJlbmd0aCAqIGIyVmVjMi5Eb3RWVihzLCBuKSxcclxuICAgICAgICAgICAgbWF4VmVsb2NpdHlWYXJpYXRpb24sXHJcbiAgICAgICAgICApICogdztcclxuICAgICAgICAvLy9iMlZlYzIgZiA9IGZuICogbjtcclxuICAgICAgICBjb25zdCBmID0gYjJWZWMyLk11bFNWKGZuLCBuLCBzX2YpO1xyXG4gICAgICAgIC8vL21fdmVsb2NpdHlCdWZmZXIuZGF0YVthXSAtPSBmO1xyXG4gICAgICAgIHZlbF9kYXRhW2FdLlNlbGZTdWIoZik7XHJcbiAgICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2JdICs9IGY7XHJcbiAgICAgICAgdmVsX2RhdGFbYl0uU2VsZkFkZChmKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlVGVuc2lsZV9zX3dlaWdodGVkTm9ybWFsID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZVRlbnNpbGVfc19zID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZVRlbnNpbGVfc19mID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVZpc2NvdXMoKTogdm9pZCB7XHJcbiAgICBjb25zdCBzX3YgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlVmlzY291c19zX3Y7XHJcbiAgICBjb25zdCBzX2YgPSBiMlBhcnRpY2xlU3lzdGVtLlNvbHZlVmlzY291c19zX2Y7XHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IHZpc2NvdXNTdHJlbmd0aCA9IHRoaXMubV9kZWYudmlzY291c1N0cmVuZ3RoO1xyXG4gICAgY29uc3QgaW52X21hc3MgPSB0aGlzLkdldFBhcnRpY2xlSW52TWFzcygpO1xyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4O1xyXG4gICAgICBpZiAodGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbYV0gJiBiMlBhcnRpY2xlRmxhZy5iMl92aXNjb3VzUGFydGljbGUpIHtcclxuICAgICAgICBjb25zdCBiID0gY29udGFjdC5ib2R5O1xyXG4gICAgICAgIGNvbnN0IHcgPSBjb250YWN0LndlaWdodDtcclxuICAgICAgICBjb25zdCBtID0gY29udGFjdC5tYXNzO1xyXG4gICAgICAgIGNvbnN0IHAgPSBwb3NfZGF0YVthXTtcclxuICAgICAgICAvLy9iMlZlYzIgdiA9IGIuR2V0TGluZWFyVmVsb2NpdHlGcm9tV29ybGRQb2ludChwKSAtIG1fdmVsb2NpdHlCdWZmZXIuZGF0YVthXTtcclxuICAgICAgICBjb25zdCB2ID0gYjJWZWMyLlN1YlZWKGIuR2V0TGluZWFyVmVsb2NpdHlGcm9tV29ybGRQb2ludChwLCBiMlZlYzIuc190MCksIHZlbF9kYXRhW2FdLCBzX3YpO1xyXG4gICAgICAgIC8vL2IyVmVjMiBmID0gdmlzY291c1N0cmVuZ3RoICogbSAqIHcgKiB2O1xyXG4gICAgICAgIGNvbnN0IGYgPSBiMlZlYzIuTXVsU1YodmlzY291c1N0cmVuZ3RoICogbSAqIHcsIHYsIHNfZik7XHJcbiAgICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdICs9IEdldFBhcnRpY2xlSW52TWFzcygpICogZjtcclxuICAgICAgICB2ZWxfZGF0YVthXS5TZWxmTXVsQWRkKGludl9tYXNzLCBmKTtcclxuICAgICAgICAvLy9iLkFwcGx5TGluZWFySW1wdWxzZSgtZiwgcCwgdHJ1ZSk7XHJcbiAgICAgICAgYi5BcHBseUxpbmVhckltcHVsc2UoZi5TZWxmTmVnKCksIHAsIHRydWUpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGlmIChjb250YWN0LmZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfdmlzY291c1BhcnRpY2xlKSB7XHJcbiAgICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgICAgLy8vYjJWZWMyIHYgPSBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYl0gLSBtX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYV07XHJcbiAgICAgICAgY29uc3QgdiA9IGIyVmVjMi5TdWJWVih2ZWxfZGF0YVtiXSwgdmVsX2RhdGFbYV0sIHNfdik7XHJcbiAgICAgICAgLy8vYjJWZWMyIGYgPSB2aXNjb3VzU3RyZW5ndGggKiB3ICogdjtcclxuICAgICAgICBjb25zdCBmID0gYjJWZWMyLk11bFNWKHZpc2NvdXNTdHJlbmd0aCAqIHcsIHYsIHNfZik7XHJcbiAgICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdICs9IGY7XHJcbiAgICAgICAgdmVsX2RhdGFbYV0uU2VsZkFkZChmKTtcclxuICAgICAgICAvLy9tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbYl0gLT0gZjtcclxuICAgICAgICB2ZWxfZGF0YVtiXS5TZWxmU3ViKGYpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVWaXNjb3VzX3NfdiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVWaXNjb3VzX3NfZiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVSZXB1bHNpdmUoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19mID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVJlcHVsc2l2ZV9zX2Y7XHJcbiAgICBjb25zdCB2ZWxfZGF0YSA9IHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgcmVwdWxzaXZlU3RyZW5ndGggPSB0aGlzLm1fZGVmLnJlcHVsc2l2ZVN0cmVuZ3RoICogdGhpcy5HZXRDcml0aWNhbFZlbG9jaXR5KHN0ZXApO1xyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fY29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IGNvbnRhY3QgPSB0aGlzLm1fY29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBpZiAoY29udGFjdC5mbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX3JlcHVsc2l2ZVBhcnRpY2xlKSB7XHJcbiAgICAgICAgY29uc3QgYSA9IGNvbnRhY3QuaW5kZXhBO1xyXG4gICAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgICBpZiAodGhpcy5tX2dyb3VwQnVmZmVyW2FdICE9PSB0aGlzLm1fZ3JvdXBCdWZmZXJbYl0pIHtcclxuICAgICAgICAgIGNvbnN0IHcgPSBjb250YWN0LndlaWdodDtcclxuICAgICAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgICAgIC8vL2IyVmVjMiBmID0gcmVwdWxzaXZlU3RyZW5ndGggKiB3ICogbjtcclxuICAgICAgICAgIGNvbnN0IGYgPSBiMlZlYzIuTXVsU1YocmVwdWxzaXZlU3RyZW5ndGggKiB3LCBuLCBzX2YpO1xyXG4gICAgICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdIC09IGY7XHJcbiAgICAgICAgICB2ZWxfZGF0YVthXS5TZWxmU3ViKGYpO1xyXG4gICAgICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2JdICs9IGY7XHJcbiAgICAgICAgICB2ZWxfZGF0YVtiXS5TZWxmQWRkKGYpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFNvbHZlUmVwdWxzaXZlX3NfZiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVQb3dkZXIoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19mID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVBvd2Rlcl9zX2Y7XHJcbiAgICBjb25zdCBwb3NfZGF0YSA9IHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IHBvd2RlclN0cmVuZ3RoID0gdGhpcy5tX2RlZi5wb3dkZXJTdHJlbmd0aCAqIHRoaXMuR2V0Q3JpdGljYWxWZWxvY2l0eShzdGVwKTtcclxuICAgIGNvbnN0IG1pbldlaWdodCA9IDEuMCAtIGIyX3BhcnRpY2xlU3RyaWRlO1xyXG4gICAgY29uc3QgaW52X21hc3MgPSB0aGlzLkdldFBhcnRpY2xlSW52TWFzcygpO1xyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4O1xyXG4gICAgICBpZiAodGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbYV0gJiBiMlBhcnRpY2xlRmxhZy5iMl9wb3dkZXJQYXJ0aWNsZSkge1xyXG4gICAgICAgIGNvbnN0IHcgPSBjb250YWN0LndlaWdodDtcclxuICAgICAgICBpZiAodyA+IG1pbldlaWdodCkge1xyXG4gICAgICAgICAgY29uc3QgYiA9IGNvbnRhY3QuYm9keTtcclxuICAgICAgICAgIGNvbnN0IG0gPSBjb250YWN0Lm1hc3M7XHJcbiAgICAgICAgICBjb25zdCBwID0gcG9zX2RhdGFbYV07XHJcbiAgICAgICAgICBjb25zdCBuID0gY29udGFjdC5ub3JtYWw7XHJcbiAgICAgICAgICBjb25zdCBmID0gYjJWZWMyLk11bFNWKHBvd2RlclN0cmVuZ3RoICogbSAqICh3IC0gbWluV2VpZ2h0KSwgbiwgc19mKTtcclxuICAgICAgICAgIHZlbF9kYXRhW2FdLlNlbGZNdWxTdWIoaW52X21hc3MsIGYpO1xyXG4gICAgICAgICAgYi5BcHBseUxpbmVhckltcHVsc2UoZiwgcCwgdHJ1ZSk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGlmIChjb250YWN0LmZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfcG93ZGVyUGFydGljbGUpIHtcclxuICAgICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgICAgaWYgKHcgPiBtaW5XZWlnaHQpIHtcclxuICAgICAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4QTtcclxuICAgICAgICAgIGNvbnN0IGIgPSBjb250YWN0LmluZGV4QjtcclxuICAgICAgICAgIGNvbnN0IG4gPSBjb250YWN0Lm5vcm1hbDtcclxuICAgICAgICAgIGNvbnN0IGYgPSBiMlZlYzIuTXVsU1YocG93ZGVyU3RyZW5ndGggKiAodyAtIG1pbldlaWdodCksIG4sIHNfZik7XHJcbiAgICAgICAgICB2ZWxfZGF0YVthXS5TZWxmU3ViKGYpO1xyXG4gICAgICAgICAgdmVsX2RhdGFbYl0uU2VsZkFkZChmKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIHN0YXRpYyByZWFkb25seSBTb2x2ZVBvd2Rlcl9zX2YgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlU29saWQoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3Qgc19mID0gYjJQYXJ0aWNsZVN5c3RlbS5Tb2x2ZVNvbGlkX3NfZjtcclxuICAgIGNvbnN0IHZlbF9kYXRhID0gdGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGE7XHJcbiAgICAvLyBhcHBsaWVzIGV4dHJhIHJlcHVsc2l2ZSBmb3JjZSBmcm9tIHNvbGlkIHBhcnRpY2xlIGdyb3Vwc1xyXG4gICAgdGhpcy5tX2RlcHRoQnVmZmVyID0gdGhpcy5SZXF1ZXN0QnVmZmVyKHRoaXMubV9kZXB0aEJ1ZmZlcik7XHJcbiAgICBjb25zdCBlamVjdGlvblN0cmVuZ3RoID0gc3RlcC5pbnZfZHQgKiB0aGlzLm1fZGVmLmVqZWN0aW9uU3RyZW5ndGg7XHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9jb250YWN0QnVmZmVyLmRhdGFba107XHJcbiAgICAgIGNvbnN0IGEgPSBjb250YWN0LmluZGV4QTtcclxuICAgICAgY29uc3QgYiA9IGNvbnRhY3QuaW5kZXhCO1xyXG4gICAgICBpZiAodGhpcy5tX2dyb3VwQnVmZmVyW2FdICE9PSB0aGlzLm1fZ3JvdXBCdWZmZXJbYl0pIHtcclxuICAgICAgICBjb25zdCB3ID0gY29udGFjdC53ZWlnaHQ7XHJcbiAgICAgICAgY29uc3QgbiA9IGNvbnRhY3Qubm9ybWFsO1xyXG4gICAgICAgIGNvbnN0IGggPSB0aGlzLm1fZGVwdGhCdWZmZXJbYV0gKyB0aGlzLm1fZGVwdGhCdWZmZXJbYl07XHJcbiAgICAgICAgY29uc3QgZiA9IGIyVmVjMi5NdWxTVihlamVjdGlvblN0cmVuZ3RoICogaCAqIHcsIG4sIHNfZik7XHJcbiAgICAgICAgdmVsX2RhdGFbYV0uU2VsZlN1YihmKTtcclxuICAgICAgICB2ZWxfZGF0YVtiXS5TZWxmQWRkKGYpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgU29sdmVTb2xpZF9zX2YgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlRm9yY2Uoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3QgdmVsX2RhdGEgPSB0aGlzLm1fdmVsb2NpdHlCdWZmZXIuZGF0YTtcclxuICAgIGNvbnN0IHZlbG9jaXR5UGVyRm9yY2UgPSBzdGVwLmR0ICogdGhpcy5HZXRQYXJ0aWNsZUludk1hc3MoKTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyBpKyspIHtcclxuICAgICAgLy8vbV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2ldICs9IHZlbG9jaXR5UGVyRm9yY2UgKiBtX2ZvcmNlQnVmZmVyW2ldO1xyXG4gICAgICB2ZWxfZGF0YVtpXS5TZWxmTXVsQWRkKHZlbG9jaXR5UGVyRm9yY2UsIHRoaXMubV9mb3JjZUJ1ZmZlcltpXSk7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1faGFzRm9yY2UgPSBmYWxzZTtcclxuICB9XHJcblxyXG4gIFNvbHZlQ29sb3JNaXhpbmcoKTogdm9pZCB7XHJcbiAgICAvLyBtaXhlcyBjb2xvciBiZXR3ZWVuIGNvbnRhY3RpbmcgcGFydGljbGVzXHJcbiAgICBjb25zdCBjb2xvck1peGluZyA9IDAuNSAqIHRoaXMubV9kZWYuY29sb3JNaXhpbmdTdHJlbmd0aDtcclxuICAgIGlmIChjb2xvck1peGluZykge1xyXG4gICAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9jb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgICBjb25zdCBhID0gY29udGFjdC5pbmRleEE7XHJcbiAgICAgICAgY29uc3QgYiA9IGNvbnRhY3QuaW5kZXhCO1xyXG4gICAgICAgIGlmIChcclxuICAgICAgICAgIHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2FdICZcclxuICAgICAgICAgIHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2JdICZcclxuICAgICAgICAgIGIyUGFydGljbGVGbGFnLmIyX2NvbG9yTWl4aW5nUGFydGljbGVcclxuICAgICAgICApIHtcclxuICAgICAgICAgIGNvbnN0IGNvbG9yQSA9IHRoaXMubV9jb2xvckJ1ZmZlci5kYXRhW2FdO1xyXG4gICAgICAgICAgY29uc3QgY29sb3JCID0gdGhpcy5tX2NvbG9yQnVmZmVyLmRhdGFbYl07XHJcbiAgICAgICAgICAvLyBVc2UgdGhlIHN0YXRpYyBtZXRob2QgdG8gZW5zdXJlIGNlcnRhaW4gY29tcGlsZXJzIGlubGluZVxyXG4gICAgICAgICAgLy8gdGhpcyBjb3JyZWN0bHkuXHJcbiAgICAgICAgICBiMkNvbG9yLk1peENvbG9ycyhjb2xvckEsIGNvbG9yQiwgY29sb3JNaXhpbmcpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgU29sdmVab21iaWUoKTogdm9pZCB7XHJcbiAgICAvLyByZW1vdmVzIHBhcnRpY2xlcyB3aXRoIHpvbWJpZSBmbGFnXHJcbiAgICBsZXQgbmV3Q291bnQgPSAwO1xyXG4gICAgY29uc3QgbmV3SW5kaWNlc0FycmF5OiBudW1iZXJbXSA9IFtdOyAvLyBUT0RPOiBzdGF0aWNcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyBpKyspIHtcclxuICAgICAgbmV3SW5kaWNlc0FycmF5W2ldID0gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcbiAgICB9XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KG5ld0luZGljZXNBcnJheS5sZW5ndGggPT09IHRoaXMubV9jb3VudCk7XHJcbiAgICBsZXQgYWxsUGFydGljbGVGbGFncyA9IDA7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgaSsrKSB7XHJcbiAgICAgIGNvbnN0IGZsYWdzID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbaV07XHJcbiAgICAgIGlmIChmbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX3pvbWJpZVBhcnRpY2xlKSB7XHJcbiAgICAgICAgY29uc3QgZGVzdHJ1Y3Rpb25MaXN0ZW5lciA9IHRoaXMubV93b3JsZC5tX2Rlc3RydWN0aW9uTGlzdGVuZXI7XHJcbiAgICAgICAgaWYgKGZsYWdzICYgYjJQYXJ0aWNsZUZsYWcuYjJfZGVzdHJ1Y3Rpb25MaXN0ZW5lclBhcnRpY2xlICYmIGRlc3RydWN0aW9uTGlzdGVuZXIpIHtcclxuICAgICAgICAgIGRlc3RydWN0aW9uTGlzdGVuZXIuU2F5R29vZGJ5ZVBhcnRpY2xlKHRoaXMsIGkpO1xyXG4gICAgICAgIH1cclxuICAgICAgICAvLyBEZXN0cm95IHBhcnRpY2xlIGhhbmRsZS5cclxuICAgICAgICBpZiAodGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyLmRhdGEpIHtcclxuICAgICAgICAgIGNvbnN0IGhhbmRsZSA9IHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgICAgaWYgKGhhbmRsZSkge1xyXG4gICAgICAgICAgICBoYW5kbGUuaW5kZXggPSBiMl9pbnZhbGlkUGFydGljbGVJbmRleDtcclxuICAgICAgICAgICAgdGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyLmRhdGFbaV0gPSBudWxsO1xyXG4gICAgICAgICAgICAvLy9tX2hhbmRsZUFsbG9jYXRvci5GcmVlKGhhbmRsZSk7XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIG5ld0luZGljZXNBcnJheVtpXSA9IGIyX2ludmFsaWRQYXJ0aWNsZUluZGV4O1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIG5ld0luZGljZXNBcnJheVtpXSA9IG5ld0NvdW50O1xyXG4gICAgICAgIGlmIChpICE9PSBuZXdDb3VudCkge1xyXG4gICAgICAgICAgLy8gVXBkYXRlIGhhbmRsZSB0byByZWZlcmVuY2UgbmV3IHBhcnRpY2xlIGluZGV4LlxyXG4gICAgICAgICAgaWYgKHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhKSB7XHJcbiAgICAgICAgICAgIGNvbnN0IGhhbmRsZSA9IHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgICAgICBpZiAoaGFuZGxlKSB7XHJcbiAgICAgICAgICAgICAgaGFuZGxlLmluZGV4ID0gbmV3Q291bnQ7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgdGhpcy5tX2hhbmRsZUluZGV4QnVmZmVyLmRhdGFbbmV3Q291bnRdID0gaGFuZGxlO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbbmV3Q291bnRdID0gdGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGFbaV07XHJcbiAgICAgICAgICBpZiAodGhpcy5tX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YSkge1xyXG4gICAgICAgICAgICB0aGlzLm1fbGFzdEJvZHlDb250YWN0U3RlcEJ1ZmZlci5kYXRhW25ld0NvdW50XSA9IHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGFbXHJcbiAgICAgICAgICAgICAgaVxyXG4gICAgICAgICAgICBdO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgaWYgKHRoaXMubV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGEpIHtcclxuICAgICAgICAgICAgdGhpcy5tX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YVtuZXdDb3VudF0gPSB0aGlzLm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgaWYgKHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhKSB7XHJcbiAgICAgICAgICAgIHRoaXMubV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhW1xyXG4gICAgICAgICAgICAgIG5ld0NvdW50XHJcbiAgICAgICAgICAgIF0gPSB0aGlzLm1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIuZGF0YVtpXTtcclxuICAgICAgICAgIH1cclxuICAgICAgICAgIHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW25ld0NvdW50XS5Db3B5KHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2ldKTtcclxuICAgICAgICAgIHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW25ld0NvdW50XS5Db3B5KHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2ldKTtcclxuICAgICAgICAgIHRoaXMubV9ncm91cEJ1ZmZlcltuZXdDb3VudF0gPSB0aGlzLm1fZ3JvdXBCdWZmZXJbaV07XHJcbiAgICAgICAgICBpZiAodGhpcy5tX2hhc0ZvcmNlKSB7XHJcbiAgICAgICAgICAgIHRoaXMubV9mb3JjZUJ1ZmZlcltuZXdDb3VudF0uQ29weSh0aGlzLm1fZm9yY2VCdWZmZXJbaV0pO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgaWYgKHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlcikge1xyXG4gICAgICAgICAgICB0aGlzLm1fc3RhdGljUHJlc3N1cmVCdWZmZXJbbmV3Q291bnRdID0gdGhpcy5tX3N0YXRpY1ByZXNzdXJlQnVmZmVyW2ldO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgaWYgKHRoaXMubV9kZXB0aEJ1ZmZlcikge1xyXG4gICAgICAgICAgICB0aGlzLm1fZGVwdGhCdWZmZXJbbmV3Q291bnRdID0gdGhpcy5tX2RlcHRoQnVmZmVyW2ldO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgaWYgKHRoaXMubV9jb2xvckJ1ZmZlci5kYXRhKSB7XHJcbiAgICAgICAgICAgIHRoaXMubV9jb2xvckJ1ZmZlci5kYXRhW25ld0NvdW50XS5Db3B5KHRoaXMubV9jb2xvckJ1ZmZlci5kYXRhW2ldKTtcclxuICAgICAgICAgIH1cclxuICAgICAgICAgIGlmICh0aGlzLm1fdXNlckRhdGFCdWZmZXIuZGF0YSkge1xyXG4gICAgICAgICAgICB0aGlzLm1fdXNlckRhdGFCdWZmZXIuZGF0YVtuZXdDb3VudF0gPSB0aGlzLm1fdXNlckRhdGFCdWZmZXIuZGF0YVtpXTtcclxuICAgICAgICAgIH1cclxuICAgICAgICAgIGlmICh0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSkge1xyXG4gICAgICAgICAgICB0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtuZXdDb3VudF0gPSB0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtpXTtcclxuICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgbmV3Q291bnQrKztcclxuICAgICAgICBhbGxQYXJ0aWNsZUZsYWdzIHw9IGZsYWdzO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gcHJlZGljYXRlIGZ1bmN0aW9uc1xyXG4gICAgY29uc3QgVGVzdCA9IHtcclxuICAgICAgLy8vc3RhdGljIGJvb2wgSXNQcm94eUludmFsaWQoY29uc3QgUHJveHkmIHByb3h5KVxyXG4gICAgICBJc1Byb3h5SW52YWxpZDogKHByb3h5OiBiMlBhcnRpY2xlU3lzdGVtX1Byb3h5KSA9PiB7XHJcbiAgICAgICAgcmV0dXJuIHByb3h5LmluZGV4IDwgMDtcclxuICAgICAgfSxcclxuICAgICAgLy8vc3RhdGljIGJvb2wgSXNDb250YWN0SW52YWxpZChjb25zdCBiMlBhcnRpY2xlQ29udGFjdCYgY29udGFjdClcclxuICAgICAgSXNDb250YWN0SW52YWxpZDogKGNvbnRhY3Q6IGIyUGFydGljbGVDb250YWN0KSA9PiB7XHJcbiAgICAgICAgcmV0dXJuIGNvbnRhY3QuaW5kZXhBIDwgMCB8fCBjb250YWN0LmluZGV4QiA8IDA7XHJcbiAgICAgIH0sXHJcbiAgICAgIC8vL3N0YXRpYyBib29sIElzQm9keUNvbnRhY3RJbnZhbGlkKGNvbnN0IGIyUGFydGljbGVCb2R5Q29udGFjdCYgY29udGFjdClcclxuICAgICAgSXNCb2R5Q29udGFjdEludmFsaWQ6IChjb250YWN0OiBiMlBhcnRpY2xlQm9keUNvbnRhY3QpID0+IHtcclxuICAgICAgICByZXR1cm4gY29udGFjdC5pbmRleCA8IDA7XHJcbiAgICAgIH0sXHJcbiAgICAgIC8vL3N0YXRpYyBib29sIElzUGFpckludmFsaWQoY29uc3QgYjJQYXJ0aWNsZVBhaXImIHBhaXIpXHJcbiAgICAgIElzUGFpckludmFsaWQ6IChwYWlyOiBiMlBhcnRpY2xlUGFpcikgPT4ge1xyXG4gICAgICAgIHJldHVybiBwYWlyLmluZGV4QSA8IDAgfHwgcGFpci5pbmRleEIgPCAwO1xyXG4gICAgICB9LFxyXG4gICAgICAvLy9zdGF0aWMgYm9vbCBJc1RyaWFkSW52YWxpZChjb25zdCBiMlBhcnRpY2xlVHJpYWQmIHRyaWFkKVxyXG4gICAgICBJc1RyaWFkSW52YWxpZDogKHRyaWFkOiBiMlBhcnRpY2xlVHJpYWQpID0+IHtcclxuICAgICAgICByZXR1cm4gdHJpYWQuaW5kZXhBIDwgMCB8fCB0cmlhZC5pbmRleEIgPCAwIHx8IHRyaWFkLmluZGV4QyA8IDA7XHJcbiAgICAgIH0sXHJcbiAgICB9O1xyXG5cclxuICAgIC8vIHVwZGF0ZSBwcm94aWVzXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9wcm94eUJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IHByb3h5ID0gdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGFba107XHJcbiAgICAgIHByb3h5LmluZGV4ID0gbmV3SW5kaWNlc0FycmF5W3Byb3h5LmluZGV4XTtcclxuICAgIH1cclxuICAgIHRoaXMubV9wcm94eUJ1ZmZlci5SZW1vdmVJZihUZXN0LklzUHJveHlJbnZhbGlkKTtcclxuXHJcbiAgICAvLyB1cGRhdGUgY29udGFjdHNcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29udGFjdC5pbmRleEEgPSBuZXdJbmRpY2VzQXJyYXlbY29udGFjdC5pbmRleEFdO1xyXG4gICAgICBjb250YWN0LmluZGV4QiA9IG5ld0luZGljZXNBcnJheVtjb250YWN0LmluZGV4Ql07XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fY29udGFjdEJ1ZmZlci5SZW1vdmVJZihUZXN0LklzQ29udGFjdEludmFsaWQpO1xyXG5cclxuICAgIC8vIHVwZGF0ZSBwYXJ0aWNsZS1ib2R5IGNvbnRhY3RzXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IGNvbnRhY3QgPSB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29udGFjdC5pbmRleCA9IG5ld0luZGljZXNBcnJheVtjb250YWN0LmluZGV4XTtcclxuICAgIH1cclxuICAgIHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5SZW1vdmVJZihUZXN0LklzQm9keUNvbnRhY3RJbnZhbGlkKTtcclxuXHJcbiAgICAvLyB1cGRhdGUgcGFpcnNcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3BhaXJCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBwYWlyID0gdGhpcy5tX3BhaXJCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgcGFpci5pbmRleEEgPSBuZXdJbmRpY2VzQXJyYXlbcGFpci5pbmRleEFdO1xyXG4gICAgICBwYWlyLmluZGV4QiA9IG5ld0luZGljZXNBcnJheVtwYWlyLmluZGV4Ql07XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fcGFpckJ1ZmZlci5SZW1vdmVJZihUZXN0LklzUGFpckludmFsaWQpO1xyXG5cclxuICAgIC8vIHVwZGF0ZSB0cmlhZHNcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX3RyaWFkQnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgdHJpYWQgPSB0aGlzLm1fdHJpYWRCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgdHJpYWQuaW5kZXhBID0gbmV3SW5kaWNlc0FycmF5W3RyaWFkLmluZGV4QV07XHJcbiAgICAgIHRyaWFkLmluZGV4QiA9IG5ld0luZGljZXNBcnJheVt0cmlhZC5pbmRleEJdO1xyXG4gICAgICB0cmlhZC5pbmRleEMgPSBuZXdJbmRpY2VzQXJyYXlbdHJpYWQuaW5kZXhDXTtcclxuICAgIH1cclxuICAgIHRoaXMubV90cmlhZEJ1ZmZlci5SZW1vdmVJZihUZXN0LklzVHJpYWRJbnZhbGlkKTtcclxuXHJcbiAgICAvLyBVcGRhdGUgbGlmZXRpbWUgaW5kaWNlcy5cclxuICAgIGlmICh0aGlzLm1faW5kZXhCeUV4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEpIHtcclxuICAgICAgbGV0IHdyaXRlT2Zmc2V0ID0gMDtcclxuICAgICAgZm9yIChsZXQgcmVhZE9mZnNldCA9IDA7IHJlYWRPZmZzZXQgPCB0aGlzLm1fY291bnQ7IHJlYWRPZmZzZXQrKykge1xyXG4gICAgICAgIGNvbnN0IG5ld0luZGV4ID0gbmV3SW5kaWNlc0FycmF5W3RoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YVtyZWFkT2Zmc2V0XV07XHJcbiAgICAgICAgaWYgKG5ld0luZGV4ICE9PSBiMl9pbnZhbGlkUGFydGljbGVJbmRleCkge1xyXG4gICAgICAgICAgdGhpcy5tX2luZGV4QnlFeHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhW3dyaXRlT2Zmc2V0KytdID0gbmV3SW5kZXg7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gdXBkYXRlIGdyb3Vwc1xyXG4gICAgZm9yIChsZXQgZ3JvdXAgPSB0aGlzLm1fZ3JvdXBMaXN0OyBncm91cDsgZ3JvdXAgPSBncm91cC5HZXROZXh0KCkpIHtcclxuICAgICAgbGV0IGZpcnN0SW5kZXggPSBuZXdDb3VudDtcclxuICAgICAgbGV0IGxhc3RJbmRleCA9IDA7XHJcbiAgICAgIGxldCBtb2RpZmllZCA9IGZhbHNlO1xyXG4gICAgICBmb3IgKGxldCBpID0gZ3JvdXAubV9maXJzdEluZGV4OyBpIDwgZ3JvdXAubV9sYXN0SW5kZXg7IGkrKykge1xyXG4gICAgICAgIGNvbnN0IGogPSBuZXdJbmRpY2VzQXJyYXlbaV07XHJcbiAgICAgICAgaWYgKGogPj0gMCkge1xyXG4gICAgICAgICAgZmlyc3RJbmRleCA9IGIyTWluSW50KGZpcnN0SW5kZXgsIGopO1xyXG4gICAgICAgICAgbGFzdEluZGV4ID0gYjJNYXhJbnQobGFzdEluZGV4LCBqICsgMSk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIG1vZGlmaWVkID0gdHJ1ZTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgICAgaWYgKGZpcnN0SW5kZXggPCBsYXN0SW5kZXgpIHtcclxuICAgICAgICBncm91cC5tX2ZpcnN0SW5kZXggPSBmaXJzdEluZGV4O1xyXG4gICAgICAgIGdyb3VwLm1fbGFzdEluZGV4ID0gbGFzdEluZGV4O1xyXG4gICAgICAgIGlmIChtb2RpZmllZCkge1xyXG4gICAgICAgICAgaWYgKGdyb3VwLm1fZ3JvdXBGbGFncyAmIGIyUGFydGljbGVHcm91cEZsYWcuYjJfc29saWRQYXJ0aWNsZUdyb3VwKSB7XHJcbiAgICAgICAgICAgIHRoaXMuU2V0R3JvdXBGbGFncyhcclxuICAgICAgICAgICAgICBncm91cCxcclxuICAgICAgICAgICAgICBncm91cC5tX2dyb3VwRmxhZ3MgfCBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3BhcnRpY2xlR3JvdXBOZWVkc1VwZGF0ZURlcHRoLFxyXG4gICAgICAgICAgICApO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICBncm91cC5tX2ZpcnN0SW5kZXggPSAwO1xyXG4gICAgICAgIGdyb3VwLm1fbGFzdEluZGV4ID0gMDtcclxuICAgICAgICBpZiAoIShncm91cC5tX2dyb3VwRmxhZ3MgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3BhcnRpY2xlR3JvdXBDYW5CZUVtcHR5KSkge1xyXG4gICAgICAgICAgdGhpcy5TZXRHcm91cEZsYWdzKFxyXG4gICAgICAgICAgICBncm91cCxcclxuICAgICAgICAgICAgZ3JvdXAubV9ncm91cEZsYWdzIHwgYjJQYXJ0aWNsZUdyb3VwRmxhZy5iMl9wYXJ0aWNsZUdyb3VwV2lsbEJlRGVzdHJveWVkLFxyXG4gICAgICAgICAgKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAvLyB1cGRhdGUgcGFydGljbGUgY291bnRcclxuICAgIHRoaXMubV9jb3VudCA9IG5ld0NvdW50O1xyXG4gICAgdGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgPSBhbGxQYXJ0aWNsZUZsYWdzO1xyXG4gICAgdGhpcy5tX25lZWRzVXBkYXRlQWxsUGFydGljbGVGbGFncyA9IGZhbHNlO1xyXG5cclxuICAgIC8vIGRlc3Ryb3kgYm9kaWVzIHdpdGggbm8gcGFydGljbGVzXHJcbiAgICBmb3IgKGxldCBncm91cCA9IHRoaXMubV9ncm91cExpc3Q7IGdyb3VwOyApIHtcclxuICAgICAgY29uc3QgbmV4dCA9IGdyb3VwLkdldE5leHQoKTtcclxuICAgICAgaWYgKGdyb3VwLm1fZ3JvdXBGbGFncyAmIGIyUGFydGljbGVHcm91cEZsYWcuYjJfcGFydGljbGVHcm91cFdpbGxCZURlc3Ryb3llZCkge1xyXG4gICAgICAgIHRoaXMuRGVzdHJveVBhcnRpY2xlR3JvdXAoZ3JvdXApO1xyXG4gICAgICB9XHJcbiAgICAgIGdyb3VwID0gbmV4dDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIERlc3Ryb3kgYWxsIHBhcnRpY2xlcyB3aGljaCBoYXZlIG91dGxpdmVkIHRoZWlyIGxpZmV0aW1lcyBzZXRcclxuICAgKiBieSBTZXRQYXJ0aWNsZUxpZmV0aW1lKCkuXHJcbiAgICovXHJcbiAgU29sdmVMaWZldGltZXMoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgLy8gVXBkYXRlIHRoZSB0aW1lIGVsYXBzZWQuXHJcbiAgICB0aGlzLm1fdGltZUVsYXBzZWQgPSB0aGlzLkxpZmV0aW1lVG9FeHBpcmF0aW9uVGltZShzdGVwLmR0KTtcclxuICAgIC8vIEdldCB0aGUgZmxvb3IgKG5vbi1mcmFjdGlvbmFsIGNvbXBvbmVudCkgb2YgdGhlIGVsYXBzZWQgdGltZS5cclxuICAgIGNvbnN0IHF1YW50aXplZFRpbWVFbGFwc2VkID0gdGhpcy5HZXRRdWFudGl6ZWRUaW1lRWxhcHNlZCgpO1xyXG5cclxuICAgIGNvbnN0IGV4cGlyYXRpb25UaW1lcyA9IHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgZXhwaXJhdGlvblRpbWVJbmRpY2VzID0gdGhpcy5tX2luZGV4QnlFeHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhO1xyXG4gICAgY29uc3QgcGFydGljbGVDb3VudCA9IHRoaXMuR2V0UGFydGljbGVDb3VudCgpO1xyXG4gICAgLy8gU29ydCB0aGUgbGlmZXRpbWUgYnVmZmVyIGlmIGl0J3MgcmVxdWlyZWQuXHJcbiAgICBpZiAodGhpcy5tX2V4cGlyYXRpb25UaW1lQnVmZmVyUmVxdWlyZXNTb3J0aW5nKSB7XHJcbiAgICAgIC8vL2NvbnN0IEV4cGlyYXRpb25UaW1lQ29tcGFyYXRvciBleHBpcmF0aW9uVGltZUNvbXBhcmF0b3IoZXhwaXJhdGlvblRpbWVzKTtcclxuICAgICAgLy8vc3RkOjpzb3J0KGV4cGlyYXRpb25UaW1lSW5kaWNlcywgZXhwaXJhdGlvblRpbWVJbmRpY2VzICsgcGFydGljbGVDb3VudCwgZXhwaXJhdGlvblRpbWVDb21wYXJhdG9yKTtcclxuXHJcbiAgICAgIC8qKlxyXG4gICAgICAgKiBDb21wYXJlIHRoZSBsaWZldGltZSBvZiBwYXJ0aWNsZUluZGV4QSBhbmQgcGFydGljbGVJbmRleEJcclxuICAgICAgICogcmV0dXJuaW5nIHRydWUgaWYgdGhlIGxpZmV0aW1lIG9mIEEgaXMgZ3JlYXRlciB0aGFuIEIgZm9yXHJcbiAgICAgICAqIHBhcnRpY2xlcyB0aGF0IHdpbGwgZXhwaXJlLiAgSWYgZWl0aGVyIHBhcnRpY2xlJ3MgbGlmZXRpbWUgaXNcclxuICAgICAgICogaW5maW5pdGUgKDw9IDAuMGYpIHRoaXMgZnVuY3Rpb24gcmV0dXJuIHRydWUgaWYgdGhlIGxpZmV0aW1lXHJcbiAgICAgICAqIG9mIEEgaXMgbGVzc2VyIHRoYW4gQi4gV2hlbiB1c2VkIHdpdGggc3RkOjpzb3J0KCkgdGhpc1xyXG4gICAgICAgKiByZXN1bHRzIGluIGFuIGFycmF5IG9mIHBhcnRpY2xlIGluZGljaWVzIHNvcnRlZCBpbiByZXZlcnNlXHJcbiAgICAgICAqIG9yZGVyIGJ5IHBhcnRpY2xlIGxpZmV0aW1lLlxyXG4gICAgICAgKlxyXG4gICAgICAgKiBGb3IgZXhhbXBsZSwgdGhlIHNldCBvZiBsaWZldGltZXNcclxuICAgICAgICogKDEuMCwgMC43LCAwLjMsIDAuMCwgLTEuMCwgMi4wKVxyXG4gICAgICAgKiB3b3VsZCBiZSBzb3J0ZWQgYXNcclxuICAgICAgICogKDAuMCwgMS4wLCAtMi4wLCAxLjAsIDAuNywgMC4zKVxyXG4gICAgICAgKi9cclxuICAgICAgY29uc3QgRXhwaXJhdGlvblRpbWVDb21wYXJhdG9yID0gKFxyXG4gICAgICAgIHBhcnRpY2xlSW5kZXhBOiBudW1iZXIsXHJcbiAgICAgICAgcGFydGljbGVJbmRleEI6IG51bWJlcixcclxuICAgICAgKTogYm9vbGVhbiA9PiB7XHJcbiAgICAgICAgY29uc3QgZXhwaXJhdGlvblRpbWVBID0gZXhwaXJhdGlvblRpbWVzW3BhcnRpY2xlSW5kZXhBXTtcclxuICAgICAgICBjb25zdCBleHBpcmF0aW9uVGltZUIgPSBleHBpcmF0aW9uVGltZXNbcGFydGljbGVJbmRleEJdO1xyXG4gICAgICAgIGNvbnN0IGluZmluaXRlRXhwaXJhdGlvblRpbWVBID0gZXhwaXJhdGlvblRpbWVBIDw9IDAuMDtcclxuICAgICAgICBjb25zdCBpbmZpbml0ZUV4cGlyYXRpb25UaW1lQiA9IGV4cGlyYXRpb25UaW1lQiA8PSAwLjA7XHJcbiAgICAgICAgcmV0dXJuIGluZmluaXRlRXhwaXJhdGlvblRpbWVBID09PSBpbmZpbml0ZUV4cGlyYXRpb25UaW1lQlxyXG4gICAgICAgICAgPyBleHBpcmF0aW9uVGltZUEgPiBleHBpcmF0aW9uVGltZUJcclxuICAgICAgICAgIDogaW5maW5pdGVFeHBpcmF0aW9uVGltZUE7XHJcbiAgICAgIH07XHJcblxyXG4gICAgICBzdGRfc29ydChleHBpcmF0aW9uVGltZUluZGljZXMsIDAsIHBhcnRpY2xlQ291bnQsIEV4cGlyYXRpb25UaW1lQ29tcGFyYXRvcik7XHJcblxyXG4gICAgICB0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXJSZXF1aXJlc1NvcnRpbmcgPSBmYWxzZTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBEZXN0cm95IHBhcnRpY2xlcyB3aGljaCBoYXZlIGV4cGlyZWQuXHJcbiAgICBmb3IgKGxldCBpID0gcGFydGljbGVDb3VudCAtIDE7IGkgPj0gMDsgLS1pKSB7XHJcbiAgICAgIGNvbnN0IHBhcnRpY2xlSW5kZXggPSBleHBpcmF0aW9uVGltZUluZGljZXNbaV07XHJcbiAgICAgIGNvbnN0IGV4cGlyYXRpb25UaW1lID0gZXhwaXJhdGlvblRpbWVzW3BhcnRpY2xlSW5kZXhdO1xyXG4gICAgICAvLyBJZiBubyBwYXJ0aWNsZXMgbmVlZCB0byBiZSBkZXN0cm95ZWQsIHNraXAgdGhpcy5cclxuICAgICAgaWYgKHF1YW50aXplZFRpbWVFbGFwc2VkIDwgZXhwaXJhdGlvblRpbWUgfHwgZXhwaXJhdGlvblRpbWUgPD0gMCkge1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcbiAgICAgIC8vIERlc3Ryb3kgdGhpcyBwYXJ0aWNsZS5cclxuICAgICAgdGhpcy5EZXN0cm95UGFydGljbGUocGFydGljbGVJbmRleCk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBSb3RhdGVCdWZmZXIoc3RhcnQ6IG51bWJlciwgbWlkOiBudW1iZXIsIGVuZDogbnVtYmVyKTogdm9pZCB7XHJcbiAgICAvLyBtb3ZlIHRoZSBwYXJ0aWNsZXMgYXNzaWduZWQgdG8gdGhlIGdpdmVuIGdyb3VwIHRvd2FyZCB0aGUgZW5kIG9mIGFycmF5XHJcbiAgICBpZiAoc3RhcnQgPT09IG1pZCB8fCBtaWQgPT09IGVuZCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChtaWQgPj0gc3RhcnQgJiYgbWlkIDw9IGVuZCk7XHJcblxyXG4gICAgLy8vc3RkOjpyb3RhdGUobV9mbGFnc0J1ZmZlci5kYXRhICsgc3RhcnQsIG1fZmxhZ3NCdWZmZXIuZGF0YSArIG1pZCwgbV9mbGFnc0J1ZmZlci5kYXRhICsgZW5kKTtcclxuICAgIHN0ZF9yb3RhdGUodGhpcy5tX2ZsYWdzQnVmZmVyLmRhdGEsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICBpZiAodGhpcy5tX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YSkge1xyXG4gICAgICAvLy9zdGQ6OnJvdGF0ZShtX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YSArIHN0YXJ0LCBtX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YSArIG1pZCwgbV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGEgKyBlbmQpO1xyXG4gICAgICBzdGRfcm90YXRlKHRoaXMubV9sYXN0Qm9keUNvbnRhY3RTdGVwQnVmZmVyLmRhdGEsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YSkge1xyXG4gICAgICAvLy9zdGQ6OnJvdGF0ZShtX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YSArIHN0YXJ0LCBtX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YSArIG1pZCwgbV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGEgKyBlbmQpO1xyXG4gICAgICBzdGRfcm90YXRlKHRoaXMubV9ib2R5Q29udGFjdENvdW50QnVmZmVyLmRhdGEsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX2NvbnNlY3V0aXZlQ29udGFjdFN0ZXBzQnVmZmVyLmRhdGEpIHtcclxuICAgICAgLy8vc3RkOjpyb3RhdGUobV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhICsgc3RhcnQsIG1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIuZGF0YSArIG1pZCwgbV9jb25zZWN1dGl2ZUNvbnRhY3RTdGVwc0J1ZmZlci5kYXRhICsgZW5kKTtcclxuICAgICAgc3RkX3JvdGF0ZSh0aGlzLm1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIuZGF0YSwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgIH1cclxuICAgIC8vL3N0ZDo6cm90YXRlKG1fcG9zaXRpb25CdWZmZXIuZGF0YSArIHN0YXJ0LCBtX3Bvc2l0aW9uQnVmZmVyLmRhdGEgKyBtaWQsIG1fcG9zaXRpb25CdWZmZXIuZGF0YSArIGVuZCk7XHJcbiAgICBzdGRfcm90YXRlKHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgLy8vc3RkOjpyb3RhdGUobV92ZWxvY2l0eUJ1ZmZlci5kYXRhICsgc3RhcnQsIG1fdmVsb2NpdHlCdWZmZXIuZGF0YSArIG1pZCwgbV92ZWxvY2l0eUJ1ZmZlci5kYXRhICsgZW5kKTtcclxuICAgIHN0ZF9yb3RhdGUodGhpcy5tX3ZlbG9jaXR5QnVmZmVyLmRhdGEsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICAvLy9zdGQ6OnJvdGF0ZShtX2dyb3VwQnVmZmVyICsgc3RhcnQsIG1fZ3JvdXBCdWZmZXIgKyBtaWQsIG1fZ3JvdXBCdWZmZXIgKyBlbmQpO1xyXG4gICAgc3RkX3JvdGF0ZSh0aGlzLm1fZ3JvdXBCdWZmZXIsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICBpZiAodGhpcy5tX2hhc0ZvcmNlKSB7XHJcbiAgICAgIC8vL3N0ZDo6cm90YXRlKG1fZm9yY2VCdWZmZXIgKyBzdGFydCwgbV9mb3JjZUJ1ZmZlciArIG1pZCwgbV9mb3JjZUJ1ZmZlciArIGVuZCk7XHJcbiAgICAgIHN0ZF9yb3RhdGUodGhpcy5tX2ZvcmNlQnVmZmVyLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlcikge1xyXG4gICAgICAvLy9zdGQ6OnJvdGF0ZShtX3N0YXRpY1ByZXNzdXJlQnVmZmVyICsgc3RhcnQsIG1fc3RhdGljUHJlc3N1cmVCdWZmZXIgKyBtaWQsIG1fc3RhdGljUHJlc3N1cmVCdWZmZXIgKyBlbmQpO1xyXG4gICAgICBzdGRfcm90YXRlKHRoaXMubV9zdGF0aWNQcmVzc3VyZUJ1ZmZlciwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fZGVwdGhCdWZmZXIpIHtcclxuICAgICAgLy8vc3RkOjpyb3RhdGUobV9kZXB0aEJ1ZmZlciArIHN0YXJ0LCBtX2RlcHRoQnVmZmVyICsgbWlkLCBtX2RlcHRoQnVmZmVyICsgZW5kKTtcclxuICAgICAgc3RkX3JvdGF0ZSh0aGlzLm1fZGVwdGhCdWZmZXIsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5tX2NvbG9yQnVmZmVyLmRhdGEpIHtcclxuICAgICAgLy8vc3RkOjpyb3RhdGUobV9jb2xvckJ1ZmZlci5kYXRhICsgc3RhcnQsIG1fY29sb3JCdWZmZXIuZGF0YSArIG1pZCwgbV9jb2xvckJ1ZmZlci5kYXRhICsgZW5kKTtcclxuICAgICAgc3RkX3JvdGF0ZSh0aGlzLm1fY29sb3JCdWZmZXIuZGF0YSwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fdXNlckRhdGFCdWZmZXIuZGF0YSkge1xyXG4gICAgICAvLy9zdGQ6OnJvdGF0ZShtX3VzZXJEYXRhQnVmZmVyLmRhdGEgKyBzdGFydCwgbV91c2VyRGF0YUJ1ZmZlci5kYXRhICsgbWlkLCBtX3VzZXJEYXRhQnVmZmVyLmRhdGEgKyBlbmQpO1xyXG4gICAgICBzdGRfcm90YXRlKHRoaXMubV91c2VyRGF0YUJ1ZmZlci5kYXRhLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFVwZGF0ZSBoYW5kbGUgaW5kaWNlcy5cclxuICAgIGlmICh0aGlzLm1faGFuZGxlSW5kZXhCdWZmZXIuZGF0YSkge1xyXG4gICAgICAvLy9zdGQ6OnJvdGF0ZShtX2hhbmRsZUluZGV4QnVmZmVyLmRhdGEgKyBzdGFydCwgbV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhICsgbWlkLCBtX2hhbmRsZUluZGV4QnVmZmVyLmRhdGEgKyBlbmQpO1xyXG4gICAgICBzdGRfcm90YXRlKHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgICBmb3IgKGxldCBpID0gc3RhcnQ7IGkgPCBlbmQ7ICsraSkge1xyXG4gICAgICAgIGNvbnN0IGhhbmRsZSA9IHRoaXMubV9oYW5kbGVJbmRleEJ1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgIGlmIChoYW5kbGUpIHtcclxuICAgICAgICAgIGhhbmRsZS5pbmRleCA9IG5ld0luZGljZXMoaGFuZGxlLmluZGV4LCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGlmICh0aGlzLm1fZXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YSkge1xyXG4gICAgICAvLy9zdGQ6OnJvdGF0ZShtX2V4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEgKyBzdGFydCwgbV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhICsgbWlkLCBtX2V4cGlyYXRpb25UaW1lQnVmZmVyLmRhdGEgKyBlbmQpO1xyXG4gICAgICBzdGRfcm90YXRlKHRoaXMubV9leHBpcmF0aW9uVGltZUJ1ZmZlci5kYXRhLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgICAvLyBVcGRhdGUgZXhwaXJhdGlvbiB0aW1lIGJ1ZmZlciBpbmRpY2VzLlxyXG4gICAgICBjb25zdCBwYXJ0aWNsZUNvdW50ID0gdGhpcy5HZXRQYXJ0aWNsZUNvdW50KCk7XHJcbiAgICAgIGNvbnN0IGluZGV4QnlFeHBpcmF0aW9uVGltZSA9IHRoaXMubV9pbmRleEJ5RXhwaXJhdGlvblRpbWVCdWZmZXIuZGF0YTtcclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBwYXJ0aWNsZUNvdW50OyArK2kpIHtcclxuICAgICAgICBpbmRleEJ5RXhwaXJhdGlvblRpbWVbaV0gPSBuZXdJbmRpY2VzKGluZGV4QnlFeHBpcmF0aW9uVGltZVtpXSwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIHVwZGF0ZSBwcm94aWVzXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV9wcm94eUJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IHByb3h5ID0gdGhpcy5tX3Byb3h5QnVmZmVyLmRhdGFba107XHJcbiAgICAgIHByb3h5LmluZGV4ID0gbmV3SW5kaWNlcyhwcm94eS5pbmRleCwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyB1cGRhdGUgY29udGFjdHNcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2NvbnRhY3RCdWZmZXIuY291bnQ7IGsrKykge1xyXG4gICAgICBjb25zdCBjb250YWN0ID0gdGhpcy5tX2NvbnRhY3RCdWZmZXIuZGF0YVtrXTtcclxuICAgICAgY29udGFjdC5pbmRleEEgPSBuZXdJbmRpY2VzKGNvbnRhY3QuaW5kZXhBLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgICBjb250YWN0LmluZGV4QiA9IG5ld0luZGljZXMoY29udGFjdC5pbmRleEIsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gdXBkYXRlIHBhcnRpY2xlLWJvZHkgY29udGFjdHNcclxuICAgIGZvciAobGV0IGsgPSAwOyBrIDwgdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmNvdW50OyBrKyspIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBjb250YWN0LmluZGV4ID0gbmV3SW5kaWNlcyhjb250YWN0LmluZGV4LCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIHVwZGF0ZSBwYWlyc1xyXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCB0aGlzLm1fcGFpckJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IHBhaXIgPSB0aGlzLm1fcGFpckJ1ZmZlci5kYXRhW2tdO1xyXG4gICAgICBwYWlyLmluZGV4QSA9IG5ld0luZGljZXMocGFpci5pbmRleEEsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICAgIHBhaXIuaW5kZXhCID0gbmV3SW5kaWNlcyhwYWlyLmluZGV4Qiwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyB1cGRhdGUgdHJpYWRzXHJcbiAgICBmb3IgKGxldCBrID0gMDsgayA8IHRoaXMubV90cmlhZEJ1ZmZlci5jb3VudDsgaysrKSB7XHJcbiAgICAgIGNvbnN0IHRyaWFkID0gdGhpcy5tX3RyaWFkQnVmZmVyLmRhdGFba107XHJcbiAgICAgIHRyaWFkLmluZGV4QSA9IG5ld0luZGljZXModHJpYWQuaW5kZXhBLCBzdGFydCwgbWlkLCBlbmQpO1xyXG4gICAgICB0cmlhZC5pbmRleEIgPSBuZXdJbmRpY2VzKHRyaWFkLmluZGV4Qiwgc3RhcnQsIG1pZCwgZW5kKTtcclxuICAgICAgdHJpYWQuaW5kZXhDID0gbmV3SW5kaWNlcyh0cmlhZC5pbmRleEMsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gdXBkYXRlIGdyb3Vwc1xyXG4gICAgZm9yIChsZXQgZ3JvdXAgPSB0aGlzLm1fZ3JvdXBMaXN0OyBncm91cDsgZ3JvdXAgPSBncm91cC5HZXROZXh0KCkpIHtcclxuICAgICAgZ3JvdXAubV9maXJzdEluZGV4ID0gbmV3SW5kaWNlcyhncm91cC5tX2ZpcnN0SW5kZXgsIHN0YXJ0LCBtaWQsIGVuZCk7XHJcbiAgICAgIGdyb3VwLm1fbGFzdEluZGV4ID0gbmV3SW5kaWNlcyhncm91cC5tX2xhc3RJbmRleCAtIDEsIHN0YXJ0LCBtaWQsIGVuZCkgKyAxO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgR2V0Q3JpdGljYWxWZWxvY2l0eShzdGVwOiBiMlRpbWVTdGVwKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fcGFydGljbGVEaWFtZXRlciAqIHN0ZXAuaW52X2R0O1xyXG4gIH1cclxuXHJcbiAgR2V0Q3JpdGljYWxWZWxvY2l0eVNxdWFyZWQoc3RlcDogYjJUaW1lU3RlcCk6IG51bWJlciB7XHJcbiAgICBjb25zdCB2ZWxvY2l0eSA9IHRoaXMuR2V0Q3JpdGljYWxWZWxvY2l0eShzdGVwKTtcclxuICAgIHJldHVybiB2ZWxvY2l0eSAqIHZlbG9jaXR5O1xyXG4gIH1cclxuXHJcbiAgR2V0Q3JpdGljYWxQcmVzc3VyZShzdGVwOiBiMlRpbWVTdGVwKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fZGVmLmRlbnNpdHkgKiB0aGlzLkdldENyaXRpY2FsVmVsb2NpdHlTcXVhcmVkKHN0ZXApO1xyXG4gIH1cclxuXHJcbiAgR2V0UGFydGljbGVTdHJpZGUoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiBiMl9wYXJ0aWNsZVN0cmlkZSAqIHRoaXMubV9wYXJ0aWNsZURpYW1ldGVyO1xyXG4gIH1cclxuXHJcbiAgR2V0UGFydGljbGVNYXNzKCk6IG51bWJlciB7XHJcbiAgICBjb25zdCBzdHJpZGUgPSB0aGlzLkdldFBhcnRpY2xlU3RyaWRlKCk7XHJcbiAgICByZXR1cm4gdGhpcy5tX2RlZi5kZW5zaXR5ICogc3RyaWRlICogc3RyaWRlO1xyXG4gIH1cclxuXHJcbiAgR2V0UGFydGljbGVJbnZNYXNzKCk6IG51bWJlciB7XHJcbiAgICAvLy9yZXR1cm4gMS43Nzc3NzcgKiB0aGlzLm1faW52ZXJzZURlbnNpdHkgKiB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogdGhpcy5tX2ludmVyc2VEaWFtZXRlcjtcclxuICAgIC8vIG1hc3MgPSBkZW5zaXR5ICogc3RyaWRlXjIsIHNvIHdlIHRha2UgdGhlIGludmVyc2Ugb2YgdGhpcy5cclxuICAgIGNvbnN0IGludmVyc2VTdHJpZGUgPSB0aGlzLm1faW52ZXJzZURpYW1ldGVyICogKDEuMCAvIGIyX3BhcnRpY2xlU3RyaWRlKTtcclxuICAgIHJldHVybiB0aGlzLm1faW52ZXJzZURlbnNpdHkgKiBpbnZlcnNlU3RyaWRlICogaW52ZXJzZVN0cmlkZTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgd29ybGQncyBjb250YWN0IGZpbHRlciBpZiBhbnkgcGFydGljbGVzIHdpdGggdGhlXHJcbiAgICogYjJfY29udGFjdEZpbHRlclBhcnRpY2xlIGZsYWcgYXJlIHByZXNlbnQgaW4gdGhlIHN5c3RlbS5cclxuICAgKi9cclxuICBHZXRGaXh0dXJlQ29udGFjdEZpbHRlcigpOiBiMkNvbnRhY3RGaWx0ZXIgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fYWxsUGFydGljbGVGbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX2ZpeHR1cmVDb250YWN0RmlsdGVyUGFydGljbGVcclxuICAgICAgPyB0aGlzLm1fd29ybGQubV9jb250YWN0TWFuYWdlci5tX2NvbnRhY3RGaWx0ZXJcclxuICAgICAgOiBudWxsO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogR2V0IHRoZSB3b3JsZCdzIGNvbnRhY3QgZmlsdGVyIGlmIGFueSBwYXJ0aWNsZXMgd2l0aCB0aGVcclxuICAgKiBiMl9wYXJ0aWNsZUNvbnRhY3RGaWx0ZXJQYXJ0aWNsZSBmbGFnIGFyZSBwcmVzZW50IGluIHRoZVxyXG4gICAqIHN5c3RlbS5cclxuICAgKi9cclxuICBHZXRQYXJ0aWNsZUNvbnRhY3RGaWx0ZXIoKTogYjJDb250YWN0RmlsdGVyIHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9wYXJ0aWNsZUNvbnRhY3RGaWx0ZXJQYXJ0aWNsZVxyXG4gICAgICA/IHRoaXMubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLm1fY29udGFjdEZpbHRlclxyXG4gICAgICA6IG51bGw7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIHdvcmxkJ3MgY29udGFjdCBsaXN0ZW5lciBpZiBhbnkgcGFydGljbGVzIHdpdGggdGhlXHJcbiAgICogYjJfZml4dHVyZUNvbnRhY3RMaXN0ZW5lclBhcnRpY2xlIGZsYWcgYXJlIHByZXNlbnQgaW4gdGhlXHJcbiAgICogc3lzdGVtLlxyXG4gICAqL1xyXG4gIEdldEZpeHR1cmVDb250YWN0TGlzdGVuZXIoKTogYjJDb250YWN0TGlzdGVuZXIgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fYWxsUGFydGljbGVGbGFncyAmIGIyUGFydGljbGVGbGFnLmIyX2ZpeHR1cmVDb250YWN0TGlzdGVuZXJQYXJ0aWNsZVxyXG4gICAgICA/IHRoaXMubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLm1fY29udGFjdExpc3RlbmVyXHJcbiAgICAgIDogbnVsbDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgd29ybGQncyBjb250YWN0IGxpc3RlbmVyIGlmIGFueSBwYXJ0aWNsZXMgd2l0aCB0aGVcclxuICAgKiBiMl9wYXJ0aWNsZUNvbnRhY3RMaXN0ZW5lclBhcnRpY2xlIGZsYWcgYXJlIHByZXNlbnQgaW4gdGhlXHJcbiAgICogc3lzdGVtLlxyXG4gICAqL1xyXG4gIEdldFBhcnRpY2xlQ29udGFjdExpc3RlbmVyKCk6IGIyQ29udGFjdExpc3RlbmVyIHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FsbFBhcnRpY2xlRmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl9wYXJ0aWNsZUNvbnRhY3RMaXN0ZW5lclBhcnRpY2xlXHJcbiAgICAgID8gdGhpcy5tX3dvcmxkLm1fY29udGFjdE1hbmFnZXIubV9jb250YWN0TGlzdGVuZXJcclxuICAgICAgOiBudWxsO1xyXG4gIH1cclxuXHJcbiAgU2V0VXNlck92ZXJyaWRhYmxlQnVmZmVyPFQ+KGJ1ZmZlcjogYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8VD4sIGRhdGE6IFRbXSk6IHZvaWQge1xyXG4gICAgYnVmZmVyLmRhdGEgPSBkYXRhO1xyXG4gICAgYnVmZmVyLnVzZXJTdXBwbGllZENhcGFjaXR5ID0gZGF0YS5sZW5ndGg7XHJcbiAgfVxyXG5cclxuICBTZXRHcm91cEZsYWdzKGdyb3VwOiBiMlBhcnRpY2xlR3JvdXAsIG5ld0ZsYWdzOiBiMlBhcnRpY2xlR3JvdXBGbGFnKTogdm9pZCB7XHJcbiAgICBjb25zdCBvbGRGbGFncyA9IGdyb3VwLm1fZ3JvdXBGbGFncztcclxuICAgIGlmICgob2xkRmxhZ3MgXiBuZXdGbGFncykgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3NvbGlkUGFydGljbGVHcm91cCkge1xyXG4gICAgICAvLyBJZiB0aGUgYjJfc29saWRQYXJ0aWNsZUdyb3VwIGZsYWcgY2hhbmdlZCBzY2hlZHVsZSBkZXB0aCB1cGRhdGUuXHJcbiAgICAgIG5ld0ZsYWdzIHw9IGIyUGFydGljbGVHcm91cEZsYWcuYjJfcGFydGljbGVHcm91cE5lZWRzVXBkYXRlRGVwdGg7XHJcbiAgICB9XHJcbiAgICBpZiAob2xkRmxhZ3MgJiB+bmV3RmxhZ3MpIHtcclxuICAgICAgLy8gSWYgYW55IGZsYWdzIG1pZ2h0IGJlIHJlbW92ZWRcclxuICAgICAgdGhpcy5tX25lZWRzVXBkYXRlQWxsR3JvdXBGbGFncyA9IHRydWU7XHJcbiAgICB9XHJcbiAgICBpZiAofnRoaXMubV9hbGxHcm91cEZsYWdzICYgbmV3RmxhZ3MpIHtcclxuICAgICAgLy8gSWYgYW55IGZsYWdzIHdlcmUgYWRkZWRcclxuICAgICAgaWYgKG5ld0ZsYWdzICYgYjJQYXJ0aWNsZUdyb3VwRmxhZy5iMl9zb2xpZFBhcnRpY2xlR3JvdXApIHtcclxuICAgICAgICB0aGlzLm1fZGVwdGhCdWZmZXIgPSB0aGlzLlJlcXVlc3RCdWZmZXIodGhpcy5tX2RlcHRoQnVmZmVyKTtcclxuICAgICAgfVxyXG4gICAgICB0aGlzLm1fYWxsR3JvdXBGbGFncyB8PSBuZXdGbGFncztcclxuICAgIH1cclxuICAgIGdyb3VwLm1fZ3JvdXBGbGFncyA9IG5ld0ZsYWdzO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIEJvZHlDb250YWN0Q29tcGFyZShsaHM6IGIyUGFydGljbGVCb2R5Q29udGFjdCwgcmhzOiBiMlBhcnRpY2xlQm9keUNvbnRhY3QpOiBib29sZWFuIHtcclxuICAgIGlmIChsaHMuaW5kZXggPT09IHJocy5pbmRleCkge1xyXG4gICAgICAvLyBTdWJzb3J0IGJ5IHdlaWdodCwgZGVjcmVhc2luZy5cclxuICAgICAgcmV0dXJuIGxocy53ZWlnaHQgPiByaHMud2VpZ2h0O1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGxocy5pbmRleCA8IHJocy5pbmRleDtcclxuICB9XHJcblxyXG4gIFJlbW92ZVNwdXJpb3VzQm9keUNvbnRhY3RzKCk6IHZvaWQge1xyXG4gICAgLy8gQXQgdGhpcyBwb2ludCB3ZSBoYXZlIGEgbGlzdCBvZiBjb250YWN0IGNhbmRpZGF0ZXMgYmFzZWQgb24gQUFCQlxyXG4gICAgLy8gb3ZlcmxhcC5UaGUgQUFCQiBxdWVyeSB0aGF0ICBnZW5lcmF0ZWQgdGhpcyByZXR1cm5zIGFsbCBjb2xsaWRhYmxlXHJcbiAgICAvLyBmaXh0dXJlcyBvdmVybGFwcGluZyBwYXJ0aWNsZSBib3VuZGluZyBib3hlcy4gIFRoaXMgYnJlYWtzIGRvd24gYXJvdW5kXHJcbiAgICAvLyB2ZXJ0aWNlcyB3aGVyZSB0d28gc2hhcGVzIGludGVyc2VjdCwgc3VjaCBhcyBhIFwiZ3JvdW5kXCIgc3VyZmFjZSBtYWRlXHJcbiAgICAvLyBvZiBtdWx0aXBsZSBiMlBvbHlnb25TaGFwZXM7IGl0IHBvdGVudGlhbGx5IGFwcGxpZXMgYSBsb3Qgb2Ygc3B1cmlvdXNcclxuICAgIC8vIGltcHVsc2VzIGZyb20gbm9ybWFscyB0aGF0IHNob3VsZCBub3QgYWN0dWFsbHkgY29udHJpYnV0ZS4gIFNlZSB0aGVcclxuICAgIC8vIFJhbXAgZXhhbXBsZSBpbiBUZXN0YmVkLlxyXG4gICAgLy9cclxuICAgIC8vIFRvIGNvcnJlY3QgZm9yIHRoaXMsIHdlIGFwcGx5IHRoaXMgYWxnb3JpdGhtOlxyXG4gICAgLy8gICAqIHNvcnQgY29udGFjdHMgYnkgcGFydGljbGUgYW5kIHN1YnNvcnQgYnkgd2VpZ2h0IChuZWFyZXN0IHRvIGZhcnRoZXN0KVxyXG4gICAgLy8gICAqIGZvciBlYWNoIGNvbnRhY3QgcGVyIHBhcnRpY2xlOlxyXG4gICAgLy8gICAgICAtIHByb2plY3QgYSBwb2ludCBhdCB0aGUgY29udGFjdCBkaXN0YW5jZSBhbG9uZyB0aGUgaW52ZXJzZSBvZiB0aGVcclxuICAgIC8vICAgICAgICBjb250YWN0IG5vcm1hbFxyXG4gICAgLy8gICAgICAtIGlmIHRoaXMgaW50ZXJzZWN0cyB0aGUgZml4dHVyZSB0aGF0IGdlbmVyYXRlZCB0aGUgY29udGFjdCwgYXBwbHlcclxuICAgIC8vICAgICAgICAgaXQsIG90aGVyd2lzZSBkaXNjYXJkIGFzIGltcG9zc2libGVcclxuICAgIC8vICAgICAgLSByZXBlYXQgZm9yIHVwIHRvIG4gbmVhcmVzdCBjb250YWN0cywgY3VycmVudGx5IHdlIGdldCBnb29kIHJlc3VsdHNcclxuICAgIC8vICAgICAgICBmcm9tIG49My5cclxuICAgIC8vL3N0ZDo6c29ydChtX2JvZHlDb250YWN0QnVmZmVyLkJlZ2luKCksIG1fYm9keUNvbnRhY3RCdWZmZXIuRW5kKCksIGIyUGFydGljbGVTeXN0ZW06OkJvZHlDb250YWN0Q29tcGFyZSk7XHJcbiAgICBzdGRfc29ydChcclxuICAgICAgdGhpcy5tX2JvZHlDb250YWN0QnVmZmVyLmRhdGEsXHJcbiAgICAgIDAsXHJcbiAgICAgIHRoaXMubV9ib2R5Q29udGFjdEJ1ZmZlci5jb3VudCxcclxuICAgICAgYjJQYXJ0aWNsZVN5c3RlbS5Cb2R5Q29udGFjdENvbXBhcmUsXHJcbiAgICApO1xyXG5cclxuICAgIC8vL2ludDMyIGRpc2NhcmRlZCA9IDA7XHJcbiAgICAvLy9zdGQ6OnJlbW92ZV9pZihtX2JvZHlDb250YWN0QnVmZmVyLkJlZ2luKCksIG1fYm9keUNvbnRhY3RCdWZmZXIuRW5kKCksIGIyUGFydGljbGVCb2R5Q29udGFjdFJlbW92ZVByZWRpY2F0ZSh0aGlzLCAmZGlzY2FyZGVkKSk7XHJcbiAgICAvLy9cclxuICAgIC8vL21fYm9keUNvbnRhY3RCdWZmZXIuU2V0Q291bnQobV9ib2R5Q29udGFjdEJ1ZmZlci5HZXRDb3VudCgpIC0gZGlzY2FyZGVkKTtcclxuXHJcbiAgICBjb25zdCBzX24gPSBiMlBhcnRpY2xlU3lzdGVtLlJlbW92ZVNwdXJpb3VzQm9keUNvbnRhY3RzX3NfbjtcclxuICAgIGNvbnN0IHNfcG9zID0gYjJQYXJ0aWNsZVN5c3RlbS5SZW1vdmVTcHVyaW91c0JvZHlDb250YWN0c19zX3BvcztcclxuICAgIGNvbnN0IHNfbm9ybWFsID0gYjJQYXJ0aWNsZVN5c3RlbS5SZW1vdmVTcHVyaW91c0JvZHlDb250YWN0c19zX25vcm1hbDtcclxuXHJcbiAgICAvLyBNYXggbnVtYmVyIG9mIGNvbnRhY3RzIHByb2Nlc3NlZCBwZXIgcGFydGljbGUsIGZyb20gbmVhcmVzdCB0byBmYXJ0aGVzdC5cclxuICAgIC8vIFRoaXMgbXVzdCBiZSBhdCBsZWFzdCAyIGZvciBjb3JyZWN0bmVzcyB3aXRoIGNvbmNhdmUgc2hhcGVzOyAzIHdhc1xyXG4gICAgLy8gZXhwZXJpbWVudGFsbHkgYXJyaXZlZCBhdCBhcyBsb29raW5nIHJlYXNvbmFibGUuXHJcbiAgICBjb25zdCBrX21heENvbnRhY3RzUGVyUG9pbnQgPSAzO1xyXG4gICAgLy8gSW5kZXggb2YgbGFzdCBwYXJ0aWNsZSBwcm9jZXNzZWQuXHJcbiAgICBsZXQgbGFzdEluZGV4ID0gLTE7XHJcbiAgICAvLyBOdW1iZXIgb2YgY29udGFjdHMgcHJvY2Vzc2VkIGZvciB0aGUgY3VycmVudCBwYXJ0aWNsZS5cclxuICAgIGxldCBjdXJyZW50Q29udGFjdHMgPSAwO1xyXG4gICAgLy8gT3V0cHV0IHRoZSBudW1iZXIgb2YgZGlzY2FyZGVkIGNvbnRhY3RzLlxyXG4gICAgLy8gbGV0IGRpc2NhcmRlZCA9IDA7XHJcbiAgICBjb25zdCBiMlBhcnRpY2xlQm9keUNvbnRhY3RSZW1vdmVQcmVkaWNhdGUgPSAoY29udGFjdDogYjJQYXJ0aWNsZUJvZHlDb250YWN0KTogYm9vbGVhbiA9PiB7XHJcbiAgICAgIC8vIFRoaXMgaW1wbGVtZW50cyB0aGUgc2VsZWN0aW9uIGNyaXRlcmlhIGRlc2NyaWJlZCBpblxyXG4gICAgICAvLyBSZW1vdmVTcHVyaW91c0JvZHlDb250YWN0cygpLlxyXG4gICAgICAvLyBUaGlzIGZ1bmN0b3IgaXMgaXRlcmF0aW5nIHRocm91Z2ggYSBsaXN0IG9mIEJvZHkgY29udGFjdHMgcGVyXHJcbiAgICAgIC8vIFBhcnRpY2xlLCBvcmRlcmVkIGZyb20gbmVhciB0byBmYXIuICBGb3IgdXAgdG8gdGhlIG1heGltdW0gbnVtYmVyIG9mXHJcbiAgICAgIC8vIGNvbnRhY3RzIHdlIGFsbG93IHBlciBwb2ludCBwZXIgc3RlcCwgd2UgdmVyaWZ5IHRoYXQgdGhlIGNvbnRhY3RcclxuICAgICAgLy8gbm9ybWFsIG9mIHRoZSBCb2R5IHRoYXQgZ2VuZW5lcmF0ZWQgdGhlIGNvbnRhY3QgbWFrZXMgcGh5c2ljYWwgc2Vuc2VcclxuICAgICAgLy8gYnkgcHJvamVjdGluZyBhIHBvaW50IGJhY2sgYWxvbmcgdGhhdCBub3JtYWwgYW5kIHNlZWluZyBpZiBpdFxyXG4gICAgICAvLyBpbnRlcnNlY3RzIHRoZSBmaXh0dXJlIGdlbmVyYXRpbmcgdGhlIGNvbnRhY3QuXHJcblxyXG4gICAgICBpZiAoY29udGFjdC5pbmRleCAhPT0gbGFzdEluZGV4KSB7XHJcbiAgICAgICAgY3VycmVudENvbnRhY3RzID0gMDtcclxuICAgICAgICBsYXN0SW5kZXggPSBjb250YWN0LmluZGV4O1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoY3VycmVudENvbnRhY3RzKysgPiBrX21heENvbnRhY3RzUGVyUG9pbnQpIHtcclxuICAgICAgICAvLyArK2Rpc2NhcmRlZDtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gUHJvamVjdCBhbG9uZyBpbnZlcnNlIG5vcm1hbCAoYXMgcmV0dXJuZWQgaW4gdGhlIGNvbnRhY3QpIHRvIGdldCB0aGVcclxuICAgICAgLy8gcG9pbnQgdG8gY2hlY2suXHJcbiAgICAgIC8vL2IyVmVjMiBuID0gY29udGFjdC5ub3JtYWw7XHJcbiAgICAgIGNvbnN0IG4gPSBzX24uQ29weShjb250YWN0Lm5vcm1hbCk7XHJcbiAgICAgIC8vIHdlaWdodCBpcyAxLShpbnYoZGlhbWV0ZXIpICogZGlzdGFuY2UpXHJcbiAgICAgIC8vL24gKj0gc3lzdGVtLm1fcGFydGljbGVEaWFtZXRlciAqICgxIC0gY29udGFjdC53ZWlnaHQpO1xyXG4gICAgICBuLlNlbGZNdWwodGhpcy5tX3BhcnRpY2xlRGlhbWV0ZXIgKiAoMSAtIGNvbnRhY3Qud2VpZ2h0KSk7XHJcbiAgICAgIC8vL2IyVmVjMiBwb3MgPSBzeXN0ZW0ubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2NvbnRhY3QuaW5kZXhdICsgbjtcclxuICAgICAgY29uc3QgcG9zID0gYjJWZWMyLkFkZFZWKHRoaXMubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2NvbnRhY3QuaW5kZXhdLCBuLCBzX3Bvcyk7XHJcblxyXG4gICAgICAvLyBwb3MgaXMgbm93IGEgcG9pbnQgcHJvamVjdGVkIGJhY2sgYWxvbmcgdGhlIGNvbnRhY3Qgbm9ybWFsIHRvIHRoZVxyXG4gICAgICAvLyBjb250YWN0IGRpc3RhbmNlLiBJZiB0aGUgc3VyZmFjZSBtYWtlcyBzZW5zZSBmb3IgYSBjb250YWN0LCBwb3Mgd2lsbFxyXG4gICAgICAvLyBub3cgbGllIG9uIG9yIGluIHRoZSBmaXh0dXJlIGdlbmVyYXRpbmdcclxuICAgICAgaWYgKCFjb250YWN0LmZpeHR1cmUuVGVzdFBvaW50KHBvcykpIHtcclxuICAgICAgICBjb25zdCBjaGlsZENvdW50ID0gY29udGFjdC5maXh0dXJlLkdldFNoYXBlKCkuR2V0Q2hpbGRDb3VudCgpO1xyXG4gICAgICAgIGZvciAobGV0IGNoaWxkSW5kZXggPSAwOyBjaGlsZEluZGV4IDwgY2hpbGRDb3VudDsgY2hpbGRJbmRleCsrKSB7XHJcbiAgICAgICAgICBjb25zdCBub3JtYWwgPSBzX25vcm1hbDtcclxuICAgICAgICAgIGNvbnN0IGRpc3RhbmNlID0gY29udGFjdC5maXh0dXJlLkNvbXB1dGVEaXN0YW5jZShwb3MsIG5vcm1hbCwgY2hpbGRJbmRleCk7XHJcbiAgICAgICAgICBpZiAoZGlzdGFuY2UgPCBiMl9saW5lYXJTbG9wKSB7XHJcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgLy8gKytkaXNjYXJkZWQ7XHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH07XHJcbiAgICB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQgPSBzdGRfcmVtb3ZlX2lmKFxyXG4gICAgICB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuZGF0YSxcclxuICAgICAgYjJQYXJ0aWNsZUJvZHlDb250YWN0UmVtb3ZlUHJlZGljYXRlLFxyXG4gICAgICB0aGlzLm1fYm9keUNvbnRhY3RCdWZmZXIuY291bnQsXHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgUmVtb3ZlU3B1cmlvdXNCb2R5Q29udGFjdHNfc19uID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFJlbW92ZVNwdXJpb3VzQm9keUNvbnRhY3RzX3NfcG9zID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFJlbW92ZVNwdXJpb3VzQm9keUNvbnRhY3RzX3Nfbm9ybWFsID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBEZXRlY3RTdHVja1BhcnRpY2xlKHBhcnRpY2xlOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIC8vIERldGVjdCBzdHVjayBwYXJ0aWNsZXNcclxuICAgIC8vXHJcbiAgICAvLyBUaGUgYmFzaWMgYWxnb3JpdGhtIGlzIHRvIGFsbG93IHRoZSB1c2VyIHRvIHNwZWNpZnkgYW4gb3B0aW9uYWxcclxuICAgIC8vIHRocmVzaG9sZCB3aGVyZSB3ZSBkZXRlY3Qgd2hlbmV2ZXIgYSBwYXJ0aWNsZSBpcyBjb250YWN0aW5nXHJcbiAgICAvLyBtb3JlIHRoYW4gb25lIGZpeHR1cmUgZm9yIG1vcmUgdGhhbiB0aHJlc2hvbGQgY29uc2VjdXRpdmVcclxuICAgIC8vIHN0ZXBzLiBUaGlzIGlzIGNvbnNpZGVyZWQgdG8gYmUgXCJzdHVja1wiLCBhbmQgdGhlc2UgYXJlIHB1dFxyXG4gICAgLy8gaW4gYSBsaXN0IHRoZSB1c2VyIGNhbiBxdWVyeSBwZXIgc3RlcCwgaWYgZW5hYmxlZCwgdG8gZGVhbCB3aXRoXHJcbiAgICAvLyBzdWNoIHBhcnRpY2xlcy5cclxuXHJcbiAgICBpZiAodGhpcy5tX3N0dWNrVGhyZXNob2xkIDw9IDApIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIEdldCB0aGUgc3RhdGUgdmFyaWFibGVzIGZvciB0aGlzIHBhcnRpY2xlLlxyXG4gICAgLy8vaW50MzIgKiBjb25zdCBjb25zZWN1dGl2ZUNvdW50ID0gJm1fY29uc2VjdXRpdmVDb250YWN0U3RlcHNCdWZmZXIuZGF0YVtwYXJ0aWNsZV07XHJcbiAgICAvLy9pbnQzMiAqIGNvbnN0IGxhc3RTdGVwID0gJm1fbGFzdEJvZHlDb250YWN0U3RlcEJ1ZmZlci5kYXRhW3BhcnRpY2xlXTtcclxuICAgIC8vL2ludDMyICogY29uc3QgYm9keUNvdW50ID0gJm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlci5kYXRhW3BhcnRpY2xlXTtcclxuXHJcbiAgICAvLyBUaGlzIGlzIG9ubHkgY2FsbGVkIHdoZW4gdGhlcmUgaXMgYSBib2R5IGNvbnRhY3QgZm9yIHRoaXMgcGFydGljbGUuXHJcbiAgICAvLy8rKygqYm9keUNvdW50KTtcclxuICAgICsrdGhpcy5tX2JvZHlDb250YWN0Q291bnRCdWZmZXIuZGF0YVtwYXJ0aWNsZV07XHJcblxyXG4gICAgLy8gV2Ugd2FudCB0byBvbmx5IHRyaWdnZXIgZGV0ZWN0aW9uIG9uY2UgcGVyIHN0ZXAsIHRoZSBmaXJzdCB0aW1lIHdlXHJcbiAgICAvLyBjb250YWN0IG1vcmUgdGhhbiBvbmUgZml4dHVyZSBpbiBhIHN0ZXAgZm9yIGEgZ2l2ZW4gcGFydGljbGUuXHJcbiAgICAvLy9pZiAoKmJvZHlDb3VudCA9PT0gMilcclxuICAgIGlmICh0aGlzLm1fYm9keUNvbnRhY3RDb3VudEJ1ZmZlci5kYXRhW3BhcnRpY2xlXSA9PT0gMikge1xyXG4gICAgICAvLy8rKygqY29uc2VjdXRpdmVDb3VudCk7XHJcbiAgICAgICsrdGhpcy5tX2NvbnNlY3V0aXZlQ29udGFjdFN0ZXBzQnVmZmVyLmRhdGFbcGFydGljbGVdO1xyXG4gICAgICAvLy9pZiAoKmNvbnNlY3V0aXZlQ291bnQgPiBtX3N0dWNrVGhyZXNob2xkKVxyXG4gICAgICBpZiAodGhpcy5tX2NvbnNlY3V0aXZlQ29udGFjdFN0ZXBzQnVmZmVyLmRhdGFbcGFydGljbGVdID4gdGhpcy5tX3N0dWNrVGhyZXNob2xkKSB7XHJcbiAgICAgICAgLy8vaW50MzImIG5ld1N0dWNrUGFydGljbGUgPSBtX3N0dWNrUGFydGljbGVCdWZmZXIuQXBwZW5kKCk7XHJcbiAgICAgICAgLy8vbmV3U3R1Y2tQYXJ0aWNsZSA9IHBhcnRpY2xlO1xyXG4gICAgICAgIHRoaXMubV9zdHVja1BhcnRpY2xlQnVmZmVyLmRhdGFbdGhpcy5tX3N0dWNrUGFydGljbGVCdWZmZXIuQXBwZW5kKCldID0gcGFydGljbGU7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICAgIC8vLypsYXN0U3RlcCA9IG1fdGltZXN0YW1wO1xyXG4gICAgdGhpcy5tX2xhc3RCb2R5Q29udGFjdFN0ZXBCdWZmZXIuZGF0YVtwYXJ0aWNsZV0gPSB0aGlzLm1fdGltZXN0YW1wO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogRGV0ZXJtaW5lIHdoZXRoZXIgYSBwYXJ0aWNsZSBpbmRleCBpcyB2YWxpZC5cclxuICAgKi9cclxuICBWYWxpZGF0ZVBhcnRpY2xlSW5kZXgoaW5kZXg6IG51bWJlcik6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIGluZGV4ID49IDAgJiYgaW5kZXggPCB0aGlzLkdldFBhcnRpY2xlQ291bnQoKSAmJiBpbmRleCAhPT0gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIHRpbWUgZWxhcHNlZCBpblxyXG4gICAqIGIyUGFydGljbGVTeXN0ZW1EZWY6OmxpZmV0aW1lR3JhbnVsYXJpdHkuXHJcbiAgICovXHJcbiAgR2V0UXVhbnRpemVkVGltZUVsYXBzZWQoKTogbnVtYmVyIHtcclxuICAgIC8vL3JldHVybiAoaW50MzIpKG1fdGltZUVsYXBzZWQgPj4gMzIpO1xyXG4gICAgcmV0dXJuIE1hdGguZmxvb3IodGhpcy5tX3RpbWVFbGFwc2VkIC8gMHgxMDAwMDAwMDApO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQ29udmVydCBhIGxpZmV0aW1lIGluIHNlY29uZHMgdG8gYW4gZXhwaXJhdGlvbiB0aW1lLlxyXG4gICAqL1xyXG4gIExpZmV0aW1lVG9FeHBpcmF0aW9uVGltZShsaWZldGltZTogbnVtYmVyKTogbnVtYmVyIHtcclxuICAgIC8vL3JldHVybiBtX3RpbWVFbGFwc2VkICsgKGludDY0KSgobGlmZXRpbWUgLyBtX2RlZi5saWZldGltZUdyYW51bGFyaXR5KSAqIChmbG9hdDMyKSgxTEwgPDwgMzIpKTtcclxuICAgIHJldHVybiAoXHJcbiAgICAgIHRoaXMubV90aW1lRWxhcHNlZCArIE1hdGguZmxvb3IoKGxpZmV0aW1lIC8gdGhpcy5tX2RlZi5saWZldGltZUdyYW51bGFyaXR5KSAqIDB4MTAwMDAwMDAwKVxyXG4gICAgKTtcclxuICB9XHJcblxyXG4gIEZvcmNlQ2FuQmVBcHBsaWVkKGZsYWdzOiBiMlBhcnRpY2xlRmxhZyk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuICEoZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl93YWxsUGFydGljbGUpO1xyXG4gIH1cclxuXHJcbiAgUHJlcGFyZUZvcmNlQnVmZmVyKCk6IHZvaWQge1xyXG4gICAgaWYgKCF0aGlzLm1faGFzRm9yY2UpIHtcclxuICAgICAgLy8vbWVtc2V0KG1fZm9yY2VCdWZmZXIsIDAsIHNpemVvZigqbV9mb3JjZUJ1ZmZlcikgKiBtX2NvdW50KTtcclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7IGkrKykge1xyXG4gICAgICAgIHRoaXMubV9mb3JjZUJ1ZmZlcltpXS5TZXRaZXJvKCk7XHJcbiAgICAgIH1cclxuICAgICAgdGhpcy5tX2hhc0ZvcmNlID0gdHJ1ZTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIElzUmlnaWRHcm91cChncm91cDogYjJQYXJ0aWNsZUdyb3VwIHwgbnVsbCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIGdyb3VwICE9PSBudWxsICYmIChncm91cC5tX2dyb3VwRmxhZ3MgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3JpZ2lkUGFydGljbGVHcm91cCkgIT09IDA7XHJcbiAgfVxyXG5cclxuICBHZXRMaW5lYXJWZWxvY2l0eShcclxuICAgIGdyb3VwOiBiMlBhcnRpY2xlR3JvdXAgfCBudWxsLFxyXG4gICAgcGFydGljbGVJbmRleDogbnVtYmVyLFxyXG4gICAgcG9pbnQ6IGIyVmVjMixcclxuICAgIG91dDogYjJWZWMyLFxyXG4gICk6IGIyVmVjMiB7XHJcbiAgICBpZiAoZ3JvdXAgJiYgdGhpcy5Jc1JpZ2lkR3JvdXAoZ3JvdXApKSB7XHJcbiAgICAgIHJldHVybiBncm91cC5HZXRMaW5lYXJWZWxvY2l0eUZyb21Xb3JsZFBvaW50KHBvaW50LCBvdXQpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8vcmV0dXJuIG1fdmVsb2NpdHlCdWZmZXIuZGF0YVtwYXJ0aWNsZUluZGV4XTtcclxuICAgICAgcmV0dXJuIG91dC5Db3B5KHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW3BhcnRpY2xlSW5kZXhdKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIEluaXREYW1waW5nUGFyYW1ldGVyKFxyXG4gICAgaW52TWFzczogbnVtYmVyW10sXHJcbiAgICBpbnZJbmVydGlhOiBudW1iZXJbXSxcclxuICAgIHRhbmdlbnREaXN0YW5jZTogbnVtYmVyW10sXHJcbiAgICBtYXNzOiBudW1iZXIsXHJcbiAgICBpbmVydGlhOiBudW1iZXIsXHJcbiAgICBjZW50ZXI6IGIyVmVjMixcclxuICAgIHBvaW50OiBiMlZlYzIsXHJcbiAgICBub3JtYWw6IGIyVmVjMixcclxuICApOiB2b2lkIHtcclxuICAgIC8vLyppbnZNYXNzID0gbWFzcyA+IDAgPyAxIC8gbWFzcyA6IDA7XHJcbiAgICBpbnZNYXNzWzBdID0gbWFzcyA+IDAgPyAxIC8gbWFzcyA6IDA7XHJcbiAgICAvLy8qaW52SW5lcnRpYSA9IGluZXJ0aWEgPiAwID8gMSAvIGluZXJ0aWEgOiAwO1xyXG4gICAgaW52SW5lcnRpYVswXSA9IGluZXJ0aWEgPiAwID8gMSAvIGluZXJ0aWEgOiAwO1xyXG4gICAgLy8vKnRhbmdlbnREaXN0YW5jZSA9IGIyQ3Jvc3MocG9pbnQgLSBjZW50ZXIsIG5vcm1hbCk7XHJcbiAgICB0YW5nZW50RGlzdGFuY2VbMF0gPSBiMlZlYzIuQ3Jvc3NWVihiMlZlYzIuU3ViVlYocG9pbnQsIGNlbnRlciwgYjJWZWMyLnNfdDApLCBub3JtYWwpO1xyXG4gIH1cclxuXHJcbiAgSW5pdERhbXBpbmdQYXJhbWV0ZXJXaXRoUmlnaWRHcm91cE9yUGFydGljbGUoXHJcbiAgICBpbnZNYXNzOiBudW1iZXJbXSxcclxuICAgIGludkluZXJ0aWE6IG51bWJlcltdLFxyXG4gICAgdGFuZ2VudERpc3RhbmNlOiBudW1iZXJbXSxcclxuICAgIGlzUmlnaWRHcm91cDogYm9vbGVhbixcclxuICAgIGdyb3VwOiBiMlBhcnRpY2xlR3JvdXAgfCBudWxsLFxyXG4gICAgcGFydGljbGVJbmRleDogbnVtYmVyLFxyXG4gICAgcG9pbnQ6IGIyVmVjMixcclxuICAgIG5vcm1hbDogYjJWZWMyLFxyXG4gICk6IHZvaWQge1xyXG4gICAgaWYgKGdyb3VwICYmIGlzUmlnaWRHcm91cCkge1xyXG4gICAgICB0aGlzLkluaXREYW1waW5nUGFyYW1ldGVyKFxyXG4gICAgICAgIGludk1hc3MsXHJcbiAgICAgICAgaW52SW5lcnRpYSxcclxuICAgICAgICB0YW5nZW50RGlzdGFuY2UsXHJcbiAgICAgICAgZ3JvdXAuR2V0TWFzcygpLFxyXG4gICAgICAgIGdyb3VwLkdldEluZXJ0aWEoKSxcclxuICAgICAgICBncm91cC5HZXRDZW50ZXIoKSxcclxuICAgICAgICBwb2ludCxcclxuICAgICAgICBub3JtYWwsXHJcbiAgICAgICk7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICBjb25zdCBmbGFncyA9IHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW3BhcnRpY2xlSW5kZXhdO1xyXG4gICAgICB0aGlzLkluaXREYW1waW5nUGFyYW1ldGVyKFxyXG4gICAgICAgIGludk1hc3MsXHJcbiAgICAgICAgaW52SW5lcnRpYSxcclxuICAgICAgICB0YW5nZW50RGlzdGFuY2UsXHJcbiAgICAgICAgZmxhZ3MgJiBiMlBhcnRpY2xlRmxhZy5iMl93YWxsUGFydGljbGUgPyAwIDogdGhpcy5HZXRQYXJ0aWNsZU1hc3MoKSxcclxuICAgICAgICAwLFxyXG4gICAgICAgIHBvaW50LFxyXG4gICAgICAgIHBvaW50LFxyXG4gICAgICAgIG5vcm1hbCxcclxuICAgICAgKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIENvbXB1dGVEYW1waW5nSW1wdWxzZShcclxuICAgIGludk1hc3NBOiBudW1iZXIsXHJcbiAgICBpbnZJbmVydGlhQTogbnVtYmVyLFxyXG4gICAgdGFuZ2VudERpc3RhbmNlQTogbnVtYmVyLFxyXG4gICAgaW52TWFzc0I6IG51bWJlcixcclxuICAgIGludkluZXJ0aWFCOiBudW1iZXIsXHJcbiAgICB0YW5nZW50RGlzdGFuY2VCOiBudW1iZXIsXHJcbiAgICBub3JtYWxWZWxvY2l0eTogbnVtYmVyLFxyXG4gICk6IG51bWJlciB7XHJcbiAgICBjb25zdCBpbnZNYXNzID1cclxuICAgICAgaW52TWFzc0EgK1xyXG4gICAgICBpbnZJbmVydGlhQSAqIHRhbmdlbnREaXN0YW5jZUEgKiB0YW5nZW50RGlzdGFuY2VBICtcclxuICAgICAgaW52TWFzc0IgK1xyXG4gICAgICBpbnZJbmVydGlhQiAqIHRhbmdlbnREaXN0YW5jZUIgKiB0YW5nZW50RGlzdGFuY2VCO1xyXG4gICAgcmV0dXJuIGludk1hc3MgPiAwID8gbm9ybWFsVmVsb2NpdHkgLyBpbnZNYXNzIDogMDtcclxuICB9XHJcblxyXG4gIEFwcGx5RGFtcGluZyhcclxuICAgIGludk1hc3M6IG51bWJlcixcclxuICAgIGludkluZXJ0aWE6IG51bWJlcixcclxuICAgIHRhbmdlbnREaXN0YW5jZTogbnVtYmVyLFxyXG4gICAgaXNSaWdpZEdyb3VwOiBib29sZWFuLFxyXG4gICAgZ3JvdXA6IGIyUGFydGljbGVHcm91cCB8IG51bGwsXHJcbiAgICBwYXJ0aWNsZUluZGV4OiBudW1iZXIsXHJcbiAgICBpbXB1bHNlOiBudW1iZXIsXHJcbiAgICBub3JtYWw6IGIyVmVjMixcclxuICApOiB2b2lkIHtcclxuICAgIGlmIChncm91cCAmJiBpc1JpZ2lkR3JvdXApIHtcclxuICAgICAgLy8vZ3JvdXAubV9saW5lYXJWZWxvY2l0eSArPSBpbXB1bHNlICogaW52TWFzcyAqIG5vcm1hbDtcclxuICAgICAgZ3JvdXAubV9saW5lYXJWZWxvY2l0eS5TZWxmTXVsQWRkKGltcHVsc2UgKiBpbnZNYXNzLCBub3JtYWwpO1xyXG4gICAgICAvLy9ncm91cC5tX2FuZ3VsYXJWZWxvY2l0eSArPSBpbXB1bHNlICogdGFuZ2VudERpc3RhbmNlICogaW52SW5lcnRpYTtcclxuICAgICAgZ3JvdXAubV9hbmd1bGFyVmVsb2NpdHkgKz0gaW1wdWxzZSAqIHRhbmdlbnREaXN0YW5jZSAqIGludkluZXJ0aWE7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICAvLy9tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbcGFydGljbGVJbmRleF0gKz0gaW1wdWxzZSAqIGludk1hc3MgKiBub3JtYWw7XHJcbiAgICAgIHRoaXMubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW3BhcnRpY2xlSW5kZXhdLlNlbGZNdWxBZGQoaW1wdWxzZSAqIGludk1hc3MsIG5vcm1hbCk7XHJcbiAgICB9XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8VD4ge1xyXG4gIHVzZXJTdXBwbGllZENhcGFjaXR5ID0gMDtcclxuICBfZGF0YTogVFtdIHwgbnVsbCA9IG51bGw7XHJcbiAgZ2V0IGRhdGEoKTogVFtdIHtcclxuICAgIHJldHVybiB0aGlzLl9kYXRhIGFzIFRbXTtcclxuICB9IC8vIEhBQ0s6IG1heSByZXR1cm4gbnVsbFxyXG4gIHNldCBkYXRhKHZhbHVlOiBUW10pIHtcclxuICAgIHRoaXMuX2RhdGEgPSB2YWx1ZTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlU3lzdGVtX1Byb3h5IHtcclxuICBpbmRleDogbnVtYmVyID0gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcbiAgdGFnID0gMDtcclxuXHJcbiAgc3RhdGljIENvbXBhcmVQcm94eVByb3h5KGE6IGIyUGFydGljbGVTeXN0ZW1fUHJveHksIGI6IGIyUGFydGljbGVTeXN0ZW1fUHJveHkpOiBib29sZWFuIHtcclxuICAgIHJldHVybiBhLnRhZyA8IGIudGFnO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIENvbXBhcmVUYWdQcm94eShhOiBudW1iZXIsIGI6IGIyUGFydGljbGVTeXN0ZW1fUHJveHkpOiBib29sZWFuIHtcclxuICAgIHJldHVybiBhIDwgYi50YWc7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgQ29tcGFyZVByb3h5VGFnKGE6IGIyUGFydGljbGVTeXN0ZW1fUHJveHksIGI6IG51bWJlcik6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIGEudGFnIDwgYjtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlU3lzdGVtX0luc2lkZUJvdW5kc0VudW1lcmF0b3Ige1xyXG4gIG1fc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtO1xyXG4gIG1feExvd2VyOiBudW1iZXI7XHJcbiAgbV94VXBwZXI6IG51bWJlcjtcclxuICBtX3lMb3dlcjogbnVtYmVyO1xyXG4gIG1feVVwcGVyOiBudW1iZXI7XHJcbiAgbV9maXJzdDogbnVtYmVyO1xyXG4gIG1fbGFzdDogbnVtYmVyO1xyXG5cclxuICAvKipcclxuICAgKiBJbnNpZGVCb3VuZHNFbnVtZXJhdG9yIGVudW1lcmF0ZXMgYWxsIHBhcnRpY2xlcyBpbnNpZGUgdGhlXHJcbiAgICogZ2l2ZW4gYm91bmRzLlxyXG4gICAqXHJcbiAgICogQ29uc3RydWN0IGFuIGVudW1lcmF0b3Igd2l0aCBib3VuZHMgb2YgdGFncyBhbmQgYSByYW5nZSBvZlxyXG4gICAqIHByb3hpZXMuXHJcbiAgICovXHJcbiAgY29uc3RydWN0b3Ioc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtLCBsb3dlcjogbnVtYmVyLCB1cHBlcjogbnVtYmVyLCBmaXJzdDogbnVtYmVyLCBsYXN0OiBudW1iZXIpIHtcclxuICAgIHRoaXMubV9zeXN0ZW0gPSBzeXN0ZW07XHJcbiAgICB0aGlzLm1feExvd2VyID0gKGxvd2VyICYgYjJQYXJ0aWNsZVN5c3RlbS54TWFzaykgPj4+IDA7XHJcbiAgICB0aGlzLm1feFVwcGVyID0gKHVwcGVyICYgYjJQYXJ0aWNsZVN5c3RlbS54TWFzaykgPj4+IDA7XHJcbiAgICB0aGlzLm1feUxvd2VyID0gKGxvd2VyICYgYjJQYXJ0aWNsZVN5c3RlbS55TWFzaykgPj4+IDA7XHJcbiAgICB0aGlzLm1feVVwcGVyID0gKHVwcGVyICYgYjJQYXJ0aWNsZVN5c3RlbS55TWFzaykgPj4+IDA7XHJcbiAgICB0aGlzLm1fZmlyc3QgPSBmaXJzdDtcclxuICAgIHRoaXMubV9sYXN0ID0gbGFzdDtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2ZpcnN0IDw9IHRoaXMubV9sYXN0KTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCBpbmRleCBvZiB0aGUgbmV4dCBwYXJ0aWNsZS4gUmV0dXJuc1xyXG4gICAqIGIyX2ludmFsaWRQYXJ0aWNsZUluZGV4IGlmIHRoZXJlIGFyZSBubyBtb3JlIHBhcnRpY2xlcy5cclxuICAgKi9cclxuICBHZXROZXh0KCk6IG51bWJlciB7XHJcbiAgICB3aGlsZSAodGhpcy5tX2ZpcnN0IDwgdGhpcy5tX2xhc3QpIHtcclxuICAgICAgY29uc3QgeFRhZyA9XHJcbiAgICAgICAgKHRoaXMubV9zeXN0ZW0ubV9wcm94eUJ1ZmZlci5kYXRhW3RoaXMubV9maXJzdF0udGFnICYgYjJQYXJ0aWNsZVN5c3RlbS54TWFzaykgPj4+IDA7XHJcbiAgICAgIGlmICghIUIyX0FTU0VSVCAmJiAhIUIyX0RFQlVHKSB7XHJcbiAgICAgICAgLy8gQjJfQVNTRVJUIC0+IEIyX0FTU0VSVF9FTkFCTEVEID8/XHJcbiAgICAgICAgY29uc3QgeVRhZyA9XHJcbiAgICAgICAgICAodGhpcy5tX3N5c3RlbS5tX3Byb3h5QnVmZmVyLmRhdGFbdGhpcy5tX2ZpcnN0XS50YWcgJiBiMlBhcnRpY2xlU3lzdGVtLnlNYXNrKSA+Pj4gMDtcclxuICAgICAgICBiMkFzc2VydCh5VGFnID49IHRoaXMubV95TG93ZXIpO1xyXG4gICAgICAgIGIyQXNzZXJ0KHlUYWcgPD0gdGhpcy5tX3lVcHBlcik7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHhUYWcgPj0gdGhpcy5tX3hMb3dlciAmJiB4VGFnIDw9IHRoaXMubV94VXBwZXIpIHtcclxuICAgICAgICByZXR1cm4gdGhpcy5tX3N5c3RlbS5tX3Byb3h5QnVmZmVyLmRhdGFbdGhpcy5tX2ZpcnN0KytdLmluZGV4O1xyXG4gICAgICB9XHJcbiAgICAgIHRoaXMubV9maXJzdCsrO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGIyX2ludmFsaWRQYXJ0aWNsZUluZGV4O1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSB7XHJcbiAgLyoqXHJcbiAgICogVGhlIGhlYWQgb2YgdGhlIGxpc3QuXHJcbiAgICovXHJcbiAgbGlzdCE6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZTtcclxuICAvKipcclxuICAgKiBUaGUgbmV4dCBub2RlIGluIHRoZSBsaXN0LlxyXG4gICAqL1xyXG4gIG5leHQ6IGIyUGFydGljbGVTeXN0ZW1fUGFydGljbGVMaXN0Tm9kZSB8IG51bGwgPSBudWxsO1xyXG4gIC8qKlxyXG4gICAqIE51bWJlciBvZiBlbnRyaWVzIGluIHRoZSBsaXN0LiBWYWxpZCBvbmx5IGZvciB0aGUgbm9kZSBhdCB0aGVcclxuICAgKiBoZWFkIG9mIHRoZSBsaXN0LlxyXG4gICAqL1xyXG4gIGNvdW50ID0gMDtcclxuICAvKipcclxuICAgKiBQYXJ0aWNsZSBpbmRleC5cclxuICAgKi9cclxuICBpbmRleCA9IDA7XHJcbn1cclxuXHJcbi8qKlxyXG4gKiBAY29uc3RydWN0b3JcclxuICovXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlU3lzdGVtX0ZpeGVkU2V0QWxsb2NhdG9yPFQ+IHtcclxuICBBbGxvY2F0ZShpdGVtU2l6ZTogbnVtYmVyLCBjb3VudDogbnVtYmVyKTogbnVtYmVyIHtcclxuICAgIC8vIFRPRE9cclxuICAgIHJldHVybiBjb3VudDtcclxuICB9XHJcblxyXG4gIENsZWFyKCk6IHZvaWQge1xyXG4gICAgLy8gVE9ET1xyXG4gIH1cclxuXHJcbiAgR2V0Q291bnQoKTogbnVtYmVyIHtcclxuICAgIC8vIFRPRE9cclxuICAgIHJldHVybiAwO1xyXG4gIH1cclxuXHJcbiAgSW52YWxpZGF0ZShpdGVtSW5kZXg6IG51bWJlcik6IHZvaWQge1xyXG4gICAgLy8gVE9ET1xyXG4gIH1cclxuXHJcbiAgR2V0VmFsaWRCdWZmZXIoKTogYm9vbGVhbltdIHtcclxuICAgIC8vIFRPRE9cclxuICAgIHJldHVybiBbXTtcclxuICB9XHJcblxyXG4gIEdldEJ1ZmZlcigpOiBUW10ge1xyXG4gICAgLy8gVE9ET1xyXG4gICAgcmV0dXJuIFtdO1xyXG4gIH1cclxuXHJcbiAgU2V0Q291bnQoY291bnQ6IG51bWJlcik6IHZvaWQge1xyXG4gICAgLy8gVE9ET1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1fRml4dHVyZVBhcnRpY2xlIHtcclxuICBmaXJzdDogYjJGaXh0dXJlO1xyXG4gIHNlY29uZDogbnVtYmVyID0gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcblxyXG4gIGNvbnN0cnVjdG9yKGZpeHR1cmU6IGIyRml4dHVyZSwgcGFydGljbGU6IG51bWJlcikge1xyXG4gICAgdGhpcy5maXJzdCA9IGZpeHR1cmU7XHJcbiAgICB0aGlzLnNlY29uZCA9IHBhcnRpY2xlO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1fRml4dHVyZVBhcnRpY2xlU2V0IGV4dGVuZHMgYjJQYXJ0aWNsZVN5c3RlbV9GaXhlZFNldEFsbG9jYXRvcjxcclxuICBiMlBhcnRpY2xlU3lzdGVtX0ZpeHR1cmVQYXJ0aWNsZVxyXG4+IHtcclxuICBJbml0aWFsaXplKFxyXG4gICAgYm9keUNvbnRhY3RCdWZmZXI6IGIyR3Jvd2FibGVCdWZmZXI8YjJQYXJ0aWNsZUJvZHlDb250YWN0PixcclxuICAgIGZsYWdzQnVmZmVyOiBiMlBhcnRpY2xlU3lzdGVtX1VzZXJPdmVycmlkYWJsZUJ1ZmZlcjxiMlBhcnRpY2xlRmxhZz4sXHJcbiAgKTogdm9pZCB7XHJcbiAgICAvLyBUT0RPXHJcbiAgfVxyXG5cclxuICBGaW5kKHBhaXI6IGIyUGFydGljbGVTeXN0ZW1fRml4dHVyZVBhcnRpY2xlKTogbnVtYmVyIHtcclxuICAgIC8vIFRPRE9cclxuICAgIHJldHVybiBiMl9pbnZhbGlkUGFydGljbGVJbmRleDtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlUGFpciB7XHJcbiAgZmlyc3Q6IG51bWJlciA9IGIyX2ludmFsaWRQYXJ0aWNsZUluZGV4O1xyXG4gIHNlY29uZDogbnVtYmVyID0gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcblxyXG4gIGNvbnN0cnVjdG9yKHBhcnRpY2xlQTogbnVtYmVyLCBwYXJ0aWNsZUI6IG51bWJlcikge1xyXG4gICAgdGhpcy5maXJzdCA9IHBhcnRpY2xlQTtcclxuICAgIHRoaXMuc2Vjb25kID0gcGFydGljbGVCO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVQYWlyU2V0IGV4dGVuZHMgYjJQYXJ0aWNsZVN5c3RlbV9GaXhlZFNldEFsbG9jYXRvcjxcclxuICBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlUGFpclxyXG4+IHtcclxuICBJbml0aWFsaXplKFxyXG4gICAgY29udGFjdEJ1ZmZlcjogYjJHcm93YWJsZUJ1ZmZlcjxiMlBhcnRpY2xlQ29udGFjdD4sXHJcbiAgICBmbGFnc0J1ZmZlcjogYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8YjJQYXJ0aWNsZUZsYWc+LFxyXG4gICk6IHZvaWQge1xyXG4gICAgLy8gVE9ET1xyXG4gIH1cclxuXHJcbiAgRmluZChwYWlyOiBiMlBhcnRpY2xlU3lzdGVtX1BhcnRpY2xlUGFpcik6IG51bWJlciB7XHJcbiAgICAvLyBUT0RPXHJcbiAgICByZXR1cm4gYjJfaW52YWxpZFBhcnRpY2xlSW5kZXg7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYXJ0aWNsZVN5c3RlbV9Db25uZWN0aW9uRmlsdGVyIHtcclxuICAvKipcclxuICAgKiBJcyB0aGUgcGFydGljbGUgbmVjZXNzYXJ5IGZvciBjb25uZWN0aW9uP1xyXG4gICAqIEEgcGFpciBvciBhIHRyaWFkIHNob3VsZCBjb250YWluIGF0IGxlYXN0IG9uZSAnbmVjZXNzYXJ5J1xyXG4gICAqIHBhcnRpY2xlLlxyXG4gICAqL1xyXG4gIElzTmVjZXNzYXJ5KGluZGV4OiBudW1iZXIpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQW4gYWRkaXRpb25hbCBjb25kaXRpb24gZm9yIGNyZWF0aW5nIGEgcGFpci5cclxuICAgKi9cclxuICBTaG91bGRDcmVhdGVQYWlyKGE6IG51bWJlciwgYjogbnVtYmVyKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gdHJ1ZTtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEFuIGFkZGl0aW9uYWwgY29uZGl0aW9uIGZvciBjcmVhdGluZyBhIHRyaWFkLlxyXG4gICAqL1xyXG4gIFNob3VsZENyZWF0ZVRyaWFkKGE6IG51bWJlciwgYjogbnVtYmVyLCBjOiBudW1iZXIpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1fRGVzdHJveVBhcnRpY2xlc0luU2hhcGVDYWxsYmFjayBleHRlbmRzIGIyUXVlcnlDYWxsYmFjayB7XHJcbiAgbV9zeXN0ZW06IGIyUGFydGljbGVTeXN0ZW07XHJcbiAgbV9zaGFwZTogYjJTaGFwZTtcclxuICBtX3hmOiBiMlRyYW5zZm9ybTtcclxuICBtX2NhbGxEZXN0cnVjdGlvbkxpc3RlbmVyID0gZmFsc2U7XHJcbiAgbV9kZXN0cm95ZWQgPSAwO1xyXG5cclxuICBjb25zdHJ1Y3RvcihcclxuICAgIHN5c3RlbTogYjJQYXJ0aWNsZVN5c3RlbSxcclxuICAgIHNoYXBlOiBiMlNoYXBlLFxyXG4gICAgeGY6IGIyVHJhbnNmb3JtLFxyXG4gICAgY2FsbERlc3RydWN0aW9uTGlzdGVuZXI6IGJvb2xlYW4sXHJcbiAgKSB7XHJcbiAgICBzdXBlcigpO1xyXG4gICAgdGhpcy5tX3N5c3RlbSA9IHN5c3RlbTtcclxuICAgIHRoaXMubV9zaGFwZSA9IHNoYXBlO1xyXG4gICAgdGhpcy5tX3hmID0geGY7XHJcbiAgICB0aGlzLm1fY2FsbERlc3RydWN0aW9uTGlzdGVuZXIgPSBjYWxsRGVzdHJ1Y3Rpb25MaXN0ZW5lcjtcclxuICAgIHRoaXMubV9kZXN0cm95ZWQgPSAwO1xyXG4gIH1cclxuXHJcbiAgUmVwb3J0Rml4dHVyZShmaXh0dXJlOiBiMkZpeHR1cmUpOiBib29sZWFuIHtcclxuICAgIHJldHVybiBmYWxzZTtcclxuICB9XHJcblxyXG4gIFJlcG9ydFBhcnRpY2xlKHBhcnRpY2xlU3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtLCBpbmRleDogbnVtYmVyKTogYm9vbGVhbiB7XHJcbiAgICBpZiAocGFydGljbGVTeXN0ZW0gIT09IHRoaXMubV9zeXN0ZW0pIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChpbmRleCA+PSAwICYmIGluZGV4IDwgdGhpcy5tX3N5c3RlbS5tX2NvdW50KTtcclxuICAgIGlmICh0aGlzLm1fc2hhcGUuVGVzdFBvaW50KHRoaXMubV94ZiwgdGhpcy5tX3N5c3RlbS5tX3Bvc2l0aW9uQnVmZmVyLmRhdGFbaW5kZXhdKSkge1xyXG4gICAgICB0aGlzLm1fc3lzdGVtLkRlc3Ryb3lQYXJ0aWNsZShpbmRleCwgdGhpcy5tX2NhbGxEZXN0cnVjdGlvbkxpc3RlbmVyKTtcclxuICAgICAgdGhpcy5tX2Rlc3Ryb3llZCsrO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRydWU7XHJcbiAgfVxyXG5cclxuICBEZXN0cm95ZWQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fZGVzdHJveWVkO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1fSm9pblBhcnRpY2xlR3JvdXBzRmlsdGVyIGV4dGVuZHMgYjJQYXJ0aWNsZVN5c3RlbV9Db25uZWN0aW9uRmlsdGVyIHtcclxuICBtX3RocmVzaG9sZCA9IDA7XHJcblxyXG4gIGNvbnN0cnVjdG9yKHRocmVzaG9sZDogbnVtYmVyKSB7XHJcbiAgICBzdXBlcigpO1xyXG4gICAgdGhpcy5tX3RocmVzaG9sZCA9IHRocmVzaG9sZDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEFuIGFkZGl0aW9uYWwgY29uZGl0aW9uIGZvciBjcmVhdGluZyBhIHBhaXIuXHJcbiAgICovXHJcbiAgU2hvdWxkQ3JlYXRlUGFpcihhOiBudW1iZXIsIGI6IG51bWJlcik6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIChcclxuICAgICAgKGEgPCB0aGlzLm1fdGhyZXNob2xkICYmIHRoaXMubV90aHJlc2hvbGQgPD0gYikgfHxcclxuICAgICAgKGIgPCB0aGlzLm1fdGhyZXNob2xkICYmIHRoaXMubV90aHJlc2hvbGQgPD0gYSlcclxuICAgICk7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBBbiBhZGRpdGlvbmFsIGNvbmRpdGlvbiBmb3IgY3JlYXRpbmcgYSB0cmlhZC5cclxuICAgKi9cclxuICBTaG91bGRDcmVhdGVUcmlhZChhOiBudW1iZXIsIGI6IG51bWJlciwgYzogbnVtYmVyKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gKFxyXG4gICAgICAoYSA8IHRoaXMubV90aHJlc2hvbGQgfHwgYiA8IHRoaXMubV90aHJlc2hvbGQgfHwgYyA8IHRoaXMubV90aHJlc2hvbGQpICYmXHJcbiAgICAgICh0aGlzLm1fdGhyZXNob2xkIDw9IGEgfHwgdGhpcy5tX3RocmVzaG9sZCA8PSBiIHx8IHRoaXMubV90aHJlc2hvbGQgPD0gYylcclxuICAgICk7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYXJ0aWNsZVN5c3RlbV9Db21wb3NpdGVTaGFwZSBleHRlbmRzIGIyU2hhcGUge1xyXG4gIGNvbnN0cnVjdG9yKHNoYXBlczogYjJTaGFwZVtdLCBzaGFwZUNvdW50PzogbnVtYmVyKSB7XHJcbiAgICBzdXBlcihiMlNoYXBlVHlwZS5lX3Vua25vd24sIDAuMCk7XHJcbiAgICB0aGlzLm1fc2hhcGVzID0gc2hhcGVzO1xyXG4gICAgdGhpcy5tX3NoYXBlQ291bnQgPSBzaGFwZUNvdW50ID8/IHNoYXBlcy5sZW5ndGg7XHJcbiAgfVxyXG5cclxuICBtX3NoYXBlczogYjJTaGFwZVtdO1xyXG4gIG1fc2hhcGVDb3VudCA9IDA7XHJcblxyXG4gIENsb25lKCk6IGIyU2hhcGUge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICB9XHJcblxyXG4gIEdldENoaWxkQ291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiAxO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQHNlZSBiMlNoYXBlOjpUZXN0UG9pbnRcclxuICAgKi9cclxuICBUZXN0UG9pbnQoeGY6IGIyVHJhbnNmb3JtLCBwOiBYWSk6IGJvb2xlYW4ge1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fc2hhcGVDb3VudDsgaSsrKSB7XHJcbiAgICAgIGlmICh0aGlzLm1fc2hhcGVzW2ldLlRlc3RQb2ludCh4ZiwgcCkpIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQHNlZSBiMlNoYXBlOjpDb21wdXRlRGlzdGFuY2VcclxuICAgKi9cclxuICBDb21wdXRlRGlzdGFuY2UoeGY6IGIyVHJhbnNmb3JtLCBwOiBiMlZlYzIsIG5vcm1hbDogYjJWZWMyLCBjaGlsZEluZGV4OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgICByZXR1cm4gMDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEltcGxlbWVudCBiMlNoYXBlLlxyXG4gICAqL1xyXG4gIFJheUNhc3QoXHJcbiAgICBvdXRwdXQ6IGIyUmF5Q2FzdE91dHB1dCxcclxuICAgIGlucHV0OiBiMlJheUNhc3RJbnB1dCxcclxuICAgIHhmOiBiMlRyYW5zZm9ybSxcclxuICAgIGNoaWxkSW5kZXg6IG51bWJlcixcclxuICApOiBib29sZWFuIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpO1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxuXHJcbiAgLyoqXHJcbiAgICogQHNlZSBiMlNoYXBlOjpDb21wdXRlQUFCQlxyXG4gICAqL1xyXG4gIENvbXB1dGVBQUJCKGFhYmI6IGIyQUFCQiwgeGY6IGIyVHJhbnNmb3JtLCBjaGlsZEluZGV4OiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGNvbnN0IHNfc3ViYWFiYiA9IG5ldyBiMkFBQkIoKTtcclxuICAgIGFhYmIubG93ZXJCb3VuZC54ID0gK2IyX21heEZsb2F0O1xyXG4gICAgYWFiYi5sb3dlckJvdW5kLnkgPSArYjJfbWF4RmxvYXQ7XHJcbiAgICBhYWJiLnVwcGVyQm91bmQueCA9IC1iMl9tYXhGbG9hdDtcclxuICAgIGFhYmIudXBwZXJCb3VuZC55ID0gLWIyX21heEZsb2F0O1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChjaGlsZEluZGV4ID09PSAwKTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX3NoYXBlQ291bnQ7IGkrKykge1xyXG4gICAgICBjb25zdCBjaGlsZENvdW50ID0gdGhpcy5tX3NoYXBlc1tpXS5HZXRDaGlsZENvdW50KCk7XHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgY2hpbGRDb3VudDsgaisrKSB7XHJcbiAgICAgICAgY29uc3Qgc3ViYWFiYiA9IHNfc3ViYWFiYjtcclxuICAgICAgICB0aGlzLm1fc2hhcGVzW2ldLkNvbXB1dGVBQUJCKHN1YmFhYmIsIHhmLCBqKTtcclxuICAgICAgICBhYWJiLkNvbWJpbmUxKHN1YmFhYmIpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBAc2VlIGIyU2hhcGU6OkNvbXB1dGVNYXNzXHJcbiAgICovXHJcbiAgQ29tcHV0ZU1hc3MobWFzc0RhdGE6IGIyTWFzc0RhdGEsIGRlbnNpdHk6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgfVxyXG5cclxuICBTZXR1cERpc3RhbmNlUHJveHkocHJveHk6IGIyRGlzdGFuY2VQcm94eSwgaW5kZXg6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgfVxyXG5cclxuICBDb21wdXRlU3VibWVyZ2VkQXJlYShub3JtYWw6IGIyVmVjMiwgb2Zmc2V0OiBudW1iZXIsIHhmOiBiMlRyYW5zZm9ybSwgYzogYjJWZWMyKTogbnVtYmVyIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZmFsc2UpO1xyXG4gICAgcmV0dXJuIDA7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYXJ0aWNsZVN5c3RlbV9SZWFjdGl2ZUZpbHRlciBleHRlbmRzIGIyUGFydGljbGVTeXN0ZW1fQ29ubmVjdGlvbkZpbHRlciB7XHJcbiAgbV9mbGFnc0J1ZmZlcjogYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8YjJQYXJ0aWNsZUZsYWc+O1xyXG5cclxuICBjb25zdHJ1Y3RvcihmbGFnc0J1ZmZlcjogYjJQYXJ0aWNsZVN5c3RlbV9Vc2VyT3ZlcnJpZGFibGVCdWZmZXI8YjJQYXJ0aWNsZUZsYWc+KSB7XHJcbiAgICBzdXBlcigpO1xyXG4gICAgdGhpcy5tX2ZsYWdzQnVmZmVyID0gZmxhZ3NCdWZmZXI7XHJcbiAgfVxyXG5cclxuICBJc05lY2Vzc2FyeShpbmRleDogbnVtYmVyKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gKHRoaXMubV9mbGFnc0J1ZmZlci5kYXRhW2luZGV4XSAmIGIyUGFydGljbGVGbGFnLmIyX3JlYWN0aXZlUGFydGljbGUpICE9PSAwO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVTeXN0ZW1fVXBkYXRlQm9keUNvbnRhY3RzQ2FsbGJhY2sgZXh0ZW5kcyBiMkZpeHR1cmVQYXJ0aWNsZVF1ZXJ5Q2FsbGJhY2sge1xyXG4gIG1fY29udGFjdEZpbHRlcjogYjJDb250YWN0RmlsdGVyIHwgbnVsbCA9IG51bGw7XHJcblxyXG4gIGNvbnN0cnVjdG9yKHN5c3RlbTogYjJQYXJ0aWNsZVN5c3RlbSwgY29udGFjdEZpbHRlcjogYjJDb250YWN0RmlsdGVyIHwgbnVsbCA9IG51bGwpIHtcclxuICAgIHN1cGVyKHN5c3RlbSk7IC8vIGJhc2UgY2xhc3MgY29uc3RydWN0b3JcclxuICAgIHRoaXMubV9jb250YWN0RmlsdGVyID0gY29udGFjdEZpbHRlcjtcclxuICB9XHJcblxyXG4gIFNob3VsZENvbGxpZGVGaXh0dXJlUGFydGljbGUoXHJcbiAgICBmaXh0dXJlOiBiMkZpeHR1cmUsXHJcbiAgICBwYXJ0aWNsZVN5c3RlbTogYjJQYXJ0aWNsZVN5c3RlbSxcclxuICAgIHBhcnRpY2xlSW5kZXg6IG51bWJlcixcclxuICApOiBib29sZWFuIHtcclxuICAgIC8vIENhbGwgdGhlIGNvbnRhY3QgZmlsdGVyIGlmIGl0J3Mgc2V0LCB0byBkZXRlcm1pbmUgd2hldGhlciB0b1xyXG4gICAgLy8gZmlsdGVyIHRoaXMgY29udGFjdC4gIFJldHVybnMgdHJ1ZSBpZiBjb250YWN0IGNhbGN1bGF0aW9ucyBzaG91bGRcclxuICAgIC8vIGJlIHBlcmZvcm1lZCwgZmFsc2Ugb3RoZXJ3aXNlLlxyXG4gICAgaWYgKHRoaXMubV9jb250YWN0RmlsdGVyKSB7XHJcbiAgICAgIGNvbnN0IGZsYWdzID0gdGhpcy5tX3N5c3RlbS5HZXRGbGFnc0J1ZmZlcigpO1xyXG4gICAgICBpZiAoZmxhZ3NbcGFydGljbGVJbmRleF0gJiBiMlBhcnRpY2xlRmxhZy5iMl9maXh0dXJlQ29udGFjdEZpbHRlclBhcnRpY2xlKSB7XHJcbiAgICAgICAgcmV0dXJuIHRoaXMubV9jb250YWN0RmlsdGVyLlNob3VsZENvbGxpZGVGaXh0dXJlUGFydGljbGUoXHJcbiAgICAgICAgICBmaXh0dXJlLFxyXG4gICAgICAgICAgdGhpcy5tX3N5c3RlbSxcclxuICAgICAgICAgIHBhcnRpY2xlSW5kZXgsXHJcbiAgICAgICAgKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRydWU7XHJcbiAgfVxyXG5cclxuICBSZXBvcnRGaXh0dXJlQW5kUGFydGljbGUoZml4dHVyZTogYjJGaXh0dXJlLCBjaGlsZEluZGV4OiBudW1iZXIsIGE6IG51bWJlcik6IHZvaWQge1xyXG4gICAgY29uc3Qgc19uID0gYjJQYXJ0aWNsZVN5c3RlbV9VcGRhdGVCb2R5Q29udGFjdHNDYWxsYmFjay5SZXBvcnRGaXh0dXJlQW5kUGFydGljbGVfc19uO1xyXG4gICAgY29uc3Qgc19ycCA9IGIyUGFydGljbGVTeXN0ZW1fVXBkYXRlQm9keUNvbnRhY3RzQ2FsbGJhY2suUmVwb3J0Rml4dHVyZUFuZFBhcnRpY2xlX3NfcnA7XHJcbiAgICBjb25zdCBhcCA9IHRoaXMubV9zeXN0ZW0ubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2FdO1xyXG4gICAgY29uc3QgbiA9IHNfbjtcclxuICAgIGNvbnN0IGQgPSBmaXh0dXJlLkNvbXB1dGVEaXN0YW5jZShhcCwgbiwgY2hpbGRJbmRleCk7XHJcbiAgICBpZiAoXHJcbiAgICAgIGQgPCB0aGlzLm1fc3lzdGVtLm1fcGFydGljbGVEaWFtZXRlciAmJlxyXG4gICAgICB0aGlzLlNob3VsZENvbGxpZGVGaXh0dXJlUGFydGljbGUoZml4dHVyZSwgdGhpcy5tX3N5c3RlbSwgYSlcclxuICAgICkge1xyXG4gICAgICBjb25zdCBiID0gZml4dHVyZS5HZXRCb2R5KCk7XHJcbiAgICAgIGNvbnN0IGJwID0gYi5HZXRXb3JsZENlbnRlcigpO1xyXG4gICAgICBjb25zdCBibSA9IGIuR2V0TWFzcygpO1xyXG4gICAgICBjb25zdCBiSSA9IGIuR2V0SW5lcnRpYSgpIC0gYm0gKiBiLkdldExvY2FsQ2VudGVyKCkuTGVuZ3RoU3F1YXJlZCgpO1xyXG4gICAgICBjb25zdCBpbnZCbSA9IGJtID4gMCA/IDEgLyBibSA6IDA7XHJcbiAgICAgIGNvbnN0IGludkJJID0gYkkgPiAwID8gMSAvIGJJIDogMDtcclxuICAgICAgY29uc3QgaW52QW0gPVxyXG4gICAgICAgIHRoaXMubV9zeXN0ZW0ubV9mbGFnc0J1ZmZlci5kYXRhW2FdICYgYjJQYXJ0aWNsZUZsYWcuYjJfd2FsbFBhcnRpY2xlXHJcbiAgICAgICAgICA/IDBcclxuICAgICAgICAgIDogdGhpcy5tX3N5c3RlbS5HZXRQYXJ0aWNsZUludk1hc3MoKTtcclxuICAgICAgLy8vYjJWZWMyIHJwID0gYXAgLSBicDtcclxuICAgICAgY29uc3QgcnAgPSBiMlZlYzIuU3ViVlYoYXAsIGJwLCBzX3JwKTtcclxuICAgICAgY29uc3QgcnBuID0gYjJWZWMyLkNyb3NzVlYocnAsIG4pO1xyXG4gICAgICBjb25zdCBpbnZNID0gaW52QW0gKyBpbnZCbSArIGludkJJICogcnBuICogcnBuO1xyXG5cclxuICAgICAgLy8vYjJQYXJ0aWNsZUJvZHlDb250YWN0JiBjb250YWN0ID0gbV9zeXN0ZW0ubV9ib2R5Q29udGFjdEJ1ZmZlci5BcHBlbmQoKTtcclxuICAgICAgY29uc3QgY29udGFjdCA9IHRoaXMubV9zeXN0ZW0ubV9ib2R5Q29udGFjdEJ1ZmZlci5kYXRhW1xyXG4gICAgICAgIHRoaXMubV9zeXN0ZW0ubV9ib2R5Q29udGFjdEJ1ZmZlci5BcHBlbmQoKVxyXG4gICAgICBdO1xyXG4gICAgICBjb250YWN0LmluZGV4ID0gYTtcclxuICAgICAgY29udGFjdC5ib2R5ID0gYjtcclxuICAgICAgY29udGFjdC5maXh0dXJlID0gZml4dHVyZTtcclxuICAgICAgY29udGFjdC53ZWlnaHQgPSAxIC0gZCAqIHRoaXMubV9zeXN0ZW0ubV9pbnZlcnNlRGlhbWV0ZXI7XHJcbiAgICAgIC8vL2NvbnRhY3Qubm9ybWFsID0gLW47XHJcbiAgICAgIGNvbnRhY3Qubm9ybWFsLkNvcHkobi5TZWxmTmVnKCkpO1xyXG4gICAgICBjb250YWN0Lm1hc3MgPSBpbnZNID4gMCA/IDEgLyBpbnZNIDogMDtcclxuICAgICAgdGhpcy5tX3N5c3RlbS5EZXRlY3RTdHVja1BhcnRpY2xlKGEpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX24gPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX3JwID0gbmV3IGIyVmVjMigpO1xyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYXJ0aWNsZVN5c3RlbV9Tb2x2ZUNvbGxpc2lvbkNhbGxiYWNrIGV4dGVuZHMgYjJGaXh0dXJlUGFydGljbGVRdWVyeUNhbGxiYWNrIHtcclxuICBtX3N0ZXA6IGIyVGltZVN0ZXA7XHJcblxyXG4gIGNvbnN0cnVjdG9yKHN5c3RlbTogYjJQYXJ0aWNsZVN5c3RlbSwgc3RlcDogYjJUaW1lU3RlcCkge1xyXG4gICAgc3VwZXIoc3lzdGVtKTsgLy8gYmFzZSBjbGFzcyBjb25zdHJ1Y3RvclxyXG4gICAgdGhpcy5tX3N0ZXAgPSBzdGVwO1xyXG4gIH1cclxuXHJcbiAgUmVwb3J0Rml4dHVyZUFuZFBhcnRpY2xlKGZpeHR1cmU6IGIyRml4dHVyZSwgY2hpbGRJbmRleDogbnVtYmVyLCBhOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGNvbnN0IHNfcDEgPSBiMlBhcnRpY2xlU3lzdGVtX1NvbHZlQ29sbGlzaW9uQ2FsbGJhY2suUmVwb3J0Rml4dHVyZUFuZFBhcnRpY2xlX3NfcDE7XHJcbiAgICBjb25zdCBzX291dHB1dCA9IGIyUGFydGljbGVTeXN0ZW1fU29sdmVDb2xsaXNpb25DYWxsYmFjay5SZXBvcnRGaXh0dXJlQW5kUGFydGljbGVfc19vdXRwdXQ7XHJcbiAgICBjb25zdCBzX2lucHV0ID0gYjJQYXJ0aWNsZVN5c3RlbV9Tb2x2ZUNvbGxpc2lvbkNhbGxiYWNrLlJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX2lucHV0O1xyXG4gICAgY29uc3Qgc19wID0gYjJQYXJ0aWNsZVN5c3RlbV9Tb2x2ZUNvbGxpc2lvbkNhbGxiYWNrLlJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX3A7XHJcbiAgICBjb25zdCBzX3YgPSBiMlBhcnRpY2xlU3lzdGVtX1NvbHZlQ29sbGlzaW9uQ2FsbGJhY2suUmVwb3J0Rml4dHVyZUFuZFBhcnRpY2xlX3NfdjtcclxuICAgIGNvbnN0IHNfZiA9IGIyUGFydGljbGVTeXN0ZW1fU29sdmVDb2xsaXNpb25DYWxsYmFjay5SZXBvcnRGaXh0dXJlQW5kUGFydGljbGVfc19mO1xyXG5cclxuICAgIGNvbnN0IGJvZHkgPSBmaXh0dXJlLkdldEJvZHkoKTtcclxuICAgIGNvbnN0IGFwID0gdGhpcy5tX3N5c3RlbS5tX3Bvc2l0aW9uQnVmZmVyLmRhdGFbYV07XHJcbiAgICBjb25zdCBhdiA9IHRoaXMubV9zeXN0ZW0ubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdO1xyXG4gICAgY29uc3Qgb3V0cHV0ID0gc19vdXRwdXQ7XHJcbiAgICBjb25zdCBpbnB1dCA9IHNfaW5wdXQ7XHJcbiAgICBpZiAodGhpcy5tX3N5c3RlbS5tX2l0ZXJhdGlvbkluZGV4ID09PSAwKSB7XHJcbiAgICAgIC8vIFB1dCAnYXAnIGluIHRoZSBsb2NhbCBzcGFjZSBvZiB0aGUgcHJldmlvdXMgZnJhbWVcclxuICAgICAgLy8vYjJWZWMyIHAxID0gYjJNdWxUKGJvZHkubV94ZjAsIGFwKTtcclxuICAgICAgY29uc3QgcDEgPSBiMlRyYW5zZm9ybS5NdWxUWFYoYm9keS5tX3hmMCwgYXAsIHNfcDEpO1xyXG4gICAgICBpZiAoZml4dHVyZS5HZXRTaGFwZSgpLkdldFR5cGUoKSA9PT0gYjJTaGFwZVR5cGUuZV9jaXJjbGVTaGFwZSkge1xyXG4gICAgICAgIC8vIE1ha2UgcmVsYXRpdmUgdG8gdGhlIGNlbnRlciBvZiB0aGUgY2lyY2xlXHJcbiAgICAgICAgLy8vcDEgLT0gYm9keS5HZXRMb2NhbENlbnRlcigpO1xyXG4gICAgICAgIHAxLlNlbGZTdWIoYm9keS5HZXRMb2NhbENlbnRlcigpKTtcclxuICAgICAgICAvLyBSZS1hcHBseSByb3RhdGlvbiBhYm91dCB0aGUgY2VudGVyIG9mIHRoZSBjaXJjbGVcclxuICAgICAgICAvLy9wMSA9IGIyTXVsKGJvZHkubV94ZjAucSwgcDEpO1xyXG4gICAgICAgIGIyUm90Lk11bFJWKGJvZHkubV94ZjAucSwgcDEsIHAxKTtcclxuICAgICAgICAvLyBTdWJ0cmFjdCByb3RhdGlvbiBvZiB0aGUgY3VycmVudCBmcmFtZVxyXG4gICAgICAgIC8vL3AxID0gYjJNdWxUKGJvZHkubV94Zi5xLCBwMSk7XHJcbiAgICAgICAgYjJSb3QuTXVsVFJWKGJvZHkubV94Zi5xLCBwMSwgcDEpO1xyXG4gICAgICAgIC8vIFJldHVybiB0byBsb2NhbCBzcGFjZVxyXG4gICAgICAgIC8vL3AxICs9IGJvZHkuR2V0TG9jYWxDZW50ZXIoKTtcclxuICAgICAgICBwMS5TZWxmQWRkKGJvZHkuR2V0TG9jYWxDZW50ZXIoKSk7XHJcbiAgICAgIH1cclxuICAgICAgLy8gUmV0dXJuIHRvIGdsb2JhbCBzcGFjZSBhbmQgYXBwbHkgcm90YXRpb24gb2YgY3VycmVudCBmcmFtZVxyXG4gICAgICAvLy9pbnB1dC5wMSA9IGIyTXVsKGJvZHkubV94ZiwgcDEpO1xyXG4gICAgICBiMlRyYW5zZm9ybS5NdWxYVihib2R5Lm1feGYsIHAxLCBpbnB1dC5wMSk7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICAvLy9pbnB1dC5wMSA9IGFwO1xyXG4gICAgICBpbnB1dC5wMS5Db3B5KGFwKTtcclxuICAgIH1cclxuICAgIC8vL2lucHV0LnAyID0gYXAgKyBtX3N0ZXAuZHQgKiBhdjtcclxuICAgIGIyVmVjMi5BZGRWTXVsU1YoYXAsIHRoaXMubV9zdGVwLmR0LCBhdiwgaW5wdXQucDIpO1xyXG4gICAgaW5wdXQubWF4RnJhY3Rpb24gPSAxO1xyXG4gICAgaWYgKGZpeHR1cmUuUmF5Q2FzdChvdXRwdXQsIGlucHV0LCBjaGlsZEluZGV4KSkge1xyXG4gICAgICBjb25zdCBuID0gb3V0cHV0Lm5vcm1hbDtcclxuICAgICAgLy8vYjJWZWMyIHAgPSAoMSAtIG91dHB1dC5mcmFjdGlvbikgKiBpbnB1dC5wMSArIG91dHB1dC5mcmFjdGlvbiAqIGlucHV0LnAyICsgYjJfbGluZWFyU2xvcCAqIG47XHJcbiAgICAgIGNvbnN0IHAgPSBzX3A7XHJcbiAgICAgIHAueCA9ICgxIC0gb3V0cHV0LmZyYWN0aW9uKSAqIGlucHV0LnAxLnggKyBvdXRwdXQuZnJhY3Rpb24gKiBpbnB1dC5wMi54ICsgYjJfbGluZWFyU2xvcCAqIG4ueDtcclxuICAgICAgcC55ID0gKDEgLSBvdXRwdXQuZnJhY3Rpb24pICogaW5wdXQucDEueSArIG91dHB1dC5mcmFjdGlvbiAqIGlucHV0LnAyLnkgKyBiMl9saW5lYXJTbG9wICogbi55O1xyXG4gICAgICAvLy9iMlZlYzIgdiA9IG1fc3RlcC5pbnZfZHQgKiAocCAtIGFwKTtcclxuICAgICAgY29uc3QgdiA9IHNfdjtcclxuICAgICAgdi54ID0gdGhpcy5tX3N0ZXAuaW52X2R0ICogKHAueCAtIGFwLngpO1xyXG4gICAgICB2LnkgPSB0aGlzLm1fc3RlcC5pbnZfZHQgKiAocC55IC0gYXAueSk7XHJcbiAgICAgIC8vL21fc3lzdGVtLm1fdmVsb2NpdHlCdWZmZXIuZGF0YVthXSA9IHY7XHJcbiAgICAgIHRoaXMubV9zeXN0ZW0ubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2FdLkNvcHkodik7XHJcbiAgICAgIC8vL2IyVmVjMiBmID0gbV9zdGVwLmludl9kdCAqIG1fc3lzdGVtLkdldFBhcnRpY2xlTWFzcygpICogKGF2IC0gdik7XHJcbiAgICAgIGNvbnN0IGYgPSBzX2Y7XHJcbiAgICAgIGYueCA9IHRoaXMubV9zdGVwLmludl9kdCAqIHRoaXMubV9zeXN0ZW0uR2V0UGFydGljbGVNYXNzKCkgKiAoYXYueCAtIHYueCk7XHJcbiAgICAgIGYueSA9IHRoaXMubV9zdGVwLmludl9kdCAqIHRoaXMubV9zeXN0ZW0uR2V0UGFydGljbGVNYXNzKCkgKiAoYXYueSAtIHYueSk7XHJcbiAgICAgIHRoaXMubV9zeXN0ZW0uUGFydGljbGVBcHBseUZvcmNlKGEsIGYpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX3AxID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBSZXBvcnRGaXh0dXJlQW5kUGFydGljbGVfc19vdXRwdXQgPSBuZXcgYjJSYXlDYXN0T3V0cHV0KCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX2lucHV0ID0gbmV3IGIyUmF5Q2FzdElucHV0KCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX3AgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX3YgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFJlcG9ydEZpeHR1cmVBbmRQYXJ0aWNsZV9zX2YgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFJlcG9ydFBhcnRpY2xlKHN5c3RlbTogYjJQYXJ0aWNsZVN5c3RlbSwgaW5kZXg6IG51bWJlcik6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxufVxyXG5cclxuLy8gI2VuZGlmXHJcbiJdfQ==