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
import { b2Assert } from '../common/b2Settings';
import { b2Transform, b2Vec2 } from '../common/b2Math';
import { b2Color } from '../common/b2Draw';
export class b2ParticleGroupDef {
    constructor() {
        this.flags = 0;
        this.groupFlags = 0;
        this.position = new b2Vec2();
        this.angle = 0.0;
        this.linearVelocity = new b2Vec2();
        this.angularVelocity = 0.0;
        this.color = new b2Color();
        this.strength = 1.0;
        this.shapeCount = 0;
        this.stride = 0;
        this.particleCount = 0;
        this.lifetime = 0;
        this.userData = null;
        this.group = null;
    }
}
export class b2ParticleGroup {
    constructor(system) {
        this.m_firstIndex = 0;
        this.m_lastIndex = 0;
        this.m_groupFlags = 0;
        this.m_strength = 1.0;
        this.m_prev = null;
        this.m_next = null;
        this.m_timestamp = -1;
        this.m_mass = 0.0;
        this.m_inertia = 0.0;
        this.m_center = new b2Vec2();
        this.m_linearVelocity = new b2Vec2();
        this.m_angularVelocity = 0.0;
        this.m_transform = new b2Transform();
        ///m_transform.SetIdentity();
        this.m_userData = null;
        this.m_system = system;
    }
    GetNext() {
        return this.m_next;
    }
    GetParticleSystem() {
        return this.m_system;
    }
    GetParticleCount() {
        return this.m_lastIndex - this.m_firstIndex;
    }
    GetBufferIndex() {
        return this.m_firstIndex;
    }
    ContainsParticle(index) {
        return this.m_firstIndex <= index && index < this.m_lastIndex;
    }
    GetAllParticleFlags() {
        if (!this.m_system.m_flagsBuffer.data) {
            throw new Error();
        }
        let flags = 0;
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            flags |= this.m_system.m_flagsBuffer.data[i];
        }
        return flags;
    }
    GetGroupFlags() {
        return this.m_groupFlags;
    }
    SetGroupFlags(flags) {
        !!B2_DEBUG && b2Assert((flags & 24 /* b2_particleGroupInternalMask */) === 0);
        flags |= this.m_groupFlags & 24 /* b2_particleGroupInternalMask */;
        this.m_system.SetGroupFlags(this, flags);
    }
    GetMass() {
        this.UpdateStatistics();
        return this.m_mass;
    }
    GetInertia() {
        this.UpdateStatistics();
        return this.m_inertia;
    }
    GetCenter() {
        this.UpdateStatistics();
        return this.m_center;
    }
    GetLinearVelocity() {
        this.UpdateStatistics();
        return this.m_linearVelocity;
    }
    GetAngularVelocity() {
        this.UpdateStatistics();
        return this.m_angularVelocity;
    }
    GetTransform() {
        return this.m_transform;
    }
    GetPosition() {
        return this.m_transform.p;
    }
    GetAngle() {
        return this.m_transform.q.GetAngle();
    }
    GetLinearVelocityFromWorldPoint(worldPoint, out) {
        const s_t0 = b2ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.UpdateStatistics();
        ///  return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_center);
        return b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Vec2.SubVV(worldPoint, this.m_center, s_t0), out);
    }
    GetUserData() {
        return this.m_userData;
    }
    SetUserData(data) {
        this.m_userData = data;
    }
    ApplyForce(force) {
        this.m_system.ApplyForce(this.m_firstIndex, this.m_lastIndex, force);
    }
    ApplyLinearImpulse(impulse) {
        this.m_system.ApplyLinearImpulse(this.m_firstIndex, this.m_lastIndex, impulse);
    }
    DestroyParticles(callDestructionListener) {
        if (this.m_system.m_world.IsLocked()) {
            throw new Error();
        }
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            this.m_system.DestroyParticle(i, callDestructionListener);
        }
    }
    UpdateStatistics() {
        if (!this.m_system.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_system.m_velocityBuffer.data) {
            throw new Error();
        }
        const p = new b2Vec2();
        const v = new b2Vec2();
        if (this.m_timestamp !== this.m_system.m_timestamp) {
            const m = this.m_system.GetParticleMass();
            ///  this.m_mass = 0;
            this.m_mass = m * (this.m_lastIndex - this.m_firstIndex);
            this.m_center.SetZero();
            this.m_linearVelocity.SetZero();
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                ///  this.m_mass += m;
                ///  this.m_center += m * this.m_system.m_positionBuffer.data[i];
                this.m_center.SelfMulAdd(m, this.m_system.m_positionBuffer.data[i]);
                ///  this.m_linearVelocity += m * this.m_system.m_velocityBuffer.data[i];
                this.m_linearVelocity.SelfMulAdd(m, this.m_system.m_velocityBuffer.data[i]);
            }
            if (this.m_mass > 0) {
                const inv_mass = 1 / this.m_mass;
                ///this.m_center *= 1 / this.m_mass;
                this.m_center.SelfMul(inv_mass);
                ///this.m_linearVelocity *= 1 / this.m_mass;
                this.m_linearVelocity.SelfMul(inv_mass);
            }
            this.m_inertia = 0;
            this.m_angularVelocity = 0;
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                ///b2Vec2 p = this.m_system.m_positionBuffer.data[i] - this.m_center;
                b2Vec2.SubVV(this.m_system.m_positionBuffer.data[i], this.m_center, p);
                ///b2Vec2 v = this.m_system.m_velocityBuffer.data[i] - this.m_linearVelocity;
                b2Vec2.SubVV(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity, v);
                this.m_inertia += m * b2Vec2.DotVV(p, p);
                this.m_angularVelocity += m * b2Vec2.CrossVV(p, v);
            }
            if (this.m_inertia > 0) {
                this.m_angularVelocity *= 1 / this.m_inertia;
            }
            this.m_timestamp = this.m_system.m_timestamp;
        }
    }
}
b2ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0 = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQYXJ0aWNsZUdyb3VwLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL3BhcnRpY2xlL2IyUGFydGljbGVHcm91cC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFBRSxRQUFRLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUNoRCxPQUFPLEVBQUUsV0FBVyxFQUFFLE1BQU0sRUFBTSxNQUFNLGtCQUFrQixDQUFDO0FBQzNELE9BQU8sRUFBRSxPQUFPLEVBQVEsTUFBTSxrQkFBa0IsQ0FBQztBQTBDakQsTUFBTSxPQUFPLGtCQUFrQjtJQUEvQjtRQUNFLFVBQUssR0FBbUIsQ0FBQyxDQUFDO1FBQzFCLGVBQVUsR0FBd0IsQ0FBQyxDQUFDO1FBQzNCLGFBQVEsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQ3pDLFVBQUssR0FBRyxHQUFHLENBQUM7UUFDSCxtQkFBYyxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDL0Msb0JBQWUsR0FBRyxHQUFHLENBQUM7UUFDYixVQUFLLEdBQVksSUFBSSxPQUFPLEVBQUUsQ0FBQztRQUN4QyxhQUFRLEdBQUcsR0FBRyxDQUFDO1FBR2YsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUNmLFdBQU0sR0FBRyxDQUFDLENBQUM7UUFDWCxrQkFBYSxHQUFHLENBQUMsQ0FBQztRQUVsQixhQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2IsYUFBUSxHQUFRLElBQUksQ0FBQztRQUNyQixVQUFLLEdBQTJCLElBQUksQ0FBQztJQUN2QyxDQUFDO0NBQUE7QUFFRCxNQUFNLE9BQU8sZUFBZTtJQWtCMUIsWUFBWSxNQUF3QjtRQWhCcEMsaUJBQVksR0FBRyxDQUFDLENBQUM7UUFDakIsZ0JBQVcsR0FBRyxDQUFDLENBQUM7UUFDaEIsaUJBQVksR0FBd0IsQ0FBQyxDQUFDO1FBQ3RDLGVBQVUsR0FBRyxHQUFHLENBQUM7UUFDakIsV0FBTSxHQUEy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QUFFLENBQUMsQ0FBQyxDQUFDO2dCQUN6QyxJQUFJLENBQUMsaUJBQWlCLElBQUksQ0FBQyxHQUFHLE1BQU0sQ0FBQyxPQUFPLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO2FBQ3BEO1lBQ0QsSUFBSSxJQUFJLENBQUMsU0FBUyxHQUFHLENBQUMsRUFBRTtnQkFDdEIsSUFBSSxDQUFDLGlCQUFpQixJQUFJLENBQUMsR0FBRyxJQUFJLENBQUMsU0FBUyxDQUFDO2FBQzlDO1lBQ0QsSUFBSSxDQUFDLFdBQVcsR0FBRyxJQUFJLENBQUMsUUFBUSxDQUFDLFdBQVcsQ0FBQztTQUM5QztJQUNILENBQUM7O0FBeEVlLG9EQUFvQyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMTMgR29vZ2xlLCBJbmMuXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7IGIyQXNzZXJ0IH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMlRyYW5zZm9ybSwgYjJWZWMyLCBYWSB9IGZyb20gJy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMkNvbG9yLCBSR0JBIH0gZnJvbSAnLi4vY29tbW9uL2IyRHJhdyc7XHJcbmltcG9ydCB7IGIyU2hhcGUgfSBmcm9tICcuLi9jb2xsaXNpb24vc2hhcGVzL2IyU2hhcGUnO1xyXG5pbXBvcnQgeyBiMlBhcnRpY2xlRmxhZyB9IGZyb20gJy4vYjJQYXJ0aWNsZSc7XHJcbmltcG9ydCB7IGIyUGFydGljbGVTeXN0ZW0gfSBmcm9tICcuL2IyUGFydGljbGVTeXN0ZW0nO1xyXG5cclxuZXhwb3J0IGNvbnN0IGVudW0gYjJQYXJ0aWNsZUdyb3VwRmxhZyB7XHJcbiAgbm9uZSA9IDAsXHJcblxyXG4gIC8vLyBQcmV2ZW50cyBvdmVybGFwcGluZyBvciBsZWFraW5nLlxyXG4gIGIyX3NvbGlkUGFydGljbGVHcm91cCA9IDEgPDwgMCxcclxuICAvLy8gS2VlcHMgaXRzIHNoYXBlLlxyXG4gIGIyX3JpZ2lkUGFydGljbGVHcm91cCA9IDEgPDwgMSxcclxuICAvLy8gV29uJ3QgYmUgZGVzdHJveWVkIGlmIGl0IGdldHMgZW1wdHkuXHJcbiAgYjJfcGFydGljbGVHcm91cENhbkJlRW1wdHkgPSAxIDw8IDIsXHJcbiAgLy8vIFdpbGwgYmUgZGVzdHJveWVkIG9uIG5leHQgc2ltdWxhdGlvbiBzdGVwLlxyXG4gIGIyX3BhcnRpY2xlR3JvdXBXaWxsQmVEZXN0cm95ZWQgPSAxIDw8IDMsXHJcbiAgLy8vIFVwZGF0ZXMgZGVwdGggZGF0YSBvbiBuZXh0IHNpbXVsYXRpb24gc3RlcC5cclxuICBiMl9wYXJ0aWNsZUdyb3VwTmVlZHNVcGRhdGVEZXB0aCA9IDEgPDwgNCxcclxuXHJcbiAgYjJfcGFydGljbGVHcm91cEludGVybmFsTWFzayA9IGIyX3BhcnRpY2xlR3JvdXBXaWxsQmVEZXN0cm95ZWQgfCBiMl9wYXJ0aWNsZUdyb3VwTmVlZHNVcGRhdGVEZXB0aCxcclxufVxyXG5cclxuZXhwb3J0IGludGVyZmFjZSBiMklQYXJ0aWNsZUdyb3VwRGVmIHtcclxuICBmbGFncz86IGIyUGFydGljbGVGbGFnO1xyXG4gIGdyb3VwRmxhZ3M/OiBiMlBhcnRpY2xlR3JvdXBGbGFnO1xyXG4gIHBvc2l0aW9uPzogWFk7XHJcbiAgYW5nbGU/OiBudW1iZXI7XHJcbiAgbGluZWFyVmVsb2NpdHk/OiBYWTtcclxuICBhbmd1bGFyVmVsb2NpdHk/OiBudW1iZXI7XHJcbiAgY29sb3I/OiBSR0JBO1xyXG4gIHN0cmVuZ3RoPzogbnVtYmVyO1xyXG4gIHNoYXBlPzogYjJTaGFwZTtcclxuICBzaGFwZXM/OiBiMlNoYXBlW107XHJcbiAgc2hhcGVDb3VudD86IG51bWJlcjtcclxuICBzdHJpZGU/OiBudW1iZXI7XHJcbiAgcGFydGljbGVDb3VudD86IG51bWJlcjtcclxuICBwb3NpdGlvbkRhdGE/OiBYWVtdO1xyXG4gIGxpZmV0aW1lPzogbnVtYmVyO1xyXG4gIHVzZXJEYXRhPzogYW55O1xyXG4gIGdyb3VwPzogYjJQYXJ0aWNsZUdyb3VwIHwgbnVsbDtcclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUGFydGljbGVHcm91cERlZiBpbXBsZW1lbnRzIGIySVBhcnRpY2xlR3JvdXBEZWYge1xyXG4gIGZsYWdzOiBiMlBhcnRpY2xlRmxhZyA9IDA7XHJcbiAgZ3JvdXBGbGFnczogYjJQYXJ0aWNsZUdyb3VwRmxhZyA9IDA7XHJcbiAgcmVhZG9ubHkgcG9zaXRpb246IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBhbmdsZSA9IDAuMDtcclxuICByZWFkb25seSBsaW5lYXJWZWxvY2l0eTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIGFuZ3VsYXJWZWxvY2l0eSA9IDAuMDtcclxuICByZWFkb25seSBjb2xvcjogYjJDb2xvciA9IG5ldyBiMkNvbG9yKCk7XHJcbiAgc3RyZW5ndGggPSAxLjA7XHJcbiAgc2hhcGU/OiBiMlNoYXBlO1xyXG4gIHNoYXBlcz86IGIyU2hhcGVbXTtcclxuICBzaGFwZUNvdW50ID0gMDtcclxuICBzdHJpZGUgPSAwO1xyXG4gIHBhcnRpY2xlQ291bnQgPSAwO1xyXG4gIHBvc2l0aW9uRGF0YT86IGIyVmVjMltdO1xyXG4gIGxpZmV0aW1lID0gMDtcclxuICB1c2VyRGF0YTogYW55ID0gbnVsbDtcclxuICBncm91cDogYjJQYXJ0aWNsZUdyb3VwIHwgbnVsbCA9IG51bGw7XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBhcnRpY2xlR3JvdXAge1xyXG4gIHJlYWRvbmx5IG1fc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtO1xyXG4gIG1fZmlyc3RJbmRleCA9IDA7XHJcbiAgbV9sYXN0SW5kZXggPSAwO1xyXG4gIG1fZ3JvdXBGbGFnczogYjJQYXJ0aWNsZUdyb3VwRmxhZyA9IDA7XHJcbiAgbV9zdHJlbmd0aCA9IDEuMDtcclxuICBtX3ByZXY6IGIyUGFydGljbGVHcm91cCB8IG51bGwgPSBudWxsO1xyXG4gIG1fbmV4dDogYjJQYXJ0aWNsZUdyb3VwIHwgbnVsbCA9IG51bGw7XHJcbiAgbV90aW1lc3RhbXAgPSAtMTtcclxuICBtX21hc3MgPSAwLjA7XHJcbiAgbV9pbmVydGlhID0gMC4wO1xyXG4gIHJlYWRvbmx5IG1fY2VudGVyOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9saW5lYXJWZWxvY2l0eTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1fYW5ndWxhclZlbG9jaXR5ID0gMC4wO1xyXG4gIHJlYWRvbmx5IG1fdHJhbnNmb3JtOiBiMlRyYW5zZm9ybSA9IG5ldyBiMlRyYW5zZm9ybSgpO1xyXG4gIC8vL21fdHJhbnNmb3JtLlNldElkZW50aXR5KCk7XHJcbiAgbV91c2VyRGF0YTogYW55ID0gbnVsbDtcclxuXHJcbiAgY29uc3RydWN0b3Ioc3lzdGVtOiBiMlBhcnRpY2xlU3lzdGVtKSB7XHJcbiAgICB0aGlzLm1fc3lzdGVtID0gc3lzdGVtO1xyXG4gIH1cclxuXHJcbiAgR2V0TmV4dCgpOiBiMlBhcnRpY2xlR3JvdXAgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fbmV4dDtcclxuICB9XHJcblxyXG4gIEdldFBhcnRpY2xlU3lzdGVtKCk6IGIyUGFydGljbGVTeXN0ZW0ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9zeXN0ZW07XHJcbiAgfVxyXG5cclxuICBHZXRQYXJ0aWNsZUNvdW50KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2xhc3RJbmRleCAtIHRoaXMubV9maXJzdEluZGV4O1xyXG4gIH1cclxuXHJcbiAgR2V0QnVmZmVySW5kZXgoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fZmlyc3RJbmRleDtcclxuICB9XHJcblxyXG4gIENvbnRhaW5zUGFydGljbGUoaW5kZXg6IG51bWJlcik6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9maXJzdEluZGV4IDw9IGluZGV4ICYmIGluZGV4IDwgdGhpcy5tX2xhc3RJbmRleDtcclxuICB9XHJcblxyXG4gIEdldEFsbFBhcnRpY2xlRmxhZ3MoKTogYjJQYXJ0aWNsZUZsYWcge1xyXG4gICAgaWYgKCF0aGlzLm1fc3lzdGVtLm1fZmxhZ3NCdWZmZXIuZGF0YSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuICAgIGxldCBmbGFncyA9IDA7XHJcbiAgICBmb3IgKGxldCBpID0gdGhpcy5tX2ZpcnN0SW5kZXg7IGkgPCB0aGlzLm1fbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgZmxhZ3MgfD0gdGhpcy5tX3N5c3RlbS5tX2ZsYWdzQnVmZmVyLmRhdGFbaV07XHJcbiAgICB9XHJcbiAgICByZXR1cm4gZmxhZ3M7XHJcbiAgfVxyXG5cclxuICBHZXRHcm91cEZsYWdzKCk6IGIyUGFydGljbGVHcm91cEZsYWcge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ncm91cEZsYWdzO1xyXG4gIH1cclxuXHJcbiAgU2V0R3JvdXBGbGFncyhmbGFnczogbnVtYmVyKTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KChmbGFncyAmIGIyUGFydGljbGVHcm91cEZsYWcuYjJfcGFydGljbGVHcm91cEludGVybmFsTWFzaykgPT09IDApO1xyXG4gICAgZmxhZ3MgfD0gdGhpcy5tX2dyb3VwRmxhZ3MgJiBiMlBhcnRpY2xlR3JvdXBGbGFnLmIyX3BhcnRpY2xlR3JvdXBJbnRlcm5hbE1hc2s7XHJcbiAgICB0aGlzLm1fc3lzdGVtLlNldEdyb3VwRmxhZ3ModGhpcywgZmxhZ3MpO1xyXG4gIH1cclxuXHJcbiAgR2V0TWFzcygpOiBudW1iZXIge1xyXG4gICAgdGhpcy5VcGRhdGVTdGF0aXN0aWNzKCk7XHJcbiAgICByZXR1cm4gdGhpcy5tX21hc3M7XHJcbiAgfVxyXG5cclxuICBHZXRJbmVydGlhKCk6IG51bWJlciB7XHJcbiAgICB0aGlzLlVwZGF0ZVN0YXRpc3RpY3MoKTtcclxuICAgIHJldHVybiB0aGlzLm1faW5lcnRpYTtcclxuICB9XHJcblxyXG4gIEdldENlbnRlcigpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHRoaXMuVXBkYXRlU3RhdGlzdGljcygpO1xyXG4gICAgcmV0dXJuIHRoaXMubV9jZW50ZXI7XHJcbiAgfVxyXG5cclxuICBHZXRMaW5lYXJWZWxvY2l0eSgpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHRoaXMuVXBkYXRlU3RhdGlzdGljcygpO1xyXG4gICAgcmV0dXJuIHRoaXMubV9saW5lYXJWZWxvY2l0eTtcclxuICB9XHJcblxyXG4gIEdldEFuZ3VsYXJWZWxvY2l0eSgpOiBudW1iZXIge1xyXG4gICAgdGhpcy5VcGRhdGVTdGF0aXN0aWNzKCk7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuICB9XHJcblxyXG4gIEdldFRyYW5zZm9ybSgpOiBSZWFkb25seTxiMlRyYW5zZm9ybT4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV90cmFuc2Zvcm07XHJcbiAgfVxyXG5cclxuICBHZXRQb3NpdGlvbigpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fdHJhbnNmb3JtLnA7XHJcbiAgfVxyXG5cclxuICBHZXRBbmdsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV90cmFuc2Zvcm0ucS5HZXRBbmdsZSgpO1xyXG4gIH1cclxuXHJcbiAgR2V0TGluZWFyVmVsb2NpdHlGcm9tV29ybGRQb2ludDxUIGV4dGVuZHMgWFk+KHdvcmxkUG9pbnQ6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIGNvbnN0IHNfdDAgPSBiMlBhcnRpY2xlR3JvdXAuR2V0TGluZWFyVmVsb2NpdHlGcm9tV29ybGRQb2ludF9zX3QwO1xyXG4gICAgdGhpcy5VcGRhdGVTdGF0aXN0aWNzKCk7XHJcbiAgICAvLy8gIHJldHVybiBtX2xpbmVhclZlbG9jaXR5ICsgYjJDcm9zcyhtX2FuZ3VsYXJWZWxvY2l0eSwgd29ybGRQb2ludCAtIG1fY2VudGVyKTtcclxuICAgIHJldHVybiBiMlZlYzIuQWRkVkNyb3NzU1YoXHJcbiAgICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eSxcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eSxcclxuICAgICAgYjJWZWMyLlN1YlZWKHdvcmxkUG9pbnQsIHRoaXMubV9jZW50ZXIsIHNfdDApLFxyXG4gICAgICBvdXQsXHJcbiAgICApO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IEdldExpbmVhclZlbG9jaXR5RnJvbVdvcmxkUG9pbnRfc190MCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgR2V0VXNlckRhdGEoKTogdm9pZCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3VzZXJEYXRhO1xyXG4gIH1cclxuXHJcbiAgU2V0VXNlckRhdGEoZGF0YTogYW55KTogdm9pZCB7XHJcbiAgICB0aGlzLm1fdXNlckRhdGEgPSBkYXRhO1xyXG4gIH1cclxuXHJcbiAgQXBwbHlGb3JjZShmb3JjZTogWFkpOiB2b2lkIHtcclxuICAgIHRoaXMubV9zeXN0ZW0uQXBwbHlGb3JjZSh0aGlzLm1fZmlyc3RJbmRleCwgdGhpcy5tX2xhc3RJbmRleCwgZm9yY2UpO1xyXG4gIH1cclxuXHJcbiAgQXBwbHlMaW5lYXJJbXB1bHNlKGltcHVsc2U6IFhZKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fc3lzdGVtLkFwcGx5TGluZWFySW1wdWxzZSh0aGlzLm1fZmlyc3RJbmRleCwgdGhpcy5tX2xhc3RJbmRleCwgaW1wdWxzZSk7XHJcbiAgfVxyXG5cclxuICBEZXN0cm95UGFydGljbGVzKGNhbGxEZXN0cnVjdGlvbkxpc3RlbmVyOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICBpZiAodGhpcy5tX3N5c3RlbS5tX3dvcmxkLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IHRoaXMubV9maXJzdEluZGV4OyBpIDwgdGhpcy5tX2xhc3RJbmRleDsgaSsrKSB7XHJcbiAgICAgIHRoaXMubV9zeXN0ZW0uRGVzdHJveVBhcnRpY2xlKGksIGNhbGxEZXN0cnVjdGlvbkxpc3RlbmVyKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIFVwZGF0ZVN0YXRpc3RpY3MoKTogdm9pZCB7XHJcbiAgICBpZiAoIXRoaXMubV9zeXN0ZW0ubV9wb3NpdGlvbkJ1ZmZlci5kYXRhKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG4gICAgaWYgKCF0aGlzLm1fc3lzdGVtLm1fdmVsb2NpdHlCdWZmZXIuZGF0YSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuICAgIGNvbnN0IHAgPSBuZXcgYjJWZWMyKCk7XHJcbiAgICBjb25zdCB2ID0gbmV3IGIyVmVjMigpO1xyXG4gICAgaWYgKHRoaXMubV90aW1lc3RhbXAgIT09IHRoaXMubV9zeXN0ZW0ubV90aW1lc3RhbXApIHtcclxuICAgICAgY29uc3QgbSA9IHRoaXMubV9zeXN0ZW0uR2V0UGFydGljbGVNYXNzKCk7XHJcbiAgICAgIC8vLyAgdGhpcy5tX21hc3MgPSAwO1xyXG4gICAgICB0aGlzLm1fbWFzcyA9IG0gKiAodGhpcy5tX2xhc3RJbmRleCAtIHRoaXMubV9maXJzdEluZGV4KTtcclxuICAgICAgdGhpcy5tX2NlbnRlci5TZXRaZXJvKCk7XHJcbiAgICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eS5TZXRaZXJvKCk7XHJcbiAgICAgIGZvciAobGV0IGkgPSB0aGlzLm1fZmlyc3RJbmRleDsgaSA8IHRoaXMubV9sYXN0SW5kZXg7IGkrKykge1xyXG4gICAgICAgIC8vLyAgdGhpcy5tX21hc3MgKz0gbTtcclxuICAgICAgICAvLy8gIHRoaXMubV9jZW50ZXIgKz0gbSAqIHRoaXMubV9zeXN0ZW0ubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgIHRoaXMubV9jZW50ZXIuU2VsZk11bEFkZChtLCB0aGlzLm1fc3lzdGVtLm1fcG9zaXRpb25CdWZmZXIuZGF0YVtpXSk7XHJcbiAgICAgICAgLy8vICB0aGlzLm1fbGluZWFyVmVsb2NpdHkgKz0gbSAqIHRoaXMubV9zeXN0ZW0ubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2ldO1xyXG4gICAgICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eS5TZWxmTXVsQWRkKG0sIHRoaXMubV9zeXN0ZW0ubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2ldKTtcclxuICAgICAgfVxyXG4gICAgICBpZiAodGhpcy5tX21hc3MgPiAwKSB7XHJcbiAgICAgICAgY29uc3QgaW52X21hc3MgPSAxIC8gdGhpcy5tX21hc3M7XHJcbiAgICAgICAgLy8vdGhpcy5tX2NlbnRlciAqPSAxIC8gdGhpcy5tX21hc3M7XHJcbiAgICAgICAgdGhpcy5tX2NlbnRlci5TZWxmTXVsKGludl9tYXNzKTtcclxuICAgICAgICAvLy90aGlzLm1fbGluZWFyVmVsb2NpdHkgKj0gMSAvIHRoaXMubV9tYXNzO1xyXG4gICAgICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eS5TZWxmTXVsKGludl9tYXNzKTtcclxuICAgICAgfVxyXG4gICAgICB0aGlzLm1faW5lcnRpYSA9IDA7XHJcbiAgICAgIHRoaXMubV9hbmd1bGFyVmVsb2NpdHkgPSAwO1xyXG4gICAgICBmb3IgKGxldCBpID0gdGhpcy5tX2ZpcnN0SW5kZXg7IGkgPCB0aGlzLm1fbGFzdEluZGV4OyBpKyspIHtcclxuICAgICAgICAvLy9iMlZlYzIgcCA9IHRoaXMubV9zeXN0ZW0ubV9wb3NpdGlvbkJ1ZmZlci5kYXRhW2ldIC0gdGhpcy5tX2NlbnRlcjtcclxuICAgICAgICBiMlZlYzIuU3ViVlYodGhpcy5tX3N5c3RlbS5tX3Bvc2l0aW9uQnVmZmVyLmRhdGFbaV0sIHRoaXMubV9jZW50ZXIsIHApO1xyXG4gICAgICAgIC8vL2IyVmVjMiB2ID0gdGhpcy5tX3N5c3RlbS5tX3ZlbG9jaXR5QnVmZmVyLmRhdGFbaV0gLSB0aGlzLm1fbGluZWFyVmVsb2NpdHk7XHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9zeXN0ZW0ubV92ZWxvY2l0eUJ1ZmZlci5kYXRhW2ldLCB0aGlzLm1fbGluZWFyVmVsb2NpdHksIHYpO1xyXG4gICAgICAgIHRoaXMubV9pbmVydGlhICs9IG0gKiBiMlZlYzIuRG90VlYocCwgcCk7XHJcbiAgICAgICAgdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eSArPSBtICogYjJWZWMyLkNyb3NzVlYocCwgdik7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9pbmVydGlhID4gMCkge1xyXG4gICAgICAgIHRoaXMubV9hbmd1bGFyVmVsb2NpdHkgKj0gMSAvIHRoaXMubV9pbmVydGlhO1xyXG4gICAgICB9XHJcbiAgICAgIHRoaXMubV90aW1lc3RhbXAgPSB0aGlzLm1fc3lzdGVtLm1fdGltZXN0YW1wO1xyXG4gICAgfVxyXG4gIH1cclxufVxyXG4iXX0=