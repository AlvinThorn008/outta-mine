/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
import { b2_epsilon, b2_pi, b2Assert, b2Maybe } from '../../common/b2Settings';
import { b2IsValid, b2Mat22, b2Rot, b2Transform, b2Vec2 } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
export class b2MouseJointDef extends b2JointDef {
    constructor() {
        super(5 /* e_mouseJoint */);
        this.target = new b2Vec2();
        this.maxForce = 0;
        this.frequencyHz = 5;
        this.dampingRatio = 0.7;
    }
}
export class b2MouseJoint extends b2Joint {
    constructor(def) {
        super(def);
        this.m_localAnchorB = new b2Vec2();
        this.m_targetA = new b2Vec2();
        this.m_frequencyHz = 0;
        this.m_dampingRatio = 0;
        this.m_beta = 0;
        // Solver shared
        this.m_impulse = new b2Vec2();
        this.m_maxForce = 0;
        this.m_gamma = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_rB = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_invMassB = 0;
        this.m_invIB = 0;
        this.m_mass = new b2Mat22();
        this.m_C = new b2Vec2();
        this.m_qB = new b2Rot();
        this.m_lalcB = new b2Vec2();
        this.m_K = new b2Mat22();
        this.m_targetA.Copy(b2Maybe(def.target, b2Vec2.ZERO));
        !!B2_DEBUG && b2Assert(this.m_targetA.IsValid());
        b2Transform.MulTXV(this.m_bodyB.GetTransform(), this.m_targetA, this.m_localAnchorB);
        this.m_maxForce = b2Maybe(def.maxForce, 0);
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_maxForce) && this.m_maxForce >= 0);
        this.m_impulse.SetZero();
        this.m_frequencyHz = b2Maybe(def.frequencyHz, 0);
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_frequencyHz) && this.m_frequencyHz >= 0);
        this.m_dampingRatio = b2Maybe(def.dampingRatio, 0);
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_dampingRatio) && this.m_dampingRatio >= 0);
        this.m_beta = 0;
        this.m_gamma = 0;
    }
    SetTarget(target) {
        if (!this.m_bodyB.IsAwake()) {
            this.m_bodyB.SetAwake(true);
        }
        this.m_targetA.Copy(target);
    }
    GetTarget() {
        return this.m_targetA;
    }
    SetMaxForce(maxForce) {
        this.m_maxForce = maxForce;
    }
    GetMaxForce() {
        return this.m_maxForce;
    }
    SetFrequency(hz) {
        this.m_frequencyHz = hz;
    }
    GetFrequency() {
        return this.m_frequencyHz;
    }
    SetDampingRatio(ratio) {
        this.m_dampingRatio = ratio;
    }
    GetDampingRatio() {
        return this.m_dampingRatio;
    }
    InitVelocityConstraints(data) {
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIB = this.m_bodyB.m_invI;
        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const qB = this.m_qB.SetAngle(aB);
        const mass = this.m_bodyB.GetMass();
        // Frequency
        const omega = 2 * b2_pi * this.m_frequencyHz;
        // Damping coefficient
        const d = 2 * mass * this.m_dampingRatio * omega;
        // Spring stiffness
        const k = mass * (omega * omega);
        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        const h = data.step.dt;
        !!B2_DEBUG && b2Assert(d + h * k > b2_epsilon);
        this.m_gamma = h * (d + h * k);
        if (this.m_gamma !== 0) {
            this.m_gamma = 1 / this.m_gamma;
        }
        this.m_beta = h * k * this.m_gamma;
        // Compute the effective mass matrix.
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        const K = this.m_K;
        K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
        K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;
        K.GetInverse(this.m_mass);
        // m_C = cB + m_rB - m_targetA;
        this.m_C.x = cB.x + this.m_rB.x - this.m_targetA.x;
        this.m_C.y = cB.y + this.m_rB.y - this.m_targetA.y;
        // m_C *= m_beta;
        this.m_C.SelfMul(this.m_beta);
        // Cheat with some damping
        wB *= 0.98;
        if (data.step.warmStarting) {
            this.m_impulse.SelfMul(data.step.dtRatio);
            // vB += m_invMassB * m_impulse;
            vB.x += this.m_invMassB * this.m_impulse.x;
            vB.y += this.m_invMassB * this.m_impulse.y;
            wB += this.m_invIB * b2Vec2.CrossVV(this.m_rB, this.m_impulse);
        }
        else {
            this.m_impulse.SetZero();
        }
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // Cdot = v + cross(w, r)
        // b2Vec2 Cdot = vB + b2Cross(wB, m_rB);
        const Cdot = b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2MouseJoint.SolveVelocityConstraints_s_Cdot);
        //  b2Vec2 impulse = b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));
        const impulse = b2Mat22.MulMV(this.m_mass, b2Vec2
            .AddVV(Cdot, b2Vec2.AddVV(this.m_C, b2Vec2.MulSV(this.m_gamma, this.m_impulse, b2Vec2.s_t0), b2Vec2.s_t0), b2Vec2.s_t0)
            .SelfNeg(), b2MouseJoint.SolveVelocityConstraints_s_impulse);
        // b2Vec2 oldImpulse = m_impulse;
        const oldImpulse = b2MouseJoint.SolveVelocityConstraints_s_oldImpulse.Copy(this.m_impulse);
        // m_impulse += impulse;
        this.m_impulse.SelfAdd(impulse);
        const maxImpulse = data.step.dt * this.m_maxForce;
        if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
            this.m_impulse.SelfMul(maxImpulse / this.m_impulse.Length());
        }
        // impulse = m_impulse - oldImpulse;
        b2Vec2.SubVV(this.m_impulse, oldImpulse, impulse);
        // vB += m_invMassB * impulse;
        vB.SelfMulAdd(this.m_invMassB, impulse);
        wB += this.m_invIB * b2Vec2.CrossVV(this.m_rB, impulse);
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        return true;
    }
    GetAnchorA(out) {
        out.x = this.m_targetA.x;
        out.y = this.m_targetA.y;
        return out;
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        return b2Vec2.MulSV(inv_dt, this.m_impulse, out);
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    ShiftOrigin(newOrigin) {
        this.m_targetA.SelfSub(newOrigin);
    }
}
b2MouseJoint.SolveVelocityConstraints_s_Cdot = new b2Vec2();
b2MouseJoint.SolveVelocityConstraints_s_impulse = new b2Vec2();
b2MouseJoint.SolveVelocityConstraints_s_oldImpulse = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJNb3VzZUpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2pvaW50cy9iMk1vdXNlSm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQUUsVUFBVSxFQUFFLEtBQUssRUFBRSxRQUFRLEVBQUUsT0FBTyxFQUFFLE1BQU0seUJBQXlCLENBQUM7QUFDL0UsT0FBTyxFQUFFLFNBQVMsRUFBRSxPQUFPLEVBQUUsS0FBSyxFQUFFLFdBQVcsRUFBRSxNQUFNLEVBQU0sTUFBTSxxQkFBcUIsQ0FBQztBQUN6RixPQUFPLEVBQWUsT0FBTyxFQUFFLFVBQVUsRUFBZSxNQUFNLFdBQVcsQ0FBQztBQWExRSwrREFBK0Q7QUFDL0QseUNBQXlDO0FBQ3pDLE1BQU0sT0FBTyxlQUFnQixTQUFRLFVBQVU7SUFTN0M7UUFDRSxLQUFLLHNCQUEwQixDQUFDO1FBVHpCLFdBQU0sR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBRXZDLGFBQVEsR0FBRyxDQUFDLENBQUM7UUFFYixnQkFBVyxHQUFHLENBQUMsQ0FBQztRQUVoQixpQkFBWSxHQUFHLEdBQUcsQ0FBQztJQUluQixDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sWUFBYSxTQUFRLE9BQU87SUF5QnZDLFlBQVksR0FBcUI7UUFDL0IsS0FBSyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBekJKLG1CQUFjLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUN0QyxjQUFTLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUMxQyxrQkFBYSxHQUFHLENBQUMsQ0FBQztRQUNsQixtQkFBYyxHQUFHLENBQUMsQ0FBQztRQUNuQixXQUFNLEdBQUcsQ0FBQyxDQUFDO1FBRVgsZ0JBQWdCO1FBQ1AsY0FBUyxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDMUMsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUNmLFlB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SUFDeEMsQ0FBQztJQUVELHdCQUF3QixDQUFDLElBQWtCO1FBQ3pDLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztJQUVELFVBQVUsQ0FBZSxHQUFNO1FBQzdCLEdBQUcsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxDQUFDLENBQUM7UUFDekIsR0FBRyxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsU0FBUyxDQUFDLENBQUMsQ0FBQztRQUN6QixPQUFPLEdBQUcsQ0FBQztJQUNiLENBQUM7SUFFRCxVQUFVLENBQWUsR0FBTTtRQUM3QixPQUFPLElBQUksQ0FBQyxPQUFPLENBQUMsYUFBYSxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsR0FBRyxDQUFDLENBQUM7SUFDOUQsQ0FBQztJQUVELGdCQUFnQixDQUFlLE1BQWMsRUFBRSxHQUFNO1FBQ25ELE9BQU8sTUFBTSxDQUFDLEtBQUssQ0FBQyxNQUFNLEVBQUUsSUFBSSxDQUFDLFNBQVMsRUFBRSxHQUFHLENBQUMsQ0FBQztJQUNuRCxDQUFDO0lBRUQsaUJBQWlCLENBQUMsTUFBYztRQUM5QixPQUFPLENBQUMsQ0FBQztJQUNYLENBQUM7SUFFRCxXQUFXLENBQUMsU0FBaUI7UUFDM0IsSUFBSSxDQUFDLFNBQVMsQ0FBQyxPQUFPLENBQUMsU0FBUyxDQUFDLENBQUM7SUFDcEMsQ0FBQzs7QUE1RWMsNENBQStCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUMvQywrQ0FBa0MsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ2xELGtEQUFxQyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMDYtMjAwNyBFcmluIENhdHRvIGh0dHA6Ly93d3cuYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7IGIyX2Vwc2lsb24sIGIyX3BpLCBiMkFzc2VydCwgYjJNYXliZSB9IGZyb20gJy4uLy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJJc1ZhbGlkLCBiMk1hdDIyLCBiMlJvdCwgYjJUcmFuc2Zvcm0sIGIyVmVjMiwgWFkgfSBmcm9tICcuLi8uLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJJSm9pbnREZWYsIGIySm9pbnQsIGIySm9pbnREZWYsIGIySm9pbnRUeXBlIH0gZnJvbSAnLi9iMkpvaW50JztcclxuaW1wb3J0IHsgYjJTb2x2ZXJEYXRhIH0gZnJvbSAnLi4vYjJUaW1lU3RlcCc7XHJcblxyXG5leHBvcnQgaW50ZXJmYWNlIGIySU1vdXNlSm9pbnREZWYgZXh0ZW5kcyBiMklKb2ludERlZiB7XHJcbiAgdGFyZ2V0PzogWFk7XHJcblxyXG4gIG1heEZvcmNlPzogbnVtYmVyO1xyXG5cclxuICBmcmVxdWVuY3lIej86IG51bWJlcjtcclxuXHJcbiAgZGFtcGluZ1JhdGlvPzogbnVtYmVyO1xyXG59XHJcblxyXG4vLy8gTW91c2Ugam9pbnQgZGVmaW5pdGlvbi4gVGhpcyByZXF1aXJlcyBhIHdvcmxkIHRhcmdldCBwb2ludCxcclxuLy8vIHR1bmluZyBwYXJhbWV0ZXJzLCBhbmQgdGhlIHRpbWUgc3RlcC5cclxuZXhwb3J0IGNsYXNzIGIyTW91c2VKb2ludERlZiBleHRlbmRzIGIySm9pbnREZWYgaW1wbGVtZW50cyBiMklNb3VzZUpvaW50RGVmIHtcclxuICByZWFkb25seSB0YXJnZXQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgbWF4Rm9yY2UgPSAwO1xyXG5cclxuICBmcmVxdWVuY3lIeiA9IDU7XHJcblxyXG4gIGRhbXBpbmdSYXRpbyA9IDAuNztcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICBzdXBlcihiMkpvaW50VHlwZS5lX21vdXNlSm9pbnQpO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyTW91c2VKb2ludCBleHRlbmRzIGIySm9pbnQge1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV90YXJnZXRBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgbV9mcmVxdWVuY3lIeiA9IDA7XHJcbiAgbV9kYW1waW5nUmF0aW8gPSAwO1xyXG4gIG1fYmV0YSA9IDA7XHJcblxyXG4gIC8vIFNvbHZlciBzaGFyZWRcclxuICByZWFkb25seSBtX2ltcHVsc2U6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBtX21heEZvcmNlID0gMDtcclxuICBtX2dhbW1hID0gMDtcclxuXHJcbiAgLy8gU29sdmVyIHRlbXBcclxuICBtX2luZGV4QSA9IDA7XHJcbiAgbV9pbmRleEIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsQ2VudGVyQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1faW52TWFzc0IgPSAwO1xyXG4gIG1faW52SUIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fbWFzczogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcbiAgcmVhZG9ubHkgbV9DOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9xQjogYjJSb3QgPSBuZXcgYjJSb3QoKTtcclxuICByZWFkb25seSBtX2xhbGNCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9LOiBiMk1hdDIyID0gbmV3IGIyTWF0MjIoKTtcclxuXHJcbiAgY29uc3RydWN0b3IoZGVmOiBiMklNb3VzZUpvaW50RGVmKSB7XHJcbiAgICBzdXBlcihkZWYpO1xyXG5cclxuICAgIHRoaXMubV90YXJnZXRBLkNvcHkoYjJNYXliZShkZWYudGFyZ2V0LCBiMlZlYzIuWkVSTykpO1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzLm1fdGFyZ2V0QS5Jc1ZhbGlkKCkpO1xyXG4gICAgYjJUcmFuc2Zvcm0uTXVsVFhWKHRoaXMubV9ib2R5Qi5HZXRUcmFuc2Zvcm0oKSwgdGhpcy5tX3RhcmdldEEsIHRoaXMubV9sb2NhbEFuY2hvckIpO1xyXG5cclxuICAgIHRoaXMubV9tYXhGb3JjZSA9IGIyTWF5YmUoZGVmLm1heEZvcmNlLCAwKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYjJJc1ZhbGlkKHRoaXMubV9tYXhGb3JjZSkgJiYgdGhpcy5tX21heEZvcmNlID49IDApO1xyXG4gICAgdGhpcy5tX2ltcHVsc2UuU2V0WmVybygpO1xyXG5cclxuICAgIHRoaXMubV9mcmVxdWVuY3lIeiA9IGIyTWF5YmUoZGVmLmZyZXF1ZW5jeUh6LCAwKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYjJJc1ZhbGlkKHRoaXMubV9mcmVxdWVuY3lIeikgJiYgdGhpcy5tX2ZyZXF1ZW5jeUh6ID49IDApO1xyXG4gICAgdGhpcy5tX2RhbXBpbmdSYXRpbyA9IGIyTWF5YmUoZGVmLmRhbXBpbmdSYXRpbywgMCk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGIySXNWYWxpZCh0aGlzLm1fZGFtcGluZ1JhdGlvKSAmJiB0aGlzLm1fZGFtcGluZ1JhdGlvID49IDApO1xyXG5cclxuICAgIHRoaXMubV9iZXRhID0gMDtcclxuICAgIHRoaXMubV9nYW1tYSA9IDA7XHJcbiAgfVxyXG5cclxuICBTZXRUYXJnZXQodGFyZ2V0OiBiMlZlYzIpOiB2b2lkIHtcclxuICAgIGlmICghdGhpcy5tX2JvZHlCLklzQXdha2UoKSkge1xyXG4gICAgICB0aGlzLm1fYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fdGFyZ2V0QS5Db3B5KHRhcmdldCk7XHJcbiAgfVxyXG5cclxuICBHZXRUYXJnZXQoKSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3RhcmdldEE7XHJcbiAgfVxyXG5cclxuICBTZXRNYXhGb3JjZShtYXhGb3JjZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fbWF4Rm9yY2UgPSBtYXhGb3JjZTtcclxuICB9XHJcblxyXG4gIEdldE1heEZvcmNlKCkge1xyXG4gICAgcmV0dXJuIHRoaXMubV9tYXhGb3JjZTtcclxuICB9XHJcblxyXG4gIFNldEZyZXF1ZW5jeShoejogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZnJlcXVlbmN5SHogPSBoejtcclxuICB9XHJcblxyXG4gIEdldEZyZXF1ZW5jeSgpIHtcclxuICAgIHJldHVybiB0aGlzLm1fZnJlcXVlbmN5SHo7XHJcbiAgfVxyXG5cclxuICBTZXREYW1waW5nUmF0aW8ocmF0aW86IG51bWJlcikge1xyXG4gICAgdGhpcy5tX2RhbXBpbmdSYXRpbyA9IHJhdGlvO1xyXG4gIH1cclxuXHJcbiAgR2V0RGFtcGluZ1JhdGlvKCkge1xyXG4gICAgcmV0dXJuIHRoaXMubV9kYW1waW5nUmF0aW87XHJcbiAgfVxyXG5cclxuICBJbml0VmVsb2NpdHlDb25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiB2b2lkIHtcclxuICAgIHRoaXMubV9pbmRleEIgPSB0aGlzLm1fYm9keUIubV9pc2xhbmRJbmRleDtcclxuICAgIHRoaXMubV9sb2NhbENlbnRlckIuQ29weSh0aGlzLm1fYm9keUIubV9zd2VlcC5sb2NhbENlbnRlcik7XHJcbiAgICB0aGlzLm1faW52TWFzc0IgPSB0aGlzLm1fYm9keUIubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX2ludklCID0gdGhpcy5tX2JvZHlCLm1faW52STtcclxuXHJcbiAgICBjb25zdCBjQiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBjb25zdCBhQiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmE7XHJcbiAgICBjb25zdCB2QiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52O1xyXG4gICAgbGV0IHdCID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgY29uc3QgcUIgPSB0aGlzLm1fcUIuU2V0QW5nbGUoYUIpO1xyXG5cclxuICAgIGNvbnN0IG1hc3MgPSB0aGlzLm1fYm9keUIuR2V0TWFzcygpO1xyXG5cclxuICAgIC8vIEZyZXF1ZW5jeVxyXG4gICAgY29uc3Qgb21lZ2EgPSAyICogYjJfcGkgKiB0aGlzLm1fZnJlcXVlbmN5SHo7XHJcblxyXG4gICAgLy8gRGFtcGluZyBjb2VmZmljaWVudFxyXG4gICAgY29uc3QgZCA9IDIgKiBtYXNzICogdGhpcy5tX2RhbXBpbmdSYXRpbyAqIG9tZWdhO1xyXG5cclxuICAgIC8vIFNwcmluZyBzdGlmZm5lc3NcclxuICAgIGNvbnN0IGsgPSBtYXNzICogKG9tZWdhICogb21lZ2EpO1xyXG5cclxuICAgIC8vIG1hZ2ljIGZvcm11bGFzXHJcbiAgICAvLyBnYW1tYSBoYXMgdW5pdHMgb2YgaW52ZXJzZSBtYXNzLlxyXG4gICAgLy8gYmV0YSBoYXMgdW5pdHMgb2YgaW52ZXJzZSB0aW1lLlxyXG4gICAgY29uc3QgaCA9IGRhdGEuc3RlcC5kdDtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZCArIGggKiBrID4gYjJfZXBzaWxvbik7XHJcbiAgICB0aGlzLm1fZ2FtbWEgPSBoICogKGQgKyBoICogayk7XHJcbiAgICBpZiAodGhpcy5tX2dhbW1hICE9PSAwKSB7XHJcbiAgICAgIHRoaXMubV9nYW1tYSA9IDEgLyB0aGlzLm1fZ2FtbWE7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fYmV0YSA9IGggKiBrICogdGhpcy5tX2dhbW1hO1xyXG5cclxuICAgIC8vIENvbXB1dGUgdGhlIGVmZmVjdGl2ZSBtYXNzIG1hdHJpeC5cclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgYjJSb3QuTXVsUlYocUIsIHRoaXMubV9sYWxjQiwgdGhpcy5tX3JCKTtcclxuXHJcbiAgICAvLyBLICAgID0gWygxL20xICsgMS9tMikgKiBleWUoMikgLSBza2V3KHIxKSAqIGludkkxICogc2tldyhyMSkgLSBza2V3KHIyKSAqIGludkkyICogc2tldyhyMildXHJcbiAgICAvLyAgICAgID0gWzEvbTErMS9tMiAgICAgMCAgICBdICsgaW52STEgKiBbcjEueSpyMS55IC1yMS54KnIxLnldICsgaW52STIgKiBbcjEueSpyMS55IC1yMS54KnIxLnldXHJcbiAgICAvLyAgICAgICAgWyAgICAwICAgICAxL20xKzEvbTJdICAgICAgICAgICBbLXIxLngqcjEueSByMS54KnIxLnhdICAgICAgICAgICBbLXIxLngqcjEueSByMS54KnIxLnhdXHJcbiAgICBjb25zdCBLID0gdGhpcy5tX0s7XHJcbiAgICBLLmV4LnggPSB0aGlzLm1faW52TWFzc0IgKyB0aGlzLm1faW52SUIgKiB0aGlzLm1fckIueSAqIHRoaXMubV9yQi55ICsgdGhpcy5tX2dhbW1hO1xyXG4gICAgSy5leC55ID0gLXRoaXMubV9pbnZJQiAqIHRoaXMubV9yQi54ICogdGhpcy5tX3JCLnk7XHJcbiAgICBLLmV5LnggPSBLLmV4Lnk7XHJcbiAgICBLLmV5LnkgPSB0aGlzLm1faW52TWFzc0IgKyB0aGlzLm1faW52SUIgKiB0aGlzLm1fckIueCAqIHRoaXMubV9yQi54ICsgdGhpcy5tX2dhbW1hO1xyXG5cclxuICAgIEsuR2V0SW52ZXJzZSh0aGlzLm1fbWFzcyk7XHJcblxyXG4gICAgLy8gbV9DID0gY0IgKyBtX3JCIC0gbV90YXJnZXRBO1xyXG4gICAgdGhpcy5tX0MueCA9IGNCLnggKyB0aGlzLm1fckIueCAtIHRoaXMubV90YXJnZXRBLng7XHJcbiAgICB0aGlzLm1fQy55ID0gY0IueSArIHRoaXMubV9yQi55IC0gdGhpcy5tX3RhcmdldEEueTtcclxuICAgIC8vIG1fQyAqPSBtX2JldGE7XHJcbiAgICB0aGlzLm1fQy5TZWxmTXVsKHRoaXMubV9iZXRhKTtcclxuXHJcbiAgICAvLyBDaGVhdCB3aXRoIHNvbWUgZGFtcGluZ1xyXG4gICAgd0IgKj0gMC45ODtcclxuXHJcbiAgICBpZiAoZGF0YS5zdGVwLndhcm1TdGFydGluZykge1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS5TZWxmTXVsKGRhdGEuc3RlcC5kdFJhdGlvKTtcclxuICAgICAgLy8gdkIgKz0gbV9pbnZNYXNzQiAqIG1faW1wdWxzZTtcclxuICAgICAgdkIueCArPSB0aGlzLm1faW52TWFzc0IgKiB0aGlzLm1faW1wdWxzZS54O1xyXG4gICAgICB2Qi55ICs9IHRoaXMubV9pbnZNYXNzQiAqIHRoaXMubV9pbXB1bHNlLnk7XHJcbiAgICAgIHdCICs9IHRoaXMubV9pbnZJQiAqIGIyVmVjMi5Dcm9zc1ZWKHRoaXMubV9yQiwgdGhpcy5tX2ltcHVsc2UpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgdGhpcy5tX2ltcHVsc2UuU2V0WmVybygpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52ID0gdkI7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udyA9IHdCO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfQ2RvdCA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19pbXB1bHNlID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX29sZEltcHVsc2UgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiB2b2lkIHtcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgLy8gQ2RvdCA9IHYgKyBjcm9zcyh3LCByKVxyXG4gICAgLy8gYjJWZWMyIENkb3QgPSB2QiArIGIyQ3Jvc3Mod0IsIG1fckIpO1xyXG4gICAgY29uc3QgQ2RvdDogYjJWZWMyID0gYjJWZWMyLkFkZFZDcm9zc1NWKFxyXG4gICAgICB2QixcclxuICAgICAgd0IsXHJcbiAgICAgIHRoaXMubV9yQixcclxuICAgICAgYjJNb3VzZUpvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX0Nkb3QsXHJcbiAgICApO1xyXG4gICAgLy8gIGIyVmVjMiBpbXB1bHNlID0gYjJNdWwobV9tYXNzLCAtKENkb3QgKyBtX0MgKyBtX2dhbW1hICogbV9pbXB1bHNlKSk7XHJcbiAgICBjb25zdCBpbXB1bHNlOiBiMlZlYzIgPSBiMk1hdDIyLk11bE1WKFxyXG4gICAgICB0aGlzLm1fbWFzcyxcclxuICAgICAgYjJWZWMyXHJcbiAgICAgICAgLkFkZFZWKFxyXG4gICAgICAgICAgQ2RvdCxcclxuICAgICAgICAgIGIyVmVjMi5BZGRWVihcclxuICAgICAgICAgICAgdGhpcy5tX0MsXHJcbiAgICAgICAgICAgIGIyVmVjMi5NdWxTVih0aGlzLm1fZ2FtbWEsIHRoaXMubV9pbXB1bHNlLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgICAgIGIyVmVjMi5zX3QwLFxyXG4gICAgICAgICAgKSxcclxuICAgICAgICAgIGIyVmVjMi5zX3QwLFxyXG4gICAgICAgIClcclxuICAgICAgICAuU2VsZk5lZygpLFxyXG4gICAgICBiMk1vdXNlSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfaW1wdWxzZSxcclxuICAgICk7XHJcblxyXG4gICAgLy8gYjJWZWMyIG9sZEltcHVsc2UgPSBtX2ltcHVsc2U7XHJcbiAgICBjb25zdCBvbGRJbXB1bHNlID0gYjJNb3VzZUpvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX29sZEltcHVsc2UuQ29weSh0aGlzLm1faW1wdWxzZSk7XHJcbiAgICAvLyBtX2ltcHVsc2UgKz0gaW1wdWxzZTtcclxuICAgIHRoaXMubV9pbXB1bHNlLlNlbGZBZGQoaW1wdWxzZSk7XHJcbiAgICBjb25zdCBtYXhJbXB1bHNlOiBudW1iZXIgPSBkYXRhLnN0ZXAuZHQgKiB0aGlzLm1fbWF4Rm9yY2U7XHJcbiAgICBpZiAodGhpcy5tX2ltcHVsc2UuTGVuZ3RoU3F1YXJlZCgpID4gbWF4SW1wdWxzZSAqIG1heEltcHVsc2UpIHtcclxuICAgICAgdGhpcy5tX2ltcHVsc2UuU2VsZk11bChtYXhJbXB1bHNlIC8gdGhpcy5tX2ltcHVsc2UuTGVuZ3RoKCkpO1xyXG4gICAgfVxyXG4gICAgLy8gaW1wdWxzZSA9IG1faW1wdWxzZSAtIG9sZEltcHVsc2U7XHJcbiAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2ltcHVsc2UsIG9sZEltcHVsc2UsIGltcHVsc2UpO1xyXG5cclxuICAgIC8vIHZCICs9IG1faW52TWFzc0IgKiBpbXB1bHNlO1xyXG4gICAgdkIuU2VsZk11bEFkZCh0aGlzLm1faW52TWFzc0IsIGltcHVsc2UpO1xyXG4gICAgd0IgKz0gdGhpcy5tX2ludklCICogYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JCLCBpbXB1bHNlKTtcclxuXHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udiA9IHZCO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLncgPSB3QjtcclxuICB9XHJcblxyXG4gIFNvbHZlUG9zaXRpb25Db25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQTxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgb3V0LnggPSB0aGlzLm1fdGFyZ2V0QS54O1xyXG4gICAgb3V0LnkgPSB0aGlzLm1fdGFyZ2V0QS55O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIEdldEFuY2hvckI8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUIuR2V0V29ybGRQb2ludCh0aGlzLm1fbG9jYWxBbmNob3JCLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgR2V0UmVhY3Rpb25Gb3JjZTxUIGV4dGVuZHMgWFk+KGludl9kdDogbnVtYmVyLCBvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiBiMlZlYzIuTXVsU1YoaW52X2R0LCB0aGlzLm1faW1wdWxzZSwgb3V0KTtcclxuICB9XHJcblxyXG4gIEdldFJlYWN0aW9uVG9ycXVlKGludl9kdDogbnVtYmVyKTogbnVtYmVyIHtcclxuICAgIHJldHVybiAwO1xyXG4gIH1cclxuXHJcbiAgU2hpZnRPcmlnaW4obmV3T3JpZ2luOiBiMlZlYzIpIHtcclxuICAgIHRoaXMubV90YXJnZXRBLlNlbGZTdWIobmV3T3JpZ2luKTtcclxuICB9XHJcbn1cclxuIl19