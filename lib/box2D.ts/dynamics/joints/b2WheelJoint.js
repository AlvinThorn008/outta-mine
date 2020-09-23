/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
import { b2_linearSlop, b2_pi, b2Assert, b2Maybe } from '../../common/b2Settings';
import { b2Abs, b2Clamp, b2Rot, b2Vec2 } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
export class b2WheelJointDef extends b2JointDef {
    constructor() {
        super(7 /* e_wheelJoint */);
        this.localAnchorA = new b2Vec2(0, 0);
        this.localAnchorB = new b2Vec2(0, 0);
        this.localAxisA = new b2Vec2(1, 0);
        this.enableMotor = false;
        this.maxMotorTorque = 0;
        this.motorSpeed = 0;
        this.frequencyHz = 2;
        this.dampingRatio = 0.7;
    }
    Initialize(bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
    }
}
export class b2WheelJoint extends b2Joint {
    constructor(def) {
        super(def);
        this.m_frequencyHz = 0;
        this.m_dampingRatio = 0;
        // Solver shared
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_localXAxisA = new b2Vec2();
        this.m_localYAxisA = new b2Vec2();
        this.m_impulse = 0;
        this.m_motorImpulse = 0;
        this.m_springImpulse = 0;
        this.m_maxMotorTorque = 0;
        this.m_motorSpeed = 0;
        this.m_enableMotor = false;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_ax = new b2Vec2();
        this.m_ay = new b2Vec2();
        this.m_sAx = 0;
        this.m_sBx = 0;
        this.m_sAy = 0;
        this.m_sBy = 0;
        this.m_mass = 0;
        this.m_motorMass = 0;
        this.m_springMass = 0;
        this.m_bias = 0;
        this.m_gamma = 0;
        this.m_qA = new b2Rot();
        this.m_qB = new b2Rot();
        this.m_lalcA = new b2Vec2();
        this.m_lalcB = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_frequencyHz = b2Maybe(def.frequencyHz, 2);
        this.m_dampingRatio = b2Maybe(def.dampingRatio, 0.7);
        this.m_localAnchorA.Copy(b2Maybe(def.localAnchorA, b2Vec2.ZERO));
        this.m_localAnchorB.Copy(b2Maybe(def.localAnchorB, b2Vec2.ZERO));
        this.m_localXAxisA.Copy(b2Maybe(def.localAxisA, b2Vec2.UNITX));
        b2Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA);
        this.m_maxMotorTorque = b2Maybe(def.maxMotorTorque, 0);
        this.m_motorSpeed = b2Maybe(def.motorSpeed, 0);
        this.m_enableMotor = b2Maybe(def.enableMotor, false);
        this.m_ax.SetZero();
        this.m_ay.SetZero();
    }
    GetMotorSpeed() {
        return this.m_motorSpeed;
    }
    GetMaxMotorTorque() {
        return this.m_maxMotorTorque;
    }
    SetSpringFrequencyHz(hz) {
        this.m_frequencyHz = hz;
    }
    GetSpringFrequencyHz() {
        return this.m_frequencyHz;
    }
    SetSpringDampingRatio(ratio) {
        this.m_dampingRatio = ratio;
    }
    GetSpringDampingRatio() {
        return this.m_dampingRatio;
    }
    InitVelocityConstraints(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const cA = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // Compute the effective masses.
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = cB + rB - cA - rA;
        const d = b2Vec2.SubVV(b2Vec2.AddVV(cB, rB, b2Vec2.s_t0), b2Vec2.AddVV(cA, rA, b2Vec2.s_t1), b2WheelJoint.InitVelocityConstraints_s_d);
        // Point to line constraint
        {
            // m_ay = b2Mul(qA, m_localYAxisA);
            b2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);
            // m_sAy = b2Cross(d + rA, m_ay);
            this.m_sAy = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), this.m_ay);
            // m_sBy = b2Cross(rB, m_ay);
            this.m_sBy = b2Vec2.CrossVV(rB, this.m_ay);
            this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;
            if (this.m_mass > 0) {
                this.m_mass = 1 / this.m_mass;
            }
        }
        // Spring constraint
        this.m_springMass = 0;
        this.m_bias = 0;
        this.m_gamma = 0;
        if (this.m_frequencyHz > 0) {
            // m_ax = b2Mul(qA, m_localXAxisA);
            b2Rot.MulRV(qA, this.m_localXAxisA, this.m_ax);
            // m_sAx = b2Cross(d + rA, m_ax);
            this.m_sAx = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), this.m_ax);
            // m_sBx = b2Cross(rB, m_ax);
            this.m_sBx = b2Vec2.CrossVV(rB, this.m_ax);
            const invMass = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;
            if (invMass > 0) {
                this.m_springMass = 1 / invMass;
                const C = b2Vec2.DotVV(d, this.m_ax);
                // Frequency
                const omega = 2 * b2_pi * this.m_frequencyHz;
                // Damping coefficient
                const damp = 2 * this.m_springMass * this.m_dampingRatio * omega;
                // Spring stiffness
                const k = this.m_springMass * omega * omega;
                // magic formulas
                const h = data.step.dt;
                this.m_gamma = h * (damp + h * k);
                if (this.m_gamma > 0) {
                    this.m_gamma = 1 / this.m_gamma;
                }
                this.m_bias = C * h * k * this.m_gamma;
                this.m_springMass = invMass + this.m_gamma;
                if (this.m_springMass > 0) {
                    this.m_springMass = 1 / this.m_springMass;
                }
            }
        }
        else {
            this.m_springImpulse = 0;
        }
        // Rotational motor
        if (this.m_enableMotor) {
            this.m_motorMass = iA + iB;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        }
        else {
            this.m_motorMass = 0;
            this.m_motorImpulse = 0;
        }
        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse *= data.step.dtRatio;
            this.m_springImpulse *= data.step.dtRatio;
            this.m_motorImpulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
            const P = b2Vec2.AddVV(b2Vec2.MulSV(this.m_impulse, this.m_ay, b2Vec2.s_t0), b2Vec2.MulSV(this.m_springImpulse, this.m_ax, b2Vec2.s_t1), b2WheelJoint.InitVelocityConstraints_s_P);
            // float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
            const LA = this.m_impulse * this.m_sAy + this.m_springImpulse * this.m_sAx + this.m_motorImpulse;
            // float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;
            const LB = this.m_impulse * this.m_sBy + this.m_springImpulse * this.m_sBx + this.m_motorImpulse;
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            wA -= this.m_invIA * LA;
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
            wB += this.m_invIB * LB;
        }
        else {
            this.m_impulse = 0;
            this.m_springImpulse = 0;
            this.m_motorImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // Solve spring constraint
        {
            const Cdot = b2Vec2.DotVV(this.m_ax, b2Vec2.SubVV(vB, vA, b2Vec2.s_t0)) +
                this.m_sBx * wB -
                this.m_sAx * wA;
            const impulse = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
            this.m_springImpulse += impulse;
            // b2Vec2 P = impulse * m_ax;
            const P = b2Vec2.MulSV(impulse, this.m_ax, b2WheelJoint.SolveVelocityConstraints_s_P);
            const LA = impulse * this.m_sAx;
            const LB = impulse * this.m_sBx;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // Solve rotational motor constraint
        {
            const Cdot = wB - wA - this.m_motorSpeed;
            let impulse = -this.m_motorMass * Cdot;
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve point to line constraint
        {
            const Cdot = b2Vec2.DotVV(this.m_ay, b2Vec2.SubVV(vB, vA, b2Vec2.s_t0)) +
                this.m_sBy * wB -
                this.m_sAy * wA;
            const impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;
            // b2Vec2 P = impulse * m_ay;
            const P = b2Vec2.MulSV(impulse, this.m_ay, b2WheelJoint.SolveVelocityConstraints_s_P);
            const LA = impulse * this.m_sAy;
            const LB = impulse * this.m_sBy;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = (cB - cA) + rB - rA;
        const d = b2Vec2.AddVV(b2Vec2.SubVV(cB, cA, b2Vec2.s_t0), b2Vec2.SubVV(rB, rA, b2Vec2.s_t1), b2WheelJoint.SolvePositionConstraints_s_d);
        // b2Vec2 ay = b2Mul(qA, m_localYAxisA);
        const ay = b2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);
        // float32 sAy = b2Cross(d + rA, ay);
        const sAy = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), ay);
        // float32 sBy = b2Cross(rB, ay);
        const sBy = b2Vec2.CrossVV(rB, ay);
        // float32 C = b2Dot(d, ay);
        const C = b2Vec2.DotVV(d, this.m_ay);
        const k = this.m_invMassA +
            this.m_invMassB +
            this.m_invIA * this.m_sAy * this.m_sAy +
            this.m_invIB * this.m_sBy * this.m_sBy;
        let impulse;
        if (k !== 0) {
            impulse = -C / k;
        }
        else {
            impulse = 0;
        }
        // b2Vec2 P = impulse * ay;
        const P = b2Vec2.MulSV(impulse, ay, b2WheelJoint.SolvePositionConstraints_s_P);
        const LA = impulse * sAy;
        const LB = impulse * sBy;
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * LA;
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        aB += this.m_invIB * LB;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return b2Abs(C) <= b2_linearSlop;
    }
    GetDefinition(def) {
        !!B2_DEBUG && b2Assert(false); // TODO
        return def;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
        out.x = inv_dt * (this.m_impulse * this.m_ay.x + this.m_springImpulse * this.m_ax.x);
        out.y = inv_dt * (this.m_impulse * this.m_ay.y + this.m_springImpulse * this.m_ax.y);
        return out;
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    GetLocalAxisA() {
        return this.m_localXAxisA;
    }
    GetJointTranslation() {
        return this.GetPrismaticJointTranslation();
    }
    GetJointLinearSpeed() {
        return this.GetPrismaticJointSpeed();
    }
    GetJointAngle() {
        return this.GetRevoluteJointAngle();
    }
    GetJointAngularSpeed() {
        return this.GetRevoluteJointSpeed();
    }
    GetPrismaticJointTranslation() {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;
        const pA = bA.GetWorldPoint(this.m_localAnchorA, new b2Vec2());
        const pB = bB.GetWorldPoint(this.m_localAnchorB, new b2Vec2());
        const d = b2Vec2.SubVV(pB, pA, new b2Vec2());
        const axis = bA.GetWorldVector(this.m_localXAxisA, new b2Vec2());
        const translation = b2Vec2.DotVV(d, axis);
        return translation;
    }
    GetPrismaticJointSpeed() {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;
        // b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
        b2Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
        const rA = b2Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
        b2Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
        const rB = b2Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
        // b2Vec2 pA = bA->m_sweep.c + rA;
        const pA = b2Vec2.AddVV(bA.m_sweep.c, rA, b2Vec2.s_t0); // pA uses s_t0
        // b2Vec2 pB = bB->m_sweep.c + rB;
        const pB = b2Vec2.AddVV(bB.m_sweep.c, rB, b2Vec2.s_t1); // pB uses s_t1
        // b2Vec2 d = pB - pA;
        const d = b2Vec2.SubVV(pB, pA, b2Vec2.s_t2); // d uses s_t2
        // b2Vec2 axis = b2Mul(bA.m_xf.q, m_localXAxisA);
        const axis = bA.GetWorldVector(this.m_localXAxisA, new b2Vec2());
        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;
        // float32 speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
        const speed = b2Vec2.DotVV(d, b2Vec2.CrossSV(wA, axis, b2Vec2.s_t0)) +
            b2Vec2.DotVV(axis, b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, rA, b2Vec2.s_t1), b2Vec2.s_t0));
        return speed;
    }
    GetRevoluteJointAngle() {
        // b2Body* bA = this.m_bodyA;
        // b2Body* bB = this.m_bodyB;
        // return bB->this.m_sweep.a - bA->this.m_sweep.a;
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a;
    }
    GetRevoluteJointSpeed() {
        const wA = this.m_bodyA.m_angularVelocity;
        const wB = this.m_bodyB.m_angularVelocity;
        return wB - wA;
    }
    IsMotorEnabled() {
        return this.m_enableMotor;
    }
    EnableMotor(flag) {
        if (flag !== this.m_enableMotor) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableMotor = flag;
        }
    }
    SetMotorSpeed(speed) {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
    }
    SetMaxMotorTorque(force) {
        if (force !== this.m_maxMotorTorque) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorTorque = force;
        }
    }
    GetMotorTorque(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
}
b2WheelJoint.InitVelocityConstraints_s_d = new b2Vec2();
b2WheelJoint.InitVelocityConstraints_s_P = new b2Vec2();
b2WheelJoint.SolveVelocityConstraints_s_P = new b2Vec2();
b2WheelJoint.SolvePositionConstraints_s_d = new b2Vec2();
b2WheelJoint.SolvePositionConstraints_s_P = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJXaGVlbEpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2pvaW50cy9iMldoZWVsSm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQUUsYUFBYSxFQUFFLEtBQUssRUFBRSxRQUFRLEVBQUUsT0FBTyxFQUFFLE1BQU0seUJBQXlCLENBQUM7QUFDbEYsT0FBTyxFQUFFLEtBQUssRUFBRSxPQUFPLEVBQUUsS0FBSyxFQUFFLE1BQU0sRUFBTSxNQUFNLHFCQUFxQixDQUFDO0FBQ3hFLE9BQU8sRUFBZSxPQUFPLEVBQUUsVUFBVSxFQUFlLE1BQU0sV0FBVyxDQUFDO0FBc0IxRSw0REFBNEQ7QUFDNUQsdUVBQXVFO0FBQ3ZFLG9FQUFvRTtBQUNwRSxzRUFBc0U7QUFDdEUscUVBQXFFO0FBQ3JFLGtFQUFrRTtBQUNsRSxNQUFNLE9BQU8sZUFBZ0IsU0FBUSxVQUFVO0lBaUI3QztRQUNFLEtBQUssc0JBQTBCLENBQUM7UUFqQnpCLGlCQUFZLEdBQVcsSUFBSSxNQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRXhDLGlCQUFZLEdBQVcsSUFBSSxNQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRXhDLGVBQVUsR0FBVyxJQUFJLE1BQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFL0MsZ0JBQVcsR0FBRyxLQUFLLENBQUM7UUFFcEIsbUJBQWMsR0FBRyxDQUFDLENBQUM7UUFFbkIsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUVmLGdCQUFXLEdBQUcsQ0FBQyxDQUFDO1FBRWhCLGlCQUFZLEdBQUcsR0FBRyxDQUFDO0lBSW5CLENBQUM7SUFFRCxVQUFVLENBQUMsRUFBVSxFQUFFLEVBQVUsRUFBRSxNQUFjLEVBQUUsSUFBWTtRQUM3RCxJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztRQUNoQixJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztRQUNoQixJQUFJLENBQUMsS0FBSyxDQUFDLGFBQWEsQ0FBQyxNQUFNLEVBQUUsSUFBSSxDQUFDLFlBQVksQ0FBQyxDQUFDO1FBQ3BELElBQUksQ0FBQyxLQUFLLENBQUMsYUFBYSxDQUFDLE1BQU0sRUFBRSxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUM7UUFDcEQsSUFBSSxDQUFDLEtBQUssQ0FBQyxjQUFjLENBQUMsSUFBSSxFQUFFLElBQUksQ0FBQyxVQUFVLENBQUMsQ0FBQztJQUNuRCxDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sWUFBYSxTQUFRLE9BQU87SUFpRHZDLFlBQVksR0FBcUI7UUFDL0IsS0FBSyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBakRiLGtCQUFhLEdBQUcsQ0FBQyxDQUFDO1FBQ2xCLG1CQUFjLEdBQUcsQ0FBQyxDQUFDO1FBRW5CLGdCQUFnQjtRQUNQLG1CQUFjLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUN0QyxtQkFBYyxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDdEMsa0JBQWEsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQ3JDLGtCQUFhLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUU5QyxjQUFTLEdBQUcsQ0FBQyxDQUFDO1FBQ2QsbUJBQWMsR0FBRyxDQUFDLENBQUM7UUFDbkIsb0JBQWUsR0FBRyxDQUFDLENBQUM7UUFFcEIscUJBQWdCLEdBQUcsQ0FBQyxDQUFDO1FBQ3JCLGlCQUFZLEdBQUcsQ0FBQyxDQUFDO1FBQ2pCLGtCQUFhLEdBQUcsS0FBSyxDQUFDO1FBRXRCLGNBQWM7UUFDZCxhQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2IsYUFBUSxHQUFHLENBQUMsQ0FBQztRQUNKLG1CQUFjLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUN0QyxtQkFBYyxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDL0MsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUNmLGVBQVUsR0FBRyxDQUFDLENBQUM7UUFDZixZQUFPLEdBQUcsQ0FBQyxDQUFDO1FBQ1osWUFBTyxHQUFHLENBQUMsQ0FBQztRQUVILFNBQUksR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQzVCLFNBQUksR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQ3JDLFVBQUssR0FBRyxDQUFDLENBQUM7UUFDVixVQUFLLEdBQUcsQ0FBQyxDQUFDO1FBQ1YsVUFBSyxHQUFHLENBQUMsQ0FBQztRQUNWLFVBQUssR0FBRyxDQUFDLENBQUM7UUFFVixXQUFNLEdBQUcsQ0FBQyxDQUFDO1FBQ1gsZ0JBQVcsR0FBRyxDQUFDLENBQUM7UUFDaEIsaUJBQVksR0FBRyxDQUFDLENBQUM7UUFFakIsV0FBTSxHQUFHLENBQUMsQ0FBQztRQUNYLFlBQU8sR0FBRyxDQUFDLENBQUM7UUFFSCxTQUFJLEdBQVUsSUFBSSxLQUFLLEVBQUUsQ0FBQztRQUMxQixTQUFJLEdBQVUsSUFBSSxLQUFLLEVBQUUsQ0FBQztRQUMxQixZQUFPLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUMvQixZQUFPLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUMvQixTQUFJLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUM1QixTQUFJLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUtuQyxJQUFJLENBQUMsYUFBYSxHQUFHLE9BQU8sQ0FBQyxHQUFHLENBQUMsV0FBVyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ2pELElBQUksQ0FBQyxjQUFjLEdBQUcsT0FBTyxDQUFDLEdBQUcsQ0FBQyxZQUFZLEVBQUUsR0FBRyxDQUFDLENBQUM7UUFFckQsSUFBSSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEdBQUcsQ0FBQyxZQUFZLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUM7UUFDakUsSUFBSSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEdBQUcsQ0FBQyxZQUFZLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUM7UUFDakUsSUFBSSxDQUFDLGFBQWEsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEdBQUcsQ0FBQyxVQUFVLEVBQUUsTUFBTSxDQUFDLEtBQUssQ0FBQyxDQUFDLENBQUM7UUFDL0QsTUFBTSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsYUFBYSxFQUFFLElBQUksQ0FBQyxhQUFhLENBQUMsQ0FBQztRQUV6RCxJQUFJLENBQUMsZ0JBQWdCLEdBQUcsT0FBTyxDQUFDLEdBQUcsQ0FBQyxjQUFjLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdkQsSUFBSSxDQUFDLFlBQVksR0FBRyxPQUFPLENBQUMsR0FBRyxDQUFDLFVBQVUsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUMvQyxJQUFJLENBQUMsYUFBYSxHQUFHLE9BQU8sQ0FBQyxHQUFHLENBQUMsV0FBVyxFQUFFLEtBQUssQ0FBQyxDQUFDO1FBRXJELElBQUksQ0FBQyxJQUFJLENBQUMsT0FBTyxFQUFFLENBQUM7UUFDcEIsSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLEVBQUUsQ0FBQztJQUN0QixDQUFDO0lBRUQsYUFBYTtRQUNYLE9BQU8sSUFBSSxDQUFDLFlBQVksQ0FBQztJQUMzQixDQUFDO0lBRUQsaUJBQWlCO1FBQ2YsT0FBTyxJQUFJLENBQUMsZ0JBQWdCLENBQUM7SUFDL0IsQ0FBQztJQUVELG9CQUFvQixDQUFDLEVBQVU7UUFDN0IsSUFBSSxDQUFDLGFBQWEsR0FBRyxFQUFFLENBQUM7SUFDMUIsQ0FBQztJQUVELG9CQUFvQjtRQUNsQixPQUFPLElBQUksQ0FBQyxhQUFhLENBQUM7SUFDNUIsQ0FBQztJQUVELHFCQUFxQixDQUFDLEtBQWE7UUFDakMsSUFBSSxDQUFDLGNBQWMsR0FBRyxLQUFLLENBQUM7SUFDOUIsQ0FBQztJQUVELHFCQUFxQjtRQUNuQixPQUFPLElBQUksQ0FBQyxjQUFjLENBQUM7SUFDN0IsQ0FBQztJQUtELHVCQUF1QixDQUFDLElBQWtCO1FBQ3hDLElBQUksQ0FBQyxRQUFRLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxhQUFhLENBQUM7UUFDM0MsSUFBSSxDQUFDLFFBQVEsR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLGFBQWEsQ0FBQztRQUMzQyxJQUFJLENBQUMsY0FBYyxDQUFDLElBQUksQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLE9BQU8sQ0FBQyxXQUFXLENBQUMsQ0FBQztRQUMzRCxJQUFJLENBQUMsY0FBYyxDQUFDLElBQUksQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLE9BQU8sQ0FBQyxXQUFXLENBQUMsQ0FBQztRQUMzRCxJQUFJLENBQUMsVUFBVSxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsU0FBUyxDQUFDO1FBQ3pDLElBQUksQ0FBQyxVQUFVLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxTQUFTLENBQUM7UUFDekMsSUFBSSxDQUFDLE9BQU8sR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLE1BQU0sQ0FBQztRQUNuQyxJQUFJLENBQUMsT0FBTyxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsTUFBTSxDQUFDO1FBRW5DLE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLEVBQ2hDLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDO1FBQy9CLE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxPQUFPLEVBQzdCLEVBQUUsR0FBVyxJQUFJLENBQUMsT0FBTyxDQUFDO1FBRTVCLE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNuRCxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDbkQsTUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ3BELElBQUksRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUVsRCxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDbkQsTUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ25ELE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNwRCxJQUFJLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFFbEQsTUFBTSxFQUFFLEdBQVUsSUFBSSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsRUFBRSxDQUFDLEVBQ3RDLEVBQUUsR0FBVSxJQUFJLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUVyQyxnQ0FBZ0M7UUFDaEMsMERBQTBEO1FBQzFELE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLGNBQWMsRUFBRSxJQUFJLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsQ0FBQztRQUNyRSxNQUFNLEVBQUUsR0FBVyxLQUFLLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxJQUFJLENBQUMsT0FBTyxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztRQUM1RCwwREFBMEQ7UUFDMUQsTUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3JFLE1BQU0sRUFBRSxHQUFXLEtBQUssQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLElBQUksQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO1FBQzVELGdDQUFnQztRQUNoQyxNQUFNLENBQUMsR0FBVyxNQUFNLENBQUMsS0FBSyxDQUM1QixNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUNqQyxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUNqQyxZQUFZLENBQUMsMkJBQTJCLENBQ3pDLENBQUM7UUFFRiwyQkFBMkI7UUFDM0I7WUFDRSxtQ0FBbUM7WUFDbkMsS0FBSyxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsSUFBSSxDQUFDLGFBQWEsRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7WUFDL0MsaUNBQWlDO1lBQ2pDLElBQUksQ0FBQyxLQUFLLEdBQUcsTUFBTSxDQUFDLE9BQU8sQ0FBQyxNQUFNLENBQUMsS0FBSyxDQUFDLENBQUMsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUN6RSw2QkFBNkI7WUFDN0IsSUFBSSxDQUFDLEtBQUssR0FBRyxNQUFNLENBQUMsT0FBTyxDQUFDLEVBQUUsRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7WUFFM0MsSUFBSSxDQUFDLE1BQU0sR0FBRyxFQUFFLEdBQUcsRUFBRSxHQUFHLEVBQUUsR0FBRyxJQUFJLENBQUMsS0FBSyxHQUFHLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxHQUFHLElBQUksQ0FBQyxLQUFLLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUVwRixJQUFJLElBQUksQ0FBQyxNQUFNLEdBQUcsQ0FBQyxFQUFFO2dCQUNuQixJQUFJLENBQUMsTUFBTSxHQUFHLENBQUMsR0FBRyxJQUFJLENBQUMsTUFBTSxDQUFDO2FBQy9CO1NBQ0Y7UUFFRCxvQkFBb0I7UUFDcEIsSUFBSSxDQUFDLFlBQVksR0FBRyxDQUFDLENBQUM7UUFDdEIsSUFBSSxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7UUFDaEIsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLENBQUM7UUFDakIsSUFBSSxJQUFJLENBQUMsYUFBYSxHQUFHLENBQUMsRUFBRTtZQUMxQixtQ0FBbUM7WUFDbkMsS0FBSyxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsSUFBSSxDQUFDLGFBQWEsRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7WUFDL0MsaUNBQWlDO1lBQ2pDLElBQUksQ0FBQyxLQUFLLEdBQUcsTUFBTSxDQUFDLE9BQU8sQ0FBQyxNQUFNLENBQUMsS0FBSyxDQUFDLENBQUMsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUN6RSw2QkFBNkI7WUFDN0IsSUFBSSxDQUFDLEtBQUssR0FBRyxNQUFNLENBQUMsT0FBTyxDQUFDLEVBQUUsRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7WUFFM0MsTUFBTSxPQUFPLEdBQVcsRUFBRSxHQUFHLEVBQUUsR0FBRyxFQUFFLEdBQUcsSUFBSSxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsR0FBRyxJQUFJLENBQUMsS0FBSyxHQUFHLElBQUksQ0FBQyxLQUFLLENBQUM7WUFFOUYsSUFBSSxPQUFPLEdBQUcsQ0FBQyxFQUFFO2dCQUNmLElBQUksQ0FBQyxZQUFZLEdBQUcsQ0FBQyxHQUFHLE9BQU8sQ0FBQztnQkFFaEMsTUFBTSxDQUFDLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO2dCQUU3QyxZQUFZO2dCQUNaLE1BQU0sS0FBSyxHQUFXLENBQUMsR0FBRyxLQUFLLEdBQUcsSUFBSSxDQUFDLGFBQWEsQ0FBQztnQkFFckQsc0JBQXNCO2dCQUN0QixNQUFNLElBQUksR0FBVyxDQUFDLEdBQUcsSUFBSSxDQUFDLFlBQVksR0FBRyxJQUFJLENBQUMsY0FBYyxHQUFHLEtBQUssQ0FBQztnQkFFekUsbUJBQW1CO2dCQUNuQixNQUFNLENBQUMsR0FBVyxJQUFJLENBQUMsWUFBWSxHQUFHLEtBQUssR0FBRyxLQUFLLENBQUM7Z0JBRXBELGlCQUFpQjtnQkFDakIsTUFBTSxDQUFDLEdBQVcsSUFBSSxDQUFDLElBQUksQ0FBQyxFQUFFLENBQUM7Z0JBQy9CLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxHQUFHLENBQUMsSUFBSSxHQUFHLENBQUMsR0FBRyxDQUFDLENBQUMsQ0FBQztnQkFDbEMsSUFBSSxJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsRUFBRTtvQkFDcEIsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQztpQkFDakM7Z0JBRUQsSUFBSSxDQUFDLE1BQU0sR0FBRyxDQUFDLEdBQUcsQ0FBQyxHQUFHLENBQUMsR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDO2dCQUV2QyxJQUFJLENBQUMsWUFBWSxHQUFHLE9BQU8sR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDO2dCQUMzQyxJQUFJLElBQUksQ0FBQyxZQUFZLEdBQUcsQ0FBQyxFQUFFO29CQUN6QixJQUFJLENBQUMsWUFBWSxHQUFHLENBQUMsR0FBRyxJQUFJLENBQUMsWUFBWSxDQUFDO2lCQUMzQzthQUNGO1NBQ0Y7YUFBTTtZQUNMLElBQUksQ0FBQyxlQUFlLEdBQUcsQ0FBQyxDQUFDO1NBQzFCO1FBRUQsbUJBQW1CO1FBQ25CLElBQUksSUFBSSxDQUFDLGFBQWEsRUFBRTtZQUN0QixJQUFJLENBQUMsV0FBVyxHQUFHLEVBQUUsR0FBRyxFQUFFLENBQUM7WUFDM0IsSUFBSSxJQUFJLENBQUMsV0FBVyxHQUFHLENBQUMsRUFBRTtnQkFDeEIsSUFBSSxDQUFDLFdBQVcsR0FBRyxDQUFDLEdBQUcsSUFBSSxDQUFDLFdBQVcsQ0FBQzthQUN6QztTQUNGO2FBQU07WUFDTCxJQUFJLENBQUMsV0FBVyxHQUFHLENBQUMsQ0FBQztZQUNyQixJQUFJLENBQUMsY0FBYyxHQUFHLENBQUMsQ0FBQztTQUN6QjtRQUVELElBQUksSUFBSSxDQUFDLElBQUksQ0FBQyxZQUFZLEVBQUU7WUFDMUIsa0NBQWtDO1lBQ2xDLElBQUksQ0FBQyxTQUFTLElBQUksSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUM7WUFDcEMsSUFBSSxDQUFDLGVBQWUsSUFBSSxJQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQztZQUMxQyxJQUFJLENBQUMsY0FBYyxJQUFJLElBQUksQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDO1lBRXpDLHdEQUF3RDtZQUN4RCxNQUFNLENBQUMsR0FBVyxNQUFNLENBQUMsS0FBSyxDQUM1QixNQUFNLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxTQUFTLEVBQUUsSUFBSSxDQUFDLElBQUksRUFBRSxNQUFNLENBQUMsSUFBSSxDQUFDLEVBQ3BELE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLGVBQWUsRUFBRSxJQUFJLENBQUMsSUFBSSxFQUFFLE1BQU0sQ0FBQyxJQUFJLENBQUMsRUFDMUQsWUFBWSxDQUFDLDJCQUEyQixDQUN6QyxDQUFDO1lBQ0YsNkVBQTZFO1lBQzdFLE1BQU0sRUFBRSxHQUNOLElBQUksQ0FBQyxTQUFTLEdBQUcsSUFBSSxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUMsZUFBZSxHQUFHLElBQUksQ0FBQyxLQUFLLEdBQUcsSUFBSSxDQUFDLGNBQWMsQ0FBQztZQUN4Riw2RUFBNkU7WUFDN0UsTUFBTSxFQUFFLEdBQ04sSUFBSSxDQUFDLFNBQVMsR0FBRyxJQUFJLENBQUMsS0FBSyxHQUFHLElBQUksQ0FBQyxlQUFlLEdBQUcsSUFBSSxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUMsY0FBYyxDQUFDO1lBRXhGLHdCQUF3QjtZQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsRUFBRSxDQUFDO1lBRXhCLHdCQUF3QjtZQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsRUFBRSxDQUFDO1NBQ3pCO2FBQU07WUFDTCxJQUFJLENBQUMsU0FBUyxHQUFHLENBQUMsQ0FBQztZQUNuQixJQUFJLENBQUMsZUFBZSxHQUFHLENBQUMsQ0FBQztZQUN6QixJQUFJLENBQUMsY0FBYyxHQUFHLENBQUMsQ0FBQztTQUN6QjtRQUVELHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO1FBQ3RDLHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO0lBQ3hDLENBQUM7SUFJRCx3QkFBd0IsQ0FBQyxJQUFrQjtRQUN6QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxFQUNoQyxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQztRQUMvQixNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsT0FBTyxFQUM3QixFQUFFLEdBQVcsSUFBSSxDQUFDLE9BQU8sQ0FBQztRQUU1QixNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEQsSUFBSSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ2xELE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNwRCxJQUFJLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFFbEQsMEJBQTBCO1FBQzFCO1lBQ0UsTUFBTSxJQUFJLEdBQ1IsTUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxNQUFNLENBQUMsSUFBSSxDQUFDLENBQUM7Z0JBQzFELElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRTtnQkFDZixJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztZQUNsQixNQUFNLE9BQU8sR0FDWCxDQUFDLElBQUksQ0FBQyxZQUFZLEdBQUcsQ0FBQyxJQUFJLEdBQUcsSUFBSSxDQUFDLE1BQU0sR0FBRyxJQUFJLENBQUMsT0FBTyxHQUFHLElBQUksQ0FBQyxlQUFlLENBQUMsQ0FBQztZQUNsRixJQUFJLENBQUMsZUFBZSxJQUFJLE9BQU8sQ0FBQztZQUVoQyw2QkFBNkI7WUFDN0IsTUFBTSxDQUFDLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksRUFBRSxZQUFZLENBQUMsNEJBQTRCLENBQUMsQ0FBQztZQUM5RixNQUFNLEVBQUUsR0FBVyxPQUFPLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUN4QyxNQUFNLEVBQUUsR0FBVyxPQUFPLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUV4QyxnQkFBZ0I7WUFDaEIsRUFBRSxDQUFDLFVBQVUsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDckIsRUFBRSxJQUFJLEVBQUUsR0FBRyxFQUFFLENBQUM7WUFFZCxnQkFBZ0I7WUFDaEIsRUFBRSxDQUFDLFVBQVUsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDckIsRUFBRSxJQUFJLEVBQUUsR0FBRyxFQUFFLENBQUM7U0FDZjtRQUVELG9DQUFvQztRQUNwQztZQUNFLE1BQU0sSUFBSSxHQUFXLEVBQUUsR0FBRyxFQUFFLEdBQUcsSUFBSSxDQUFDLFlBQVksQ0FBQztZQUNqRCxJQUFJLE9BQU8sR0FBVyxDQUFDLElBQUksQ0FBQyxXQUFXLEdBQUcsSUFBSSxDQUFDO1lBRS9DLE1BQU0sVUFBVSxHQUFXLElBQUksQ0FBQyxjQUFjLENBQUM7WUFDL0MsTUFBTSxVQUFVLEdBQVcsSUFBSSxDQUFDLElBQUksQ0FBQyxFQUFFLEdBQUcsSUFBSSxDQUFDLGdCQUFnQixDQUFDO1lBQ2hFLElBQUksQ0FBQyxjQUFjLEdBQUcsT0FBTyxDQUFDLElBQUksQ0FBQyxjQUFjLEdBQUcsT0FBTyxFQUFFLENBQUMsVUFBVSxFQUFFLFVBQVUsQ0FBQyxDQUFDO1lBQ3RGLE9BQU8sR0FBRyxJQUFJLENBQUMsY0FBYyxHQUFHLFVBQVUsQ0FBQztZQUUzQyxFQUFFLElBQUksRUFBRSxHQUFHLE9BQU8sQ0FBQztZQUNuQixFQUFFLElBQUksRUFBRSxHQUFHLE9BQU8sQ0FBQztTQUNwQjtRQUVELGlDQUFpQztRQUNqQztZQUNFLE1BQU0sSUFBSSxHQUNSLE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLElBQUksRUFBRSxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxDQUFDO2dCQUMxRCxJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUU7Z0JBQ2YsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7WUFDbEIsTUFBTSxPQUFPLEdBQVcsQ0FBQyxJQUFJLENBQUMsTUFBTSxHQUFHLElBQUksQ0FBQztZQUM1QyxJQUFJLENBQUMsU0FBUyxJQUFJLE9BQU8sQ0FBQztZQUUxQiw2QkFBNkI7WUFDN0IsTUFBTSxDQUFDLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksRUFBRSxZQUFZLENBQUMsNEJBQTRCLENBQUMsQ0FBQztZQUM5RixNQUFNLEVBQUUsR0FBVyxPQUFPLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUN4QyxNQUFNLEVBQUUsR0FBVyxPQUFPLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUV4QyxnQkFBZ0I7WUFDaEIsRUFBRSxDQUFDLFVBQVUsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDckIsRUFBRSxJQUFJLEVBQUUsR0FBRyxFQUFFLENBQUM7WUFFZCxnQkFBZ0I7WUFDaEIsRUFBRSxDQUFDLFVBQVUsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDckIsRUFBRSxJQUFJLEVBQUUsR0FBRyxFQUFFLENBQUM7U0FDZjtRQUVELHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO1FBQ3RDLHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO0lBQ3hDLENBQUM7SUFLRCx3QkFBd0IsQ0FBQyxJQUFrQjtRQUN6QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDbkQsSUFBSSxFQUFFLEdBQVcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ2pELE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNuRCxJQUFJLEVBQUUsR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFFakQsTUFBTSxFQUFFLEdBQVUsSUFBSSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsRUFBRSxDQUFDLEVBQ3RDLEVBQUUsR0FBVSxJQUFJLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUVyQywwREFBMEQ7UUFDMUQsTUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3JFLE1BQU0sRUFBRSxHQUFXLEtBQUssQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLElBQUksQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO1FBQzVELDBEQUEwRDtRQUMxRCxNQUFNLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLGNBQWMsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUM7UUFDckUsTUFBTSxFQUFFLEdBQVcsS0FBSyxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsSUFBSSxDQUFDLE9BQU8sRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7UUFDNUQsa0NBQWtDO1FBQ2xDLE1BQU0sQ0FBQyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQzVCLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxNQUFNLENBQUMsSUFBSSxDQUFDLEVBQ2pDLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxNQUFNLENBQUMsSUFBSSxDQUFDLEVBQ2pDLFlBQVksQ0FBQyw0QkFBNEIsQ0FDMUMsQ0FBQztRQUVGLHdDQUF3QztRQUN4QyxNQUFNLEVBQUUsR0FBVyxLQUFLLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxJQUFJLENBQUMsYUFBYSxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztRQUVsRSxxQ0FBcUM7UUFDckMsTUFBTSxHQUFHLEdBQUcsTUFBTSxDQUFDLE9BQU8sQ0FBQyxNQUFNLENBQUMsS0FBSyxDQUFDLENBQUMsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDO1FBQ2pFLGlDQUFpQztRQUNqQyxNQUFNLEdBQUcsR0FBRyxNQUFNLENBQUMsT0FBTyxDQUFDLEVBQUUsRUFBRSxFQUFFLENBQUMsQ0FBQztRQUVuQyw0QkFBNEI7UUFDNUIsTUFBTSxDQUFDLEdBQVcsTUFBTSxDQUFDLEtBQUssQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO1FBRTdDLE1BQU0sQ0FBQyxHQUNMLElBQUksQ0FBQyxVQUFVO1lBQ2YsSUFBSSxDQUFDLFVBQVU7WUFDZixJQUFJLENBQUMsT0FBTyxHQUFHLElBQUksQ0FBQyxLQUFLLEdBQUcsSUFBSSxDQUFDLEtBQUs7WUFDdEMsSUFBSSxDQUFDLE9BQU8sR0FBRyxJQUFJLENBQUMsS0FBSyxHQUFHLElBQUksQ0FBQyxLQUFLLENBQUM7UUFFekMsSUFBSSxPQUFlLENBQUM7UUFDcEIsSUFBSSxDQUFDLEtBQUssQ0FBQyxFQUFFO1lBQ1gsT0FBTyxHQUFHLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQztTQUNsQjthQUFNO1lBQ0wsT0FBTyxHQUFHLENBQUMsQ0FBQztTQUNiO1FBRUQsMkJBQTJCO1FBQzNCLE1BQU0sQ0FBQyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsT0FBTyxFQUFFLEVBQUUsRUFBRSxZQUFZLENBQUMsNEJBQTRCLENBQUMsQ0FBQztRQUN2RixNQUFNLEVBQUUsR0FBVyxPQUFPLEdBQUcsR0FBRyxDQUFDO1FBQ2pDLE1BQU0sRUFBRSxHQUFXLE9BQU8sR0FBRyxHQUFHLENBQUM7UUFFakMsd0JBQXdCO1FBQ3hCLEVBQUUsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNsQyxFQUFFLElBQUksSUFBSSxDQUFDLE9BQU8sR0FBRyxFQUFFLENBQUM7UUFDeEIsd0JBQXdCO1FBQ3hCLEVBQUUsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNsQyxFQUFFLElBQUksSUFBSSxDQUFDLE9BQU8sR0FBRyxFQUFFLENBQUM7UUFFeEIsd0NBQXdDO1FBQ3hDLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDckMsd0NBQXdDO1FBQ3hDLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFFckMsT0FBTyxLQUFLLENBQUMsQ0FBQyxDQUFDLElBQUksYUFBYSxDQUFDO0lBQ25DLENBQUM7SUFFRCxhQUFhLENBQUMsR0FBb0I7UUFDaEMsQ0FBQyxDQUFDLFFBQVEsSUFBSSxRQUFRLENBQUMsS0FBSyxDQUFDLENBQUMsQ0FBQyxPQUFPO1FBQ3RDLE9BQU8sR0FBRyxDQUFDO0lBQ2IsQ0FBQztJQUVELFVBQVUsQ0FBZSxHQUFNO1FBQzdCLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQyxhQUFhLENBQUMsSUFBSSxDQUFDLGNBQWMsRUFBRSxHQUFHLENBQUMsQ0FBQztJQUM5RCxDQUFDO0lBRUQsVUFBVSxDQUFlLEdBQU07UUFDN0IsT0FBTyxJQUFJLENBQUMsT0FBTyxDQUFDLGFBQWEsQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLEdBQUcsQ0FBQyxDQUFDO0lBQzlELENBQUM7SUFFRCxnQkFBZ0IsQ0FBZSxNQUFjLEVBQUUsR0FBTTtRQUNuRCwrREFBK0Q7UUFDL0QsR0FBRyxDQUFDLENBQUMsR0FBRyxNQUFNLEdBQUcsQ0FBQyxJQUFJLENBQUMsU0FBUyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxlQUFlLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNyRixHQUFHLENBQUMsQ0FBQyxHQUFHLE1BQU0sR0FBRyxDQUFDLElBQUksQ0FBQyxTQUFTLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLGVBQWUsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ3JGLE9BQU8sR0FBRyxDQUFDO0lBQ2IsQ0FBQztJQUVELGlCQUFpQixDQUFDLE1BQWM7UUFDOUIsT0FBTyxNQUFNLEdBQUcsSUFBSSxDQUFDLGNBQWMsQ0FBQztJQUN0QyxDQUFDO0lBRUQsZUFBZTtRQUNiLE9BQU8sSUFBSSxDQUFDLGNBQWMsQ0FBQztJQUM3QixDQUFDO0lBRUQsZUFBZTtRQUNiLE9BQU8sSUFBSSxDQUFDLGNBQWMsQ0FBQztJQUM3QixDQUFDO0lBRUQsYUFBYTtRQUNYLE9BQU8sSUFBSSxDQUFDLGFBQWEsQ0FBQztJQUM1QixDQUFDO0lBRUQsbUJBQW1CO1FBQ2pCLE9BQU8sSUFBSSxDQUFDLDRCQUE0QixFQUFFLENBQUM7SUFDN0MsQ0FBQztJQUVELG1CQUFtQjtRQUNqQixPQUFPLElBQUksQ0FBQyxzQkFBc0IsRUFBRSxDQUFDO0lBQ3ZDLENBQUM7SUFFRCxhQUFhO1FBQ1gsT0FBTyxJQUFJLENBQUMscUJBQXFCLEVBQUUsQ0FBQztJQUN0QyxDQUFDO0lBRUQsb0JBQW9CO1FBQ2xCLE9BQU8sSUFBSSxDQUFDLHFCQUFxQixFQUFFLENBQUM7SUFDdEMsQ0FBQztJQUVELDRCQUE0QjtRQUMxQixNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsT0FBTyxDQUFDO1FBQ2hDLE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxPQUFPLENBQUM7UUFFaEMsTUFBTSxFQUFFLEdBQVcsRUFBRSxDQUFDLGFBQWEsQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLElBQUksTUFBTSxFQUFFLENBQUMsQ0FBQztRQUN2RSxNQUFNLEVBQUUsR0FBVyxFQUFFLENBQUMsYUFBYSxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxNQUFNLEVBQUUsQ0FBQyxDQUFDO1FBQ3ZFLE1BQU0sQ0FBQyxHQUFXLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxJQUFJLE1BQU0sRUFBRSxDQUFDLENBQUM7UUFDckQsTUFBTSxJQUFJLEdBQVcsRUFBRSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsYUFBYSxFQUFFLElBQUksTUFBTSxFQUFFLENBQUMsQ0FBQztRQUV6RSxNQUFNLFdBQVcsR0FBVyxNQUFNLENBQUMsS0FBSyxDQUFDLENBQUMsRUFBRSxJQUFJLENBQUMsQ0FBQztRQUNsRCxPQUFPLFdBQVcsQ0FBQztJQUNyQixDQUFDO0lBRUQsc0JBQXNCO1FBQ3BCLE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxPQUFPLENBQUM7UUFDaEMsTUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLE9BQU8sQ0FBQztRQUVoQywyRUFBMkU7UUFDM0UsTUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLEVBQUUsQ0FBQyxPQUFPLENBQUMsV0FBVyxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsQ0FBQztRQUN4RSxNQUFNLEVBQUUsR0FBRyxLQUFLLENBQUMsS0FBSyxDQUFDLEVBQUUsQ0FBQyxJQUFJLENBQUMsQ0FBQyxFQUFFLElBQUksQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO1FBQzNELDJFQUEyRTtRQUMzRSxNQUFNLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsRUFBRSxDQUFDLE9BQU8sQ0FBQyxXQUFXLEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3hFLE1BQU0sRUFBRSxHQUFHLEtBQUssQ0FBQyxLQUFLLENBQUMsRUFBRSxDQUFDLElBQUksQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLE9BQU8sRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7UUFDM0Qsa0NBQWtDO1FBQ2xDLE1BQU0sRUFBRSxHQUFHLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxDQUFDLE9BQU8sQ0FBQyxDQUFDLEVBQUUsRUFBRSxFQUFFLE1BQU0sQ0FBQyxJQUFJLENBQUMsQ0FBQyxDQUFDLGVBQWU7UUFDdkUsa0NBQWtDO1FBQ2xDLE1BQU0sRUFBRSxHQUFHLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxDQUFDLE9BQU8sQ0FBQyxDQUFDLEVBQUUsRUFBRSxFQUFFLE1BQU0sQ0FBQyxJQUFJLENBQUMsQ0FBQyxDQUFDLGVBQWU7UUFDdkUsc0JBQXNCO1FBQ3RCLE1BQU0sQ0FBQyxHQUFHLE1BQU0sQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxNQUFNLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQyxjQUFjO1FBQzNELGlEQUFpRDtRQUNqRCxNQUFNLElBQUksR0FBRyxFQUFFLENBQUMsY0FBYyxDQUFDLElBQUksQ0FBQyxhQUFhLEVBQUUsSUFBSSxNQUFNLEVBQUUsQ0FBQyxDQUFDO1FBRWpFLE1BQU0sRUFBRSxHQUFHLEVBQUUsQ0FBQyxnQkFBZ0IsQ0FBQztRQUMvQixNQUFNLEVBQUUsR0FBRyxFQUFFLENBQUMsZ0JBQWdCLENBQUM7UUFDL0IsTUFBTSxFQUFFLEdBQUcsRUFBRSxDQUFDLGlCQUFpQixDQUFDO1FBQ2hDLE1BQU0sRUFBRSxHQUFHLEVBQUUsQ0FBQyxpQkFBaUIsQ0FBQztRQUVoQywwR0FBMEc7UUFDMUcsTUFBTSxLQUFLLEdBQ1QsTUFBTSxDQUFDLEtBQUssQ0FBQyxDQUFDLEVBQUUsTUFBTSxDQUFDLE9BQU8sQ0FBQyxFQUFFLEVBQUUsSUFBSSxFQUFFLE1BQU0sQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUN0RCxNQUFNLENBQUMsS0FBSyxDQUNWLElBQUksRUFDSixNQUFNLENBQUMsS0FBSyxDQUNWLE1BQU0sQ0FBQyxXQUFXLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUMzQyxNQUFNLENBQUMsV0FBVyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsRUFBRSxFQUFFLE1BQU0sQ0FBQyxJQUFJLENBQUMsRUFDM0MsTUFBTSxDQUFDLElBQUksQ0FDWixDQUNGLENBQUM7UUFDSixPQUFPLEtBQUssQ0FBQztJQUNmLENBQUM7SUFFRCxxQkFBcUI7UUFDbkIsNkJBQTZCO1FBQzdCLDZCQUE2QjtRQUM3QixrREFBa0Q7UUFDbEQsT0FBTyxJQUFJLENBQUMsT0FBTyxDQUFDLE9BQU8sQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxPQUFPLENBQUMsQ0FBQyxDQUFDO0lBQ3pELENBQUM7SUFFRCxxQkFBcUI7UUFDbkIsTUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxpQkFBaUIsQ0FBQztRQUNsRCxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsT0FBTyxDQUFDLGlCQUFpQixDQUFDO1FBQ2xELE9BQU8sRUFBRSxHQUFHLEVBQUUsQ0FBQztJQUNqQixDQUFDO0lBRUQsY0FBYztRQUNaLE9BQU8sSUFBSSxDQUFDLGFBQWEsQ0FBQztJQUM1QixDQUFDO0lBRUQsV0FBVyxDQUFDLElBQWE7UUFDdkIsSUFBSSxJQUFJLEtBQUssSUFBSSxDQUFDLGFBQWEsRUFBRTtZQUMvQixJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUM1QixJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUM1QixJQUFJLENBQUMsYUFBYSxHQUFHLElBQUksQ0FBQztTQUMzQjtJQUNILENBQUM7SUFFRCxhQUFhLENBQUMsS0FBYTtRQUN6QixJQUFJLEtBQUssS0FBSyxJQUFJLENBQUMsWUFBWSxFQUFFO1lBQy9CLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQzVCLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQzVCLElBQUksQ0FBQyxZQUFZLEdBQUcsS0FBSyxDQUFDO1NBQzNCO0lBQ0gsQ0FBQztJQUVELGlCQUFpQixDQUFDLEtBQWE7UUFDN0IsSUFBSSxLQUFLLEtBQUssSUFBSSxDQUFDLGdCQUFnQixFQUFFO1lBQ25DLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQzVCLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQzVCLElBQUksQ0FBQyxnQkFBZ0IsR0FBRyxLQUFLLENBQUM7U0FDL0I7SUFDSCxDQUFDO0lBRUQsY0FBYyxDQUFDLE1BQWM7UUFDM0IsT0FBTyxNQUFNLEdBQUcsSUFBSSxDQUFDLGNBQWMsQ0FBQztJQUN0QyxDQUFDOztBQW5jYyx3Q0FBMkIsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQzNDLHdDQUEyQixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUEySjNDLHlDQUE0QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFnRjVDLHlDQUE0QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDNUMseUNBQTRCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDExIEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJfbGluZWFyU2xvcCwgYjJfcGksIGIyQXNzZXJ0LCBiMk1heWJlIH0gZnJvbSAnLi4vLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMkFicywgYjJDbGFtcCwgYjJSb3QsIGIyVmVjMiwgWFkgfSBmcm9tICcuLi8uLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJJSm9pbnREZWYsIGIySm9pbnQsIGIySm9pbnREZWYsIGIySm9pbnRUeXBlIH0gZnJvbSAnLi9iMkpvaW50JztcclxuaW1wb3J0IHsgYjJTb2x2ZXJEYXRhIH0gZnJvbSAnLi4vYjJUaW1lU3RlcCc7XHJcbmltcG9ydCB7IGIyQm9keSB9IGZyb20gJy4uL2IyQm9keSc7XHJcblxyXG5leHBvcnQgaW50ZXJmYWNlIGIySVdoZWVsSm9pbnREZWYgZXh0ZW5kcyBiMklKb2ludERlZiB7XHJcbiAgbG9jYWxBbmNob3JBPzogWFk7XHJcblxyXG4gIGxvY2FsQW5jaG9yQj86IFhZO1xyXG5cclxuICBsb2NhbEF4aXNBPzogWFk7XHJcblxyXG4gIGVuYWJsZU1vdG9yPzogYm9vbGVhbjtcclxuXHJcbiAgbWF4TW90b3JUb3JxdWU/OiBudW1iZXI7XHJcblxyXG4gIG1vdG9yU3BlZWQ/OiBudW1iZXI7XHJcblxyXG4gIGZyZXF1ZW5jeUh6PzogbnVtYmVyO1xyXG5cclxuICBkYW1waW5nUmF0aW8/OiBudW1iZXI7XHJcbn1cclxuXHJcbi8vLyBXaGVlbCBqb2ludCBkZWZpbml0aW9uLiBUaGlzIHJlcXVpcmVzIGRlZmluaW5nIGEgbGluZSBvZlxyXG4vLy8gbW90aW9uIHVzaW5nIGFuIGF4aXMgYW5kIGFuIGFuY2hvciBwb2ludC4gVGhlIGRlZmluaXRpb24gdXNlcyBsb2NhbFxyXG4vLy8gYW5jaG9yIHBvaW50cyBhbmQgYSBsb2NhbCBheGlzIHNvIHRoYXQgdGhlIGluaXRpYWwgY29uZmlndXJhdGlvblxyXG4vLy8gY2FuIHZpb2xhdGUgdGhlIGNvbnN0cmFpbnQgc2xpZ2h0bHkuIFRoZSBqb2ludCB0cmFuc2xhdGlvbiBpcyB6ZXJvXHJcbi8vLyB3aGVuIHRoZSBsb2NhbCBhbmNob3IgcG9pbnRzIGNvaW5jaWRlIGluIHdvcmxkIHNwYWNlLiBVc2luZyBsb2NhbFxyXG4vLy8gYW5jaG9ycyBhbmQgYSBsb2NhbCBheGlzIGhlbHBzIHdoZW4gc2F2aW5nIGFuZCBsb2FkaW5nIGEgZ2FtZS5cclxuZXhwb3J0IGNsYXNzIGIyV2hlZWxKb2ludERlZiBleHRlbmRzIGIySm9pbnREZWYgaW1wbGVtZW50cyBiMklXaGVlbEpvaW50RGVmIHtcclxuICByZWFkb25seSBsb2NhbEFuY2hvckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoMCwgMCk7XHJcblxyXG4gIHJlYWRvbmx5IGxvY2FsQW5jaG9yQjogYjJWZWMyID0gbmV3IGIyVmVjMigwLCAwKTtcclxuXHJcbiAgcmVhZG9ubHkgbG9jYWxBeGlzQTogYjJWZWMyID0gbmV3IGIyVmVjMigxLCAwKTtcclxuXHJcbiAgZW5hYmxlTW90b3IgPSBmYWxzZTtcclxuXHJcbiAgbWF4TW90b3JUb3JxdWUgPSAwO1xyXG5cclxuICBtb3RvclNwZWVkID0gMDtcclxuXHJcbiAgZnJlcXVlbmN5SHogPSAyO1xyXG5cclxuICBkYW1waW5nUmF0aW8gPSAwLjc7XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgc3VwZXIoYjJKb2ludFR5cGUuZV93aGVlbEpvaW50KTtcclxuICB9XHJcblxyXG4gIEluaXRpYWxpemUoYkE6IGIyQm9keSwgYkI6IGIyQm9keSwgYW5jaG9yOiBiMlZlYzIsIGF4aXM6IGIyVmVjMik6IHZvaWQge1xyXG4gICAgdGhpcy5ib2R5QSA9IGJBO1xyXG4gICAgdGhpcy5ib2R5QiA9IGJCO1xyXG4gICAgdGhpcy5ib2R5QS5HZXRMb2NhbFBvaW50KGFuY2hvciwgdGhpcy5sb2NhbEFuY2hvckEpO1xyXG4gICAgdGhpcy5ib2R5Qi5HZXRMb2NhbFBvaW50KGFuY2hvciwgdGhpcy5sb2NhbEFuY2hvckIpO1xyXG4gICAgdGhpcy5ib2R5QS5HZXRMb2NhbFZlY3RvcihheGlzLCB0aGlzLmxvY2FsQXhpc0EpO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyV2hlZWxKb2ludCBleHRlbmRzIGIySm9pbnQge1xyXG4gIG1fZnJlcXVlbmN5SHogPSAwO1xyXG4gIG1fZGFtcGluZ1JhdGlvID0gMDtcclxuXHJcbiAgLy8gU29sdmVyIHNoYXJlZFxyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbEFuY2hvckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsWEF4aXNBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbFlBeGlzQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBtX2ltcHVsc2UgPSAwO1xyXG4gIG1fbW90b3JJbXB1bHNlID0gMDtcclxuICBtX3NwcmluZ0ltcHVsc2UgPSAwO1xyXG5cclxuICBtX21heE1vdG9yVG9ycXVlID0gMDtcclxuICBtX21vdG9yU3BlZWQgPSAwO1xyXG4gIG1fZW5hYmxlTW90b3IgPSBmYWxzZTtcclxuXHJcbiAgLy8gU29sdmVyIHRlbXBcclxuICBtX2luZGV4QSA9IDA7XHJcbiAgbV9pbmRleEIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxDZW50ZXJBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbENlbnRlckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBtX2ludk1hc3NBID0gMDtcclxuICBtX2ludk1hc3NCID0gMDtcclxuICBtX2ludklBID0gMDtcclxuICBtX2ludklCID0gMDtcclxuXHJcbiAgcmVhZG9ubHkgbV9heDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fYXk6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBtX3NBeCA9IDA7XHJcbiAgbV9zQnggPSAwO1xyXG4gIG1fc0F5ID0gMDtcclxuICBtX3NCeSA9IDA7XHJcblxyXG4gIG1fbWFzcyA9IDA7XHJcbiAgbV9tb3Rvck1hc3MgPSAwO1xyXG4gIG1fc3ByaW5nTWFzcyA9IDA7XHJcblxyXG4gIG1fYmlhcyA9IDA7XHJcbiAgbV9nYW1tYSA9IDA7XHJcblxyXG4gIHJlYWRvbmx5IG1fcUE6IGIyUm90ID0gbmV3IGIyUm90KCk7XHJcbiAgcmVhZG9ubHkgbV9xQjogYjJSb3QgPSBuZXcgYjJSb3QoKTtcclxuICByZWFkb25seSBtX2xhbGNBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sYWxjQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIGNvbnN0cnVjdG9yKGRlZjogYjJJV2hlZWxKb2ludERlZikge1xyXG4gICAgc3VwZXIoZGVmKTtcclxuXHJcbiAgICB0aGlzLm1fZnJlcXVlbmN5SHogPSBiMk1heWJlKGRlZi5mcmVxdWVuY3lIeiwgMik7XHJcbiAgICB0aGlzLm1fZGFtcGluZ1JhdGlvID0gYjJNYXliZShkZWYuZGFtcGluZ1JhdGlvLCAwLjcpO1xyXG5cclxuICAgIHRoaXMubV9sb2NhbEFuY2hvckEuQ29weShiMk1heWJlKGRlZi5sb2NhbEFuY2hvckEsIGIyVmVjMi5aRVJPKSk7XHJcbiAgICB0aGlzLm1fbG9jYWxBbmNob3JCLkNvcHkoYjJNYXliZShkZWYubG9jYWxBbmNob3JCLCBiMlZlYzIuWkVSTykpO1xyXG4gICAgdGhpcy5tX2xvY2FsWEF4aXNBLkNvcHkoYjJNYXliZShkZWYubG9jYWxBeGlzQSwgYjJWZWMyLlVOSVRYKSk7XHJcbiAgICBiMlZlYzIuQ3Jvc3NPbmVWKHRoaXMubV9sb2NhbFhBeGlzQSwgdGhpcy5tX2xvY2FsWUF4aXNBKTtcclxuXHJcbiAgICB0aGlzLm1fbWF4TW90b3JUb3JxdWUgPSBiMk1heWJlKGRlZi5tYXhNb3RvclRvcnF1ZSwgMCk7XHJcbiAgICB0aGlzLm1fbW90b3JTcGVlZCA9IGIyTWF5YmUoZGVmLm1vdG9yU3BlZWQsIDApO1xyXG4gICAgdGhpcy5tX2VuYWJsZU1vdG9yID0gYjJNYXliZShkZWYuZW5hYmxlTW90b3IsIGZhbHNlKTtcclxuXHJcbiAgICB0aGlzLm1fYXguU2V0WmVybygpO1xyXG4gICAgdGhpcy5tX2F5LlNldFplcm8oKTtcclxuICB9XHJcblxyXG4gIEdldE1vdG9yU3BlZWQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fbW90b3JTcGVlZDtcclxuICB9XHJcblxyXG4gIEdldE1heE1vdG9yVG9ycXVlKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX21heE1vdG9yVG9ycXVlO1xyXG4gIH1cclxuXHJcbiAgU2V0U3ByaW5nRnJlcXVlbmN5SHooaHo6IG51bWJlcik6IHZvaWQge1xyXG4gICAgdGhpcy5tX2ZyZXF1ZW5jeUh6ID0gaHo7XHJcbiAgfVxyXG5cclxuICBHZXRTcHJpbmdGcmVxdWVuY3lIeigpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9mcmVxdWVuY3lIejtcclxuICB9XHJcblxyXG4gIFNldFNwcmluZ0RhbXBpbmdSYXRpbyhyYXRpbzogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZGFtcGluZ1JhdGlvID0gcmF0aW87XHJcbiAgfVxyXG5cclxuICBHZXRTcHJpbmdEYW1waW5nUmF0aW8oKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fZGFtcGluZ1JhdGlvO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19kID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICB0aGlzLm1faW5kZXhBID0gdGhpcy5tX2JvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhCID0gdGhpcy5tX2JvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1fbG9jYWxDZW50ZXJBLkNvcHkodGhpcy5tX2JvZHlBLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX2xvY2FsQ2VudGVyQi5Db3B5KHRoaXMubV9ib2R5Qi5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMubV9pbnZNYXNzQSA9IHRoaXMubV9ib2R5QS5tX2ludk1hc3M7XHJcbiAgICB0aGlzLm1faW52TWFzc0IgPSB0aGlzLm1fYm9keUIubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX2ludklBID0gdGhpcy5tX2JvZHlBLm1faW52STtcclxuICAgIHRoaXMubV9pbnZJQiA9IHRoaXMubV9ib2R5Qi5tX2ludkk7XHJcblxyXG4gICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgbUI6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQjtcclxuICAgIGNvbnN0IGlBOiBudW1iZXIgPSB0aGlzLm1faW52SUEsXHJcbiAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgY29uc3QgY0E6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmM7XHJcbiAgICBjb25zdCBhQTogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYTtcclxuICAgIGNvbnN0IHZBOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udjtcclxuICAgIGxldCB3QTogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnc7XHJcblxyXG4gICAgY29uc3QgY0I6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBjb25zdCBhQjogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYTtcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgY29uc3QgcUE6IGIyUm90ID0gdGhpcy5tX3FBLlNldEFuZ2xlKGFBKSxcclxuICAgICAgcUI6IGIyUm90ID0gdGhpcy5tX3FCLlNldEFuZ2xlKGFCKTtcclxuXHJcbiAgICAvLyBDb21wdXRlIHRoZSBlZmZlY3RpdmUgbWFzc2VzLlxyXG4gICAgLy8gYjJWZWMyIHJBID0gYjJNdWwocUEsIG1fbG9jYWxBbmNob3JBIC0gbV9sb2NhbENlbnRlckEpO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckEsIHRoaXMubV9sb2NhbENlbnRlckEsIHRoaXMubV9sYWxjQSk7XHJcbiAgICBjb25zdCByQTogYjJWZWMyID0gYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sYWxjQSwgdGhpcy5tX3JBKTtcclxuICAgIC8vIGIyVmVjMiByQiA9IGIyTXVsKHFCLCBtX2xvY2FsQW5jaG9yQiAtIG1fbG9jYWxDZW50ZXJCKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgY29uc3QgckI6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFCLCB0aGlzLm1fbGFsY0IsIHRoaXMubV9yQik7XHJcbiAgICAvLyBiMlZlYzIgZCA9IGNCICsgckIgLSBjQSAtIHJBO1xyXG4gICAgY29uc3QgZDogYjJWZWMyID0gYjJWZWMyLlN1YlZWKFxyXG4gICAgICBiMlZlYzIuQWRkVlYoY0IsIHJCLCBiMlZlYzIuc190MCksXHJcbiAgICAgIGIyVmVjMi5BZGRWVihjQSwgckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgYjJXaGVlbEpvaW50LkluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZCxcclxuICAgICk7XHJcblxyXG4gICAgLy8gUG9pbnQgdG8gbGluZSBjb25zdHJhaW50XHJcbiAgICB7XHJcbiAgICAgIC8vIG1fYXkgPSBiMk11bChxQSwgbV9sb2NhbFlBeGlzQSk7XHJcbiAgICAgIGIyUm90Lk11bFJWKHFBLCB0aGlzLm1fbG9jYWxZQXhpc0EsIHRoaXMubV9heSk7XHJcbiAgICAgIC8vIG1fc0F5ID0gYjJDcm9zcyhkICsgckEsIG1fYXkpO1xyXG4gICAgICB0aGlzLm1fc0F5ID0gYjJWZWMyLkNyb3NzVlYoYjJWZWMyLkFkZFZWKGQsIHJBLCBiMlZlYzIuc190MCksIHRoaXMubV9heSk7XHJcbiAgICAgIC8vIG1fc0J5ID0gYjJDcm9zcyhyQiwgbV9heSk7XHJcbiAgICAgIHRoaXMubV9zQnkgPSBiMlZlYzIuQ3Jvc3NWVihyQiwgdGhpcy5tX2F5KTtcclxuXHJcbiAgICAgIHRoaXMubV9tYXNzID0gbUEgKyBtQiArIGlBICogdGhpcy5tX3NBeSAqIHRoaXMubV9zQXkgKyBpQiAqIHRoaXMubV9zQnkgKiB0aGlzLm1fc0J5O1xyXG5cclxuICAgICAgaWYgKHRoaXMubV9tYXNzID4gMCkge1xyXG4gICAgICAgIHRoaXMubV9tYXNzID0gMSAvIHRoaXMubV9tYXNzO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gU3ByaW5nIGNvbnN0cmFpbnRcclxuICAgIHRoaXMubV9zcHJpbmdNYXNzID0gMDtcclxuICAgIHRoaXMubV9iaWFzID0gMDtcclxuICAgIHRoaXMubV9nYW1tYSA9IDA7XHJcbiAgICBpZiAodGhpcy5tX2ZyZXF1ZW5jeUh6ID4gMCkge1xyXG4gICAgICAvLyBtX2F4ID0gYjJNdWwocUEsIG1fbG9jYWxYQXhpc0EpO1xyXG4gICAgICBiMlJvdC5NdWxSVihxQSwgdGhpcy5tX2xvY2FsWEF4aXNBLCB0aGlzLm1fYXgpO1xyXG4gICAgICAvLyBtX3NBeCA9IGIyQ3Jvc3MoZCArIHJBLCBtX2F4KTtcclxuICAgICAgdGhpcy5tX3NBeCA9IGIyVmVjMi5Dcm9zc1ZWKGIyVmVjMi5BZGRWVihkLCByQSwgYjJWZWMyLnNfdDApLCB0aGlzLm1fYXgpO1xyXG4gICAgICAvLyBtX3NCeCA9IGIyQ3Jvc3MockIsIG1fYXgpO1xyXG4gICAgICB0aGlzLm1fc0J4ID0gYjJWZWMyLkNyb3NzVlYockIsIHRoaXMubV9heCk7XHJcblxyXG4gICAgICBjb25zdCBpbnZNYXNzOiBudW1iZXIgPSBtQSArIG1CICsgaUEgKiB0aGlzLm1fc0F4ICogdGhpcy5tX3NBeCArIGlCICogdGhpcy5tX3NCeCAqIHRoaXMubV9zQng7XHJcblxyXG4gICAgICBpZiAoaW52TWFzcyA+IDApIHtcclxuICAgICAgICB0aGlzLm1fc3ByaW5nTWFzcyA9IDEgLyBpbnZNYXNzO1xyXG5cclxuICAgICAgICBjb25zdCBDOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoZCwgdGhpcy5tX2F4KTtcclxuXHJcbiAgICAgICAgLy8gRnJlcXVlbmN5XHJcbiAgICAgICAgY29uc3Qgb21lZ2E6IG51bWJlciA9IDIgKiBiMl9waSAqIHRoaXMubV9mcmVxdWVuY3lIejtcclxuXHJcbiAgICAgICAgLy8gRGFtcGluZyBjb2VmZmljaWVudFxyXG4gICAgICAgIGNvbnN0IGRhbXA6IG51bWJlciA9IDIgKiB0aGlzLm1fc3ByaW5nTWFzcyAqIHRoaXMubV9kYW1waW5nUmF0aW8gKiBvbWVnYTtcclxuXHJcbiAgICAgICAgLy8gU3ByaW5nIHN0aWZmbmVzc1xyXG4gICAgICAgIGNvbnN0IGs6IG51bWJlciA9IHRoaXMubV9zcHJpbmdNYXNzICogb21lZ2EgKiBvbWVnYTtcclxuXHJcbiAgICAgICAgLy8gbWFnaWMgZm9ybXVsYXNcclxuICAgICAgICBjb25zdCBoOiBudW1iZXIgPSBkYXRhLnN0ZXAuZHQ7XHJcbiAgICAgICAgdGhpcy5tX2dhbW1hID0gaCAqIChkYW1wICsgaCAqIGspO1xyXG4gICAgICAgIGlmICh0aGlzLm1fZ2FtbWEgPiAwKSB7XHJcbiAgICAgICAgICB0aGlzLm1fZ2FtbWEgPSAxIC8gdGhpcy5tX2dhbW1hO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgdGhpcy5tX2JpYXMgPSBDICogaCAqIGsgKiB0aGlzLm1fZ2FtbWE7XHJcblxyXG4gICAgICAgIHRoaXMubV9zcHJpbmdNYXNzID0gaW52TWFzcyArIHRoaXMubV9nYW1tYTtcclxuICAgICAgICBpZiAodGhpcy5tX3NwcmluZ01hc3MgPiAwKSB7XHJcbiAgICAgICAgICB0aGlzLm1fc3ByaW5nTWFzcyA9IDEgLyB0aGlzLm1fc3ByaW5nTWFzcztcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9zcHJpbmdJbXB1bHNlID0gMDtcclxuICAgIH1cclxuXHJcbiAgICAvLyBSb3RhdGlvbmFsIG1vdG9yXHJcbiAgICBpZiAodGhpcy5tX2VuYWJsZU1vdG9yKSB7XHJcbiAgICAgIHRoaXMubV9tb3Rvck1hc3MgPSBpQSArIGlCO1xyXG4gICAgICBpZiAodGhpcy5tX21vdG9yTWFzcyA+IDApIHtcclxuICAgICAgICB0aGlzLm1fbW90b3JNYXNzID0gMSAvIHRoaXMubV9tb3Rvck1hc3M7XHJcbiAgICAgIH1cclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9tb3Rvck1hc3MgPSAwO1xyXG4gICAgICB0aGlzLm1fbW90b3JJbXB1bHNlID0gMDtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoZGF0YS5zdGVwLndhcm1TdGFydGluZykge1xyXG4gICAgICAvLyBBY2NvdW50IGZvciB2YXJpYWJsZSB0aW1lIHN0ZXAuXHJcbiAgICAgIHRoaXMubV9pbXB1bHNlICo9IGRhdGEuc3RlcC5kdFJhdGlvO1xyXG4gICAgICB0aGlzLm1fc3ByaW5nSW1wdWxzZSAqPSBkYXRhLnN0ZXAuZHRSYXRpbztcclxuICAgICAgdGhpcy5tX21vdG9ySW1wdWxzZSAqPSBkYXRhLnN0ZXAuZHRSYXRpbztcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBQID0gbV9pbXB1bHNlICogbV9heSArIG1fc3ByaW5nSW1wdWxzZSAqIG1fYXg7XHJcbiAgICAgIGNvbnN0IFA6IGIyVmVjMiA9IGIyVmVjMi5BZGRWVihcclxuICAgICAgICBiMlZlYzIuTXVsU1YodGhpcy5tX2ltcHVsc2UsIHRoaXMubV9heSwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgIGIyVmVjMi5NdWxTVih0aGlzLm1fc3ByaW5nSW1wdWxzZSwgdGhpcy5tX2F4LCBiMlZlYzIuc190MSksXHJcbiAgICAgICAgYjJXaGVlbEpvaW50LkluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCxcclxuICAgICAgKTtcclxuICAgICAgLy8gZmxvYXQzMiBMQSA9IG1faW1wdWxzZSAqIG1fc0F5ICsgbV9zcHJpbmdJbXB1bHNlICogbV9zQXggKyBtX21vdG9ySW1wdWxzZTtcclxuICAgICAgY29uc3QgTEE6IG51bWJlciA9XHJcbiAgICAgICAgdGhpcy5tX2ltcHVsc2UgKiB0aGlzLm1fc0F5ICsgdGhpcy5tX3NwcmluZ0ltcHVsc2UgKiB0aGlzLm1fc0F4ICsgdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuICAgICAgLy8gZmxvYXQzMiBMQiA9IG1faW1wdWxzZSAqIG1fc0J5ICsgbV9zcHJpbmdJbXB1bHNlICogbV9zQnggKyBtX21vdG9ySW1wdWxzZTtcclxuICAgICAgY29uc3QgTEI6IG51bWJlciA9XHJcbiAgICAgICAgdGhpcy5tX2ltcHVsc2UgKiB0aGlzLm1fc0J5ICsgdGhpcy5tX3NwcmluZ0ltcHVsc2UgKiB0aGlzLm1fc0J4ICsgdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuXHJcbiAgICAgIC8vIHZBIC09IG1faW52TWFzc0EgKiBQO1xyXG4gICAgICB2QS5TZWxmTXVsU3ViKHRoaXMubV9pbnZNYXNzQSwgUCk7XHJcbiAgICAgIHdBIC09IHRoaXMubV9pbnZJQSAqIExBO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbV9pbnZNYXNzQiAqIFA7XHJcbiAgICAgIHZCLlNlbGZNdWxBZGQodGhpcy5tX2ludk1hc3NCLCBQKTtcclxuICAgICAgd0IgKz0gdGhpcy5tX2ludklCICogTEI7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1faW1wdWxzZSA9IDA7XHJcbiAgICAgIHRoaXMubV9zcHJpbmdJbXB1bHNlID0gMDtcclxuICAgICAgdGhpcy5tX21vdG9ySW1wdWxzZSA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnYgPSB2QTtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53ID0gd0E7XHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udiA9IHZCO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLncgPSB3QjtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1AgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiB2b2lkIHtcclxuICAgIGNvbnN0IG1BOiBudW1iZXIgPSB0aGlzLm1faW52TWFzc0EsXHJcbiAgICAgIG1COiBudW1iZXIgPSB0aGlzLm1faW52TWFzc0I7XHJcbiAgICBjb25zdCBpQTogbnVtYmVyID0gdGhpcy5tX2ludklBLFxyXG4gICAgICBpQjogbnVtYmVyID0gdGhpcy5tX2ludklCO1xyXG5cclxuICAgIGNvbnN0IHZBOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udjtcclxuICAgIGxldCB3QTogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnc7XHJcbiAgICBjb25zdCB2QjogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnY7XHJcbiAgICBsZXQgd0I6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS53O1xyXG5cclxuICAgIC8vIFNvbHZlIHNwcmluZyBjb25zdHJhaW50XHJcbiAgICB7XHJcbiAgICAgIGNvbnN0IENkb3Q6IG51bWJlciA9XHJcbiAgICAgICAgYjJWZWMyLkRvdFZWKHRoaXMubV9heCwgYjJWZWMyLlN1YlZWKHZCLCB2QSwgYjJWZWMyLnNfdDApKSArXHJcbiAgICAgICAgdGhpcy5tX3NCeCAqIHdCIC1cclxuICAgICAgICB0aGlzLm1fc0F4ICogd0E7XHJcbiAgICAgIGNvbnN0IGltcHVsc2U6IG51bWJlciA9XHJcbiAgICAgICAgLXRoaXMubV9zcHJpbmdNYXNzICogKENkb3QgKyB0aGlzLm1fYmlhcyArIHRoaXMubV9nYW1tYSAqIHRoaXMubV9zcHJpbmdJbXB1bHNlKTtcclxuICAgICAgdGhpcy5tX3NwcmluZ0ltcHVsc2UgKz0gaW1wdWxzZTtcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBQID0gaW1wdWxzZSAqIG1fYXg7XHJcbiAgICAgIGNvbnN0IFA6IGIyVmVjMiA9IGIyVmVjMi5NdWxTVihpbXB1bHNlLCB0aGlzLm1fYXgsIGIyV2hlZWxKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QKTtcclxuICAgICAgY29uc3QgTEE6IG51bWJlciA9IGltcHVsc2UgKiB0aGlzLm1fc0F4O1xyXG4gICAgICBjb25zdCBMQjogbnVtYmVyID0gaW1wdWxzZSAqIHRoaXMubV9zQng7XHJcblxyXG4gICAgICAvLyB2QSAtPSBtQSAqIFA7XHJcbiAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgICB3QSAtPSBpQSAqIExBO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgd0IgKz0gaUIgKiBMQjtcclxuICAgIH1cclxuXHJcbiAgICAvLyBTb2x2ZSByb3RhdGlvbmFsIG1vdG9yIGNvbnN0cmFpbnRcclxuICAgIHtcclxuICAgICAgY29uc3QgQ2RvdDogbnVtYmVyID0gd0IgLSB3QSAtIHRoaXMubV9tb3RvclNwZWVkO1xyXG4gICAgICBsZXQgaW1wdWxzZTogbnVtYmVyID0gLXRoaXMubV9tb3Rvck1hc3MgKiBDZG90O1xyXG5cclxuICAgICAgY29uc3Qgb2xkSW1wdWxzZTogbnVtYmVyID0gdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuICAgICAgY29uc3QgbWF4SW1wdWxzZTogbnVtYmVyID0gZGF0YS5zdGVwLmR0ICogdGhpcy5tX21heE1vdG9yVG9ycXVlO1xyXG4gICAgICB0aGlzLm1fbW90b3JJbXB1bHNlID0gYjJDbGFtcCh0aGlzLm1fbW90b3JJbXB1bHNlICsgaW1wdWxzZSwgLW1heEltcHVsc2UsIG1heEltcHVsc2UpO1xyXG4gICAgICBpbXB1bHNlID0gdGhpcy5tX21vdG9ySW1wdWxzZSAtIG9sZEltcHVsc2U7XHJcblxyXG4gICAgICB3QSAtPSBpQSAqIGltcHVsc2U7XHJcbiAgICAgIHdCICs9IGlCICogaW1wdWxzZTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBTb2x2ZSBwb2ludCB0byBsaW5lIGNvbnN0cmFpbnRcclxuICAgIHtcclxuICAgICAgY29uc3QgQ2RvdDogbnVtYmVyID1cclxuICAgICAgICBiMlZlYzIuRG90VlYodGhpcy5tX2F5LCBiMlZlYzIuU3ViVlYodkIsIHZBLCBiMlZlYzIuc190MCkpICtcclxuICAgICAgICB0aGlzLm1fc0J5ICogd0IgLVxyXG4gICAgICAgIHRoaXMubV9zQXkgKiB3QTtcclxuICAgICAgY29uc3QgaW1wdWxzZTogbnVtYmVyID0gLXRoaXMubV9tYXNzICogQ2RvdDtcclxuICAgICAgdGhpcy5tX2ltcHVsc2UgKz0gaW1wdWxzZTtcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBQID0gaW1wdWxzZSAqIG1fYXk7XHJcbiAgICAgIGNvbnN0IFA6IGIyVmVjMiA9IGIyVmVjMi5NdWxTVihpbXB1bHNlLCB0aGlzLm1fYXksIGIyV2hlZWxKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QKTtcclxuICAgICAgY29uc3QgTEE6IG51bWJlciA9IGltcHVsc2UgKiB0aGlzLm1fc0F5O1xyXG4gICAgICBjb25zdCBMQjogbnVtYmVyID0gaW1wdWxzZSAqIHRoaXMubV9zQnk7XHJcblxyXG4gICAgICAvLyB2QSAtPSBtQSAqIFA7XHJcbiAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgICB3QSAtPSBpQSAqIExBO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgd0IgKz0gaUIgKiBMQjtcclxuICAgIH1cclxuXHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udiA9IHZBO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLncgPSB3QTtcclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52ID0gdkI7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udyA9IHdCO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfZCA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19QID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogYm9vbGVhbiB7XHJcbiAgICBjb25zdCBjQTogYjJWZWMyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYztcclxuICAgIGxldCBhQTogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYTtcclxuICAgIGNvbnN0IGNCOiBiMlZlYzIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5jO1xyXG4gICAgbGV0IGFCOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hO1xyXG5cclxuICAgIGNvbnN0IHFBOiBiMlJvdCA9IHRoaXMubV9xQS5TZXRBbmdsZShhQSksXHJcbiAgICAgIHFCOiBiMlJvdCA9IHRoaXMubV9xQi5TZXRBbmdsZShhQik7XHJcblxyXG4gICAgLy8gYjJWZWMyIHJBID0gYjJNdWwocUEsIG1fbG9jYWxBbmNob3JBIC0gbV9sb2NhbENlbnRlckEpO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckEsIHRoaXMubV9sb2NhbENlbnRlckEsIHRoaXMubV9sYWxjQSk7XHJcbiAgICBjb25zdCByQTogYjJWZWMyID0gYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sYWxjQSwgdGhpcy5tX3JBKTtcclxuICAgIC8vIGIyVmVjMiByQiA9IGIyTXVsKHFCLCBtX2xvY2FsQW5jaG9yQiAtIG1fbG9jYWxDZW50ZXJCKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgY29uc3QgckI6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFCLCB0aGlzLm1fbGFsY0IsIHRoaXMubV9yQik7XHJcbiAgICAvLyBiMlZlYzIgZCA9IChjQiAtIGNBKSArIHJCIC0gckE7XHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBiMlZlYzIuQWRkVlYoXHJcbiAgICAgIGIyVmVjMi5TdWJWVihjQiwgY0EsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgYjJWZWMyLlN1YlZWKHJCLCByQSwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICBiMldoZWVsSm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfZCxcclxuICAgICk7XHJcblxyXG4gICAgLy8gYjJWZWMyIGF5ID0gYjJNdWwocUEsIG1fbG9jYWxZQXhpc0EpO1xyXG4gICAgY29uc3QgYXk6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFBLCB0aGlzLm1fbG9jYWxZQXhpc0EsIHRoaXMubV9heSk7XHJcblxyXG4gICAgLy8gZmxvYXQzMiBzQXkgPSBiMkNyb3NzKGQgKyByQSwgYXkpO1xyXG4gICAgY29uc3Qgc0F5ID0gYjJWZWMyLkNyb3NzVlYoYjJWZWMyLkFkZFZWKGQsIHJBLCBiMlZlYzIuc190MCksIGF5KTtcclxuICAgIC8vIGZsb2F0MzIgc0J5ID0gYjJDcm9zcyhyQiwgYXkpO1xyXG4gICAgY29uc3Qgc0J5ID0gYjJWZWMyLkNyb3NzVlYockIsIGF5KTtcclxuXHJcbiAgICAvLyBmbG9hdDMyIEMgPSBiMkRvdChkLCBheSk7XHJcbiAgICBjb25zdCBDOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoZCwgdGhpcy5tX2F5KTtcclxuXHJcbiAgICBjb25zdCBrOiBudW1iZXIgPVxyXG4gICAgICB0aGlzLm1faW52TWFzc0EgK1xyXG4gICAgICB0aGlzLm1faW52TWFzc0IgK1xyXG4gICAgICB0aGlzLm1faW52SUEgKiB0aGlzLm1fc0F5ICogdGhpcy5tX3NBeSArXHJcbiAgICAgIHRoaXMubV9pbnZJQiAqIHRoaXMubV9zQnkgKiB0aGlzLm1fc0J5O1xyXG5cclxuICAgIGxldCBpbXB1bHNlOiBudW1iZXI7XHJcbiAgICBpZiAoayAhPT0gMCkge1xyXG4gICAgICBpbXB1bHNlID0gLUMgLyBrO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgaW1wdWxzZSA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gYjJWZWMyIFAgPSBpbXB1bHNlICogYXk7XHJcbiAgICBjb25zdCBQOiBiMlZlYzIgPSBiMlZlYzIuTXVsU1YoaW1wdWxzZSwgYXksIGIyV2hlZWxKb2ludC5Tb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19QKTtcclxuICAgIGNvbnN0IExBOiBudW1iZXIgPSBpbXB1bHNlICogc0F5O1xyXG4gICAgY29uc3QgTEI6IG51bWJlciA9IGltcHVsc2UgKiBzQnk7XHJcblxyXG4gICAgLy8gY0EgLT0gbV9pbnZNYXNzQSAqIFA7XHJcbiAgICBjQS5TZWxmTXVsU3ViKHRoaXMubV9pbnZNYXNzQSwgUCk7XHJcbiAgICBhQSAtPSB0aGlzLm1faW52SUEgKiBMQTtcclxuICAgIC8vIGNCICs9IG1faW52TWFzc0IgKiBQO1xyXG4gICAgY0IuU2VsZk11bEFkZCh0aGlzLm1faW52TWFzc0IsIFApO1xyXG4gICAgYUIgKz0gdGhpcy5tX2ludklCICogTEI7XHJcblxyXG4gICAgLy8gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYyA9IGNBO1xyXG4gICAgZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYSA9IGFBO1xyXG4gICAgLy8gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYyA9IGNCO1xyXG4gICAgZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYSA9IGFCO1xyXG5cclxuICAgIHJldHVybiBiMkFicyhDKSA8PSBiMl9saW5lYXJTbG9wO1xyXG4gIH1cclxuXHJcbiAgR2V0RGVmaW5pdGlvbihkZWY6IGIyV2hlZWxKb2ludERlZik6IGIyV2hlZWxKb2ludERlZiB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZhbHNlKTsgLy8gVE9ET1xyXG4gICAgcmV0dXJuIGRlZjtcclxuICB9XHJcblxyXG4gIEdldEFuY2hvckE8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUEuR2V0V29ybGRQb2ludCh0aGlzLm1fbG9jYWxBbmNob3JBLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQjxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5Qi5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckIsIG91dCk7XHJcbiAgfVxyXG5cclxuICBHZXRSZWFjdGlvbkZvcmNlPFQgZXh0ZW5kcyBYWT4oaW52X2R0OiBudW1iZXIsIG91dDogVCk6IFQge1xyXG4gICAgLy8gcmV0dXJuIGludl9kdCAqIChtX2ltcHVsc2UgKiBtX2F5ICsgbV9zcHJpbmdJbXB1bHNlICogbV9heCk7XHJcbiAgICBvdXQueCA9IGludl9kdCAqICh0aGlzLm1faW1wdWxzZSAqIHRoaXMubV9heS54ICsgdGhpcy5tX3NwcmluZ0ltcHVsc2UgKiB0aGlzLm1fYXgueCk7XHJcbiAgICBvdXQueSA9IGludl9kdCAqICh0aGlzLm1faW1wdWxzZSAqIHRoaXMubV9heS55ICsgdGhpcy5tX3NwcmluZ0ltcHVsc2UgKiB0aGlzLm1fYXgueSk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgR2V0UmVhY3Rpb25Ub3JxdWUoaW52X2R0OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIGludl9kdCAqIHRoaXMubV9tb3RvckltcHVsc2U7XHJcbiAgfVxyXG5cclxuICBHZXRMb2NhbEFuY2hvckEoKTogUmVhZG9ubHk8YjJWZWMyPiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2xvY2FsQW5jaG9yQTtcclxuICB9XHJcblxyXG4gIEdldExvY2FsQW5jaG9yQigpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxBbmNob3JCO1xyXG4gIH1cclxuXHJcbiAgR2V0TG9jYWxBeGlzQSgpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxYQXhpc0E7XHJcbiAgfVxyXG5cclxuICBHZXRKb2ludFRyYW5zbGF0aW9uKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5HZXRQcmlzbWF0aWNKb2ludFRyYW5zbGF0aW9uKCk7XHJcbiAgfVxyXG5cclxuICBHZXRKb2ludExpbmVhclNwZWVkKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5HZXRQcmlzbWF0aWNKb2ludFNwZWVkKCk7XHJcbiAgfVxyXG5cclxuICBHZXRKb2ludEFuZ2xlKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5HZXRSZXZvbHV0ZUpvaW50QW5nbGUoKTtcclxuICB9XHJcblxyXG4gIEdldEpvaW50QW5ndWxhclNwZWVkKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5HZXRSZXZvbHV0ZUpvaW50U3BlZWQoKTtcclxuICB9XHJcblxyXG4gIEdldFByaXNtYXRpY0pvaW50VHJhbnNsYXRpb24oKTogbnVtYmVyIHtcclxuICAgIGNvbnN0IGJBOiBiMkJvZHkgPSB0aGlzLm1fYm9keUE7XHJcbiAgICBjb25zdCBiQjogYjJCb2R5ID0gdGhpcy5tX2JvZHlCO1xyXG5cclxuICAgIGNvbnN0IHBBOiBiMlZlYzIgPSBiQS5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckEsIG5ldyBiMlZlYzIoKSk7XHJcbiAgICBjb25zdCBwQjogYjJWZWMyID0gYkIuR2V0V29ybGRQb2ludCh0aGlzLm1fbG9jYWxBbmNob3JCLCBuZXcgYjJWZWMyKCkpO1xyXG4gICAgY29uc3QgZDogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHBCLCBwQSwgbmV3IGIyVmVjMigpKTtcclxuICAgIGNvbnN0IGF4aXM6IGIyVmVjMiA9IGJBLkdldFdvcmxkVmVjdG9yKHRoaXMubV9sb2NhbFhBeGlzQSwgbmV3IGIyVmVjMigpKTtcclxuXHJcbiAgICBjb25zdCB0cmFuc2xhdGlvbjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGQsIGF4aXMpO1xyXG4gICAgcmV0dXJuIHRyYW5zbGF0aW9uO1xyXG4gIH1cclxuXHJcbiAgR2V0UHJpc21hdGljSm9pbnRTcGVlZCgpOiBudW1iZXIge1xyXG4gICAgY29uc3QgYkE6IGIyQm9keSA9IHRoaXMubV9ib2R5QTtcclxuICAgIGNvbnN0IGJCOiBiMkJvZHkgPSB0aGlzLm1fYm9keUI7XHJcblxyXG4gICAgLy8gYjJWZWMyIHJBID0gYjJNdWwoYkEtPm1feGYucSwgbV9sb2NhbEFuY2hvckEgLSBiQS0+bV9zd2VlcC5sb2NhbENlbnRlcik7XHJcbiAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2xvY2FsQW5jaG9yQSwgYkEubV9zd2VlcC5sb2NhbENlbnRlciwgdGhpcy5tX2xhbGNBKTtcclxuICAgIGNvbnN0IHJBID0gYjJSb3QuTXVsUlYoYkEubV94Zi5xLCB0aGlzLm1fbGFsY0EsIHRoaXMubV9yQSk7XHJcbiAgICAvLyBiMlZlYzIgckIgPSBiMk11bChiQi0+bV94Zi5xLCBtX2xvY2FsQW5jaG9yQiAtIGJCLT5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCBiQi5tX3N3ZWVwLmxvY2FsQ2VudGVyLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgY29uc3QgckIgPSBiMlJvdC5NdWxSVihiQi5tX3hmLnEsIHRoaXMubV9sYWxjQiwgdGhpcy5tX3JCKTtcclxuICAgIC8vIGIyVmVjMiBwQSA9IGJBLT5tX3N3ZWVwLmMgKyByQTtcclxuICAgIGNvbnN0IHBBID0gYjJWZWMyLkFkZFZWKGJBLm1fc3dlZXAuYywgckEsIGIyVmVjMi5zX3QwKTsgLy8gcEEgdXNlcyBzX3QwXHJcbiAgICAvLyBiMlZlYzIgcEIgPSBiQi0+bV9zd2VlcC5jICsgckI7XHJcbiAgICBjb25zdCBwQiA9IGIyVmVjMi5BZGRWVihiQi5tX3N3ZWVwLmMsIHJCLCBiMlZlYzIuc190MSk7IC8vIHBCIHVzZXMgc190MVxyXG4gICAgLy8gYjJWZWMyIGQgPSBwQiAtIHBBO1xyXG4gICAgY29uc3QgZCA9IGIyVmVjMi5TdWJWVihwQiwgcEEsIGIyVmVjMi5zX3QyKTsgLy8gZCB1c2VzIHNfdDJcclxuICAgIC8vIGIyVmVjMiBheGlzID0gYjJNdWwoYkEubV94Zi5xLCBtX2xvY2FsWEF4aXNBKTtcclxuICAgIGNvbnN0IGF4aXMgPSBiQS5HZXRXb3JsZFZlY3Rvcih0aGlzLm1fbG9jYWxYQXhpc0EsIG5ldyBiMlZlYzIoKSk7XHJcblxyXG4gICAgY29uc3QgdkEgPSBiQS5tX2xpbmVhclZlbG9jaXR5O1xyXG4gICAgY29uc3QgdkIgPSBiQi5tX2xpbmVhclZlbG9jaXR5O1xyXG4gICAgY29uc3Qgd0EgPSBiQS5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuICAgIGNvbnN0IHdCID0gYkIubV9hbmd1bGFyVmVsb2NpdHk7XHJcblxyXG4gICAgLy8gZmxvYXQzMiBzcGVlZCA9IGIyRG90KGQsIGIyQ3Jvc3Mod0EsIGF4aXMpKSArIGIyRG90KGF4aXMsIHZCICsgYjJDcm9zcyh3QiwgckIpIC0gdkEgLSBiMkNyb3NzKHdBLCByQSkpO1xyXG4gICAgY29uc3Qgc3BlZWQgPVxyXG4gICAgICBiMlZlYzIuRG90VlYoZCwgYjJWZWMyLkNyb3NzU1Yod0EsIGF4aXMsIGIyVmVjMi5zX3QwKSkgK1xyXG4gICAgICBiMlZlYzIuRG90VlYoXHJcbiAgICAgICAgYXhpcyxcclxuICAgICAgICBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkIsIHdCLCByQiwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgICAgYjJWZWMyLkFkZFZDcm9zc1NWKHZBLCB3QSwgckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICAgIGIyVmVjMi5zX3QwLFxyXG4gICAgICAgICksXHJcbiAgICAgICk7XHJcbiAgICByZXR1cm4gc3BlZWQ7XHJcbiAgfVxyXG5cclxuICBHZXRSZXZvbHV0ZUpvaW50QW5nbGUoKTogbnVtYmVyIHtcclxuICAgIC8vIGIyQm9keSogYkEgPSB0aGlzLm1fYm9keUE7XHJcbiAgICAvLyBiMkJvZHkqIGJCID0gdGhpcy5tX2JvZHlCO1xyXG4gICAgLy8gcmV0dXJuIGJCLT50aGlzLm1fc3dlZXAuYSAtIGJBLT50aGlzLm1fc3dlZXAuYTtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUIubV9zd2VlcC5hIC0gdGhpcy5tX2JvZHlBLm1fc3dlZXAuYTtcclxuICB9XHJcblxyXG4gIEdldFJldm9sdXRlSm9pbnRTcGVlZCgpOiBudW1iZXIge1xyXG4gICAgY29uc3Qgd0E6IG51bWJlciA9IHRoaXMubV9ib2R5QS5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuICAgIGNvbnN0IHdCOiBudW1iZXIgPSB0aGlzLm1fYm9keUIubV9hbmd1bGFyVmVsb2NpdHk7XHJcbiAgICByZXR1cm4gd0IgLSB3QTtcclxuICB9XHJcblxyXG4gIElzTW90b3JFbmFibGVkKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9lbmFibGVNb3RvcjtcclxuICB9XHJcblxyXG4gIEVuYWJsZU1vdG9yKGZsYWc6IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGlmIChmbGFnICE9PSB0aGlzLm1fZW5hYmxlTW90b3IpIHtcclxuICAgICAgdGhpcy5tX2JvZHlBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9lbmFibGVNb3RvciA9IGZsYWc7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTZXRNb3RvclNwZWVkKHNwZWVkOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGlmIChzcGVlZCAhPT0gdGhpcy5tX21vdG9yU3BlZWQpIHtcclxuICAgICAgdGhpcy5tX2JvZHlBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9tb3RvclNwZWVkID0gc3BlZWQ7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTZXRNYXhNb3RvclRvcnF1ZShmb3JjZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICBpZiAoZm9yY2UgIT09IHRoaXMubV9tYXhNb3RvclRvcnF1ZSkge1xyXG4gICAgICB0aGlzLm1fYm9keUEuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9ib2R5Qi5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX21heE1vdG9yVG9ycXVlID0gZm9yY2U7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBHZXRNb3RvclRvcnF1ZShpbnZfZHQ6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICByZXR1cm4gaW52X2R0ICogdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuICB9XHJcbn1cclxuIl19