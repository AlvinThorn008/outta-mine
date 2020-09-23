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
import { b2_angularSlop, b2_linearSlop, b2_maxAngularCorrection, b2Maybe, } from '../../common/b2Settings';
import { b2Abs, b2Clamp, b2Mat22, b2Mat33, b2Rot, b2Vec2, b2Vec3 } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
export class b2RevoluteJointDef extends b2JointDef {
    constructor() {
        super(1 /* e_revoluteJoint */);
        this.localAnchorA = new b2Vec2(0, 0);
        this.localAnchorB = new b2Vec2(0, 0);
        this.referenceAngle = 0;
        this.enableLimit = false;
        this.lowerAngle = 0;
        this.upperAngle = 0;
        this.enableMotor = false;
        this.motorSpeed = 0;
        this.maxMotorTorque = 0;
    }
    Initialize(bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}
export class b2RevoluteJoint extends b2Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_impulse = new b2Vec3();
        this.m_motorImpulse = 0;
        this.m_enableMotor = false;
        this.m_maxMotorTorque = 0;
        this.m_motorSpeed = 0;
        this.m_enableLimit = false;
        this.m_referenceAngle = 0;
        this.m_lowerAngle = 0;
        this.m_upperAngle = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_mass = new b2Mat33(); // effective mass for point-to-point constraint.
        this.m_motorMass = 0; // effective mass for motor/limit angular constraint.
        this.m_limitState = 0 /* e_inactiveLimit */;
        this.m_qA = new b2Rot();
        this.m_qB = new b2Rot();
        this.m_lalcA = new b2Vec2();
        this.m_lalcB = new b2Vec2();
        this.m_K = new b2Mat22();
        this.m_localAnchorA.Copy(b2Maybe(def.localAnchorA, b2Vec2.ZERO));
        this.m_localAnchorB.Copy(b2Maybe(def.localAnchorB, b2Vec2.ZERO));
        this.m_referenceAngle = b2Maybe(def.referenceAngle, 0);
        this.m_impulse.SetZero();
        this.m_motorImpulse = 0;
        this.m_lowerAngle = b2Maybe(def.lowerAngle, 0);
        this.m_upperAngle = b2Maybe(def.upperAngle, 0);
        this.m_maxMotorTorque = b2Maybe(def.maxMotorTorque, 0);
        this.m_motorSpeed = b2Maybe(def.motorSpeed, 0);
        this.m_enableLimit = b2Maybe(def.enableLimit, false);
        this.m_enableMotor = b2Maybe(def.enableMotor, false);
        this.m_limitState = 0 /* e_inactiveLimit */;
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
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // b2Rot qA(aA), qB(aB);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const fixedRotation = iA + iB === 0;
        this.m_mass.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        this.m_mass.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        this.m_mass.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        this.m_mass.ex.y = this.m_mass.ey.x;
        this.m_mass.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        this.m_mass.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        this.m_mass.ex.z = this.m_mass.ez.x;
        this.m_mass.ey.z = this.m_mass.ez.y;
        this.m_mass.ez.z = iA + iB;
        this.m_motorMass = iA + iB;
        if (this.m_motorMass > 0) {
            this.m_motorMass = 1 / this.m_motorMass;
        }
        if (!this.m_enableMotor || fixedRotation) {
            this.m_motorImpulse = 0;
        }
        if (this.m_enableLimit && !fixedRotation) {
            const jointAngle = aB - aA - this.m_referenceAngle;
            if (b2Abs(this.m_upperAngle - this.m_lowerAngle) < 2 * b2_angularSlop) {
                this.m_limitState = 3 /* e_equalLimits */;
            }
            else if (jointAngle <= this.m_lowerAngle) {
                if (this.m_limitState !== 1 /* e_atLowerLimit */) {
                    this.m_impulse.z = 0;
                }
                this.m_limitState = 1 /* e_atLowerLimit */;
            }
            else if (jointAngle >= this.m_upperAngle) {
                if (this.m_limitState !== 2 /* e_atUpperLimit */) {
                    this.m_impulse.z = 0;
                }
                this.m_limitState = 2 /* e_atUpperLimit */;
            }
            else {
                this.m_limitState = 0 /* e_inactiveLimit */;
                this.m_impulse.z = 0;
            }
        }
        else {
            this.m_limitState = 0 /* e_inactiveLimit */;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.SelfMul(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            // b2Vec2 P(m_impulse.x, m_impulse.y);
            const P = b2RevoluteJoint.InitVelocityConstraints_s_P.Set(this.m_impulse.x, this.m_impulse.y);
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Vec2.CrossVV(this.m_rA, P) + this.m_motorImpulse + this.m_impulse.z);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Vec2.CrossVV(this.m_rB, P) + this.m_motorImpulse + this.m_impulse.z);
        }
        else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const fixedRotation = iA + iB === 0;
        // Solve motor constraint.
        if (this.m_enableMotor && this.m_limitState !== 3 /* e_equalLimits */ && !fixedRotation) {
            const Cdot = wB - wA - this.m_motorSpeed;
            let impulse = -this.m_motorMass * Cdot;
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve limit constraint.
        if (this.m_enableLimit &&
            this.m_limitState !== 0 /* e_inactiveLimit */ &&
            !fixedRotation) {
            // b2Vec2 Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
            const Cdot1 = b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Vec2.s_t1), b2RevoluteJoint.SolveVelocityConstraints_s_Cdot1);
            const Cdot2 = wB - wA;
            // b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // b2Vec3 impulse = -this.m_mass.Solve33(Cdot);
            const impulse_v3 = this.m_mass
                .Solve33(Cdot1.x, Cdot1.y, Cdot2, b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v3)
                .SelfNeg();
            if (this.m_limitState === 3 /* e_equalLimits */) {
                this.m_impulse.SelfAdd(impulse_v3);
            }
            else if (this.m_limitState === 1 /* e_atLowerLimit */) {
                const newImpulse = this.m_impulse.z + impulse_v3.z;
                if (newImpulse < 0) {
                    // b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
                    const rhs_x = -Cdot1.x + this.m_impulse.z * this.m_mass.ez.x;
                    const rhs_y = -Cdot1.y + this.m_impulse.z * this.m_mass.ez.y;
                    const reduced_v2 = this.m_mass.Solve22(rhs_x, rhs_y, b2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2);
                    impulse_v3.x = reduced_v2.x;
                    impulse_v3.y = reduced_v2.y;
                    impulse_v3.z = -this.m_impulse.z;
                    this.m_impulse.x += reduced_v2.x;
                    this.m_impulse.y += reduced_v2.y;
                    this.m_impulse.z = 0;
                }
                else {
                    this.m_impulse.SelfAdd(impulse_v3);
                }
            }
            else if (this.m_limitState === 2 /* e_atUpperLimit */) {
                const newImpulse = this.m_impulse.z + impulse_v3.z;
                if (newImpulse > 0) {
                    // b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
                    const rhs_x = -Cdot1.x + this.m_impulse.z * this.m_mass.ez.x;
                    const rhs_y = -Cdot1.y + this.m_impulse.z * this.m_mass.ez.y;
                    const reduced_v2 = this.m_mass.Solve22(rhs_x, rhs_y, b2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2);
                    impulse_v3.x = reduced_v2.x;
                    impulse_v3.y = reduced_v2.y;
                    impulse_v3.z = -this.m_impulse.z;
                    this.m_impulse.x += reduced_v2.x;
                    this.m_impulse.y += reduced_v2.y;
                    this.m_impulse.z = 0;
                }
                else {
                    this.m_impulse.SelfAdd(impulse_v3);
                }
            }
            // b2Vec2 P(impulse.x, impulse.y);
            const P = b2RevoluteJoint.SolveVelocityConstraints_s_P.Set(impulse_v3.x, impulse_v3.y);
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Vec2.CrossVV(this.m_rA, P) + impulse_v3.z);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Vec2.CrossVV(this.m_rB, P) + impulse_v3.z);
        }
        else {
            // Solve point-to-point constraint
            // b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
            const Cdot_v2 = b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Vec2.s_t1), b2RevoluteJoint.SolveVelocityConstraints_s_Cdot_v2);
            // b2Vec2 impulse = m_mass.Solve22(-Cdot);
            const impulse_v2 = this.m_mass.Solve22(-Cdot_v2.x, -Cdot_v2.y, b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v2);
            this.m_impulse.x += impulse_v2.x;
            this.m_impulse.y += impulse_v2.y;
            // vA -= mA * impulse;
            vA.SelfMulSub(mA, impulse_v2);
            wA -= iA * b2Vec2.CrossVV(this.m_rA, impulse_v2);
            // vB += mB * impulse;
            vB.SelfMulAdd(mB, impulse_v2);
            wB += iB * b2Vec2.CrossVV(this.m_rB, impulse_v2);
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
        // b2Rot qA(aA), qB(aB);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        let angularError = 0;
        let positionError = 0;
        const fixedRotation = this.m_invIA + this.m_invIB === 0;
        // Solve angular limit constraint.
        if (this.m_enableLimit &&
            this.m_limitState !== 0 /* e_inactiveLimit */ &&
            !fixedRotation) {
            const angle = aB - aA - this.m_referenceAngle;
            let limitImpulse = 0;
            if (this.m_limitState === 3 /* e_equalLimits */) {
                // Prevent large angular corrections
                const C = b2Clamp(angle - this.m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
                limitImpulse = -this.m_motorMass * C;
                angularError = b2Abs(C);
            }
            else if (this.m_limitState === 1 /* e_atLowerLimit */) {
                let C = angle - this.m_lowerAngle;
                angularError = -C;
                // Prevent large angular corrections and allow some slop.
                C = b2Clamp(C + b2_angularSlop, -b2_maxAngularCorrection, 0);
                limitImpulse = -this.m_motorMass * C;
            }
            else if (this.m_limitState === 2 /* e_atUpperLimit */) {
                let C = angle - this.m_upperAngle;
                angularError = C;
                // Prevent large angular corrections and allow some slop.
                C = b2Clamp(C - b2_angularSlop, 0, b2_maxAngularCorrection);
                limitImpulse = -this.m_motorMass * C;
            }
            aA -= this.m_invIA * limitImpulse;
            aB += this.m_invIB * limitImpulse;
        }
        // Solve point-to-point constraint.
        {
            qA.SetAngle(aA);
            qB.SetAngle(aB);
            // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
            b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
            const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
            // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
            b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
            const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
            // b2Vec2 C = cB + rB - cA - rA;
            const C_v2 = b2Vec2.SubVV(b2Vec2.AddVV(cB, rB, b2Vec2.s_t0), b2Vec2.AddVV(cA, rA, b2Vec2.s_t1), b2RevoluteJoint.SolvePositionConstraints_s_C_v2);
            // positionError = C.Length();
            positionError = C_v2.Length();
            const mA = this.m_invMassA, mB = this.m_invMassB;
            const iA = this.m_invIA, iB = this.m_invIB;
            const K = this.m_K;
            K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
            K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
            // b2Vec2 impulse = -K.Solve(C);
            const impulse = K.Solve(C_v2.x, C_v2.y, b2RevoluteJoint.SolvePositionConstraints_s_impulse).SelfNeg();
            // cA -= mA * impulse;
            cA.SelfMulSub(mA, impulse);
            aA -= iA * b2Vec2.CrossVV(rA, impulse);
            // cB += mB * impulse;
            cB.SelfMulAdd(mB, impulse);
            aB += iB * b2Vec2.CrossVV(rB, impulse);
        }
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // b2Vec2 P(this.m_impulse.x, this.m_impulse.y);
        // return inv_dt * P;
        out.x = inv_dt * this.m_impulse.x;
        out.y = inv_dt * this.m_impulse.y;
        return out;
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_impulse.z;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    GetReferenceAngle() {
        return this.m_referenceAngle;
    }
    GetJointAngle() {
        // b2Body* bA = this.m_bodyA;
        // b2Body* bB = this.m_bodyB;
        // return bB->this.m_sweep.a - bA->this.m_sweep.a - this.m_referenceAngle;
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
    }
    GetJointSpeed() {
        // b2Body* bA = this.m_bodyA;
        // b2Body* bB = this.m_bodyB;
        // return bB->this.m_angularVelocity - bA->this.m_angularVelocity;
        return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
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
    GetMotorTorque(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    GetMotorSpeed() {
        return this.m_motorSpeed;
    }
    SetMaxMotorTorque(torque) {
        if (torque !== this.m_maxMotorTorque) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorTorque = torque;
        }
    }
    GetMaxMotorTorque() {
        return this.m_maxMotorTorque;
    }
    IsLimitEnabled() {
        return this.m_enableLimit;
    }
    EnableLimit(flag) {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0;
        }
    }
    GetLowerLimit() {
        return this.m_lowerAngle;
    }
    GetUpperLimit() {
        return this.m_upperAngle;
    }
    SetLimits(lower, upper) {
        if (lower !== this.m_lowerAngle || upper !== this.m_upperAngle) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_impulse.z = 0;
            this.m_lowerAngle = lower;
            this.m_upperAngle = upper;
        }
    }
    SetMotorSpeed(speed) {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
    }
}
b2RevoluteJoint.InitVelocityConstraints_s_P = new b2Vec2();
b2RevoluteJoint.SolveVelocityConstraints_s_P = new b2Vec2();
b2RevoluteJoint.SolveVelocityConstraints_s_Cdot_v2 = new b2Vec2();
b2RevoluteJoint.SolveVelocityConstraints_s_Cdot1 = new b2Vec2();
b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v3 = new b2Vec3();
b2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2 = new b2Vec2();
b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v2 = new b2Vec2();
b2RevoluteJoint.SolvePositionConstraints_s_C_v2 = new b2Vec2();
b2RevoluteJoint.SolvePositionConstraints_s_impulse = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJSZXZvbHV0ZUpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2pvaW50cy9iMlJldm9sdXRlSm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQ0wsY0FBYyxFQUNkLGFBQWEsRUFDYix1QkFBdUIsRUFDdkIsT0FBTyxHQUNSLE1BQU0seUJBQXlCLENBQUM7QUFDakMsT0FBTyxFQUFFLEtBQUssRUFBRSxPQUFPLEVBQUUsT0FBTyxFQUFFLE9BQU8sRUFBRSxLQUFLLEVBQUUsTUFBTSxFQUFFLE1BQU0sRUFBTSxNQUFNLHFCQUFxQixDQUFDO0FBRWxHLE9BQU8sRUFBZSxPQUFPLEVBQUUsVUFBVSxFQUE2QixNQUFNLFdBQVcsQ0FBQztBQXVCeEYsd0RBQXdEO0FBQ3hELDREQUE0RDtBQUM1RCw4REFBOEQ7QUFDOUQseURBQXlEO0FBQ3pELDZEQUE2RDtBQUM3RCx5Q0FBeUM7QUFDekMsK0RBQStEO0FBQy9ELDJDQUEyQztBQUMzQywyREFBMkQ7QUFDM0QsbUVBQW1FO0FBQ25FLGlDQUFpQztBQUNqQyxNQUFNLE9BQU8sa0JBQW1CLFNBQVEsVUFBVTtJQW1CaEQ7UUFDRSxLQUFLLHlCQUE2QixDQUFDO1FBbkI1QixpQkFBWSxHQUFXLElBQUksTUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUV4QyxpQkFBWSxHQUFXLElBQUksTUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUVqRCxtQkFBYyxHQUFHLENBQUMsQ0FBQztRQUVuQixnQkFBVyxHQUFHLEtBQUssQ0FBQztRQUVwQixlQUFVLEdBQUcsQ0FBQyxDQUFDO1FBRWYsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUVmLGdCQUFXLEdBQUcsS0FBSyxDQUFDO1FBRXBCLGVBQVUsR0FBRyxDQUFDLENBQUM7UUFFZixtQkFBYyxHQUFHLENBQUMs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TyxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUM1QixJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUM1QixJQUFJLENBQUMsU0FBUyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUM7WUFDckIsSUFBSSxDQUFDLFlBQVksR0FBRyxLQUFLLENBQUM7WUFDMUIsSUFBSSxDQUFDLFlBQVksR0FBRyxLQUFLLENBQUM7U0FDM0I7SUFDSCxDQUFDO0lBRUQsYUFBYSxDQUFDLEtBQWE7UUFDekIsSUFBSSxLQUFLLEtBQUssSUFBSSxDQUFDLFlBQVksRUFBRTtZQUMvQixJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUM1QixJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUM1QixJQUFJLENBQUMsWUFBWSxHQUFHLEtBQUssQ0FBQztTQUMzQjtJQUNILENBQUM7O0FBdGVjLDJDQUEyQixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFxSDNDLDRDQUE0QixHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUFDcEQsa0RBQWtDLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUMxRCxnREFBZ0MsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ3hELHFEQUFxQyxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUFDN0QscURBQXFDLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUM3RCxxREFBcUMsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBMEk3RCwrQ0FBK0IsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQy9DLGtEQUFrQyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMDYtMjAxMSBFcmluIENhdHRvIGh0dHA6Ly93d3cuYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7XHJcbiAgYjJfYW5ndWxhclNsb3AsXHJcbiAgYjJfbGluZWFyU2xvcCxcclxuICBiMl9tYXhBbmd1bGFyQ29ycmVjdGlvbixcclxuICBiMk1heWJlLFxyXG59IGZyb20gJy4uLy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJBYnMsIGIyQ2xhbXAsIGIyTWF0MjIsIGIyTWF0MzMsIGIyUm90LCBiMlZlYzIsIGIyVmVjMywgWFkgfSBmcm9tICcuLi8uLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJCb2R5IH0gZnJvbSAnLi4vYjJCb2R5JztcclxuaW1wb3J0IHsgYjJJSm9pbnREZWYsIGIySm9pbnQsIGIySm9pbnREZWYsIGIySm9pbnRUeXBlLCBiMkxpbWl0U3RhdGUgfSBmcm9tICcuL2IySm9pbnQnO1xyXG5pbXBvcnQgeyBiMlNvbHZlckRhdGEgfSBmcm9tICcuLi9iMlRpbWVTdGVwJztcclxuXHJcbmV4cG9ydCBpbnRlcmZhY2UgYjJJUmV2b2x1dGVKb2ludERlZiBleHRlbmRzIGIySUpvaW50RGVmIHtcclxuICBsb2NhbEFuY2hvckE/OiBYWTtcclxuXHJcbiAgbG9jYWxBbmNob3JCPzogWFk7XHJcblxyXG4gIHJlZmVyZW5jZUFuZ2xlPzogbnVtYmVyO1xyXG5cclxuICBlbmFibGVMaW1pdD86IGJvb2xlYW47XHJcblxyXG4gIGxvd2VyQW5nbGU/OiBudW1iZXI7XHJcblxyXG4gIHVwcGVyQW5nbGU/OiBudW1iZXI7XHJcblxyXG4gIGVuYWJsZU1vdG9yPzogYm9vbGVhbjtcclxuXHJcbiAgbW90b3JTcGVlZD86IG51bWJlcjtcclxuXHJcbiAgbWF4TW90b3JUb3JxdWU/OiBudW1iZXI7XHJcbn1cclxuXHJcbi8vLyBSZXZvbHV0ZSBqb2ludCBkZWZpbml0aW9uLiBUaGlzIHJlcXVpcmVzIGRlZmluaW5nIGFuXHJcbi8vLyBhbmNob3IgcG9pbnQgd2hlcmUgdGhlIGJvZGllcyBhcmUgam9pbmVkLiBUaGUgZGVmaW5pdGlvblxyXG4vLy8gdXNlcyBsb2NhbCBhbmNob3IgcG9pbnRzIHNvIHRoYXQgdGhlIGluaXRpYWwgY29uZmlndXJhdGlvblxyXG4vLy8gY2FuIHZpb2xhdGUgdGhlIGNvbnN0cmFpbnQgc2xpZ2h0bHkuIFlvdSBhbHNvIG5lZWQgdG9cclxuLy8vIHNwZWNpZnkgdGhlIGluaXRpYWwgcmVsYXRpdmUgYW5nbGUgZm9yIGpvaW50IGxpbWl0cy4gVGhpc1xyXG4vLy8gaGVscHMgd2hlbiBzYXZpbmcgYW5kIGxvYWRpbmcgYSBnYW1lLlxyXG4vLy8gVGhlIGxvY2FsIGFuY2hvciBwb2ludHMgYXJlIG1lYXN1cmVkIGZyb20gdGhlIGJvZHkncyBvcmlnaW5cclxuLy8vIHJhdGhlciB0aGFuIHRoZSBjZW50ZXIgb2YgbWFzcyBiZWNhdXNlOlxyXG4vLy8gMS4geW91IG1pZ2h0IG5vdCBrbm93IHdoZXJlIHRoZSBjZW50ZXIgb2YgbWFzcyB3aWxsIGJlLlxyXG4vLy8gMi4gaWYgeW91IGFkZC9yZW1vdmUgc2hhcGVzIGZyb20gYSBib2R5IGFuZCByZWNvbXB1dGUgdGhlIG1hc3MsXHJcbi8vLyAgICB0aGUgam9pbnRzIHdpbGwgYmUgYnJva2VuLlxyXG5leHBvcnQgY2xhc3MgYjJSZXZvbHV0ZUpvaW50RGVmIGV4dGVuZHMgYjJKb2ludERlZiBpbXBsZW1lbnRzIGIySVJldm9sdXRlSm9pbnREZWYge1xyXG4gIHJlYWRvbmx5IGxvY2FsQW5jaG9yQTogYjJWZWMyID0gbmV3IGIyVmVjMigwLCAwKTtcclxuXHJcbiAgcmVhZG9ubHkgbG9jYWxBbmNob3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKDAsIDApO1xyXG5cclxuICByZWZlcmVuY2VBbmdsZSA9IDA7XHJcblxyXG4gIGVuYWJsZUxpbWl0ID0gZmFsc2U7XHJcblxyXG4gIGxvd2VyQW5nbGUgPSAwO1xyXG5cclxuICB1cHBlckFuZ2xlID0gMDtcclxuXHJcbiAgZW5hYmxlTW90b3IgPSBmYWxzZTtcclxuXHJcbiAgbW90b3JTcGVlZCA9IDA7XHJcblxyXG4gIG1heE1vdG9yVG9ycXVlID0gMDtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICBzdXBlcihiMkpvaW50VHlwZS5lX3Jldm9sdXRlSm9pbnQpO1xyXG4gIH1cclxuXHJcbiAgSW5pdGlhbGl6ZShiQTogYjJCb2R5LCBiQjogYjJCb2R5LCBhbmNob3I6IFhZKTogdm9pZCB7XHJcbiAgICB0aGlzLmJvZHlBID0gYkE7XHJcbiAgICB0aGlzLmJvZHlCID0gYkI7XHJcbiAgICB0aGlzLmJvZHlBLkdldExvY2FsUG9pbnQoYW5jaG9yLCB0aGlzLmxvY2FsQW5jaG9yQSk7XHJcbiAgICB0aGlzLmJvZHlCLkdldExvY2FsUG9pbnQoYW5jaG9yLCB0aGlzLmxvY2FsQW5jaG9yQik7XHJcbiAgICB0aGlzLnJlZmVyZW5jZUFuZ2xlID0gdGhpcy5ib2R5Qi5HZXRBbmdsZSgpIC0gdGhpcy5ib2R5QS5HZXRBbmdsZSgpO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyUmV2b2x1dGVKb2ludCBleHRlbmRzIGIySm9pbnQge1xyXG4gIC8vIFNvbHZlciBzaGFyZWRcclxuICByZWFkb25seSBtX2xvY2FsQW5jaG9yQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9pbXB1bHNlOiBiMlZlYzMgPSBuZXcgYjJWZWMzKCk7XHJcbiAgbV9tb3RvckltcHVsc2UgPSAwO1xyXG5cclxuICBtX2VuYWJsZU1vdG9yID0gZmFsc2U7XHJcbiAgbV9tYXhNb3RvclRvcnF1ZSA9IDA7XHJcbiAgbV9tb3RvclNwZWVkID0gMDtcclxuXHJcbiAgbV9lbmFibGVMaW1pdCA9IGZhbHNlO1xyXG4gIG1fcmVmZXJlbmNlQW5nbGUgPSAwO1xyXG4gIG1fbG93ZXJBbmdsZSA9IDA7XHJcbiAgbV91cHBlckFuZ2xlID0gMDtcclxuXHJcbiAgLy8gU29sdmVyIHRlbXBcclxuICBtX2luZGV4QSA9IDA7XHJcbiAgbV9pbmRleEIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbENlbnRlckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsQ2VudGVyQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1faW52TWFzc0EgPSAwO1xyXG4gIG1faW52TWFzc0IgPSAwO1xyXG4gIG1faW52SUEgPSAwO1xyXG4gIG1faW52SUIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fbWFzczogYjJNYXQzMyA9IG5ldyBiMk1hdDMzKCk7IC8vIGVmZmVjdGl2ZSBtYXNzIGZvciBwb2ludC10by1wb2ludCBjb25zdHJhaW50LlxyXG4gIG1fbW90b3JNYXNzID0gMDsgLy8gZWZmZWN0aXZlIG1hc3MgZm9yIG1vdG9yL2xpbWl0IGFuZ3VsYXIgY29uc3RyYWludC5cclxuICBtX2xpbWl0U3RhdGU6IGIyTGltaXRTdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQ7XHJcblxyXG4gIHJlYWRvbmx5IG1fcUE6IGIyUm90ID0gbmV3IGIyUm90KCk7XHJcbiAgcmVhZG9ubHkgbV9xQjogYjJSb3QgPSBuZXcgYjJSb3QoKTtcclxuICByZWFkb25seSBtX2xhbGNBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sYWxjQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fSzogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcblxyXG4gIGNvbnN0cnVjdG9yKGRlZjogYjJJUmV2b2x1dGVKb2ludERlZikge1xyXG4gICAgc3VwZXIoZGVmKTtcclxuXHJcbiAgICB0aGlzLm1fbG9jYWxBbmNob3JBLkNvcHkoYjJNYXliZShkZWYubG9jYWxBbmNob3JBLCBiMlZlYzIuWkVSTykpO1xyXG4gICAgdGhpcy5tX2xvY2FsQW5jaG9yQi5Db3B5KGIyTWF5YmUoZGVmLmxvY2FsQW5jaG9yQiwgYjJWZWMyLlpFUk8pKTtcclxuICAgIHRoaXMubV9yZWZlcmVuY2VBbmdsZSA9IGIyTWF5YmUoZGVmLnJlZmVyZW5jZUFuZ2xlLCAwKTtcclxuXHJcbiAgICB0aGlzLm1faW1wdWxzZS5TZXRaZXJvKCk7XHJcbiAgICB0aGlzLm1fbW90b3JJbXB1bHNlID0gMDtcclxuXHJcbiAgICB0aGlzLm1fbG93ZXJBbmdsZSA9IGIyTWF5YmUoZGVmLmxvd2VyQW5nbGUsIDApO1xyXG4gICAgdGhpcy5tX3VwcGVyQW5nbGUgPSBiMk1heWJlKGRlZi51cHBlckFuZ2xlLCAwKTtcclxuICAgIHRoaXMubV9tYXhNb3RvclRvcnF1ZSA9IGIyTWF5YmUoZGVmLm1heE1vdG9yVG9ycXVlLCAwKTtcclxuICAgIHRoaXMubV9tb3RvclNwZWVkID0gYjJNYXliZShkZWYubW90b3JTcGVlZCwgMCk7XHJcbiAgICB0aGlzLm1fZW5hYmxlTGltaXQgPSBiMk1heWJlKGRlZi5lbmFibGVMaW1pdCwgZmFsc2UpO1xyXG4gICAgdGhpcy5tX2VuYWJsZU1vdG9yID0gYjJNYXliZShkZWYuZW5hYmxlTW90b3IsIGZhbHNlKTtcclxuICAgIHRoaXMubV9saW1pdFN0YXRlID0gYjJMaW1pdFN0YXRlLmVfaW5hY3RpdmVMaW1pdDtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIEluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICB0aGlzLm1faW5kZXhBID0gdGhpcy5tX2JvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhCID0gdGhpcy5tX2JvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1fbG9jYWxDZW50ZXJBLkNvcHkodGhpcy5tX2JvZHlBLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX2xvY2FsQ2VudGVyQi5Db3B5KHRoaXMubV9ib2R5Qi5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMubV9pbnZNYXNzQSA9IHRoaXMubV9ib2R5QS5tX2ludk1hc3M7XHJcbiAgICB0aGlzLm1faW52TWFzc0IgPSB0aGlzLm1fYm9keUIubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX2ludklBID0gdGhpcy5tX2JvZHlBLm1faW52STtcclxuICAgIHRoaXMubV9pbnZJQiA9IHRoaXMubV9ib2R5Qi5tX2ludkk7XHJcblxyXG4gICAgY29uc3QgYUE6IG51bWJlciA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmE7XHJcbiAgICBjb25zdCB2QTogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnY7XHJcbiAgICBsZXQgd0E6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53O1xyXG5cclxuICAgIGNvbnN0IGFCOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hO1xyXG4gICAgY29uc3QgdkI6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52O1xyXG4gICAgbGV0IHdCOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udztcclxuXHJcbiAgICAvLyBiMlJvdCBxQShhQSksIHFCKGFCKTtcclxuICAgIGNvbnN0IHFBOiBiMlJvdCA9IHRoaXMubV9xQS5TZXRBbmdsZShhQSksXHJcbiAgICAgIHFCOiBiMlJvdCA9IHRoaXMubV9xQi5TZXRBbmdsZShhQik7XHJcblxyXG4gICAgLy8gbV9yQSA9IGIyTXVsKHFBLCBtX2xvY2FsQW5jaG9yQSAtIG1fbG9jYWxDZW50ZXJBKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JBLCB0aGlzLm1fbG9jYWxDZW50ZXJBLCB0aGlzLm1fbGFsY0EpO1xyXG4gICAgYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sYWxjQSwgdGhpcy5tX3JBKTtcclxuICAgIC8vIG1fckIgPSBiMk11bChxQiwgbV9sb2NhbEFuY2hvckIgLSBtX2xvY2FsQ2VudGVyQik7XHJcbiAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2xvY2FsQW5jaG9yQiwgdGhpcy5tX2xvY2FsQ2VudGVyQiwgdGhpcy5tX2xhbGNCKTtcclxuICAgIGIyUm90Lk11bFJWKHFCLCB0aGlzLm1fbGFsY0IsIHRoaXMubV9yQik7XHJcblxyXG4gICAgLy8gSiA9IFstSSAtcjFfc2tldyBJIHIyX3NrZXddXHJcbiAgICAvLyAgICAgWyAwICAgICAgIC0xIDAgICAgICAgMV1cclxuICAgIC8vIHJfc2tldyA9IFstcnk7IHJ4XVxyXG5cclxuICAgIC8vIE1hdGxhYlxyXG4gICAgLy8gSyA9IFsgbUErcjF5XjIqaUErbUIrcjJ5XjIqaUIsICAtcjF5KmlBKnIxeC1yMnkqaUIqcjJ4LCAgICAgICAgICAtcjF5KmlBLXIyeSppQl1cclxuICAgIC8vICAgICBbICAtcjF5KmlBKnIxeC1yMnkqaUIqcjJ4LCBtQStyMXheMippQSttQityMnheMippQiwgICAgICAgICAgIHIxeCppQStyMngqaUJdXHJcbiAgICAvLyAgICAgWyAgICAgICAgICAtcjF5KmlBLXIyeSppQiwgICAgICAgICAgIHIxeCppQStyMngqaUIsICAgICAgICAgICAgICAgICAgIGlBK2lCXVxyXG5cclxuICAgIGNvbnN0IG1BOiBudW1iZXIgPSB0aGlzLm1faW52TWFzc0EsXHJcbiAgICAgIG1COiBudW1iZXIgPSB0aGlzLm1faW52TWFzc0I7XHJcbiAgICBjb25zdCBpQTogbnVtYmVyID0gdGhpcy5tX2ludklBLFxyXG4gICAgICBpQjogbnVtYmVyID0gdGhpcy5tX2ludklCO1xyXG5cclxuICAgIGNvbnN0IGZpeGVkUm90YXRpb246IGJvb2xlYW4gPSBpQSArIGlCID09PSAwO1xyXG5cclxuICAgIHRoaXMubV9tYXNzLmV4LnggPSBtQSArIG1CICsgdGhpcy5tX3JBLnkgKiB0aGlzLm1fckEueSAqIGlBICsgdGhpcy5tX3JCLnkgKiB0aGlzLm1fckIueSAqIGlCO1xyXG4gICAgdGhpcy5tX21hc3MuZXkueCA9IC10aGlzLm1fckEueSAqIHRoaXMubV9yQS54ICogaUEgLSB0aGlzLm1fckIueSAqIHRoaXMubV9yQi54ICogaUI7XHJcbiAgICB0aGlzLm1fbWFzcy5lei54ID0gLXRoaXMubV9yQS55ICogaUEgLSB0aGlzLm1fckIueSAqIGlCO1xyXG4gICAgdGhpcy5tX21hc3MuZXgueSA9IHRoaXMubV9tYXNzLmV5Lng7XHJcbiAgICB0aGlzLm1fbWFzcy5leS55ID0gbUEgKyBtQiArIHRoaXMubV9yQS54ICogdGhpcy5tX3JBLnggKiBpQSArIHRoaXMubV9yQi54ICogdGhpcy5tX3JCLnggKiBpQjtcclxuICAgIHRoaXMubV9tYXNzLmV6LnkgPSB0aGlzLm1fckEueCAqIGlBICsgdGhpcy5tX3JCLnggKiBpQjtcclxuICAgIHRoaXMubV9tYXNzLmV4LnogPSB0aGlzLm1fbWFzcy5lei54O1xyXG4gICAgdGhpcy5tX21hc3MuZXkueiA9IHRoaXMubV9tYXNzLmV6Lnk7XHJcbiAgICB0aGlzLm1fbWFzcy5lei56ID0gaUEgKyBpQjtcclxuXHJcbiAgICB0aGlzLm1fbW90b3JNYXNzID0gaUEgKyBpQjtcclxuICAgIGlmICh0aGlzLm1fbW90b3JNYXNzID4gMCkge1xyXG4gICAgICB0aGlzLm1fbW90b3JNYXNzID0gMSAvIHRoaXMubV9tb3Rvck1hc3M7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKCF0aGlzLm1fZW5hYmxlTW90b3IgfHwgZml4ZWRSb3RhdGlvbikge1xyXG4gICAgICB0aGlzLm1fbW90b3JJbXB1bHNlID0gMDtcclxuICAgIH1cclxuXHJcbiAgICBpZiAodGhpcy5tX2VuYWJsZUxpbWl0ICYmICFmaXhlZFJvdGF0aW9uKSB7XHJcbiAgICAgIGNvbnN0IGpvaW50QW5nbGU6IG51bWJlciA9IGFCIC0gYUEgLSB0aGlzLm1fcmVmZXJlbmNlQW5nbGU7XHJcbiAgICAgIGlmIChiMkFicyh0aGlzLm1fdXBwZXJBbmdsZSAtIHRoaXMubV9sb3dlckFuZ2xlKSA8IDIgKiBiMl9hbmd1bGFyU2xvcCkge1xyXG4gICAgICAgIHRoaXMubV9saW1pdFN0YXRlID0gYjJMaW1pdFN0YXRlLmVfZXF1YWxMaW1pdHM7XHJcbiAgICAgIH0gZWxzZSBpZiAoam9pbnRBbmdsZSA8PSB0aGlzLm1fbG93ZXJBbmdsZSkge1xyXG4gICAgICAgIGlmICh0aGlzLm1fbGltaXRTdGF0ZSAhPT0gYjJMaW1pdFN0YXRlLmVfYXRMb3dlckxpbWl0KSB7XHJcbiAgICAgICAgICB0aGlzLm1faW1wdWxzZS56ID0gMDtcclxuICAgICAgICB9XHJcbiAgICAgICAgdGhpcy5tX2xpbWl0U3RhdGUgPSBiMkxpbWl0U3RhdGUuZV9hdExvd2VyTGltaXQ7XHJcbiAgICAgIH0gZWxzZSBpZiAoam9pbnRBbmdsZSA+PSB0aGlzLm1fdXBwZXJBbmdsZSkge1xyXG4gICAgICAgIGlmICh0aGlzLm1fbGltaXRTdGF0ZSAhPT0gYjJMaW1pdFN0YXRlLmVfYXRVcHBlckxpbWl0KSB7XHJcbiAgICAgICAgICB0aGlzLm1faW1wdWxzZS56ID0gMDtcclxuICAgICAgICB9XHJcbiAgICAgICAgdGhpcy5tX2xpbWl0U3RhdGUgPSBiMkxpbWl0U3RhdGUuZV9hdFVwcGVyTGltaXQ7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgdGhpcy5tX2xpbWl0U3RhdGUgPSBiMkxpbWl0U3RhdGUuZV9pbmFjdGl2ZUxpbWl0O1xyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLnogPSAwO1xyXG4gICAgICB9XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1fbGltaXRTdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQ7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGRhdGEuc3RlcC53YXJtU3RhcnRpbmcpIHtcclxuICAgICAgLy8gU2NhbGUgaW1wdWxzZXMgdG8gc3VwcG9ydCBhIHZhcmlhYmxlIHRpbWUgc3RlcC5cclxuICAgICAgdGhpcy5tX2ltcHVsc2UuU2VsZk11bChkYXRhLnN0ZXAuZHRSYXRpbyk7XHJcbiAgICAgIHRoaXMubV9tb3RvckltcHVsc2UgKj0gZGF0YS5zdGVwLmR0UmF0aW87XHJcblxyXG4gICAgICAvLyBiMlZlYzIgUChtX2ltcHVsc2UueCwgbV9pbXB1bHNlLnkpO1xyXG4gICAgICBjb25zdCBQOiBiMlZlYzIgPSBiMlJldm9sdXRlSm9pbnQuSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19QLlNldChcclxuICAgICAgICB0aGlzLm1faW1wdWxzZS54LFxyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLnksXHJcbiAgICAgICk7XHJcblxyXG4gICAgICAvLyB2QSAtPSBtQSAqIFA7XHJcbiAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgICB3QSAtPSBpQSAqIChiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckEsIFApICsgdGhpcy5tX21vdG9ySW1wdWxzZSArIHRoaXMubV9pbXB1bHNlLnopO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgd0IgKz0gaUIgKiAoYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JCLCBQKSArIHRoaXMubV9tb3RvckltcHVsc2UgKyB0aGlzLm1faW1wdWxzZS56KTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9pbXB1bHNlLlNldFplcm8oKTtcclxuICAgICAgdGhpcy5tX21vdG9ySW1wdWxzZSA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnYgPSB2QTtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53ID0gd0E7XHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udiA9IHZCO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLncgPSB3QjtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1A6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19DZG90X3YyOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfQ2RvdDE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19pbXB1bHNlX3YzOiBiMlZlYzMgPSBuZXcgYjJWZWMzKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfcmVkdWNlZF92MjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX2ltcHVsc2VfdjI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzKGRhdGE6IGIyU29sdmVyRGF0YSk6IHZvaWQge1xyXG4gICAgY29uc3QgdkE6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52O1xyXG4gICAgbGV0IHdBOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udztcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgbUI6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQjtcclxuICAgIGNvbnN0IGlBOiBudW1iZXIgPSB0aGlzLm1faW52SUEsXHJcbiAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgY29uc3QgZml4ZWRSb3RhdGlvbjogYm9vbGVhbiA9IGlBICsgaUIgPT09IDA7XHJcblxyXG4gICAgLy8gU29sdmUgbW90b3IgY29uc3RyYWludC5cclxuICAgIGlmICh0aGlzLm1fZW5hYmxlTW90b3IgJiYgdGhpcy5tX2xpbWl0U3RhdGUgIT09IGIyTGltaXRTdGF0ZS5lX2VxdWFsTGltaXRzICYmICFmaXhlZFJvdGF0aW9uKSB7XHJcbiAgICAgIGNvbnN0IENkb3Q6IG51bWJlciA9IHdCIC0gd0EgLSB0aGlzLm1fbW90b3JTcGVlZDtcclxuICAgICAgbGV0IGltcHVsc2U6IG51bWJlciA9IC10aGlzLm1fbW90b3JNYXNzICogQ2RvdDtcclxuICAgICAgY29uc3Qgb2xkSW1wdWxzZTogbnVtYmVyID0gdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuICAgICAgY29uc3QgbWF4SW1wdWxzZTogbnVtYmVyID0gZGF0YS5zdGVwLmR0ICogdGhpcy5tX21heE1vdG9yVG9ycXVlO1xyXG4gICAgICB0aGlzLm1fbW90b3JJbXB1bHNlID0gYjJDbGFtcCh0aGlzLm1fbW90b3JJbXB1bHNlICsgaW1wdWxzZSwgLW1heEltcHVsc2UsIG1heEltcHVsc2UpO1xyXG4gICAgICBpbXB1bHNlID0gdGhpcy5tX21vdG9ySW1wdWxzZSAtIG9sZEltcHVsc2U7XHJcblxyXG4gICAgICB3QSAtPSBpQSAqIGltcHVsc2U7XHJcbiAgICAgIHdCICs9IGlCICogaW1wdWxzZTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBTb2x2ZSBsaW1pdCBjb25zdHJhaW50LlxyXG4gICAgaWYgKFxyXG4gICAgICB0aGlzLm1fZW5hYmxlTGltaXQgJiZcclxuICAgICAgdGhpcy5tX2xpbWl0U3RhdGUgIT09IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQgJiZcclxuICAgICAgIWZpeGVkUm90YXRpb25cclxuICAgICkge1xyXG4gICAgICAvLyBiMlZlYzIgQ2RvdDEgPSB2QiArIGIyQ3Jvc3Mod0IsIG1fckIpIC0gdkEgLSBiMkNyb3NzKHdBLCBtX3JBKTtcclxuICAgICAgY29uc3QgQ2RvdDE6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihcclxuICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkIsIHdCLCB0aGlzLm1fckIsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkEsIHdBLCB0aGlzLm1fckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICBiMlJldm9sdXRlSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfQ2RvdDEsXHJcbiAgICAgICk7XHJcbiAgICAgIGNvbnN0IENkb3QyOiBudW1iZXIgPSB3QiAtIHdBO1xyXG4gICAgICAvLyBiMlZlYzMgQ2RvdChDZG90MS54LCBDZG90MS55LCBDZG90Mik7XHJcblxyXG4gICAgICAvLyBiMlZlYzMgaW1wdWxzZSA9IC10aGlzLm1fbWFzcy5Tb2x2ZTMzKENkb3QpO1xyXG4gICAgICBjb25zdCBpbXB1bHNlX3YzOiBiMlZlYzMgPSB0aGlzLm1fbWFzc1xyXG4gICAgICAgIC5Tb2x2ZTMzKENkb3QxLngsIENkb3QxLnksIENkb3QyLCBiMlJldm9sdXRlSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfaW1wdWxzZV92MylcclxuICAgICAgICAuU2VsZk5lZygpO1xyXG5cclxuICAgICAgaWYgKHRoaXMubV9saW1pdFN0YXRlID09PSBiMkxpbWl0U3RhdGUuZV9lcXVhbExpbWl0cykge1xyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLlNlbGZBZGQoaW1wdWxzZV92Myk7XHJcbiAgICAgIH0gZWxzZSBpZiAodGhpcy5tX2xpbWl0U3RhdGUgPT09IGIyTGltaXRTdGF0ZS5lX2F0TG93ZXJMaW1pdCkge1xyXG4gICAgICAgIGNvbnN0IG5ld0ltcHVsc2U6IG51bWJlciA9IHRoaXMubV9pbXB1bHNlLnogKyBpbXB1bHNlX3YzLno7XHJcbiAgICAgICAgaWYgKG5ld0ltcHVsc2UgPCAwKSB7XHJcbiAgICAgICAgICAvLyBiMlZlYzIgcmhzID0gLUNkb3QxICsgbV9pbXB1bHNlLnogKiBiMlZlYzIobV9tYXNzLmV6LngsIG1fbWFzcy5lei55KTtcclxuICAgICAgICAgIGNvbnN0IHJoc194ID0gLUNkb3QxLnggKyB0aGlzLm1faW1wdWxzZS56ICogdGhpcy5tX21hc3MuZXoueDtcclxuICAgICAgICAgIGNvbnN0IHJoc195ID0gLUNkb3QxLnkgKyB0aGlzLm1faW1wdWxzZS56ICogdGhpcy5tX21hc3MuZXoueTtcclxuICAgICAgICAgIGNvbnN0IHJlZHVjZWRfdjI6IGIyVmVjMiA9IHRoaXMubV9tYXNzLlNvbHZlMjIoXHJcbiAgICAgICAgICAgIHJoc194LFxyXG4gICAgICAgICAgICByaHNfeSxcclxuICAgICAgICAgICAgYjJSZXZvbHV0ZUpvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX3JlZHVjZWRfdjIsXHJcbiAgICAgICAgICApO1xyXG4gICAgICAgICAgaW1wdWxzZV92My54ID0gcmVkdWNlZF92Mi54O1xyXG4gICAgICAgICAgaW1wdWxzZV92My55ID0gcmVkdWNlZF92Mi55O1xyXG4gICAgICAgICAgaW1wdWxzZV92My56ID0gLXRoaXMubV9pbXB1bHNlLno7XHJcbiAgICAgICAgICB0aGlzLm1faW1wdWxzZS54ICs9IHJlZHVjZWRfdjIueDtcclxuICAgICAgICAgIHRoaXMubV9pbXB1bHNlLnkgKz0gcmVkdWNlZF92Mi55O1xyXG4gICAgICAgICAgdGhpcy5tX2ltcHVsc2UueiA9IDA7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHRoaXMubV9pbXB1bHNlLlNlbGZBZGQoaW1wdWxzZV92Myk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9IGVsc2UgaWYgKHRoaXMubV9saW1pdFN0YXRlID09PSBiMkxpbWl0U3RhdGUuZV9hdFVwcGVyTGltaXQpIHtcclxuICAgICAgICBjb25zdCBuZXdJbXB1bHNlOiBudW1iZXIgPSB0aGlzLm1faW1wdWxzZS56ICsgaW1wdWxzZV92My56O1xyXG4gICAgICAgIGlmIChuZXdJbXB1bHNlID4gMCkge1xyXG4gICAgICAgICAgLy8gYjJWZWMyIHJocyA9IC1DZG90MSArIG1faW1wdWxzZS56ICogYjJWZWMyKG1fbWFzcy5lei54LCBtX21hc3MuZXoueSk7XHJcbiAgICAgICAgICBjb25zdCByaHNfeCA9IC1DZG90MS54ICsgdGhpcy5tX2ltcHVsc2UueiAqIHRoaXMubV9tYXNzLmV6Lng7XHJcbiAgICAgICAgICBjb25zdCByaHNfeSA9IC1DZG90MS55ICsgdGhpcy5tX2ltcHVsc2UueiAqIHRoaXMubV9tYXNzLmV6Lnk7XHJcbiAgICAgICAgICBjb25zdCByZWR1Y2VkX3YyOiBiMlZlYzIgPSB0aGlzLm1fbWFzcy5Tb2x2ZTIyKFxyXG4gICAgICAgICAgICByaHNfeCxcclxuICAgICAgICAgICAgcmhzX3ksXHJcbiAgICAgICAgICAgIGIyUmV2b2x1dGVKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19yZWR1Y2VkX3YyLFxyXG4gICAgICAgICAgKTtcclxuICAgICAgICAgIGltcHVsc2VfdjMueCA9IHJlZHVjZWRfdjIueDtcclxuICAgICAgICAgIGltcHVsc2VfdjMueSA9IHJlZHVjZWRfdjIueTtcclxuICAgICAgICAgIGltcHVsc2VfdjMueiA9IC10aGlzLm1faW1wdWxzZS56O1xyXG4gICAgICAgICAgdGhpcy5tX2ltcHVsc2UueCArPSByZWR1Y2VkX3YyLng7XHJcbiAgICAgICAgICB0aGlzLm1faW1wdWxzZS55ICs9IHJlZHVjZWRfdjIueTtcclxuICAgICAgICAgIHRoaXMubV9pbXB1bHNlLnogPSAwO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICB0aGlzLm1faW1wdWxzZS5TZWxmQWRkKGltcHVsc2VfdjMpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gYjJWZWMyIFAoaW1wdWxzZS54LCBpbXB1bHNlLnkpO1xyXG4gICAgICBjb25zdCBQOiBiMlZlYzIgPSBiMlJldm9sdXRlSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUC5TZXQoXHJcbiAgICAgICAgaW1wdWxzZV92My54LFxyXG4gICAgICAgIGltcHVsc2VfdjMueSxcclxuICAgICAgKTtcclxuXHJcbiAgICAgIC8vIHZBIC09IG1BICogUDtcclxuICAgICAgdkEuU2VsZk11bFN1YihtQSwgUCk7XHJcbiAgICAgIHdBIC09IGlBICogKGIyVmVjMi5Dcm9zc1ZWKHRoaXMubV9yQSwgUCkgKyBpbXB1bHNlX3YzLnopO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgd0IgKz0gaUIgKiAoYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JCLCBQKSArIGltcHVsc2VfdjMueik7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICAvLyBTb2x2ZSBwb2ludC10by1wb2ludCBjb25zdHJhaW50XHJcbiAgICAgIC8vIGIyVmVjMiBDZG90ID0gdkIgKyBiMkNyb3NzKHdCLCBtX3JCKSAtIHZBIC0gYjJDcm9zcyh3QSwgbV9yQSk7XHJcbiAgICAgIGNvbnN0IENkb3RfdjI6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihcclxuICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkIsIHdCLCB0aGlzLm1fckIsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkEsIHdBLCB0aGlzLm1fckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICBiMlJldm9sdXRlSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfQ2RvdF92MixcclxuICAgICAgKTtcclxuICAgICAgLy8gYjJWZWMyIGltcHVsc2UgPSBtX21hc3MuU29sdmUyMigtQ2RvdCk7XHJcbiAgICAgIGNvbnN0IGltcHVsc2VfdjI6IGIyVmVjMiA9IHRoaXMubV9tYXNzLlNvbHZlMjIoXHJcbiAgICAgICAgLUNkb3RfdjIueCxcclxuICAgICAgICAtQ2RvdF92Mi55LFxyXG4gICAgICAgIGIyUmV2b2x1dGVKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19pbXB1bHNlX3YyLFxyXG4gICAgICApO1xyXG5cclxuICAgICAgdGhpcy5tX2ltcHVsc2UueCArPSBpbXB1bHNlX3YyLng7XHJcbiAgICAgIHRoaXMubV9pbXB1bHNlLnkgKz0gaW1wdWxzZV92Mi55O1xyXG5cclxuICAgICAgLy8gdkEgLT0gbUEgKiBpbXB1bHNlO1xyXG4gICAgICB2QS5TZWxmTXVsU3ViKG1BLCBpbXB1bHNlX3YyKTtcclxuICAgICAgd0EgLT0gaUEgKiBiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckEsIGltcHVsc2VfdjIpO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBpbXB1bHNlO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBpbXB1bHNlX3YyKTtcclxuICAgICAgd0IgKz0gaUIgKiBiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckIsIGltcHVsc2VfdjIpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52ID0gdkE7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udyA9IHdBO1xyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnYgPSB2QjtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS53ID0gd0I7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19DX3YyID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX2ltcHVsc2UgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlUG9zaXRpb25Db25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiBib29sZWFuIHtcclxuICAgIGNvbnN0IGNBOiBiMlZlYzIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5jO1xyXG4gICAgbGV0IGFBOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5hO1xyXG4gICAgY29uc3QgY0I6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBsZXQgYUI6IG51bWJlciA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmE7XHJcblxyXG4gICAgLy8gYjJSb3QgcUEoYUEpLCBxQihhQik7XHJcbiAgICBjb25zdCBxQTogYjJSb3QgPSB0aGlzLm1fcUEuU2V0QW5nbGUoYUEpLFxyXG4gICAgICBxQjogYjJSb3QgPSB0aGlzLm1fcUIuU2V0QW5nbGUoYUIpO1xyXG5cclxuICAgIGxldCBhbmd1bGFyRXJyb3IgPSAwO1xyXG4gICAgbGV0IHBvc2l0aW9uRXJyb3IgPSAwO1xyXG5cclxuICAgIGNvbnN0IGZpeGVkUm90YXRpb246IGJvb2xlYW4gPSB0aGlzLm1faW52SUEgKyB0aGlzLm1faW52SUIgPT09IDA7XHJcblxyXG4gICAgLy8gU29sdmUgYW5ndWxhciBsaW1pdCBjb25zdHJhaW50LlxyXG4gICAgaWYgKFxyXG4gICAgICB0aGlzLm1fZW5hYmxlTGltaXQgJiZcclxuICAgICAgdGhpcy5tX2xpbWl0U3RhdGUgIT09IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQgJiZcclxuICAgICAgIWZpeGVkUm90YXRpb25cclxuICAgICkge1xyXG4gICAgICBjb25zdCBhbmdsZTogbnVtYmVyID0gYUIgLSBhQSAtIHRoaXMubV9yZWZlcmVuY2VBbmdsZTtcclxuICAgICAgbGV0IGxpbWl0SW1wdWxzZSA9IDA7XHJcblxyXG4gICAgICBpZiAodGhpcy5tX2xpbWl0U3RhdGUgPT09IGIyTGltaXRTdGF0ZS5lX2VxdWFsTGltaXRzKSB7XHJcbiAgICAgICAgLy8gUHJldmVudCBsYXJnZSBhbmd1bGFyIGNvcnJlY3Rpb25zXHJcbiAgICAgICAgY29uc3QgQzogbnVtYmVyID0gYjJDbGFtcChcclxuICAgICAgICAgIGFuZ2xlIC0gdGhpcy5tX2xvd2VyQW5nbGUsXHJcbiAgICAgICAgICAtYjJfbWF4QW5ndWxhckNvcnJlY3Rpb24sXHJcbiAgICAgICAgICBiMl9tYXhBbmd1bGFyQ29ycmVjdGlvbixcclxuICAgICAgICApO1xyXG4gICAgICAgIGxpbWl0SW1wdWxzZSA9IC10aGlzLm1fbW90b3JNYXNzICogQztcclxuICAgICAgICBhbmd1bGFyRXJyb3IgPSBiMkFicyhDKTtcclxuICAgICAgfSBlbHNlIGlmICh0aGlzLm1fbGltaXRTdGF0ZSA9PT0gYjJMaW1pdFN0YXRlLmVfYXRMb3dlckxpbWl0KSB7XHJcbiAgICAgICAgbGV0IEM6IG51bWJlciA9IGFuZ2xlIC0gdGhpcy5tX2xvd2VyQW5nbGU7XHJcbiAgICAgICAgYW5ndWxhckVycm9yID0gLUM7XHJcblxyXG4gICAgICAgIC8vIFByZXZlbnQgbGFyZ2UgYW5ndWxhciBjb3JyZWN0aW9ucyBhbmQgYWxsb3cgc29tZSBzbG9wLlxyXG4gICAgICAgIEMgPSBiMkNsYW1wKEMgKyBiMl9hbmd1bGFyU2xvcCwgLWIyX21heEFuZ3VsYXJDb3JyZWN0aW9uLCAwKTtcclxuICAgICAgICBsaW1pdEltcHVsc2UgPSAtdGhpcy5tX21vdG9yTWFzcyAqIEM7XHJcbiAgICAgIH0gZWxzZSBpZiAodGhpcy5tX2xpbWl0U3RhdGUgPT09IGIyTGltaXRTdGF0ZS5lX2F0VXBwZXJMaW1pdCkge1xyXG4gICAgICAgIGxldCBDOiBudW1iZXIgPSBhbmdsZSAtIHRoaXMubV91cHBlckFuZ2xlO1xyXG4gICAgICAgIGFuZ3VsYXJFcnJvciA9IEM7XHJcblxyXG4gICAgICAgIC8vIFByZXZlbnQgbGFyZ2UgYW5ndWxhciBjb3JyZWN0aW9ucyBhbmQgYWxsb3cgc29tZSBzbG9wLlxyXG4gICAgICAgIEMgPSBiMkNsYW1wKEMgLSBiMl9hbmd1bGFyU2xvcCwgMCwgYjJfbWF4QW5ndWxhckNvcnJlY3Rpb24pO1xyXG4gICAgICAgIGxpbWl0SW1wdWxzZSA9IC10aGlzLm1fbW90b3JNYXNzICogQztcclxuICAgICAgfVxyXG5cclxuICAgICAgYUEgLT0gdGhpcy5tX2ludklBICogbGltaXRJbXB1bHNlO1xyXG4gICAgICBhQiArPSB0aGlzLm1faW52SUIgKiBsaW1pdEltcHVsc2U7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gU29sdmUgcG9pbnQtdG8tcG9pbnQgY29uc3RyYWludC5cclxuICAgIHtcclxuICAgICAgcUEuU2V0QW5nbGUoYUEpO1xyXG4gICAgICBxQi5TZXRBbmdsZShhQik7XHJcbiAgICAgIC8vIGIyVmVjMiByQSA9IGIyTXVsKHFBLCBtX2xvY2FsQW5jaG9yQSAtIG1fbG9jYWxDZW50ZXJBKTtcclxuICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckEsIHRoaXMubV9sb2NhbENlbnRlckEsIHRoaXMubV9sYWxjQSk7XHJcbiAgICAgIGNvbnN0IHJBOiBiMlZlYzIgPSBiMlJvdC5NdWxSVihxQSwgdGhpcy5tX2xhbGNBLCB0aGlzLm1fckEpO1xyXG4gICAgICAvLyBiMlZlYzIgckIgPSBiMk11bChxQiwgbV9sb2NhbEFuY2hvckIgLSBtX2xvY2FsQ2VudGVyQik7XHJcbiAgICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgICBjb25zdCByQjogYjJWZWMyID0gYjJSb3QuTXVsUlYocUIsIHRoaXMubV9sYWxjQiwgdGhpcy5tX3JCKTtcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBDID0gY0IgKyByQiAtIGNBIC0gckE7XHJcbiAgICAgIGNvbnN0IENfdjIgPSBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgYjJWZWMyLkFkZFZWKGNCLCByQiwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgIGIyVmVjMi5BZGRWVihjQSwgckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICBiMlJldm9sdXRlSm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfQ192MixcclxuICAgICAgKTtcclxuICAgICAgLy8gcG9zaXRpb25FcnJvciA9IEMuTGVuZ3RoKCk7XHJcbiAgICAgIHBvc2l0aW9uRXJyb3IgPSBDX3YyLkxlbmd0aCgpO1xyXG5cclxuICAgICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgICBtQjogbnVtYmVyID0gdGhpcy5tX2ludk1hc3NCO1xyXG4gICAgICBjb25zdCBpQTogbnVtYmVyID0gdGhpcy5tX2ludklBLFxyXG4gICAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgICBjb25zdCBLOiBiMk1hdDIyID0gdGhpcy5tX0s7XHJcbiAgICAgIEsuZXgueCA9IG1BICsgbUIgKyBpQSAqIHJBLnkgKiByQS55ICsgaUIgKiByQi55ICogckIueTtcclxuICAgICAgSy5leC55ID0gLWlBICogckEueCAqIHJBLnkgLSBpQiAqIHJCLnggKiByQi55O1xyXG4gICAgICBLLmV5LnggPSBLLmV4Lnk7XHJcbiAgICAgIEsuZXkueSA9IG1BICsgbUIgKyBpQSAqIHJBLnggKiByQS54ICsgaUIgKiByQi54ICogckIueDtcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBpbXB1bHNlID0gLUsuU29sdmUoQyk7XHJcbiAgICAgIGNvbnN0IGltcHVsc2U6IGIyVmVjMiA9IEsuU29sdmUoXHJcbiAgICAgICAgQ192Mi54LFxyXG4gICAgICAgIENfdjIueSxcclxuICAgICAgICBiMlJldm9sdXRlSm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfaW1wdWxzZSxcclxuICAgICAgKS5TZWxmTmVnKCk7XHJcblxyXG4gICAgICAvLyBjQSAtPSBtQSAqIGltcHVsc2U7XHJcbiAgICAgIGNBLlNlbGZNdWxTdWIobUEsIGltcHVsc2UpO1xyXG4gICAgICBhQSAtPSBpQSAqIGIyVmVjMi5Dcm9zc1ZWKHJBLCBpbXB1bHNlKTtcclxuXHJcbiAgICAgIC8vIGNCICs9IG1CICogaW1wdWxzZTtcclxuICAgICAgY0IuU2VsZk11bEFkZChtQiwgaW1wdWxzZSk7XHJcbiAgICAgIGFCICs9IGlCICogYjJWZWMyLkNyb3NzVlYockIsIGltcHVsc2UpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmMgPSBjQTtcclxuICAgIGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmEgPSBhQTtcclxuICAgIC8vIGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmMgPSBjQjtcclxuICAgIGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmEgPSBhQjtcclxuXHJcbiAgICByZXR1cm4gcG9zaXRpb25FcnJvciA8PSBiMl9saW5lYXJTbG9wICYmIGFuZ3VsYXJFcnJvciA8PSBiMl9hbmd1bGFyU2xvcDtcclxuICB9XHJcblxyXG4gIEdldEFuY2hvckE8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUEuR2V0V29ybGRQb2ludCh0aGlzLm1fbG9jYWxBbmNob3JBLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQjxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5Qi5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckIsIG91dCk7XHJcbiAgfVxyXG5cclxuICBHZXRSZWFjdGlvbkZvcmNlPFQgZXh0ZW5kcyBYWT4oaW52X2R0OiBudW1iZXIsIG91dDogVCk6IFQge1xyXG4gICAgLy8gYjJWZWMyIFAodGhpcy5tX2ltcHVsc2UueCwgdGhpcy5tX2ltcHVsc2UueSk7XHJcbiAgICAvLyByZXR1cm4gaW52X2R0ICogUDtcclxuICAgIG91dC54ID0gaW52X2R0ICogdGhpcy5tX2ltcHVsc2UueDtcclxuICAgIG91dC55ID0gaW52X2R0ICogdGhpcy5tX2ltcHVsc2UueTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBHZXRSZWFjdGlvblRvcnF1ZShpbnZfZHQ6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICByZXR1cm4gaW52X2R0ICogdGhpcy5tX2ltcHVsc2UuejtcclxuICB9XHJcblxyXG4gIEdldExvY2FsQW5jaG9yQSgpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxBbmNob3JBO1xyXG4gIH1cclxuXHJcbiAgR2V0TG9jYWxBbmNob3JCKCk6IFJlYWRvbmx5PGIyVmVjMj4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9sb2NhbEFuY2hvckI7XHJcbiAgfVxyXG5cclxuICBHZXRSZWZlcmVuY2VBbmdsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9yZWZlcmVuY2VBbmdsZTtcclxuICB9XHJcblxyXG4gIEdldEpvaW50QW5nbGUoKTogbnVtYmVyIHtcclxuICAgIC8vIGIyQm9keSogYkEgPSB0aGlzLm1fYm9keUE7XHJcbiAgICAvLyBiMkJvZHkqIGJCID0gdGhpcy5tX2JvZHlCO1xyXG4gICAgLy8gcmV0dXJuIGJCLT50aGlzLm1fc3dlZXAuYSAtIGJBLT50aGlzLm1fc3dlZXAuYSAtIHRoaXMubV9yZWZlcmVuY2VBbmdsZTtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUIubV9zd2VlcC5hIC0gdGhpcy5tX2JvZHlBLm1fc3dlZXAuYSAtIHRoaXMubV9yZWZlcmVuY2VBbmdsZTtcclxuICB9XHJcblxyXG4gIEdldEpvaW50U3BlZWQoKTogbnVtYmVyIHtcclxuICAgIC8vIGIyQm9keSogYkEgPSB0aGlzLm1fYm9keUE7XHJcbiAgICAvLyBiMkJvZHkqIGJCID0gdGhpcy5tX2JvZHlCO1xyXG4gICAgLy8gcmV0dXJuIGJCLT50aGlzLm1fYW5ndWxhclZlbG9jaXR5IC0gYkEtPnRoaXMubV9hbmd1bGFyVmVsb2NpdHk7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHlCLm1fYW5ndWxhclZlbG9jaXR5IC0gdGhpcy5tX2JvZHlBLm1fYW5ndWxhclZlbG9jaXR5O1xyXG4gIH1cclxuXHJcbiAgSXNNb3RvckVuYWJsZWQoKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2VuYWJsZU1vdG9yO1xyXG4gIH1cclxuXHJcbiAgRW5hYmxlTW90b3IoZmxhZzogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgaWYgKGZsYWcgIT09IHRoaXMubV9lbmFibGVNb3Rvcikge1xyXG4gICAgICB0aGlzLm1fYm9keUEuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9ib2R5Qi5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX2VuYWJsZU1vdG9yID0gZmxhZztcclxuICAgIH1cclxuICB9XHJcblxyXG4gIEdldE1vdG9yVG9ycXVlKGludl9kdDogbnVtYmVyKTogbnVtYmVyIHtcclxuICAgIHJldHVybiBpbnZfZHQgKiB0aGlzLm1fbW90b3JJbXB1bHNlO1xyXG4gIH1cclxuXHJcbiAgR2V0TW90b3JTcGVlZCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9tb3RvclNwZWVkO1xyXG4gIH1cclxuXHJcbiAgU2V0TWF4TW90b3JUb3JxdWUodG9ycXVlOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGlmICh0b3JxdWUgIT09IHRoaXMubV9tYXhNb3RvclRvcnF1ZSkge1xyXG4gICAgICB0aGlzLm1fYm9keUEuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9ib2R5Qi5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX21heE1vdG9yVG9ycXVlID0gdG9ycXVlO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgR2V0TWF4TW90b3JUb3JxdWUoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fbWF4TW90b3JUb3JxdWU7XHJcbiAgfVxyXG5cclxuICBJc0xpbWl0RW5hYmxlZCgpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fZW5hYmxlTGltaXQ7XHJcbiAgfVxyXG5cclxuICBFbmFibGVMaW1pdChmbGFnOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICBpZiAoZmxhZyAhPT0gdGhpcy5tX2VuYWJsZUxpbWl0KSB7XHJcbiAgICAgIHRoaXMubV9ib2R5QS5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX2JvZHlCLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fZW5hYmxlTGltaXQgPSBmbGFnO1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS56ID0gMDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIEdldExvd2VyTGltaXQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fbG93ZXJBbmdsZTtcclxuICB9XHJcblxyXG4gIEdldFVwcGVyTGltaXQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fdXBwZXJBbmdsZTtcclxuICB9XHJcblxyXG4gIFNldExpbWl0cyhsb3dlcjogbnVtYmVyLCB1cHBlcjogbnVtYmVyKTogdm9pZCB7XHJcbiAgICBpZiAobG93ZXIgIT09IHRoaXMubV9sb3dlckFuZ2xlIHx8IHVwcGVyICE9PSB0aGlzLm1fdXBwZXJBbmdsZSkge1xyXG4gICAgICB0aGlzLm1fYm9keUEuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9ib2R5Qi5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX2ltcHVsc2UueiA9IDA7XHJcbiAgICAgIHRoaXMubV9sb3dlckFuZ2xlID0gbG93ZXI7XHJcbiAgICAgIHRoaXMubV91cHBlckFuZ2xlID0gdXBwZXI7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTZXRNb3RvclNwZWVkKHNwZWVkOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGlmIChzcGVlZCAhPT0gdGhpcy5tX21vdG9yU3BlZWQpIHtcclxuICAgICAgdGhpcy5tX2JvZHlBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9tb3RvclNwZWVkID0gc3BlZWQ7XHJcbiAgICB9XHJcbiAgfVxyXG59XHJcbiJdfQ==