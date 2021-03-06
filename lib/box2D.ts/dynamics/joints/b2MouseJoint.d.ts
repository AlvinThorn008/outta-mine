import { b2Mat22, b2Rot, b2Vec2, XY } from '../../common/b2Math';
import { b2IJointDef, b2Joint, b2JointDef } from './b2Joint';
import { b2SolverData } from '../b2TimeStep';
export interface b2IMouseJointDef extends b2IJointDef {
    target?: XY;
    maxForce?: number;
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class b2MouseJointDef extends b2JointDef implements b2IMouseJointDef {
    readonly target: b2Vec2;
    maxForce: number;
    frequencyHz: number;
    dampingRatio: number;
    constructor();
}
export declare class b2MouseJoint extends b2Joint {
    readonly m_localAnchorB: b2Vec2;
    readonly m_targetA: b2Vec2;
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_beta: number;
    readonly m_impulse: b2Vec2;
    m_maxForce: number;
    m_gamma: number;
    m_indexA: number;
    m_indexB: number;
    readonly m_rB: b2Vec2;
    readonly m_localCenterB: b2Vec2;
    m_invMassB: number;
    m_invIB: number;
    readonly m_mass: b2Mat22;
    readonly m_C: b2Vec2;
    readonly m_qB: b2Rot;
    readonly m_lalcB: b2Vec2;
    readonly m_K: b2Mat22;
    constructor(def: b2IMouseJointDef);
    SetTarget(target: b2Vec2): void;
    GetTarget(): b2Vec2;
    SetMaxForce(maxForce: number): void;
    GetMaxForce(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    InitVelocityConstraints(data: b2SolverData): void;
    private static SolveVelocityConstraints_s_Cdot;
    private static SolveVelocityConstraints_s_impulse;
    private static SolveVelocityConstraints_s_oldImpulse;
    SolveVelocityConstraints(data: b2SolverData): void;
    SolvePositionConstraints(data: b2SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    ShiftOrigin(newOrigin: b2Vec2): void;
}
