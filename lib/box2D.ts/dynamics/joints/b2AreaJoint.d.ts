import { b2Vec2, XY } from '../../common/b2Math';
import { b2IJointDef, b2Joint, b2JointDef } from './b2Joint';
import { b2DistanceJoint } from './b2DistanceJoint';
import { b2SolverData } from '../b2TimeStep';
import { b2Body } from '../b2Body';
export interface b2IAreaJointDef extends b2IJointDef {
    bodies: b2Body[];
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class b2AreaJointDef extends b2JointDef implements b2IAreaJointDef {
    bodies: b2Body[];
    frequencyHz: number;
    dampingRatio: number;
    constructor();
    AddBody(body: b2Body): void;
}
export declare class b2AreaJoint extends b2Joint {
    m_bodies: b2Body[];
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_impulse: number;
    readonly m_targetLengths: number[];
    m_targetArea: number;
    readonly m_normals: b2Vec2[];
    readonly m_joints: b2DistanceJoint[];
    readonly m_deltas: b2Vec2[];
    readonly m_delta: b2Vec2;
    constructor(def: b2IAreaJointDef);
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    InitVelocityConstraints(data: b2SolverData): void;
    SolveVelocityConstraints(data: b2SolverData): void;
    SolvePositionConstraints(data: b2SolverData): boolean;
}
