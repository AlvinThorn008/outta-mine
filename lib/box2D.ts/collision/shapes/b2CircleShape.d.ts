import { b2Transform, b2Vec2, XY } from '../../common/b2Math';
import { b2AABB, b2RayCastInput, b2RayCastOutput } from '../b2Collision';
import { b2DistanceProxy } from '../b2Distance';
import { b2MassData, b2Shape } from './b2Shape';
export declare class b2CircleShape extends b2Shape {
    readonly m_p: b2Vec2;
    constructor(radius?: number);
    Set(position: XY, radius?: number): this;
    Clone(): b2CircleShape;
    Copy(other: b2CircleShape): b2CircleShape;
    GetChildCount(): number;
    private static TestPoint_s_center;
    private static TestPoint_s_d;
    TestPoint(transform: b2Transform, p: XY): boolean;
    ComputeDistance(xf: b2Transform, p: b2Vec2, normal: b2Vec2, childIndex: number): number;
    private static RayCast_s_position;
    private static RayCast_s_s;
    private static RayCast_s_r;
    RayCast(output: b2RayCastOutput, input: b2RayCastInput, transform: b2Transform, childIndex: number): boolean;
    private static ComputeAABB_s_p;
    ComputeAABB(aabb: b2AABB, transform: b2Transform, childIndex: number): void;
    ComputeMass(massData: b2MassData, density: number): void;
    SetupDistanceProxy(proxy: b2DistanceProxy, index: number): void;
    ComputeSubmergedArea(normal: b2Vec2, offset: number, xf: b2Transform, c: b2Vec2): number;
}
