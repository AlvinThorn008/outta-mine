import { b2Sweep, b2Vec2 } from '../common/b2Math';
import { b2DistanceProxy, b2SimplexCache } from './b2Distance';
declare class TOIStats {
    time: number;
    maxTime: number;
    calls: number;
    iters: number;
    maxIters: number;
    rootIters: number;
    maxRootIters: number;
    constructor();
    Reset(): void;
}
export declare const b2_toiStats: TOIStats;
export declare class b2TOIInput {
    readonly proxyA: b2DistanceProxy;
    readonly proxyB: b2DistanceProxy;
    readonly sweepA: b2Sweep;
    readonly sweepB: b2Sweep;
    tMax: number;
    constructor();
}
export declare const enum b2TOIOutputState {
    e_unknown = 0,
    e_failed = 1,
    e_overlapped = 2,
    e_touching = 3,
    e_separated = 4
}
export declare class b2TOIOutput {
    state: b2TOIOutputState;
    t: number;
    constructor();
}
export declare const enum b2SeparationFunctionType {
    e_unknown = -1,
    e_points = 0,
    e_faceA = 1,
    e_faceB = 2
}
export declare class b2SeparationFunction {
    m_proxyA: b2DistanceProxy;
    m_proxyB: b2DistanceProxy;
    readonly m_sweepA: b2Sweep;
    readonly m_sweepB: b2Sweep;
    m_type: b2SeparationFunctionType;
    readonly m_localPoint: b2Vec2;
    readonly m_axis: b2Vec2;
    Initialize(cache: b2SimplexCache, proxyA: b2DistanceProxy, sweepA: b2Sweep, proxyB: b2DistanceProxy, sweepB: b2Sweep, t1: number): number;
    FindMinSeparation(indexA: [number], indexB: [number], t: number): number;
    Evaluate(indexA: number, indexB: number, t: number): number;
}
export declare function b2TimeOfImpact(output: b2TOIOutput, input: b2TOIInput): void;
export {};
