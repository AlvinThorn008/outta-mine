import { b2Transform } from '../common/b2Math';
import { b2Manifold } from './b2Collision';
import { b2PolygonShape } from './shapes/b2PolygonShape';
export declare function b2CollidePolygons(manifold: b2Manifold, polyA: b2PolygonShape, xfA: b2Transform, polyB: b2PolygonShape, xfB: b2Transform): void;
