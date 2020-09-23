import { b2Transform } from '../common/b2Math';
import { b2Manifold } from './b2Collision';
import { b2CircleShape } from './shapes/b2CircleShape';
import { b2PolygonShape } from './shapes/b2PolygonShape';
export declare function b2CollideCircles(manifold: b2Manifold, circleA: b2CircleShape, xfA: b2Transform, circleB: b2CircleShape, xfB: b2Transform): void;
export declare function b2CollidePolygonAndCircle(manifold: b2Manifold, polygonA: b2PolygonShape, xfA: b2Transform, circleB: b2CircleShape, xfB: b2Transform): void;
