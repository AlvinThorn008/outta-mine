import { b2Assert } from '../common/b2Settings';
import { b2_angularSlop, b2_maxFloat, b2_maxManifoldPoints } from '../common/b2Settings';
import { b2Min, b2Rot, b2Transform, b2Vec2 } from '../common/b2Math';
import { b2ClipSegmentToLine, b2ClipVertex, b2ContactID, } from './b2Collision';
const b2CollideEdgeAndCircle_s_Q = new b2Vec2();
const b2CollideEdgeAndCircle_s_e = new b2Vec2();
const b2CollideEdgeAndCircle_s_d = new b2Vec2();
const b2CollideEdgeAndCircle_s_e1 = new b2Vec2();
const b2CollideEdgeAndCircle_s_e2 = new b2Vec2();
const b2CollideEdgeAndCircle_s_P = new b2Vec2();
const b2CollideEdgeAndCircle_s_n = new b2Vec2();
const b2CollideEdgeAndCircle_s_id = new b2ContactID();
export function b2CollideEdgeAndCircle(manifold, edgeA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    // Compute circle in frame of edge
    const Q = b2Transform.MulTXV(xfA, b2Transform.MulXV(xfB, circleB.m_p, b2Vec2.s_t0), b2CollideEdgeAndCircle_s_Q);
    const A = edgeA.m_vertex1;
    const B = edgeA.m_vertex2;
    const e = b2Vec2.SubVV(B, A, b2CollideEdgeAndCircle_s_e);
    // Barycentric coordinates
    const u = b2Vec2.DotVV(e, b2Vec2.SubVV(B, Q, b2Vec2.s_t0));
    const v = b2Vec2.DotVV(e, b2Vec2.SubVV(Q, A, b2Vec2.s_t0));
    const radius = edgeA.m_radius + circleB.m_radius;
    // const cf: b2ContactFeature = new b2ContactFeature();
    const id = b2CollideEdgeAndCircle_s_id;
    id.cf.indexB = 0;
    id.cf.typeB = 0 /* e_vertex */;
    // Region A
    if (v <= 0) {
        const P = A;
        const d = b2Vec2.SubVV(Q, P, b2CollideEdgeAndCircle_s_d);
        const dd = b2Vec2.DotVV(d, d);
        if (dd > radius * radius) {
            return;
        }
        // Is there an edge connected to A?
        if (edgeA.m_hasVertex0) {
            const A1 = edgeA.m_vertex0;
            const B1 = A;
            const e1 = b2Vec2.SubVV(B1, A1, b2CollideEdgeAndCircle_s_e1);
            const u1 = b2Vec2.DotVV(e1, b2Vec2.SubVV(B1, Q, b2Vec2.s_t0));
            // Is the circle in Region AB of the previous edge?
            if (u1 > 0) {
                return;
            }
        }
        id.cf.indexA = 0;
        id.cf.typeA = 0 /* e_vertex */;
        manifold.pointCount = 1;
        manifold.type = 0 /* e_circles */;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P);
        manifold.points[0].id.Copy(id);
        // manifold.points[0].id.key = 0;
        // manifold.points[0].id.cf = cf;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }
    // Region B
    if (u <= 0) {
        const P = B;
        const d = b2Vec2.SubVV(Q, P, b2CollideEdgeAndCircle_s_d);
        const dd = b2Vec2.DotVV(d, d);
        if (dd > radius * radius) {
            return;
        }
        // Is there an edge connected to B?
        if (edgeA.m_hasVertex3) {
            const B2 = edgeA.m_vertex3;
            const A2 = B;
            const e2 = b2Vec2.SubVV(B2, A2, b2CollideEdgeAndCircle_s_e2);
            const v2 = b2Vec2.DotVV(e2, b2Vec2.SubVV(Q, A2, b2Vec2.s_t0));
            // Is the circle in Region AB of the next edge?
            if (v2 > 0) {
                return;
            }
        }
        id.cf.indexA = 1;
        id.cf.typeA = 0 /* e_vertex */;
        manifold.pointCount = 1;
        manifold.type = 0 /* e_circles */;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P);
        manifold.points[0].id.Copy(id);
        // manifold.points[0].id.key = 0;
        // manifold.points[0].id.cf = cf;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }
    // Region AB
    const den = b2Vec2.DotVV(e, e);
    !!B2_DEBUG && b2Assert(den > 0);
    const P = b2CollideEdgeAndCircle_s_P;
    P.x = (1 / den) * (u * A.x + v * B.x);
    P.y = (1 / den) * (u * A.y + v * B.y);
    const d = b2Vec2.SubVV(Q, P, b2CollideEdgeAndCircle_s_d);
    const dd = b2Vec2.DotVV(d, d);
    if (dd > radius * radius) {
        return;
    }
    const n = b2CollideEdgeAndCircle_s_n.Set(-e.y, e.x);
    if (b2Vec2.DotVV(n, b2Vec2.SubVV(Q, A, b2Vec2.s_t0)) < 0) {
        n.Set(-n.x, -n.y);
    }
    n.Normalize();
    id.cf.indexA = 0;
    id.cf.typeA = 1 /* e_face */;
    manifold.pointCount = 1;
    manifold.type = 1 /* e_faceA */;
    manifold.localNormal.Copy(n);
    manifold.localPoint.Copy(A);
    manifold.points[0].id.Copy(id);
    // manifold.points[0].id.key = 0;
    // manifold.points[0].id.cf = cf;
    manifold.points[0].localPoint.Copy(circleB.m_p);
}
class b2EPAxis {
    constructor() {
        this.type = 0 /* e_unknown */;
        this.index = 0;
        this.separation = NaN;
        this.separation = 0.0;
    }
}
class b2TempPolygon {
    constructor() {
        this.vertices = [];
        this.normals = [];
        this.count = 0;
    }
}
class b2ReferenceFace {
    constructor() {
        this.i1 = 0;
        this.i2 = 0;
        this.v1 = new b2Vec2();
        this.v2 = new b2Vec2();
        this.normal = new b2Vec2();
        this.sideNormal1 = new b2Vec2();
        this.sideOffset1 = NaN;
        this.sideNormal2 = new b2Vec2();
        this.sideOffset2 = NaN;
        this.sideOffset1 = 0.0;
        this.sideOffset2 = 0.0;
    }
}
class b2EPCollider {
    constructor() {
        this.m_polygonB = new b2TempPolygon();
        this.m_xf = new b2Transform();
        this.m_centroidB = new b2Vec2();
        this.m_v0 = new b2Vec2();
        this.m_v1 = new b2Vec2();
        this.m_v2 = new b2Vec2();
        this.m_v3 = new b2Vec2();
        this.m_normal0 = new b2Vec2();
        this.m_normal1 = new b2Vec2();
        this.m_normal2 = new b2Vec2();
        this.m_normal = new b2Vec2();
        this.m_type1 = 0 /* e_isolated */;
        this.m_type2 = 0 /* e_isolated */;
        this.m_lowerLimit = new b2Vec2();
        this.m_upperLimit = new b2Vec2();
        this.m_radius = NaN;
        this.m_front = false;
        this.m_radius = 0.0;
    }
    Collide(manifold, edgeA, xfA, polygonB, xfB) {
        b2Transform.MulTXX(xfA, xfB, this.m_xf);
        b2Transform.MulXV(this.m_xf, polygonB.m_centroid, this.m_centroidB);
        this.m_v0.Copy(edgeA.m_vertex0);
        this.m_v1.Copy(edgeA.m_vertex1);
        this.m_v2.Copy(edgeA.m_vertex2);
        this.m_v3.Copy(edgeA.m_vertex3);
        const hasVertex0 = edgeA.m_hasVertex0;
        const hasVertex3 = edgeA.m_hasVertex3;
        const edge1 = b2Vec2.SubVV(this.m_v2, this.m_v1, b2EPCollider.s_edge1);
        edge1.Normalize();
        this.m_normal1.Set(edge1.y, -edge1.x);
        const offset1 = b2Vec2.DotVV(this.m_normal1, b2Vec2.SubVV(this.m_centroidB, this.m_v1, b2Vec2.s_t0));
        let offset0 = 0;
        let offset2 = 0;
        let convex1 = false;
        let convex2 = false;
        // Is there a preceding edge?
        if (hasVertex0) {
            const edge0 = b2Vec2.SubVV(this.m_v1, this.m_v0, b2EPCollider.s_edge0);
            edge0.Normalize();
            this.m_normal0.Set(edge0.y, -edge0.x);
            convex1 = b2Vec2.CrossVV(edge0, edge1) >= 0;
            offset0 = b2Vec2.DotVV(this.m_normal0, b2Vec2.SubVV(this.m_centroidB, this.m_v0, b2Vec2.s_t0));
        }
        // Is there a following edge?
        if (hasVertex3) {
            const edge2 = b2Vec2.SubVV(this.m_v3, this.m_v2, b2EPCollider.s_edge2);
            edge2.Normalize();
            this.m_normal2.Set(edge2.y, -edge2.x);
            convex2 = b2Vec2.CrossVV(edge1, edge2) > 0;
            offset2 = b2Vec2.DotVV(this.m_normal2, b2Vec2.SubVV(this.m_centroidB, this.m_v2, b2Vec2.s_t0));
        }
        // Determine front or back collision. Determine collision normal limits.
        if (hasVertex0 && hasVertex3) {
            if (convex1 && convex2) {
                this.m_front = offset0 >= 0 || offset1 >= 0 || offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else if (convex1) {
                this.m_front = offset0 >= 0 || (offset1 >= 0 && offset2 >= 0);
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else if (convex2) {
                this.m_front = offset2 >= 0 || (offset0 >= 0 && offset1 >= 0);
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
            else {
                this.m_front = offset0 >= 0 && offset1 >= 0 && offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
        }
        else if (hasVertex0) {
            if (convex1) {
                this.m_front = offset0 >= 0 || offset1 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else {
                this.m_front = offset0 >= 0 && offset1 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
        }
        else if (hasVertex3) {
            if (convex2) {
                this.m_front = offset1 >= 0 || offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
            }
            else {
                this.m_front = offset1 >= 0 && offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
            }
        }
        else {
            this.m_front = offset1 >= 0;
            if (this.m_front) {
                this.m_normal.Copy(this.m_normal1);
                this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
            }
            else {
                this.m_normal.Copy(this.m_normal1).SelfNeg();
                this.m_lowerLimit.Copy(this.m_normal1);
                this.m_upperLimit.Copy(this.m_normal1);
            }
        }
        // Get polygonB in frameA
        this.m_polygonB.count = polygonB.m_count;
        for (let i = 0; i < polygonB.m_count; ++i) {
            if (this.m_polygonB.vertices.length <= i) {
                this.m_polygonB.vertices.push(new b2Vec2());
            }
            if (this.m_polygonB.normals.length <= i) {
                this.m_polygonB.normals.push(new b2Vec2());
            }
            b2Transform.MulXV(this.m_xf, polygonB.m_vertices[i], this.m_polygonB.vertices[i]);
            b2Rot.MulRV(this.m_xf.q, polygonB.m_normals[i], this.m_polygonB.normals[i]);
        }
        this.m_radius = polygonB.m_radius + edgeA.m_radius;
        manifold.pointCount = 0;
        const edgeAxis = this.ComputeEdgeSeparation(b2EPCollider.s_edgeAxis);
        // If no valid normal can be found than this edge should not collide.
        if (edgeAxis.type === 0 /* e_unknown */) {
            return;
        }
        if (edgeAxis.separation > this.m_radius) {
            return;
        }
        const polygonAxis = this.ComputePolygonSeparation(b2EPCollider.s_polygonAxis);
        if (polygonAxis.type !== 0 /* e_unknown */ && polygonAxis.separation > this.m_radius) {
            return;
        }
        // Use hysteresis for jitter reduction.
        const k_relativeTol = 0.98;
        const k_absoluteTol = 0.001;
        let primaryAxis;
        if (polygonAxis.type === 0 /* e_unknown */) {
            primaryAxis = edgeAxis;
        }
        else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol) {
            primaryAxis = polygonAxis;
        }
        else {
            primaryAxis = edgeAxis;
        }
        const ie = b2EPCollider.s_ie;
        const rf = b2EPCollider.s_rf;
        if (primaryAxis.type === 1 /* e_edgeA */) {
            manifold.type = 1 /* e_faceA */;
            // Search for the polygon normal that is most anti-parallel to the edge normal.
            let bestIndex = 0;
            let bestValue = b2Vec2.DotVV(this.m_normal, this.m_polygonB.normals[0]);
            for (let i = 1; i < this.m_polygonB.count; ++i) {
                const value = b2Vec2.DotVV(this.m_normal, this.m_polygonB.normals[i]);
                if (value < bestValue) {
                    bestValue = value;
                    bestIndex = i;
                }
            }
            const i1 = bestIndex;
            const i2 = (i1 + 1) % this.m_polygonB.count;
            const ie0 = ie[0];
            ie0.v.Copy(this.m_polygonB.vertices[i1]);
            ie0.id.cf.indexA = 0;
            ie0.id.cf.indexB = i1;
            ie0.id.cf.typeA = 1 /* e_face */;
            ie0.id.cf.typeB = 0 /* e_vertex */;
            const ie1 = ie[1];
            ie1.v.Copy(this.m_polygonB.vertices[i2]);
            ie1.id.cf.indexA = 0;
            ie1.id.cf.indexB = i2;
            ie1.id.cf.typeA = 1 /* e_face */;
            ie1.id.cf.typeB = 0 /* e_vertex */;
            if (this.m_front) {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1.Copy(this.m_v1);
                rf.v2.Copy(this.m_v2);
                rf.normal.Copy(this.m_normal1);
            }
            else {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1.Copy(this.m_v2);
                rf.v2.Copy(this.m_v1);
                rf.normal.Copy(this.m_normal1).SelfNeg();
            }
        }
        else {
            manifold.type = 2 /* e_faceB */;
            const ie0 = ie[0];
            ie0.v.Copy(this.m_v1);
            ie0.id.cf.indexA = 0;
            ie0.id.cf.indexB = primaryAxis.index;
            ie0.id.cf.typeA = 0 /* e_vertex */;
            ie0.id.cf.typeB = 1 /* e_face */;
            const ie1 = ie[1];
            ie1.v.Copy(this.m_v2);
            ie1.id.cf.indexA = 0;
            ie1.id.cf.indexB = primaryAxis.index;
            ie1.id.cf.typeA = 0 /* e_vertex */;
            ie1.id.cf.typeB = 1 /* e_face */;
            rf.i1 = primaryAxis.index;
            rf.i2 = (rf.i1 + 1) % this.m_polygonB.count;
            rf.v1.Copy(this.m_polygonB.vertices[rf.i1]);
            rf.v2.Copy(this.m_polygonB.vertices[rf.i2]);
            rf.normal.Copy(this.m_polygonB.normals[rf.i1]);
        }
        rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
        rf.sideNormal2.Copy(rf.sideNormal1).SelfNeg();
        rf.sideOffset1 = b2Vec2.DotVV(rf.sideNormal1, rf.v1);
        rf.sideOffset2 = b2Vec2.DotVV(rf.sideNormal2, rf.v2);
        // Clip incident edge against extruded edge1 side edges.
        const clipPoints1 = b2EPCollider.s_clipPoints1;
        const clipPoints2 = b2EPCollider.s_clipPoints2;
        let np = 0;
        // Clip to box side 1
        np = b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
        if (np < b2_maxManifoldPoints) {
            return;
        }
        // Clip to negative box side 1
        np = b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
        if (np < b2_maxManifoldPoints) {
            return;
        }
        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.type === 1 /* e_edgeA */) {
            manifold.localNormal.Copy(rf.normal);
            manifold.localPoint.Copy(rf.v1);
        }
        else {
            manifold.localNormal.Copy(polygonB.m_normals[rf.i1]);
            manifold.localPoint.Copy(polygonB.m_vertices[rf.i1]);
        }
        let pointCount = 0;
        for (let i = 0; i < b2_maxManifoldPoints; ++i) {
            const separation = b2Vec2.DotVV(rf.normal, b2Vec2.SubVV(clipPoints2[i].v, rf.v1, b2Vec2.s_t0));
            if (separation <= this.m_radius) {
                const cp = manifold.points[pointCount];
                if (primaryAxis.type === 1 /* e_edgeA */) {
                    b2Transform.MulTXV(this.m_xf, clipPoints2[i].v, cp.localPoint);
                    cp.id.Copy(clipPoints2[i].id);
                }
                else {
                    cp.localPoint.Copy(clipPoints2[i].v);
                    cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
                    cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
                    cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
                    cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
                }
                ++pointCount;
            }
        }
        manifold.pointCount = pointCount;
    }
    ComputeEdgeSeparation(out) {
        const axis = out;
        axis.type = 1 /* e_edgeA */;
        axis.index = this.m_front ? 0 : 1;
        axis.separation = b2_maxFloat;
        for (let i = 0; i < this.m_polygonB.count; ++i) {
            const s = b2Vec2.DotVV(this.m_normal, b2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, b2Vec2.s_t0));
            if (s < axis.separation) {
                axis.separation = s;
            }
        }
        return axis;
    }
    ComputePolygonSeparation(out) {
        const axis = out;
        axis.type = 0 /* e_unknown */;
        axis.index = -1;
        axis.separation = -b2_maxFloat;
        const perp = b2EPCollider.s_perp.Set(-this.m_normal.y, this.m_normal.x);
        for (let i = 0; i < this.m_polygonB.count; ++i) {
            const n = b2Vec2.NegV(this.m_polygonB.normals[i], b2EPCollider.s_n);
            const s1 = b2Vec2.DotVV(n, b2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, b2Vec2.s_t0));
            const s2 = b2Vec2.DotVV(n, b2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v2, b2Vec2.s_t0));
            const s = b2Min(s1, s2);
            if (s > this.m_radius) {
                // No collision
                axis.type = 2 /* e_edgeB */;
                axis.index = i;
                axis.separation = s;
                return axis;
            }
            // Adjacency
            if (b2Vec2.DotVV(n, perp) >= 0) {
                if (b2Vec2.DotVV(b2Vec2.SubVV(n, this.m_upperLimit, b2Vec2.s_t0), this.m_normal) <
                    -b2_angularSlop) {
                    continue;
                }
            }
            else {
                if (b2Vec2.DotVV(b2Vec2.SubVV(n, this.m_lowerLimit, b2Vec2.s_t0), this.m_normal) <
                    -b2_angularSlop) {
                    continue;
                }
            }
            if (s > axis.separation) {
                axis.type = 2 /* e_edgeB */;
                axis.index = i;
                axis.separation = s;
            }
        }
        return axis;
    }
}
b2EPCollider.s_edge1 = new b2Vec2();
b2EPCollider.s_edge0 = new b2Vec2();
b2EPCollider.s_edge2 = new b2Vec2();
b2EPCollider.s_ie = b2ClipVertex.MakeArray(2);
b2EPCollider.s_rf = new b2ReferenceFace();
b2EPCollider.s_clipPoints1 = b2ClipVertex.MakeArray(2);
b2EPCollider.s_clipPoints2 = b2ClipVertex.MakeArray(2);
b2EPCollider.s_edgeAxis = new b2EPAxis();
b2EPCollider.s_polygonAxis = new b2EPAxis();
b2EPCollider.s_n = new b2Vec2();
b2EPCollider.s_perp = new b2Vec2();
const b2CollideEdgeAndPolygon_s_collider = new b2EPCollider();
export function b2CollideEdgeAndPolygon(manifold, edgeA, xfA, polygonB, xfB) {
    const collider = b2CollideEdgeAndPolygon_s_collider;
    collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaWRlRWRnZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb2xsaXNpb24vYjJDb2xsaWRlRWRnZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSxPQUFPLEVBQUUsUUFBUSxFQUFFLE1BQU0sc0JBQXNCLENBQUM7QUFDaEQsT0FBTyxFQUFFLGNBQWMsRUFBRSxXQUFXLEVBQUUsb0JBQW9CLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUN6RixPQUFPLEVBQUUsS0FBSyxFQUFFLEtBQUssRUFBRSxXQUFXLEVBQUUsTUFBTSxFQUFFLE1BQU0sa0JBQWtCLENBQUM7QUFDckUsT0FBTyxFQUNMLG1CQUFtQixFQUNuQixZQUFZLEVBRVosV0FBVyxHQUlaLE1BQU0sZUFBZSxDQUFDO0FBS3ZCLE1BQU0sMEJBQTBCLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUN4RCxNQUFNLDBCQUEwQixHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUFDeEQsTUFBTSwwQkFBMEIsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ3hELE1BQU0sMkJBQTJCLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUN6RCxNQUFNLDJCQUEyQixHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUFDekQsTUFBTSwwQkFBMEIsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ3hELE1BQU0sMEJBQTBCLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUN4RCxNQUFNLDJCQUEyQixHQUFnQixJQUFJLFdBQVcsRUFBRSxDQUFDO0FBRW5FLE1BQU0sVUFBVSxzQkFBc0IsQ0FDcEMsUUFBb0IsRUFDcEIsS0FBa0IsRUFDbEIsR0FBZ0IsRUFDaEIsT0FBc0IsRUFDdEIsR0FBZ0I7SUFFaEIsUUFBUSxDQUFDLFVBQVUsR0FBRyxDQUFDLENBQUM7SUFFeEIsa0NBQWtDO0lBQ2xDLE1BQU0sQ0FBQyxHQUFXLFdBQVcsQ0FBQyxNQUFNLENBQ2xDLEdBQUcs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QUEyRHZDLE1BQU0sa0NBQWtDLEdBQWlCLElBQUksWUFBWSxFQUFFLENBQUM7QUFFNUUsTUFBTSxVQUFVLHVCQUF1QixDQUNyQyxRQUFvQixFQUNwQixLQUFrQixFQUNsQixHQUFnQixFQUNoQixRQUF3QixFQUN4QixHQUFnQjtJQUVoQixNQUFNLFFBQVEsR0FBaUIsa0NBQWtDLENBQUM7SUFDbEUsUUFBUSxDQUFDLE9BQU8sQ0FBQyxRQUFRLEVBQUUsS0FBSyxFQUFFLEdBQUcsRUFBRSxRQUFRLEVBQUUsR0FBRyxDQUFDLENBQUM7QUFDeEQsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IGIyQXNzZXJ0IH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMl9hbmd1bGFyU2xvcCwgYjJfbWF4RmxvYXQsIGIyX21heE1hbmlmb2xkUG9pbnRzIH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMk1pbiwgYjJSb3QsIGIyVHJhbnNmb3JtLCBiMlZlYzIgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHtcclxuICBiMkNsaXBTZWdtZW50VG9MaW5lLFxyXG4gIGIyQ2xpcFZlcnRleCxcclxuICBiMkNvbnRhY3RGZWF0dXJlVHlwZSxcclxuICBiMkNvbnRhY3RJRCxcclxuICBiMk1hbmlmb2xkLFxyXG4gIGIyTWFuaWZvbGRQb2ludCxcclxuICBiMk1hbmlmb2xkVHlwZSxcclxufSBmcm9tICcuL2IyQ29sbGlzaW9uJztcclxuaW1wb3J0IHsgYjJDaXJjbGVTaGFwZSB9IGZyb20gJy4vc2hhcGVzL2IyQ2lyY2xlU2hhcGUnO1xyXG5pbXBvcnQgeyBiMlBvbHlnb25TaGFwZSB9IGZyb20gJy4vc2hhcGVzL2IyUG9seWdvblNoYXBlJztcclxuaW1wb3J0IHsgYjJFZGdlU2hhcGUgfSBmcm9tICcuL3NoYXBlcy9iMkVkZ2VTaGFwZSc7XHJcblxyXG5jb25zdCBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfUTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfZTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfZDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5jb25zdCBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfZTE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJDb2xsaWRlRWRnZUFuZENpcmNsZV9zX2UyOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyQ29sbGlkZUVkZ2VBbmRDaXJjbGVfc19QOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyQ29sbGlkZUVkZ2VBbmRDaXJjbGVfc19uOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyQ29sbGlkZUVkZ2VBbmRDaXJjbGVfc19pZDogYjJDb250YWN0SUQgPSBuZXcgYjJDb250YWN0SUQoKTtcclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlKFxyXG4gIG1hbmlmb2xkOiBiMk1hbmlmb2xkLFxyXG4gIGVkZ2VBOiBiMkVkZ2VTaGFwZSxcclxuICB4ZkE6IGIyVHJhbnNmb3JtLFxyXG4gIGNpcmNsZUI6IGIyQ2lyY2xlU2hhcGUsXHJcbiAgeGZCOiBiMlRyYW5zZm9ybSxcclxuKTogdm9pZCB7XHJcbiAgbWFuaWZvbGQucG9pbnRDb3VudCA9IDA7XHJcblxyXG4gIC8vIENvbXB1dGUgY2lyY2xlIGluIGZyYW1lIG9mIGVkZ2VcclxuICBjb25zdCBROiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxUWFYoXHJcbiAgICB4ZkEsXHJcbiAgICBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIGNpcmNsZUIubV9wLCBiMlZlYzIuc190MCksXHJcbiAgICBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfUSxcclxuICApO1xyXG5cclxuICBjb25zdCBBOiBiMlZlYzIgPSBlZGdlQS5tX3ZlcnRleDE7XHJcbiAgY29uc3QgQjogYjJWZWMyID0gZWRnZUEubV92ZXJ0ZXgyO1xyXG4gIGNvbnN0IGU6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihCLCBBLCBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfZSk7XHJcblxyXG4gIC8vIEJhcnljZW50cmljIGNvb3JkaW5hdGVzXHJcbiAgY29uc3QgdTogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGUsIGIyVmVjMi5TdWJWVihCLCBRLCBiMlZlYzIuc190MCkpO1xyXG4gIGNvbnN0IHY6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihlLCBiMlZlYzIuU3ViVlYoUSwgQSwgYjJWZWMyLnNfdDApKTtcclxuXHJcbiAgY29uc3QgcmFkaXVzOiBudW1iZXIgPSBlZGdlQS5tX3JhZGl1cyArIGNpcmNsZUIubV9yYWRpdXM7XHJcblxyXG4gIC8vIGNvbnN0IGNmOiBiMkNvbnRhY3RGZWF0dXJlID0gbmV3IGIyQ29udGFjdEZlYXR1cmUoKTtcclxuICBjb25zdCBpZDogYjJDb250YWN0SUQgPSBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfaWQ7XHJcbiAgaWQuY2YuaW5kZXhCID0gMDtcclxuICBpZC5jZi50eXBlQiA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfdmVydGV4O1xyXG5cclxuICAvLyBSZWdpb24gQVxyXG4gIGlmICh2IDw9IDApIHtcclxuICAgIGNvbnN0IFA6IGIyVmVjMiA9IEE7XHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYoUSwgUCwgYjJDb2xsaWRlRWRnZUFuZENpcmNsZV9zX2QpO1xyXG4gICAgY29uc3QgZGQ6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihkLCBkKTtcclxuICAgIGlmIChkZCA+IHJhZGl1cyAqIHJhZGl1cykge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgLy8gSXMgdGhlcmUgYW4gZWRnZSBjb25uZWN0ZWQgdG8gQT9cclxuICAgIGlmIChlZGdlQS5tX2hhc1ZlcnRleDApIHtcclxuICAgICAgY29uc3QgQTE6IGIyVmVjMiA9IGVkZ2VBLm1fdmVydGV4MDtcclxuICAgICAgY29uc3QgQjE6IGIyVmVjMiA9IEE7XHJcbiAgICAgIGNvbnN0IGUxOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYoQjEsIEExLCBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfZTEpO1xyXG4gICAgICBjb25zdCB1MTogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGUxLCBiMlZlYzIuU3ViVlYoQjEsIFEsIGIyVmVjMi5zX3QwKSk7XHJcblxyXG4gICAgICAvLyBJcyB0aGUgY2lyY2xlIGluIFJlZ2lvbiBBQiBvZiB0aGUgcHJldmlvdXMgZWRnZT9cclxuICAgICAgaWYgKHUxID4gMCkge1xyXG4gICAgICAgIHJldHVybjtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGlkLmNmLmluZGV4QSA9IDA7XHJcbiAgICBpZC5jZi50eXBlQSA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfdmVydGV4O1xyXG4gICAgbWFuaWZvbGQucG9pbnRDb3VudCA9IDE7XHJcbiAgICBtYW5pZm9sZC50eXBlID0gYjJNYW5pZm9sZFR5cGUuZV9jaXJjbGVzO1xyXG4gICAgbWFuaWZvbGQubG9jYWxOb3JtYWwuU2V0WmVybygpO1xyXG4gICAgbWFuaWZvbGQubG9jYWxQb2ludC5Db3B5KFApO1xyXG4gICAgbWFuaWZvbGQucG9pbnRzWzBdLmlkLkNvcHkoaWQpO1xyXG4gICAgLy8gbWFuaWZvbGQucG9pbnRzWzBdLmlkLmtleSA9IDA7XHJcbiAgICAvLyBtYW5pZm9sZC5wb2ludHNbMF0uaWQuY2YgPSBjZjtcclxuICAgIG1hbmlmb2xkLnBvaW50c1swXS5sb2NhbFBvaW50LkNvcHkoY2lyY2xlQi5tX3ApO1xyXG4gICAgcmV0dXJuO1xyXG4gIH1cclxuXHJcbiAgLy8gUmVnaW9uIEJcclxuICBpZiAodSA8PSAwKSB7XHJcbiAgICBjb25zdCBQOiBiMlZlYzIgPSBCO1xyXG4gICAgY29uc3QgZDogYjJWZWMyID0gYjJWZWMyLlN1YlZWKFEsIFAsIGIyQ29sbGlkZUVkZ2VBbmRDaXJjbGVfc19kKTtcclxuICAgIGNvbnN0IGRkOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoZCwgZCk7XHJcbiAgICBpZiAoZGQgPiByYWRpdXMgKiByYWRpdXMpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIElzIHRoZXJlIGFuIGVkZ2UgY29ubmVjdGVkIHRvIEI/XHJcbiAgICBpZiAoZWRnZUEubV9oYXNWZXJ0ZXgzKSB7XHJcbiAgICAgIGNvbnN0IEIyOiBiMlZlYzIgPSBlZGdlQS5tX3ZlcnRleDM7XHJcbiAgICAgIGNvbnN0IEEyOiBiMlZlYzIgPSBCO1xyXG4gICAgICBjb25zdCBlMjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKEIyLCBBMiwgYjJDb2xsaWRlRWRnZUFuZENpcmNsZV9zX2UyKTtcclxuICAgICAgY29uc3QgdjI6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihlMiwgYjJWZWMyLlN1YlZWKFEsIEEyLCBiMlZlYzIuc190MCkpO1xyXG5cclxuICAgICAgLy8gSXMgdGhlIGNpcmNsZSBpbiBSZWdpb24gQUIgb2YgdGhlIG5leHQgZWRnZT9cclxuICAgICAgaWYgKHYyID4gMCkge1xyXG4gICAgICAgIHJldHVybjtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGlkLmNmLmluZGV4QSA9IDE7XHJcbiAgICBpZC5jZi50eXBlQSA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfdmVydGV4O1xyXG4gICAgbWFuaWZvbGQucG9pbnRDb3VudCA9IDE7XHJcbiAgICBtYW5pZm9sZC50eXBlID0gYjJNYW5pZm9sZFR5cGUuZV9jaXJjbGVzO1xyXG4gICAgbWFuaWZvbGQubG9jYWxOb3JtYWwuU2V0WmVybygpO1xyXG4gICAgbWFuaWZvbGQubG9jYWxQb2ludC5Db3B5KFApO1xyXG4gICAgbWFuaWZvbGQucG9pbnRzWzBdLmlkLkNvcHkoaWQpO1xyXG4gICAgLy8gbWFuaWZvbGQucG9pbnRzWzBdLmlkLmtleSA9IDA7XHJcbiAgICAvLyBtYW5pZm9sZC5wb2ludHNbMF0uaWQuY2YgPSBjZjtcclxuICAgIG1hbmlmb2xkLnBvaW50c1swXS5sb2NhbFBvaW50LkNvcHkoY2lyY2xlQi5tX3ApO1xyXG4gICAgcmV0dXJuO1xyXG4gIH1cclxuXHJcbiAgLy8gUmVnaW9uIEFCXHJcbiAgY29uc3QgZGVuOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoZSwgZSk7XHJcbiAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChkZW4gPiAwKTtcclxuICBjb25zdCBQOiBiMlZlYzIgPSBiMkNvbGxpZGVFZGdlQW5kQ2lyY2xlX3NfUDtcclxuICBQLnggPSAoMSAvIGRlbikgKiAodSAqIEEueCArIHYgKiBCLngpO1xyXG4gIFAueSA9ICgxIC8gZGVuKSAqICh1ICogQS55ICsgdiAqIEIueSk7XHJcbiAgY29uc3QgZDogYjJWZWMyID0gYjJWZWMyLlN1YlZWKFEsIFAsIGIyQ29sbGlkZUVkZ2VBbmRDaXJjbGVfc19kKTtcclxuICBjb25zdCBkZDogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGQsIGQpO1xyXG4gIGlmIChkZCA+IHJhZGl1cyAqIHJhZGl1cykge1xyXG4gICAgcmV0dXJuO1xyXG4gIH1cclxuXHJcbiAgY29uc3QgbjogYjJWZWMyID0gYjJDb2xsaWRlRWRnZUFuZENpcmNsZV9zX24uU2V0KC1lLnksIGUueCk7XHJcbiAgaWYgKGIyVmVjMi5Eb3RWVihuLCBiMlZlYzIuU3ViVlYoUSwgQSwgYjJWZWMyLnNfdDApKSA8IDApIHtcclxuICAgIG4uU2V0KC1uLngsIC1uLnkpO1xyXG4gIH1cclxuICBuLk5vcm1hbGl6ZSgpO1xyXG5cclxuICBpZC5jZi5pbmRleEEgPSAwO1xyXG4gIGlkLmNmLnR5cGVBID0gYjJDb250YWN0RmVhdHVyZVR5cGUuZV9mYWNlO1xyXG4gIG1hbmlmb2xkLnBvaW50Q291bnQgPSAxO1xyXG4gIG1hbmlmb2xkLnR5cGUgPSBiMk1hbmlmb2xkVHlwZS5lX2ZhY2VBO1xyXG4gIG1hbmlmb2xkLmxvY2FsTm9ybWFsLkNvcHkobik7XHJcbiAgbWFuaWZvbGQubG9jYWxQb2ludC5Db3B5KEEpO1xyXG4gIG1hbmlmb2xkLnBvaW50c1swXS5pZC5Db3B5KGlkKTtcclxuICAvLyBtYW5pZm9sZC5wb2ludHNbMF0uaWQua2V5ID0gMDtcclxuICAvLyBtYW5pZm9sZC5wb2ludHNbMF0uaWQuY2YgPSBjZjtcclxuICBtYW5pZm9sZC5wb2ludHNbMF0ubG9jYWxQb2ludC5Db3B5KGNpcmNsZUIubV9wKTtcclxufVxyXG5cclxuY29uc3QgZW51bSBiMkVQQXhpc1R5cGUge1xyXG4gIGVfdW5rbm93biA9IDAsXHJcbiAgZV9lZGdlQSA9IDEsXHJcbiAgZV9lZGdlQiA9IDIsXHJcbn1cclxuXHJcbmNsYXNzIGIyRVBBeGlzIHtcclxuICB0eXBlID0gYjJFUEF4aXNUeXBlLmVfdW5rbm93bjtcclxuICBpbmRleCA9IDA7XHJcbiAgc2VwYXJhdGlvbiA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLnNlcGFyYXRpb24gPSAwLjA7XHJcbiAgfVxyXG59XHJcblxyXG5jbGFzcyBiMlRlbXBQb2x5Z29uIHtcclxuICB2ZXJ0aWNlczogYjJWZWMyW10gPSBbXTtcclxuICBub3JtYWxzOiBiMlZlYzJbXSA9IFtdO1xyXG4gIGNvdW50ID0gMDtcclxufVxyXG5cclxuY2xhc3MgYjJSZWZlcmVuY2VGYWNlIHtcclxuICBpMSA9IDA7XHJcbiAgaTIgPSAwO1xyXG4gIHJlYWRvbmx5IHYxID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IHYyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG5vcm1hbCA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBzaWRlTm9ybWFsMSA9IG5ldyBiMlZlYzIoKTtcclxuICBzaWRlT2Zmc2V0MSA9IE5hTjtcclxuICByZWFkb25seSBzaWRlTm9ybWFsMiA9IG5ldyBiMlZlYzIoKTtcclxuICBzaWRlT2Zmc2V0MiA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLnNpZGVPZmZzZXQxID0gMC4wO1xyXG4gICAgdGhpcy5zaWRlT2Zmc2V0MiA9IDAuMDtcclxuICB9XHJcbn1cclxuXHJcbmNvbnN0IGVudW0gYjJFUENvbGxpZGVyVmVydGV4VHlwZSB7XHJcbiAgZV9pc29sYXRlZCA9IDAsXHJcbiAgZV9jb25jYXZlID0gMSxcclxuICBlX2NvbnZleCA9IDIsXHJcbn1cclxuXHJcbmNsYXNzIGIyRVBDb2xsaWRlciB7XHJcbiAgcmVhZG9ubHkgbV9wb2x5Z29uQjogYjJUZW1wUG9seWdvbiA9IG5ldyBiMlRlbXBQb2x5Z29uKCk7XHJcbiAgcmVhZG9ubHkgbV94ZjogYjJUcmFuc2Zvcm0gPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICByZWFkb25seSBtX2NlbnRyb2lkQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fdjA6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX3YxOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV92MjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fdjM6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX25vcm1hbDA6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX25vcm1hbDE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX25vcm1hbDI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX25vcm1hbDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1fdHlwZTEgPSBiMkVQQ29sbGlkZXJWZXJ0ZXhUeXBlLmVfaXNvbGF0ZWQ7XHJcbiAgbV90eXBlMiA9IGIyRVBDb2xsaWRlclZlcnRleFR5cGUuZV9pc29sYXRlZDtcclxuICByZWFkb25seSBtX2xvd2VyTGltaXQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX3VwcGVyTGltaXQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBtX3JhZGl1cyA9IE5hTjtcclxuICBtX2Zyb250ID0gZmFsc2U7XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5tX3JhZGl1cyA9IDAuMDtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIHNfZWRnZTEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19lZGdlMCA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBzX2VkZ2UyID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIHNfaWUgPSBiMkNsaXBWZXJ0ZXguTWFrZUFycmF5KDIpO1xyXG4gIHByaXZhdGUgc3RhdGljIHNfcmYgPSBuZXcgYjJSZWZlcmVuY2VGYWNlKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19jbGlwUG9pbnRzMSA9IGIyQ2xpcFZlcnRleC5NYWtlQXJyYXkoMik7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19jbGlwUG9pbnRzMiA9IGIyQ2xpcFZlcnRleC5NYWtlQXJyYXkoMik7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19lZGdlQXhpcyA9IG5ldyBiMkVQQXhpcygpO1xyXG4gIHByaXZhdGUgc3RhdGljIHNfcG9seWdvbkF4aXMgPSBuZXcgYjJFUEF4aXMoKTtcclxuXHJcbiAgQ29sbGlkZShcclxuICAgIG1hbmlmb2xkOiBiMk1hbmlmb2xkLFxyXG4gICAgZWRnZUE6IGIyRWRnZVNoYXBlLFxyXG4gICAgeGZBOiBiMlRyYW5zZm9ybSxcclxuICAgIHBvbHlnb25COiBiMlBvbHlnb25TaGFwZSxcclxuICAgIHhmQjogYjJUcmFuc2Zvcm0sXHJcbiAgKTogdm9pZCB7XHJcbiAgICBiMlRyYW5zZm9ybS5NdWxUWFgoeGZBLCB4ZkIsIHRoaXMubV94Zik7XHJcblxyXG4gICAgYjJUcmFuc2Zvcm0uTXVsWFYodGhpcy5tX3hmLCBwb2x5Z29uQi5tX2NlbnRyb2lkLCB0aGlzLm1fY2VudHJvaWRCKTtcclxuXHJcbiAgICB0aGlzLm1fdjAuQ29weShlZGdlQS5tX3ZlcnRleDApO1xyXG4gICAgdGhpcy5tX3YxLkNvcHkoZWRnZUEubV92ZXJ0ZXgxKTtcclxuICAgIHRoaXMubV92Mi5Db3B5KGVkZ2VBLm1fdmVydGV4Mik7XHJcbiAgICB0aGlzLm1fdjMuQ29weShlZGdlQS5tX3ZlcnRleDMpO1xyXG5cclxuICAgIGNvbnN0IGhhc1ZlcnRleDA6IGJvb2xlYW4gPSBlZGdlQS5tX2hhc1ZlcnRleDA7XHJcbiAgICBjb25zdCBoYXNWZXJ0ZXgzOiBib29sZWFuID0gZWRnZUEubV9oYXNWZXJ0ZXgzO1xyXG5cclxuICAgIGNvbnN0IGVkZ2UxOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYodGhpcy5tX3YyLCB0aGlzLm1fdjEsIGIyRVBDb2xsaWRlci5zX2VkZ2UxKTtcclxuICAgIGVkZ2UxLk5vcm1hbGl6ZSgpO1xyXG4gICAgdGhpcy5tX25vcm1hbDEuU2V0KGVkZ2UxLnksIC1lZGdlMS54KTtcclxuICAgIGNvbnN0IG9mZnNldDE6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgdGhpcy5tX25vcm1hbDEsXHJcbiAgICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fY2VudHJvaWRCLCB0aGlzLm1fdjEsIGIyVmVjMi5zX3QwKSxcclxuICAgICk7XHJcbiAgICBsZXQgb2Zmc2V0MCA9IDA7XHJcbiAgICBsZXQgb2Zmc2V0MiA9IDA7XHJcbiAgICBsZXQgY29udmV4MSA9IGZhbHNlO1xyXG4gICAgbGV0IGNvbnZleDIgPSBmYWxzZTtcclxuXHJcbiAgICAvLyBJcyB0aGVyZSBhIHByZWNlZGluZyBlZGdlP1xyXG4gICAgaWYgKGhhc1ZlcnRleDApIHtcclxuICAgICAgY29uc3QgZWRnZTA6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVih0aGlzLm1fdjEsIHRoaXMubV92MCwgYjJFUENvbGxpZGVyLnNfZWRnZTApO1xyXG4gICAgICBlZGdlMC5Ob3JtYWxpemUoKTtcclxuICAgICAgdGhpcy5tX25vcm1hbDAuU2V0KGVkZ2UwLnksIC1lZGdlMC54KTtcclxuICAgICAgY29udmV4MSA9IGIyVmVjMi5Dcm9zc1ZWKGVkZ2UwLCBlZGdlMSkgPj0gMDtcclxuICAgICAgb2Zmc2V0MCA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgICB0aGlzLm1fbm9ybWFsMCxcclxuICAgICAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2NlbnRyb2lkQiwgdGhpcy5tX3YwLCBiMlZlYzIuc190MCksXHJcbiAgICAgICk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gSXMgdGhlcmUgYSBmb2xsb3dpbmcgZWRnZT9cclxuICAgIGlmIChoYXNWZXJ0ZXgzKSB7XHJcbiAgICAgIGNvbnN0IGVkZ2UyOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYodGhpcy5tX3YzLCB0aGlzLm1fdjIsIGIyRVBDb2xsaWRlci5zX2VkZ2UyKTtcclxuICAgICAgZWRnZTIuTm9ybWFsaXplKCk7XHJcbiAgICAgIHRoaXMubV9ub3JtYWwyLlNldChlZGdlMi55LCAtZWRnZTIueCk7XHJcbiAgICAgIGNvbnZleDIgPSBiMlZlYzIuQ3Jvc3NWVihlZGdlMSwgZWRnZTIpID4gMDtcclxuICAgICAgb2Zmc2V0MiA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgICB0aGlzLm1fbm9ybWFsMixcclxuICAgICAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2NlbnRyb2lkQiwgdGhpcy5tX3YyLCBiMlZlYzIuc190MCksXHJcbiAgICAgICk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gRGV0ZXJtaW5lIGZyb250IG9yIGJhY2sgY29sbGlzaW9uLiBEZXRlcm1pbmUgY29sbGlzaW9uIG5vcm1hbCBsaW1pdHMuXHJcbiAgICBpZiAoaGFzVmVydGV4MCAmJiBoYXNWZXJ0ZXgzKSB7XHJcbiAgICAgIGlmIChjb252ZXgxICYmIGNvbnZleDIpIHtcclxuICAgICAgICB0aGlzLm1fZnJvbnQgPSBvZmZzZXQwID49IDAgfHwgb2Zmc2V0MSA+PSAwIHx8IG9mZnNldDIgPj0gMDtcclxuICAgICAgICBpZiAodGhpcy5tX2Zyb250KSB7XHJcbiAgICAgICAgICB0aGlzLm1fbm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMCk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwyKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgdGhpcy5tX25vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9IGVsc2UgaWYgKGNvbnZleDEpIHtcclxuICAgICAgICB0aGlzLm1fZnJvbnQgPSBvZmZzZXQwID49IDAgfHwgKG9mZnNldDEgPj0gMCAmJiBvZmZzZXQyID49IDApO1xyXG4gICAgICAgIGlmICh0aGlzLm1fZnJvbnQpIHtcclxuICAgICAgICAgIHRoaXMubV9ub3JtYWwuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwwKTtcclxuICAgICAgICAgIHRoaXMubV91cHBlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICB0aGlzLm1fbm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgICAgICAgIHRoaXMubV9sb3dlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDIpLlNlbGZOZWcoKTtcclxuICAgICAgICAgIHRoaXMubV91cHBlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgICAgICB9XHJcbiAgICAgIH0gZWxzZSBpZiAoY29udmV4Mikge1xyXG4gICAgICAgIHRoaXMubV9mcm9udCA9IG9mZnNldDIgPj0gMCB8fCAob2Zmc2V0MCA+PSAwICYmIG9mZnNldDEgPj0gMCk7XHJcbiAgICAgICAgaWYgKHRoaXMubV9mcm9udCkge1xyXG4gICAgICAgICAgdGhpcy5tX25vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKTtcclxuICAgICAgICAgIHRoaXMubV9sb3dlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgICAgdGhpcy5tX3VwcGVyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMik7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHRoaXMubV9ub3JtYWwuQ29weSh0aGlzLm1fbm9ybWFsMSkuU2VsZk5lZygpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSkuU2VsZk5lZygpO1xyXG4gICAgICAgICAgdGhpcy5tX3VwcGVyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMCkuU2VsZk5lZygpO1xyXG4gICAgICAgIH1cclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICB0aGlzLm1fZnJvbnQgPSBvZmZzZXQwID49IDAgJiYgb2Zmc2V0MSA+PSAwICYmIG9mZnNldDIgPj0gMDtcclxuICAgICAgICBpZiAodGhpcy5tX2Zyb250KSB7XHJcbiAgICAgICAgICB0aGlzLm1fbm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgdGhpcy5tX25vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwyKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwwKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9IGVsc2UgaWYgKGhhc1ZlcnRleDApIHtcclxuICAgICAgaWYgKGNvbnZleDEpIHtcclxuICAgICAgICB0aGlzLm1fZnJvbnQgPSBvZmZzZXQwID49IDAgfHwgb2Zmc2V0MSA+PSAwO1xyXG4gICAgICAgIGlmICh0aGlzLm1fZnJvbnQpIHtcclxuICAgICAgICAgIHRoaXMubV9ub3JtYWwuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwwKTtcclxuICAgICAgICAgIHRoaXMubV91cHBlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgdGhpcy5tX25vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKTtcclxuICAgICAgICAgIHRoaXMubV91cHBlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgICAgICB9XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgdGhpcy5tX2Zyb250ID0gb2Zmc2V0MCA+PSAwICYmIG9mZnNldDEgPj0gMDtcclxuICAgICAgICBpZiAodGhpcy5tX2Zyb250KSB7XHJcbiAgICAgICAgICB0aGlzLm1fbm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHRoaXMubV9ub3JtYWwuQ29weSh0aGlzLm1fbm9ybWFsMSkuU2VsZk5lZygpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwwKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9IGVsc2UgaWYgKGhhc1ZlcnRleDMpIHtcclxuICAgICAgaWYgKGNvbnZleDIpIHtcclxuICAgICAgICB0aGlzLm1fZnJvbnQgPSBvZmZzZXQxID49IDAgfHwgb2Zmc2V0MiA+PSAwO1xyXG4gICAgICAgIGlmICh0aGlzLm1fZnJvbnQpIHtcclxuICAgICAgICAgIHRoaXMubV9ub3JtYWwuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwyKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgdGhpcy5tX25vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fbG93ZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKTtcclxuICAgICAgICB9XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgdGhpcy5tX2Zyb250ID0gb2Zmc2V0MSA+PSAwICYmIG9mZnNldDIgPj0gMDtcclxuICAgICAgICBpZiAodGhpcy5tX2Zyb250KSB7XHJcbiAgICAgICAgICB0aGlzLm1fbm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSkuU2VsZk5lZygpO1xyXG4gICAgICAgICAgdGhpcy5tX3VwcGVyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHRoaXMubV9ub3JtYWwuQ29weSh0aGlzLm1fbm9ybWFsMSkuU2VsZk5lZygpO1xyXG4gICAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMikuU2VsZk5lZygpO1xyXG4gICAgICAgICAgdGhpcy5tX3VwcGVyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1fZnJvbnQgPSBvZmZzZXQxID49IDA7XHJcbiAgICAgIGlmICh0aGlzLm1fZnJvbnQpIHtcclxuICAgICAgICB0aGlzLm1fbm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpO1xyXG4gICAgICAgIHRoaXMubV9sb3dlckxpbWl0LkNvcHkodGhpcy5tX25vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgICAgICB0aGlzLm1fdXBwZXJMaW1pdC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgdGhpcy5tX25vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKS5TZWxmTmVnKCk7XHJcbiAgICAgICAgdGhpcy5tX2xvd2VyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgICAgdGhpcy5tX3VwcGVyTGltaXQuQ29weSh0aGlzLm1fbm9ybWFsMSk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAvLyBHZXQgcG9seWdvbkIgaW4gZnJhbWVBXHJcbiAgICB0aGlzLm1fcG9seWdvbkIuY291bnQgPSBwb2x5Z29uQi5tX2NvdW50O1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBwb2x5Z29uQi5tX2NvdW50OyArK2kpIHtcclxuICAgICAgaWYgKHRoaXMubV9wb2x5Z29uQi52ZXJ0aWNlcy5sZW5ndGggPD0gaSkge1xyXG4gICAgICAgIHRoaXMubV9wb2x5Z29uQi52ZXJ0aWNlcy5wdXNoKG5ldyBiMlZlYzIoKSk7XHJcbiAgICAgIH1cclxuICAgICAgaWYgKHRoaXMubV9wb2x5Z29uQi5ub3JtYWxzLmxlbmd0aCA8PSBpKSB7XHJcbiAgICAgICAgdGhpcy5tX3BvbHlnb25CLm5vcm1hbHMucHVzaChuZXcgYjJWZWMyKCkpO1xyXG4gICAgICB9XHJcbiAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHRoaXMubV94ZiwgcG9seWdvbkIubV92ZXJ0aWNlc1tpXSwgdGhpcy5tX3BvbHlnb25CLnZlcnRpY2VzW2ldKTtcclxuICAgICAgYjJSb3QuTXVsUlYodGhpcy5tX3hmLnEsIHBvbHlnb25CLm1fbm9ybWFsc1tpXSwgdGhpcy5tX3BvbHlnb25CLm5vcm1hbHNbaV0pO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9yYWRpdXMgPSBwb2x5Z29uQi5tX3JhZGl1cyArIGVkZ2VBLm1fcmFkaXVzO1xyXG5cclxuICAgIG1hbmlmb2xkLnBvaW50Q291bnQgPSAwO1xyXG5cclxuICAgIGNvbnN0IGVkZ2VBeGlzOiBiMkVQQXhpcyA9IHRoaXMuQ29tcHV0ZUVkZ2VTZXBhcmF0aW9uKGIyRVBDb2xsaWRlci5zX2VkZ2VBeGlzKTtcclxuXHJcbiAgICAvLyBJZiBubyB2YWxpZCBub3JtYWwgY2FuIGJlIGZvdW5kIHRoYW4gdGhpcyBlZGdlIHNob3VsZCBub3QgY29sbGlkZS5cclxuICAgIGlmIChlZGdlQXhpcy50eXBlID09PSBiMkVQQXhpc1R5cGUuZV91bmtub3duKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoZWRnZUF4aXMuc2VwYXJhdGlvbiA+IHRoaXMubV9yYWRpdXMpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IHBvbHlnb25BeGlzOiBiMkVQQXhpcyA9IHRoaXMuQ29tcHV0ZVBvbHlnb25TZXBhcmF0aW9uKGIyRVBDb2xsaWRlci5zX3BvbHlnb25BeGlzKTtcclxuICAgIGlmIChwb2x5Z29uQXhpcy50eXBlICE9PSBiMkVQQXhpc1R5cGUuZV91bmtub3duICYmIHBvbHlnb25BeGlzLnNlcGFyYXRpb24gPiB0aGlzLm1fcmFkaXVzKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICAvLyBVc2UgaHlzdGVyZXNpcyBmb3Igaml0dGVyIHJlZHVjdGlvbi5cclxuICAgIGNvbnN0IGtfcmVsYXRpdmVUb2wgPSAwLjk4O1xyXG4gICAgY29uc3Qga19hYnNvbHV0ZVRvbCA9IDAuMDAxO1xyXG5cclxuICAgIGxldCBwcmltYXJ5QXhpczogYjJFUEF4aXM7XHJcbiAgICBpZiAocG9seWdvbkF4aXMudHlwZSA9PT0gYjJFUEF4aXNUeXBlLmVfdW5rbm93bikge1xyXG4gICAgICBwcmltYXJ5QXhpcyA9IGVkZ2VBeGlzO1xyXG4gICAgfSBlbHNlIGlmIChwb2x5Z29uQXhpcy5zZXBhcmF0aW9uID4ga19yZWxhdGl2ZVRvbCAqIGVkZ2VBeGlzLnNlcGFyYXRpb24gKyBrX2Fic29sdXRlVG9sKSB7XHJcbiAgICAgIHByaW1hcnlBeGlzID0gcG9seWdvbkF4aXM7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICBwcmltYXJ5QXhpcyA9IGVkZ2VBeGlzO1xyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IGllOiBiMkNsaXBWZXJ0ZXhbXSA9IGIyRVBDb2xsaWRlci5zX2llO1xyXG4gICAgY29uc3QgcmY6IGIyUmVmZXJlbmNlRmFjZSA9IGIyRVBDb2xsaWRlci5zX3JmO1xyXG4gICAgaWYgKHByaW1hcnlBeGlzLnR5cGUgPT09IGIyRVBBeGlzVHlwZS5lX2VkZ2VBKSB7XHJcbiAgICAgIG1hbmlmb2xkLnR5cGUgPSBiMk1hbmlmb2xkVHlwZS5lX2ZhY2VBO1xyXG5cclxuICAgICAgLy8gU2VhcmNoIGZvciB0aGUgcG9seWdvbiBub3JtYWwgdGhhdCBpcyBtb3N0IGFudGktcGFyYWxsZWwgdG8gdGhlIGVkZ2Ugbm9ybWFsLlxyXG4gICAgICBsZXQgYmVzdEluZGV4ID0gMDtcclxuICAgICAgbGV0IGJlc3RWYWx1ZTogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHRoaXMubV9ub3JtYWwsIHRoaXMubV9wb2x5Z29uQi5ub3JtYWxzWzBdKTtcclxuICAgICAgZm9yIChsZXQgaSA9IDE7IGkgPCB0aGlzLm1fcG9seWdvbkIuY291bnQ7ICsraSkge1xyXG4gICAgICAgIGNvbnN0IHZhbHVlOiBudW1iZXIgPSBiMlZlYzIuRG90VlYodGhpcy5tX25vcm1hbCwgdGhpcy5tX3BvbHlnb25CLm5vcm1hbHNbaV0pO1xyXG4gICAgICAgIGlmICh2YWx1ZSA8IGJlc3RWYWx1ZSkge1xyXG4gICAgICAgICAgYmVzdFZhbHVlID0gdmFsdWU7XHJcbiAgICAgICAgICBiZXN0SW5kZXggPSBpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgY29uc3QgaTE6IG51bWJlciA9IGJlc3RJbmRleDtcclxuICAgICAgY29uc3QgaTI6IG51bWJlciA9IChpMSArIDEpICUgdGhpcy5tX3BvbHlnb25CLmNvdW50O1xyXG5cclxuICAgICAgY29uc3QgaWUwOiBiMkNsaXBWZXJ0ZXggPSBpZVswXTtcclxuICAgICAgaWUwLnYuQ29weSh0aGlzLm1fcG9seWdvbkIudmVydGljZXNbaTFdKTtcclxuICAgICAgaWUwLmlkLmNmLmluZGV4QSA9IDA7XHJcbiAgICAgIGllMC5pZC5jZi5pbmRleEIgPSBpMTtcclxuICAgICAgaWUwLmlkLmNmLnR5cGVBID0gYjJDb250YWN0RmVhdHVyZVR5cGUuZV9mYWNlO1xyXG4gICAgICBpZTAuaWQuY2YudHlwZUIgPSBiMkNvbnRhY3RGZWF0dXJlVHlwZS5lX3ZlcnRleDtcclxuXHJcbiAgICAgIGNvbnN0IGllMTogYjJDbGlwVmVydGV4ID0gaWVbMV07XHJcbiAgICAgIGllMS52LkNvcHkodGhpcy5tX3BvbHlnb25CLnZlcnRpY2VzW2kyXSk7XHJcbiAgICAgIGllMS5pZC5jZi5pbmRleEEgPSAwO1xyXG4gICAgICBpZTEuaWQuY2YuaW5kZXhCID0gaTI7XHJcbiAgICAgIGllMS5pZC5jZi50eXBlQSA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfZmFjZTtcclxuICAgICAgaWUxLmlkLmNmLnR5cGVCID0gYjJDb250YWN0RmVhdHVyZVR5cGUuZV92ZXJ0ZXg7XHJcblxyXG4gICAgICBpZiAodGhpcy5tX2Zyb250KSB7XHJcbiAgICAgICAgcmYuaTEgPSAwO1xyXG4gICAgICAgIHJmLmkyID0gMTtcclxuICAgICAgICByZi52MS5Db3B5KHRoaXMubV92MSk7XHJcbiAgICAgICAgcmYudjIuQ29weSh0aGlzLm1fdjIpO1xyXG4gICAgICAgIHJmLm5vcm1hbC5Db3B5KHRoaXMubV9ub3JtYWwxKTtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICByZi5pMSA9IDE7XHJcbiAgICAgICAgcmYuaTIgPSAwO1xyXG4gICAgICAgIHJmLnYxLkNvcHkodGhpcy5tX3YyKTtcclxuICAgICAgICByZi52Mi5Db3B5KHRoaXMubV92MSk7XHJcbiAgICAgICAgcmYubm9ybWFsLkNvcHkodGhpcy5tX25vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgbWFuaWZvbGQudHlwZSA9IGIyTWFuaWZvbGRUeXBlLmVfZmFjZUI7XHJcblxyXG4gICAgICBjb25zdCBpZTA6IGIyQ2xpcFZlcnRleCA9IGllWzBdO1xyXG4gICAgICBpZTAudi5Db3B5KHRoaXMubV92MSk7XHJcbiAgICAgIGllMC5pZC5jZi5pbmRleEEgPSAwO1xyXG4gICAgICBpZTAuaWQuY2YuaW5kZXhCID0gcHJpbWFyeUF4aXMuaW5kZXg7XHJcbiAgICAgIGllMC5pZC5jZi50eXBlQSA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfdmVydGV4O1xyXG4gICAgICBpZTAuaWQuY2YudHlwZUIgPSBiMkNvbnRhY3RGZWF0dXJlVHlwZS5lX2ZhY2U7XHJcblxyXG4gICAgICBjb25zdCBpZTE6IGIyQ2xpcFZlcnRleCA9IGllWzFdO1xyXG4gICAgICBpZTEudi5Db3B5KHRoaXMubV92Mik7XHJcbiAgICAgIGllMS5pZC5jZi5pbmRleEEgPSAwO1xyXG4gICAgICBpZTEuaWQuY2YuaW5kZXhCID0gcHJpbWFyeUF4aXMuaW5kZXg7XHJcbiAgICAgIGllMS5pZC5jZi50eXBlQSA9IGIyQ29udGFjdEZlYXR1cmVUeXBlLmVfdmVydGV4O1xyXG4gICAgICBpZTEuaWQuY2YudHlwZUIgPSBiMkNvbnRhY3RGZWF0dXJlVHlwZS5lX2ZhY2U7XHJcblxyXG4gICAgICByZi5pMSA9IHByaW1hcnlBeGlzLmluZGV4O1xyXG4gICAgICByZi5pMiA9IChyZi5pMSArIDEpICUgdGhpcy5tX3BvbHlnb25CLmNvdW50O1xyXG4gICAgICByZi52MS5Db3B5KHRoaXMubV9wb2x5Z29uQi52ZXJ0aWNlc1tyZi5pMV0pO1xyXG4gICAgICByZi52Mi5Db3B5KHRoaXMubV9wb2x5Z29uQi52ZXJ0aWNlc1tyZi5pMl0pO1xyXG4gICAgICByZi5ub3JtYWwuQ29weSh0aGlzLm1fcG9seWdvbkIubm9ybWFsc1tyZi5pMV0pO1xyXG4gICAgfVxyXG5cclxuICAgIHJmLnNpZGVOb3JtYWwxLlNldChyZi5ub3JtYWwueSwgLXJmLm5vcm1hbC54KTtcclxuICAgIHJmLnNpZGVOb3JtYWwyLkNvcHkocmYuc2lkZU5vcm1hbDEpLlNlbGZOZWcoKTtcclxuICAgIHJmLnNpZGVPZmZzZXQxID0gYjJWZWMyLkRvdFZWKHJmLnNpZGVOb3JtYWwxLCByZi52MSk7XHJcbiAgICByZi5zaWRlT2Zmc2V0MiA9IGIyVmVjMi5Eb3RWVihyZi5zaWRlTm9ybWFsMiwgcmYudjIpO1xyXG5cclxuICAgIC8vIENsaXAgaW5jaWRlbnQgZWRnZSBhZ2FpbnN0IGV4dHJ1ZGVkIGVkZ2UxIHNpZGUgZWRnZXMuXHJcbiAgICBjb25zdCBjbGlwUG9pbnRzMTogYjJDbGlwVmVydGV4W10gPSBiMkVQQ29sbGlkZXIuc19jbGlwUG9pbnRzMTtcclxuICAgIGNvbnN0IGNsaXBQb2ludHMyOiBiMkNsaXBWZXJ0ZXhbXSA9IGIyRVBDb2xsaWRlci5zX2NsaXBQb2ludHMyO1xyXG4gICAgbGV0IG5wID0gMDtcclxuXHJcbiAgICAvLyBDbGlwIHRvIGJveCBzaWRlIDFcclxuICAgIG5wID0gYjJDbGlwU2VnbWVudFRvTGluZShjbGlwUG9pbnRzMSwgaWUsIHJmLnNpZGVOb3JtYWwxLCByZi5zaWRlT2Zmc2V0MSwgcmYuaTEpO1xyXG5cclxuICAgIGlmIChucCA8IGIyX21heE1hbmlmb2xkUG9pbnRzKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICAvLyBDbGlwIHRvIG5lZ2F0aXZlIGJveCBzaWRlIDFcclxuICAgIG5wID0gYjJDbGlwU2VnbWVudFRvTGluZShjbGlwUG9pbnRzMiwgY2xpcFBvaW50czEsIHJmLnNpZGVOb3JtYWwyLCByZi5zaWRlT2Zmc2V0MiwgcmYuaTIpO1xyXG5cclxuICAgIGlmIChucCA8IGIyX21heE1hbmlmb2xkUG9pbnRzKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICAvLyBOb3cgY2xpcFBvaW50czIgY29udGFpbnMgdGhlIGNsaXBwZWQgcG9pbnRzLlxyXG4gICAgaWYgKHByaW1hcnlBeGlzLnR5cGUgPT09IGIyRVBBeGlzVHlwZS5lX2VkZ2VBKSB7XHJcbiAgICAgIG1hbmlmb2xkLmxvY2FsTm9ybWFsLkNvcHkocmYubm9ybWFsKTtcclxuICAgICAgbWFuaWZvbGQubG9jYWxQb2ludC5Db3B5KHJmLnYxKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIG1hbmlmb2xkLmxvY2FsTm9ybWFsLkNvcHkocG9seWdvbkIubV9ub3JtYWxzW3JmLmkxXSk7XHJcbiAgICAgIG1hbmlmb2xkLmxvY2FsUG9pbnQuQ29weShwb2x5Z29uQi5tX3ZlcnRpY2VzW3JmLmkxXSk7XHJcbiAgICB9XHJcblxyXG4gICAgbGV0IHBvaW50Q291bnQgPSAwO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBiMl9tYXhNYW5pZm9sZFBvaW50czsgKytpKSB7XHJcbiAgICAgIGNvbnN0IHNlcGFyYXRpb24gPSBiMlZlYzIuRG90VlYoXHJcbiAgICAgICAgcmYubm9ybWFsLFxyXG4gICAgICAgIGIyVmVjMi5TdWJWVihjbGlwUG9pbnRzMltpXS52LCByZi52MSwgYjJWZWMyLnNfdDApLFxyXG4gICAgICApO1xyXG5cclxuICAgICAgaWYgKHNlcGFyYXRpb24gPD0gdGhpcy5tX3JhZGl1cykge1xyXG4gICAgICAgIGNvbnN0IGNwOiBiMk1hbmlmb2xkUG9pbnQgPSBtYW5pZm9sZC5wb2ludHNbcG9pbnRDb3VudF07XHJcblxyXG4gICAgICAgIGlmIChwcmltYXJ5QXhpcy50eXBlID09PSBiMkVQQXhpc1R5cGUuZV9lZGdlQSkge1xyXG4gICAgICAgICAgYjJUcmFuc2Zvcm0uTXVsVFhWKHRoaXMubV94ZiwgY2xpcFBvaW50czJbaV0udiwgY3AubG9jYWxQb2ludCk7XHJcbiAgICAgICAgICBjcC5pZC5Db3B5KGNsaXBQb2ludHMyW2ldLmlkKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgY3AubG9jYWxQb2ludC5Db3B5KGNsaXBQb2ludHMyW2ldLnYpO1xyXG4gICAgICAgICAgY3AuaWQuY2YudHlwZUEgPSBjbGlwUG9pbnRzMltpXS5pZC5jZi50eXBlQjtcclxuICAgICAgICAgIGNwLmlkLmNmLnR5cGVCID0gY2xpcFBvaW50czJbaV0uaWQuY2YudHlwZUE7XHJcbiAgICAgICAgICBjcC5pZC5jZi5pbmRleEEgPSBjbGlwUG9pbnRzMltpXS5pZC5jZi5pbmRleEI7XHJcbiAgICAgICAgICBjcC5pZC5jZi5pbmRleEIgPSBjbGlwUG9pbnRzMltpXS5pZC5jZi5pbmRleEE7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICArK3BvaW50Q291bnQ7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICBtYW5pZm9sZC5wb2ludENvdW50ID0gcG9pbnRDb3VudDtcclxuICB9XHJcblxyXG4gIENvbXB1dGVFZGdlU2VwYXJhdGlvbihvdXQ6IGIyRVBBeGlzKTogYjJFUEF4aXMge1xyXG4gICAgY29uc3QgYXhpczogYjJFUEF4aXMgPSBvdXQ7XHJcbiAgICBheGlzLnR5cGUgPSBiMkVQQXhpc1R5cGUuZV9lZGdlQTtcclxuICAgIGF4aXMuaW5kZXggPSB0aGlzLm1fZnJvbnQgPyAwIDogMTtcclxuICAgIGF4aXMuc2VwYXJhdGlvbiA9IGIyX21heEZsb2F0O1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX3BvbHlnb25CLmNvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgczogbnVtYmVyID0gYjJWZWMyLkRvdFZWKFxyXG4gICAgICAgIHRoaXMubV9ub3JtYWwsXHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9wb2x5Z29uQi52ZXJ0aWNlc1tpXSwgdGhpcy5tX3YxLCBiMlZlYzIuc190MCksXHJcbiAgICAgICk7XHJcbiAgICAgIGlmIChzIDwgYXhpcy5zZXBhcmF0aW9uKSB7XHJcbiAgICAgICAgYXhpcy5zZXBhcmF0aW9uID0gcztcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBheGlzO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19uID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIHNfcGVycCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgQ29tcHV0ZVBvbHlnb25TZXBhcmF0aW9uKG91dDogYjJFUEF4aXMpOiBiMkVQQXhpcyB7XHJcbiAgICBjb25zdCBheGlzOiBiMkVQQXhpcyA9IG91dDtcclxuICAgIGF4aXMudHlwZSA9IGIyRVBBeGlzVHlwZS5lX3Vua25vd247XHJcbiAgICBheGlzLmluZGV4ID0gLTE7XHJcbiAgICBheGlzLnNlcGFyYXRpb24gPSAtYjJfbWF4RmxvYXQ7XHJcblxyXG4gICAgY29uc3QgcGVycDogYjJWZWMyID0gYjJFUENvbGxpZGVyLnNfcGVycC5TZXQoLXRoaXMubV9ub3JtYWwueSwgdGhpcy5tX25vcm1hbC54KTtcclxuXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9wb2x5Z29uQi5jb3VudDsgKytpKSB7XHJcbiAgICAgIGNvbnN0IG46IGIyVmVjMiA9IGIyVmVjMi5OZWdWKHRoaXMubV9wb2x5Z29uQi5ub3JtYWxzW2ldLCBiMkVQQ29sbGlkZXIuc19uKTtcclxuXHJcbiAgICAgIGNvbnN0IHMxOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoXHJcbiAgICAgICAgbixcclxuICAgICAgICBiMlZlYzIuU3ViVlYodGhpcy5tX3BvbHlnb25CLnZlcnRpY2VzW2ldLCB0aGlzLm1fdjEsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgKTtcclxuICAgICAgY29uc3QgczI6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgICBuLFxyXG4gICAgICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fcG9seWdvbkIudmVydGljZXNbaV0sIHRoaXMubV92MiwgYjJWZWMyLnNfdDApLFxyXG4gICAgICApO1xyXG4gICAgICBjb25zdCBzOiBudW1iZXIgPSBiMk1pbihzMSwgczIpO1xyXG5cclxuICAgICAgaWYgKHMgPiB0aGlzLm1fcmFkaXVzKSB7XHJcbiAgICAgICAgLy8gTm8gY29sbGlzaW9uXHJcbiAgICAgICAgYXhpcy50eXBlID0gYjJFUEF4aXNUeXBlLmVfZWRnZUI7XHJcbiAgICAgICAgYXhpcy5pbmRleCA9IGk7XHJcbiAgICAgICAgYXhpcy5zZXBhcmF0aW9uID0gcztcclxuICAgICAgICByZXR1cm4gYXhpcztcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gQWRqYWNlbmN5XHJcbiAgICAgIGlmIChiMlZlYzIuRG90VlYobiwgcGVycCkgPj0gMCkge1xyXG4gICAgICAgIGlmIChcclxuICAgICAgICAgIGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYobiwgdGhpcy5tX3VwcGVyTGltaXQsIGIyVmVjMi5zX3QwKSwgdGhpcy5tX25vcm1hbCkgPFxyXG4gICAgICAgICAgLWIyX2FuZ3VsYXJTbG9wXHJcbiAgICAgICAgKSB7XHJcbiAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICB9XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgaWYgKFxyXG4gICAgICAgICAgYjJWZWMyLkRvdFZWKGIyVmVjMi5TdWJWVihuLCB0aGlzLm1fbG93ZXJMaW1pdCwgYjJWZWMyLnNfdDApLCB0aGlzLm1fbm9ybWFsKSA8XHJcbiAgICAgICAgICAtYjJfYW5ndWxhclNsb3BcclxuICAgICAgICApIHtcclxuICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgaWYgKHMgPiBheGlzLnNlcGFyYXRpb24pIHtcclxuICAgICAgICBheGlzLnR5cGUgPSBiMkVQQXhpc1R5cGUuZV9lZGdlQjtcclxuICAgICAgICBheGlzLmluZGV4ID0gaTtcclxuICAgICAgICBheGlzLnNlcGFyYXRpb24gPSBzO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIGF4aXM7XHJcbiAgfVxyXG59XHJcblxyXG5jb25zdCBiMkNvbGxpZGVFZGdlQW5kUG9seWdvbl9zX2NvbGxpZGVyOiBiMkVQQ29sbGlkZXIgPSBuZXcgYjJFUENvbGxpZGVyKCk7XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJDb2xsaWRlRWRnZUFuZFBvbHlnb24oXHJcbiAgbWFuaWZvbGQ6IGIyTWFuaWZvbGQsXHJcbiAgZWRnZUE6IGIyRWRnZVNoYXBlLFxyXG4gIHhmQTogYjJUcmFuc2Zvcm0sXHJcbiAgcG9seWdvbkI6IGIyUG9seWdvblNoYXBlLFxyXG4gIHhmQjogYjJUcmFuc2Zvcm0sXHJcbik6IHZvaWQge1xyXG4gIGNvbnN0IGNvbGxpZGVyOiBiMkVQQ29sbGlkZXIgPSBiMkNvbGxpZGVFZGdlQW5kUG9seWdvbl9zX2NvbGxpZGVyO1xyXG4gIGNvbGxpZGVyLkNvbGxpZGUobWFuaWZvbGQsIGVkZ2VBLCB4ZkEsIHBvbHlnb25CLCB4ZkIpO1xyXG59XHJcbiJdfQ==