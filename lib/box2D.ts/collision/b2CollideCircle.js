import { b2_epsilon, b2_maxFloat } from '../common/b2Settings';
import { b2Transform, b2Vec2 } from '../common/b2Math';
const b2CollideCircles_s_pA = new b2Vec2();
const b2CollideCircles_s_pB = new b2Vec2();
export function b2CollideCircles(manifold, circleA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    const pA = b2Transform.MulXV(xfA, circleA.m_p, b2CollideCircles_s_pA);
    const pB = b2Transform.MulXV(xfB, circleB.m_p, b2CollideCircles_s_pB);
    const distSqr = b2Vec2.DistanceSquaredVV(pA, pB);
    const radius = circleA.m_radius + circleB.m_radius;
    if (distSqr > radius * radius) {
        return;
    }
    manifold.type = 0 /* e_circles */;
    manifold.localPoint.Copy(circleA.m_p);
    manifold.localNormal.SetZero();
    manifold.pointCount = 1;
    manifold.points[0].localPoint.Copy(circleB.m_p);
    manifold.points[0].id.key = 0;
}
const b2CollidePolygonAndCircle_s_c = new b2Vec2();
const b2CollidePolygonAndCircle_s_cLocal = new b2Vec2();
const b2CollidePolygonAndCircle_s_faceCenter = new b2Vec2();
export function b2CollidePolygonAndCircle(manifold, polygonA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    // Compute circle position in the frame of the polygon.
    const c = b2Transform.MulXV(xfB, circleB.m_p, b2CollidePolygonAndCircle_s_c);
    const cLocal = b2Transform.MulTXV(xfA, c, b2CollidePolygonAndCircle_s_cLocal);
    // Find the min separating edge.
    let normalIndex = 0;
    let separation = -b2_maxFloat;
    const radius = polygonA.m_radius + circleB.m_radius;
    const vertexCount = polygonA.m_count;
    const vertices = polygonA.m_vertices;
    const normals = polygonA.m_normals;
    for (let i = 0; i < vertexCount; ++i) {
        const s = b2Vec2.DotVV(normals[i], b2Vec2.SubVV(cLocal, vertices[i], b2Vec2.s_t0));
        if (s > radius) {
            // Early out.
            return;
        }
        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }
    // Vertices that subtend the incident face.
    const vertIndex1 = normalIndex;
    const vertIndex2 = (vertIndex1 + 1) % vertexCount;
    const v1 = vertices[vertIndex1];
    const v2 = vertices[vertIndex2];
    // If the center is inside the polygon ...
    if (separation < b2_epsilon) {
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        manifold.localNormal.Copy(normals[normalIndex]);
        b2Vec2.MidVV(v1, v2, manifold.localPoint);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
        return;
    }
    // Compute barycentric coordinates
    const u1 = b2Vec2.DotVV(b2Vec2.SubVV(cLocal, v1, b2Vec2.s_t0), b2Vec2.SubVV(v2, v1, b2Vec2.s_t1));
    const u2 = b2Vec2.DotVV(b2Vec2.SubVV(cLocal, v2, b2Vec2.s_t0), b2Vec2.SubVV(v1, v2, b2Vec2.s_t1));
    if (u1 <= 0) {
        if (b2Vec2.DistanceSquaredVV(cLocal, v1) > radius * radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        b2Vec2.SubVV(cLocal, v1, manifold.localNormal).SelfNormalize();
        manifold.localPoint.Copy(v1);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    else if (u2 <= 0) {
        if (b2Vec2.DistanceSquaredVV(cLocal, v2) > radius * radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        b2Vec2.SubVV(cLocal, v2, manifold.localNormal).SelfNormalize();
        manifold.localPoint.Copy(v2);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    else {
        const faceCenter = b2Vec2.MidVV(v1, v2, b2CollidePolygonAndCircle_s_faceCenter);
        const separation = b2Vec2.DotVV(b2Vec2.SubVV(cLocal, faceCenter, b2Vec2.s_t1), normals[vertIndex1]);
        if (separation > radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        manifold.localNormal.Copy(normals[vertIndex1]).SelfNormalize();
        manifold.localPoint.Copy(faceCenter);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaWRlQ2lyY2xlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2NvbGxpc2lvbi9iMkNvbGxpZGVDaXJjbGUudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsT0FBTyxFQUFFLFVBQVUsRUFBRSxXQUFXLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUMvRCxPQUFPLEVBQUUsV0FBVyxFQUFFLE1BQU0sRUFBRSxNQUFNLGtCQUFrQixDQUFDO0FBS3ZELE1BQU0scUJBQXFCLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNuRCxNQUFNLHFCQUFxQixHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUFFbkQsTUFBTSxVQUFVLGdCQUFnQixDQUM5QixRQUFvQixFQUNwQixPQUFzQixFQUN0QixHQUFnQixFQUNoQixPQUFzQixFQUN0QixHQUFnQjtJQUVoQixRQUFRLENBQUMsVUFBVSxHQUFHLENBQUMsQ0FBQztJQUV4QixNQUFNLEVBQUUsR0FBRyxXQUFXLENBQUMsS0FBSyxDQUFDLEdBQUcsRUFBRSxPQUFPLENBQUMsR0FBRyxFQUFFLHFCQUFxQixDQUFDLENBQUM7SUFDdEUsTUFBTSxFQUFFLEdBQUcsV0FBVyxDQUFDLEtBQUssQ0FBQyxHQUFHLEVBQUUsT0FBTyxDQUFDLEdBQUcsRUFBRSxxQkFBcUIsQ0FBQyxDQUFDO0lBRXRFLE1BQU0sT0FBTyxHQUFXLE1BQU0sQ0FBQyxpQkFBaUIsQ0FBQyxFQUFFLEVBQUUsRUFBRSxDQUFDLENBQUM7SUFDekQsTUFBTSxNQUFNLEdBQVcsT0FBTyxDQUFDLFFBQVEsR0FBRyxPQUFPLENBQUMsUUFBUSxDQUFDO0lBQzNELElBQUksT0FBTyxHQUFHLE1BQU0sR0FBRyxNQUFNLEVBQUU7UUFDN0IsT0FBTztLQUNSO0lBRUQsUUFBUSxDQUFDLElBQUksb0JBQTJCLENBQUM7SUFDekMsUUFBUSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEdBQUcsQ0FBQyxDQUFD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a0JBQXlCLENBQUM7UUFDdkMsUUFBUSxDQUFDLFdBQVcsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLFVBQVUsQ0FBQyxDQUFDLENBQUMsYUFBYSxFQUFFLENBQUM7UUFDL0QsUUFBUSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsVUFBVSxDQUFDLENBQUM7UUFDckMsUUFBUSxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUNoRCxRQUFRLENBQUMsTUFBTSxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxHQUFHLEdBQUcsQ0FBQyxDQUFDO0tBQy9CO0FBQ0gsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IGIyX2Vwc2lsb24sIGIyX21heEZsb2F0IH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMlRyYW5zZm9ybSwgYjJWZWMyIH0gZnJvbSAnLi4vY29tbW9uL2IyTWF0aCc7XHJcbmltcG9ydCB7IGIyTWFuaWZvbGQsIGIyTWFuaWZvbGRUeXBlIH0gZnJvbSAnLi9iMkNvbGxpc2lvbic7XHJcbmltcG9ydCB7IGIyQ2lyY2xlU2hhcGUgfSBmcm9tICcuL3NoYXBlcy9iMkNpcmNsZVNoYXBlJztcclxuaW1wb3J0IHsgYjJQb2x5Z29uU2hhcGUgfSBmcm9tICcuL3NoYXBlcy9iMlBvbHlnb25TaGFwZSc7XHJcblxyXG5jb25zdCBiMkNvbGxpZGVDaXJjbGVzX3NfcEE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJDb2xsaWRlQ2lyY2xlc19zX3BCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJDb2xsaWRlQ2lyY2xlcyhcclxuICBtYW5pZm9sZDogYjJNYW5pZm9sZCxcclxuICBjaXJjbGVBOiBiMkNpcmNsZVNoYXBlLFxyXG4gIHhmQTogYjJUcmFuc2Zvcm0sXHJcbiAgY2lyY2xlQjogYjJDaXJjbGVTaGFwZSxcclxuICB4ZkI6IGIyVHJhbnNmb3JtLFxyXG4pOiB2b2lkIHtcclxuICBtYW5pZm9sZC5wb2ludENvdW50ID0gMDtcclxuXHJcbiAgY29uc3QgcEEgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkEsIGNpcmNsZUEubV9wLCBiMkNvbGxpZGVDaXJjbGVzX3NfcEEpO1xyXG4gIGNvbnN0IHBCID0gYjJUcmFuc2Zvcm0uTXVsWFYoeGZCLCBjaXJjbGVCLm1fcCwgYjJDb2xsaWRlQ2lyY2xlc19zX3BCKTtcclxuXHJcbiAgY29uc3QgZGlzdFNxcjogbnVtYmVyID0gYjJWZWMyLkRpc3RhbmNlU3F1YXJlZFZWKHBBLCBwQik7XHJcbiAgY29uc3QgcmFkaXVzOiBudW1iZXIgPSBjaXJjbGVBLm1fcmFkaXVzICsgY2lyY2xlQi5tX3JhZGl1cztcclxuICBpZiAoZGlzdFNxciA+IHJhZGl1cyAqIHJhZGl1cykge1xyXG4gICAgcmV0dXJuO1xyXG4gIH1cclxuXHJcbiAgbWFuaWZvbGQudHlwZSA9IGIyTWFuaWZvbGRUeXBlLmVfY2lyY2xlcztcclxuICBtYW5pZm9sZC5sb2NhbFBvaW50LkNvcHkoY2lyY2xlQS5tX3ApO1xyXG4gIG1hbmlmb2xkLmxvY2FsTm9ybWFsLlNldFplcm8oKTtcclxuICBtYW5pZm9sZC5wb2ludENvdW50ID0gMTtcclxuXHJcbiAgbWFuaWZvbGQucG9pbnRzWzBdLmxvY2FsUG9pbnQuQ29weShjaXJjbGVCLm1fcCk7XHJcbiAgbWFuaWZvbGQucG9pbnRzWzBdLmlkLmtleSA9IDA7XHJcbn1cclxuXHJcbmNvbnN0IGIyQ29sbGlkZVBvbHlnb25BbmRDaXJjbGVfc19jOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbmNvbnN0IGIyQ29sbGlkZVBvbHlnb25BbmRDaXJjbGVfc19jTG9jYWw6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuY29uc3QgYjJDb2xsaWRlUG9seWdvbkFuZENpcmNsZV9zX2ZhY2VDZW50ZXI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkNvbGxpZGVQb2x5Z29uQW5kQ2lyY2xlKFxyXG4gIG1hbmlmb2xkOiBiMk1hbmlmb2xkLFxyXG4gIHBvbHlnb25BOiBiMlBvbHlnb25TaGFwZSxcclxuICB4ZkE6IGIyVHJhbnNmb3JtLFxyXG4gIGNpcmNsZUI6IGIyQ2lyY2xlU2hhcGUsXHJcbiAgeGZCOiBiMlRyYW5zZm9ybSxcclxuKTogdm9pZCB7XHJcbiAgbWFuaWZvbGQucG9pbnRDb3VudCA9IDA7XHJcblxyXG4gIC8vIENvbXB1dGUgY2lyY2xlIHBvc2l0aW9uIGluIHRoZSBmcmFtZSBvZiB0aGUgcG9seWdvbi5cclxuICBjb25zdCBjOiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxYVih4ZkIsIGNpcmNsZUIubV9wLCBiMkNvbGxpZGVQb2x5Z29uQW5kQ2lyY2xlX3NfYyk7XHJcbiAgY29uc3QgY0xvY2FsOiBiMlZlYzIgPSBiMlRyYW5zZm9ybS5NdWxUWFYoeGZBLCBjLCBiMkNvbGxpZGVQb2x5Z29uQW5kQ2lyY2xlX3NfY0xvY2FsKTtcclxuXHJcbiAgLy8gRmluZCB0aGUgbWluIHNlcGFyYXRpbmcgZWRnZS5cclxuICBsZXQgbm9ybWFsSW5kZXggPSAwO1xyXG4gIGxldCBzZXBhcmF0aW9uOiBudW1iZXIgPSAtYjJfbWF4RmxvYXQ7XHJcbiAgY29uc3QgcmFkaXVzOiBudW1iZXIgPSBwb2x5Z29uQS5tX3JhZGl1cyArIGNpcmNsZUIubV9yYWRpdXM7XHJcbiAgY29uc3QgdmVydGV4Q291bnQ6IG51bWJlciA9IHBvbHlnb25BLm1fY291bnQ7XHJcbiAgY29uc3QgdmVydGljZXM6IGIyVmVjMltdID0gcG9seWdvbkEubV92ZXJ0aWNlcztcclxuICBjb25zdCBub3JtYWxzOiBiMlZlYzJbXSA9IHBvbHlnb25BLm1fbm9ybWFscztcclxuXHJcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCB2ZXJ0ZXhDb3VudDsgKytpKSB7XHJcbiAgICBjb25zdCBzOiBudW1iZXIgPSBiMlZlYzIuRG90VlYobm9ybWFsc1tpXSwgYjJWZWMyLlN1YlZWKGNMb2NhbCwgdmVydGljZXNbaV0sIGIyVmVjMi5zX3QwKSk7XHJcblxyXG4gICAgaWYgKHMgPiByYWRpdXMpIHtcclxuICAgICAgLy8gRWFybHkgb3V0LlxyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHMgPiBzZXBhcmF0aW9uKSB7XHJcbiAgICAgIHNlcGFyYXRpb24gPSBzO1xyXG4gICAgICBub3JtYWxJbmRleCA9IGk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLyBWZXJ0aWNlcyB0aGF0IHN1YnRlbmQgdGhlIGluY2lkZW50IGZhY2UuXHJcbiAgY29uc3QgdmVydEluZGV4MTogbnVtYmVyID0gbm9ybWFsSW5kZXg7XHJcbiAgY29uc3QgdmVydEluZGV4MjogbnVtYmVyID0gKHZlcnRJbmRleDEgKyAxKSAlIHZlcnRleENvdW50O1xyXG4gIGNvbnN0IHYxOiBiMlZlYzIgPSB2ZXJ0aWNlc1t2ZXJ0SW5kZXgxXTtcclxuICBjb25zdCB2MjogYjJWZWMyID0gdmVydGljZXNbdmVydEluZGV4Ml07XHJcblxyXG4gIC8vIElmIHRoZSBjZW50ZXIgaXMgaW5zaWRlIHRoZSBwb2x5Z29uIC4uLlxyXG4gIGlmIChzZXBhcmF0aW9uIDwgYjJfZXBzaWxvbikge1xyXG4gICAgbWFuaWZvbGQucG9pbnRDb3VudCA9IDE7XHJcbiAgICBtYW5pZm9sZC50eXBlID0gYjJNYW5pZm9sZFR5cGUuZV9mYWNlQTtcclxuICAgIG1hbmlmb2xkLmxvY2FsTm9ybWFsLkNvcHkobm9ybWFsc1tub3JtYWxJbmRleF0pO1xyXG4gICAgYjJWZWMyLk1pZFZWKHYxLCB2MiwgbWFuaWZvbGQubG9jYWxQb2ludCk7XHJcbiAgICBtYW5pZm9sZC5wb2ludHNbMF0ubG9jYWxQb2ludC5Db3B5KGNpcmNsZUIubV9wKTtcclxuICAgIG1hbmlmb2xkLnBvaW50c1swXS5pZC5rZXkgPSAwO1xyXG4gICAgcmV0dXJuO1xyXG4gIH1cclxuXHJcbiAgLy8gQ29tcHV0ZSBiYXJ5Y2VudHJpYyBjb29yZGluYXRlc1xyXG4gIGNvbnN0IHUxOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoXHJcbiAgICBiMlZlYzIuU3ViVlYoY0xvY2FsLCB2MSwgYjJWZWMyLnNfdDApLFxyXG4gICAgYjJWZWMyLlN1YlZWKHYyLCB2MSwgYjJWZWMyLnNfdDEpLFxyXG4gICk7XHJcbiAgY29uc3QgdTI6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihcclxuICAgIGIyVmVjMi5TdWJWVihjTG9jYWwsIHYyLCBiMlZlYzIuc190MCksXHJcbiAgICBiMlZlYzIuU3ViVlYodjEsIHYyLCBiMlZlYzIuc190MSksXHJcbiAgKTtcclxuICBpZiAodTEgPD0gMCkge1xyXG4gICAgaWYgKGIyVmVjMi5EaXN0YW5jZVNxdWFyZWRWVihjTG9jYWwsIHYxKSA+IHJhZGl1cyAqIHJhZGl1cykge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgbWFuaWZvbGQucG9pbnRDb3VudCA9IDE7XHJcbiAgICBtYW5pZm9sZC50eXBlID0gYjJNYW5pZm9sZFR5cGUuZV9mYWNlQTtcclxuICAgIGIyVmVjMi5TdWJWVihjTG9jYWwsIHYxLCBtYW5pZm9sZC5sb2NhbE5vcm1hbCkuU2VsZk5vcm1hbGl6ZSgpO1xyXG4gICAgbWFuaWZvbGQubG9jYWxQb2ludC5Db3B5KHYxKTtcclxuICAgIG1hbmlmb2xkLnBvaW50c1swXS5sb2NhbFBvaW50LkNvcHkoY2lyY2xlQi5tX3ApO1xyXG4gICAgbWFuaWZvbGQucG9pbnRzWzBdLmlkLmtleSA9IDA7XHJcbiAgfSBlbHNlIGlmICh1MiA8PSAwKSB7XHJcbiAgICBpZiAoYjJWZWMyLkRpc3RhbmNlU3F1YXJlZFZWKGNMb2NhbCwgdjIpID4gcmFkaXVzICogcmFkaXVzKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBtYW5pZm9sZC5wb2ludENvdW50ID0gMTtcclxuICAgIG1hbmlmb2xkLnR5cGUgPSBiMk1hbmlmb2xkVHlwZS5lX2ZhY2VBO1xyXG4gICAgYjJWZWMyLlN1YlZWKGNMb2NhbCwgdjIsIG1hbmlmb2xkLmxvY2FsTm9ybWFsKS5TZWxmTm9ybWFsaXplKCk7XHJcbiAgICBtYW5pZm9sZC5sb2NhbFBvaW50LkNvcHkodjIpO1xyXG4gICAgbWFuaWZvbGQucG9pbnRzWzBdLmxvY2FsUG9pbnQuQ29weShjaXJjbGVCLm1fcCk7XHJcbiAgICBtYW5pZm9sZC5wb2ludHNbMF0uaWQua2V5ID0gMDtcclxuICB9IGVsc2Uge1xyXG4gICAgY29uc3QgZmFjZUNlbnRlcjogYjJWZWMyID0gYjJWZWMyLk1pZFZWKHYxLCB2MiwgYjJDb2xsaWRlUG9seWdvbkFuZENpcmNsZV9zX2ZhY2VDZW50ZXIpO1xyXG4gICAgY29uc3Qgc2VwYXJhdGlvbiA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgYjJWZWMyLlN1YlZWKGNMb2NhbCwgZmFjZUNlbnRlciwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICBub3JtYWxzW3ZlcnRJbmRleDFdLFxyXG4gICAgKTtcclxuICAgIGlmIChzZXBhcmF0aW9uID4gcmFkaXVzKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBtYW5pZm9sZC5wb2ludENvdW50ID0gMTtcclxuICAgIG1hbmlmb2xkLnR5cGUgPSBiMk1hbmlmb2xkVHlwZS5lX2ZhY2VBO1xyXG4gICAgbWFuaWZvbGQubG9jYWxOb3JtYWwuQ29weShub3JtYWxzW3ZlcnRJbmRleDFdKS5TZWxmTm9ybWFsaXplKCk7XHJcbiAgICBtYW5pZm9sZC5sb2NhbFBvaW50LkNvcHkoZmFjZUNlbnRlcik7XHJcbiAgICBtYW5pZm9sZC5wb2ludHNbMF0ubG9jYWxQb2ludC5Db3B5KGNpcmNsZUIubV9wKTtcclxuICAgIG1hbmlmb2xkLnBvaW50c1swXS5pZC5rZXkgPSAwO1xyXG4gIH1cclxufVxyXG4iXX0=