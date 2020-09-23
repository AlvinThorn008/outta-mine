/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
// #if B2_ENABLE_CONTROLLER
import { b2Controller } from './b2Controller';
import { b2Vec2 } from '../common/b2Math';
import { b2_epsilon } from '../common/b2Settings';
import { b2Color } from '../common/b2Draw';
/**
 * Calculates buoyancy forces for fluids in the form of a half
 * plane.
 */
export class b2BuoyancyController extends b2Controller {
    constructor() {
        super(...arguments);
        /**
         * The outer surface normal
         */
        this.normal = new b2Vec2(0, 1);
        /**
         * The height of the fluid surface along the normal
         */
        this.offset = 0;
        /**
         * The fluid density
         */
        this.density = 0;
        /**
         * Fluid velocity, for drag calculations
         */
        this.velocity = new b2Vec2(0, 0);
        /**
         * Linear drag co-efficient
         */
        this.linearDrag = 0;
        /**
         * Angular drag co-efficient
         */
        this.angularDrag = 0;
        /**
         * If false, bodies are assumed to be uniformly dense, otherwise
         * use the shapes densities
         */
        this.useDensity = false; //False by default to prevent a gotcha
        /**
         * If true, gravity is taken from the world instead of the
         */
        this.useWorldGravity = true;
        /**
         * Gravity vector, if the world's gravity is not used
         */
        this.gravity = new b2Vec2(0, 0);
    }
    Step(step) {
        if (!this.m_bodyList) {
            return;
        }
        if (this.useWorldGravity) {
            this.gravity.Copy(this.m_bodyList.body.GetWorld().GetGravity());
        }
        for (let i = this.m_bodyList; i; i = i.nextBody) {
            const body = i.body;
            if (!body.IsAwake()) {
                //Buoyancy force is just a function of position,
                //so unlike most forces, it is safe to ignore sleeping bodes
                continue;
            }
            const areac = new b2Vec2();
            const massc = new b2Vec2();
            let area = 0;
            let mass = 0;
            for (let fixture = body.GetFixtureList(); fixture; fixture = fixture.m_next) {
                const sc = new b2Vec2();
                const sarea = fixture
                    .GetShape()
                    .ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
                area += sarea;
                areac.x += sarea * sc.x;
                areac.y += sarea * sc.y;
                let shapeDensity = 0;
                if (this.useDensity) {
                    //TODO: Expose density publicly
                    shapeDensity = fixture.GetDensity();
                }
                else {
                    shapeDensity = 1;
                }
                mass += sarea * shapeDensity;
                massc.x += sarea * sc.x * shapeDensity;
                massc.y += sarea * sc.y * shapeDensity;
            }
            areac.x /= area;
            areac.y /= area;
            //    b2Vec2 localCentroid = b2MulT(body->GetXForm(),areac);
            massc.x /= mass;
            massc.y /= mass;
            if (area < b2_epsilon) {
                continue;
            }
            //Buoyancy
            const buoyancyForce = this.gravity.Clone().SelfNeg();
            buoyancyForce.SelfMul(this.density * area);
            body.ApplyForce(buoyancyForce, massc);
            //Linear drag
            const dragForce = body.GetLinearVelocityFromWorldPoint(areac, new b2Vec2());
            dragForce.SelfSub(this.velocity);
            dragForce.SelfMul(-this.linearDrag * area);
            body.ApplyForce(dragForce, areac);
            //Angular drag
            //TODO: Something that makes more physical sense?
            body.ApplyTorque((-body.GetInertia() / body.GetMass()) * area * body.GetAngularVelocity() * this.angularDrag);
        }
    }
    Draw(debugDraw) {
        const r = 100;
        const p1 = new b2Vec2();
        const p2 = new b2Vec2();
        p1.x = this.normal.x * this.offset + this.normal.y * r;
        p1.y = this.normal.y * this.offset - this.normal.x * r;
        p2.x = this.normal.x * this.offset - this.normal.y * r;
        p2.y = this.normal.y * this.offset + this.normal.x * r;
        const color = new b2Color(0, 0, 0.8);
        debugDraw.DrawSegment(p1, p2, color);
    }
}
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJCdW95YW5jeUNvbnRyb2xsZXIuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9zcmMvY29udHJvbGxlcnMvYjJCdW95YW5jeUNvbnRyb2xsZXIudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCwyQkFBMkI7QUFFM0IsT0FBTyxFQUFFLFlBQVksRUFBb0IsTUFBTSxnQkFBZ0IsQ0FBQztBQUNoRSxPQUFPLEVBQUUsTUFBTSxFQUFFLE1BQU0sa0JBQWtCLENBQUM7QUFFMUMsT0FBTyxFQUFFLFVBQVUsRUFBRSxNQUFNLHNCQUFzQixDQUFDO0FBQ2xELE9BQU8sRUFBRSxPQUFPLEVBQVUsTUFBTSxrQkFBa0IsQ0FBQztBQUVuRDs7O0dBR0c7QUFDSCxNQUFNLE9BQU8sb0JBQXFCLFNBQVEsWUFBWTtJQUF0RDs7UUFDRTs7V0FFRztRQUNNLFdBQU0sR0FBRyxJQUFJLE1BQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDbkM7O1dBRUc7UUFDSCxXQUFNLEdBQUcsQ0FBQyxDQUFDO1FBQ1g7O1dBRUc7UUFDSCxZQUFPLEdBQUcsQ0FBQyxDQUFDO1FBQ1o7O1dBRUc7UUFDTSxhQUFRLEdBQUcsSUFBSSxNQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3JDOztXQUVHO1FBQ0gsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUNmOztXQUVHO1FBQ0gsZ0JBQVcsR0FBRyxDQUFDLENBQUM7UUFDaEI7OztXQUdHO1FBQ0gsZUFBVSxHQUFHLEtBQUssQ0FBQyxDQUFDLHNDQUFzQztRQUMxRDs7V0FFRztRQUNILG9CQUFlLEdBQUcsSUFBSSxDQUFDO1FBQ3ZCOztXQUVHO1FBQ00sWUFBTyxHQUFHLElBQUksTUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztJQTZFdEMsQ0FBQztJQTNFQyxJQUFJLENBQUMsSUFBZ0I7UUFDbkIsSUFBSSxDQUFDLElBQUks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Q0FBQyxHQUFHLENBQUMsQ0FBQztRQUV2RCxNQUFNLEtBQUssR0FBRyxJQUFJLE9BQU8sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxFQUFFLEdBQUcsQ0FBQyxDQUFDO1FBRXJDLFNBQVMsQ0FBQyxXQUFXLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxLQUFLLENBQUMsQ0FBQztJQUN2QyxDQUFDO0NBQ0Y7QUFFRCxTQUFTIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMDkgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG4vLyAjaWYgQjJfRU5BQkxFX0NPTlRST0xMRVJcclxuXHJcbmltcG9ydCB7IGIyQ29udHJvbGxlciwgYjJDb250cm9sbGVyRWRnZSB9IGZyb20gJy4vYjJDb250cm9sbGVyJztcclxuaW1wb3J0IHsgYjJWZWMyIH0gZnJvbSAnLi4vY29tbW9uL2IyTWF0aCc7XHJcbmltcG9ydCB7IGIyVGltZVN0ZXAgfSBmcm9tICcuLi9keW5hbWljcy9iMlRpbWVTdGVwJztcclxuaW1wb3J0IHsgYjJfZXBzaWxvbiB9IGZyb20gJy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJDb2xvciwgYjJEcmF3IH0gZnJvbSAnLi4vY29tbW9uL2IyRHJhdyc7XHJcblxyXG4vKipcclxuICogQ2FsY3VsYXRlcyBidW95YW5jeSBmb3JjZXMgZm9yIGZsdWlkcyBpbiB0aGUgZm9ybSBvZiBhIGhhbGZcclxuICogcGxhbmUuXHJcbiAqL1xyXG5leHBvcnQgY2xhc3MgYjJCdW95YW5jeUNvbnRyb2xsZXIgZXh0ZW5kcyBiMkNvbnRyb2xsZXIge1xyXG4gIC8qKlxyXG4gICAqIFRoZSBvdXRlciBzdXJmYWNlIG5vcm1hbFxyXG4gICAqL1xyXG4gIHJlYWRvbmx5IG5vcm1hbCA9IG5ldyBiMlZlYzIoMCwgMSk7XHJcbiAgLyoqXHJcbiAgICogVGhlIGhlaWdodCBvZiB0aGUgZmx1aWQgc3VyZmFjZSBhbG9uZyB0aGUgbm9ybWFsXHJcbiAgICovXHJcbiAgb2Zmc2V0ID0gMDtcclxuICAvKipcclxuICAgKiBUaGUgZmx1aWQgZGVuc2l0eVxyXG4gICAqL1xyXG4gIGRlbnNpdHkgPSAwO1xyXG4gIC8qKlxyXG4gICAqIEZsdWlkIHZlbG9jaXR5LCBmb3IgZHJhZyBjYWxjdWxhdGlvbnNcclxuICAgKi9cclxuICByZWFkb25seSB2ZWxvY2l0eSA9IG5ldyBiMlZlYzIoMCwgMCk7XHJcbiAgLyoqXHJcbiAgICogTGluZWFyIGRyYWcgY28tZWZmaWNpZW50XHJcbiAgICovXHJcbiAgbGluZWFyRHJhZyA9IDA7XHJcbiAgLyoqXHJcbiAgICogQW5ndWxhciBkcmFnIGNvLWVmZmljaWVudFxyXG4gICAqL1xyXG4gIGFuZ3VsYXJEcmFnID0gMDtcclxuICAvKipcclxuICAgKiBJZiBmYWxzZSwgYm9kaWVzIGFyZSBhc3N1bWVkIHRvIGJlIHVuaWZvcm1seSBkZW5zZSwgb3RoZXJ3aXNlXHJcbiAgICogdXNlIHRoZSBzaGFwZXMgZGVuc2l0aWVzXHJcbiAgICovXHJcbiAgdXNlRGVuc2l0eSA9IGZhbHNlOyAvL0ZhbHNlIGJ5IGRlZmF1bHQgdG8gcHJldmVudCBhIGdvdGNoYVxyXG4gIC8qKlxyXG4gICAqIElmIHRydWUsIGdyYXZpdHkgaXMgdGFrZW4gZnJvbSB0aGUgd29ybGQgaW5zdGVhZCBvZiB0aGVcclxuICAgKi9cclxuICB1c2VXb3JsZEdyYXZpdHkgPSB0cnVlO1xyXG4gIC8qKlxyXG4gICAqIEdyYXZpdHkgdmVjdG9yLCBpZiB0aGUgd29ybGQncyBncmF2aXR5IGlzIG5vdCB1c2VkXHJcbiAgICovXHJcbiAgcmVhZG9ubHkgZ3Jhdml0eSA9IG5ldyBiMlZlYzIoMCwgMCk7XHJcblxyXG4gIFN0ZXAoc3RlcDogYjJUaW1lU3RlcCkge1xyXG4gICAgaWYgKCF0aGlzLm1fYm9keUxpc3QpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMudXNlV29ybGRHcmF2aXR5KSB7XHJcbiAgICAgIHRoaXMuZ3Jhdml0eS5Db3B5KHRoaXMubV9ib2R5TGlzdC5ib2R5LkdldFdvcmxkKCkuR2V0R3Jhdml0eSgpKTtcclxuICAgIH1cclxuICAgIGZvciAobGV0IGk6IGIyQ29udHJvbGxlckVkZ2UgfCBudWxsID0gdGhpcy5tX2JvZHlMaXN0OyBpOyBpID0gaS5uZXh0Qm9keSkge1xyXG4gICAgICBjb25zdCBib2R5ID0gaS5ib2R5O1xyXG4gICAgICBpZiAoIWJvZHkuSXNBd2FrZSgpKSB7XHJcbiAgICAgICAgLy9CdW95YW5jeSBmb3JjZSBpcyBqdXN0IGEgZnVuY3Rpb24gb2YgcG9zaXRpb24sXHJcbiAgICAgICAgLy9zbyB1bmxpa2UgbW9zdCBmb3JjZXMsIGl0IGlzIHNhZmUgdG8gaWdub3JlIHNsZWVwaW5nIGJvZGVzXHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuICAgICAgY29uc3QgYXJlYWMgPSBuZXcgYjJWZWMyKCk7XHJcbiAgICAgIGNvbnN0IG1hc3NjID0gbmV3IGIyVmVjMigpO1xyXG4gICAgICBsZXQgYXJlYSA9IDA7XHJcbiAgICAgIGxldCBtYXNzID0gMDtcclxuICAgICAgZm9yIChsZXQgZml4dHVyZSA9IGJvZHkuR2V0Rml4dHVyZUxpc3QoKTsgZml4dHVyZTsgZml4dHVyZSA9IGZpeHR1cmUubV9uZXh0KSB7XHJcbiAgICAgICAgY29uc3Qgc2MgPSBuZXcgYjJWZWMyKCk7XHJcbiAgICAgICAgY29uc3Qgc2FyZWEgPSBmaXh0dXJlXHJcbiAgICAgICAgICAuR2V0U2hhcGUoKVxyXG4gICAgICAgICAgLkNvbXB1dGVTdWJtZXJnZWRBcmVhKHRoaXMubm9ybWFsLCB0aGlzLm9mZnNldCwgYm9keS5HZXRUcmFuc2Zvcm0oKSwgc2MpO1xyXG4gICAgICAgIGFyZWEgKz0gc2FyZWE7XHJcbiAgICAgICAgYXJlYWMueCArPSBzYXJlYSAqIHNjLng7XHJcbiAgICAgICAgYXJlYWMueSArPSBzYXJlYSAqIHNjLnk7XHJcbiAgICAgICAgbGV0IHNoYXBlRGVuc2l0eSA9IDA7XHJcbiAgICAgICAgaWYgKHRoaXMudXNlRGVuc2l0eSkge1xyXG4gICAgICAgICAgLy9UT0RPOiBFeHBvc2UgZGVuc2l0eSBwdWJsaWNseVxyXG4gICAgICAgICAgc2hhcGVEZW5zaXR5ID0gZml4dHVyZS5HZXREZW5zaXR5KCk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHNoYXBlRGVuc2l0eSA9IDE7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIG1hc3MgKz0gc2FyZWEgKiBzaGFwZURlbnNpdHk7XHJcbiAgICAgICAgbWFzc2MueCArPSBzYXJlYSAqIHNjLnggKiBzaGFwZURlbnNpdHk7XHJcbiAgICAgICAgbWFzc2MueSArPSBzYXJlYSAqIHNjLnkgKiBzaGFwZURlbnNpdHk7XHJcbiAgICAgIH1cclxuICAgICAgYXJlYWMueCAvPSBhcmVhO1xyXG4gICAgICBhcmVhYy55IC89IGFyZWE7XHJcbiAgICAgIC8vICAgIGIyVmVjMiBsb2NhbENlbnRyb2lkID0gYjJNdWxUKGJvZHktPkdldFhGb3JtKCksYXJlYWMpO1xyXG4gICAgICBtYXNzYy54IC89IG1hc3M7XHJcbiAgICAgIG1hc3NjLnkgLz0gbWFzcztcclxuICAgICAgaWYgKGFyZWEgPCBiMl9lcHNpbG9uKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuICAgICAgLy9CdW95YW5jeVxyXG4gICAgICBjb25zdCBidW95YW5jeUZvcmNlID0gdGhpcy5ncmF2aXR5LkNsb25lKCkuU2VsZk5lZygpO1xyXG4gICAgICBidW95YW5jeUZvcmNlLlNlbGZNdWwodGhpcy5kZW5zaXR5ICogYXJlYSk7XHJcbiAgICAgIGJvZHkuQXBwbHlGb3JjZShidW95YW5jeUZvcmNlLCBtYXNzYyk7XHJcbiAgICAgIC8vTGluZWFyIGRyYWdcclxuICAgICAgY29uc3QgZHJhZ0ZvcmNlID0gYm9keS5HZXRMaW5lYXJWZWxvY2l0eUZyb21Xb3JsZFBvaW50KGFyZWFjLCBuZXcgYjJWZWMyKCkpO1xyXG4gICAgICBkcmFnRm9yY2UuU2VsZlN1Yih0aGlzLnZlbG9jaXR5KTtcclxuICAgICAgZHJhZ0ZvcmNlLlNlbGZNdWwoLXRoaXMubGluZWFyRHJhZyAqIGFyZWEpO1xyXG4gICAgICBib2R5LkFwcGx5Rm9yY2UoZHJhZ0ZvcmNlLCBhcmVhYyk7XHJcbiAgICAgIC8vQW5ndWxhciBkcmFnXHJcbiAgICAgIC8vVE9ETzogU29tZXRoaW5nIHRoYXQgbWFrZXMgbW9yZSBwaHlzaWNhbCBzZW5zZT9cclxuICAgICAgYm9keS5BcHBseVRvcnF1ZShcclxuICAgICAgICAoLWJvZHkuR2V0SW5lcnRpYSgpIC8gYm9keS5HZXRNYXNzKCkpICogYXJlYSAqIGJvZHkuR2V0QW5ndWxhclZlbG9jaXR5KCkgKiB0aGlzLmFuZ3VsYXJEcmFnLFxyXG4gICAgICApO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgRHJhdyhkZWJ1Z0RyYXc6IGIyRHJhdykge1xyXG4gICAgY29uc3QgciA9IDEwMDtcclxuICAgIGNvbnN0IHAxID0gbmV3IGIyVmVjMigpO1xyXG4gICAgY29uc3QgcDIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgICBwMS54ID0gdGhpcy5ub3JtYWwueCAqIHRoaXMub2Zmc2V0ICsgdGhpcy5ub3JtYWwueSAqIHI7XHJcbiAgICBwMS55ID0gdGhpcy5ub3JtYWwueSAqIHRoaXMub2Zmc2V0IC0gdGhpcy5ub3JtYWwueCAqIHI7XHJcbiAgICBwMi54ID0gdGhpcy5ub3JtYWwueCAqIHRoaXMub2Zmc2V0IC0gdGhpcy5ub3JtYWwueSAqIHI7XHJcbiAgICBwMi55ID0gdGhpcy5ub3JtYWwueSAqIHRoaXMub2Zmc2V0ICsgdGhpcy5ub3JtYWwueCAqIHI7XHJcblxyXG4gICAgY29uc3QgY29sb3IgPSBuZXcgYjJDb2xvcigwLCAwLCAwLjgpO1xyXG5cclxuICAgIGRlYnVnRHJhdy5EcmF3U2VnbWVudChwMSwgcDIsIGNvbG9yKTtcclxuICB9XHJcbn1cclxuXHJcbi8vICNlbmRpZlxyXG4iXX0=