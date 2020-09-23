/*
 * Copyright (c) 2011 Erin Catto http://www.box2d.org
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
import { b2_pi, b2Assert, b2MakeNumberArray } from '../common/b2Settings';
import { b2Atan2, b2Vec2 } from '../common/b2Math';
import { b2Color } from '../common/b2Draw';
///
export class b2RopeDef {
    constructor() {
        ///
        this.vertices = [];
        ///
        this.count = 0;
        ///
        this.masses = [];
        ///
        this.gravity = new b2Vec2(0, 0);
        ///
        this.damping = 0.1;
        /// Stretching stiffness
        this.k2 = 0.9;
        /// Bending stiffness. Values above 0.5 can make the simulation blow up.
        this.k3 = 0.1;
    }
}
///
export class b2Rope {
    constructor() {
        this.m_count = 0;
        this.m_ps = [];
        this.m_p0s = [];
        this.m_vs = [];
        this.m_ims = [];
        this.m_Ls = [];
        this.m_as = [];
        this.m_gravity = new b2Vec2();
        this.m_damping = 0;
        this.m_k2 = 1;
        this.m_k3 = 0.1;
    }
    GetVertexCount() {
        return this.m_count;
    }
    GetVertices() {
        return this.m_ps;
    }
    ///
    Initialize(def) {
        !!B2_DEBUG && b2Assert(def.count >= 3);
        this.m_count = def.count;
        // this.m_ps = (b2Vec2*)b2Alloc(this.m_count * sizeof(b2Vec2));
        this.m_ps = b2Vec2.MakeArray(this.m_count);
        // this.m_p0s = (b2Vec2*)b2Alloc(this.m_count * sizeof(b2Vec2));
        this.m_p0s = b2Vec2.MakeArray(this.m_count);
        // this.m_vs = (b2Vec2*)b2Alloc(this.m_count * sizeof(b2Vec2));
        this.m_vs = b2Vec2.MakeArray(this.m_count);
        // this.m_ims = (float32*)b2Alloc(this.m_count * sizeof(float32));
        this.m_ims = b2MakeNumberArray(this.m_count);
        for (let i = 0; i < this.m_count; ++i) {
            this.m_ps[i].Copy(def.vertices[i]);
            this.m_p0s[i].Copy(def.vertices[i]);
            this.m_vs[i].SetZero();
            const m = def.masses[i];
            if (m > 0) {
                this.m_ims[i] = 1 / m;
            }
            else {
                this.m_ims[i] = 0;
            }
        }
        const count2 = this.m_count - 1;
        const count3 = this.m_count - 2;
        // this.m_Ls = (float32*)be2Alloc(count2 * sizeof(float32));
        this.m_Ls = b2MakeNumberArray(count2);
        // this.m_as = (float32*)b2Alloc(count3 * sizeof(float32));
        this.m_as = b2MakeNumberArray(count3);
        for (let i = 0; i < count2; ++i) {
            const p1 = this.m_ps[i];
            const p2 = this.m_ps[i + 1];
            this.m_Ls[i] = b2Vec2.DistanceVV(p1, p2);
        }
        for (let i = 0; i < count3; ++i) {
            const p1 = this.m_ps[i];
            const p2 = this.m_ps[i + 1];
            const p3 = this.m_ps[i + 2];
            const d1 = b2Vec2.SubVV(p2, p1, b2Vec2.s_t0);
            const d2 = b2Vec2.SubVV(p3, p2, b2Vec2.s_t1);
            const a = b2Vec2.CrossVV(d1, d2);
            const b = b2Vec2.DotVV(d1, d2);
            this.m_as[i] = b2Atan2(a, b);
        }
        this.m_gravity.Copy(def.gravity);
        this.m_damping = def.damping;
        this.m_k2 = def.k2;
        this.m_k3 = def.k3;
    }
    ///
    Step(h, iterations) {
        if (h === 0) {
            return;
        }
        const d = Math.exp(-h * this.m_damping);
        for (let i = 0; i < this.m_count; ++i) {
            this.m_p0s[i].Copy(this.m_ps[i]);
            if (this.m_ims[i] > 0) {
                this.m_vs[i].SelfMulAdd(h, this.m_gravity);
            }
            this.m_vs[i].SelfMul(d);
            this.m_ps[i].SelfMulAdd(h, this.m_vs[i]);
        }
        for (let i = 0; i < iterations; ++i) {
            this.SolveC2();
            this.SolveC3();
            this.SolveC2();
        }
        const inv_h = 1 / h;
        for (let i = 0; i < this.m_count; ++i) {
            b2Vec2.MulSV(inv_h, b2Vec2.SubVV(this.m_ps[i], this.m_p0s[i], b2Vec2.s_t0), this.m_vs[i]);
        }
    }
    SolveC2() {
        const count2 = this.m_count - 1;
        for (let i = 0; i < count2; ++i) {
            const p1 = this.m_ps[i];
            const p2 = this.m_ps[i + 1];
            const d = b2Vec2.SubVV(p2, p1, b2Rope.s_d);
            const L = d.Normalize();
            const im1 = this.m_ims[i];
            const im2 = this.m_ims[i + 1];
            if (im1 + im2 === 0) {
                continue;
            }
            const s1 = im1 / (im1 + im2);
            const s2 = im2 / (im1 + im2);
            p1.SelfMulSub(this.m_k2 * s1 * (this.m_Ls[i] - L), d);
            p2.SelfMulAdd(this.m_k2 * s2 * (this.m_Ls[i] - L), d);
            // this.m_ps[i] = p1;
            // this.m_ps[i + 1] = p2;
        }
    }
    SetAngle(angle) {
        const count3 = this.m_count - 2;
        for (let i = 0; i < count3; ++i) {
            this.m_as[i] = angle;
        }
    }
    SolveC3() {
        const count3 = this.m_count - 2;
        for (let i = 0; i < count3; ++i) {
            const p1 = this.m_ps[i];
            const p2 = this.m_ps[i + 1];
            const p3 = this.m_ps[i + 2];
            const m1 = this.m_ims[i];
            const m2 = this.m_ims[i + 1];
            const m3 = this.m_ims[i + 2];
            const d1 = b2Vec2.SubVV(p2, p1, b2Rope.s_d1);
            const d2 = b2Vec2.SubVV(p3, p2, b2Rope.s_d2);
            const L1sqr = d1.LengthSquared();
            const L2sqr = d2.LengthSquared();
            if (L1sqr * L2sqr === 0) {
                continue;
            }
            const a = b2Vec2.CrossVV(d1, d2);
            const b = b2Vec2.DotVV(d1, d2);
            let angle = b2Atan2(a, b);
            const Jd1 = b2Vec2.MulSV(-1 / L1sqr, d1.SelfSkew(), b2Rope.s_Jd1);
            const Jd2 = b2Vec2.MulSV(1 / L2sqr, d2.SelfSkew(), b2Rope.s_Jd2);
            const J1 = b2Vec2.NegV(Jd1, b2Rope.s_J1);
            const J2 = b2Vec2.SubVV(Jd1, Jd2, b2Rope.s_J2);
            const J3 = Jd2;
            let mass = m1 * b2Vec2.DotVV(J1, J1) + m2 * b2Vec2.DotVV(J2, J2) + m3 * b2Vec2.DotVV(J3, J3);
            if (mass === 0) {
                continue;
            }
            mass = 1 / mass;
            let C = angle - this.m_as[i];
            while (C > b2_pi) {
                angle -= 2 * b2_pi;
                C = angle - this.m_as[i];
            }
            while (C < -b2_pi) {
                angle += 2 * b2_pi;
                C = angle - this.m_as[i];
            }
            const impulse = -this.m_k3 * mass * C;
            p1.SelfMulAdd(m1 * impulse, J1);
            p2.SelfMulAdd(m2 * impulse, J2);
            p3.SelfMulAdd(m3 * impulse, J3);
            // this.m_ps[i] = p1;
            // this.m_ps[i + 1] = p2;
            // this.m_ps[i + 2] = p3;
        }
    }
    Draw(draw) {
        const c = new b2Color(0.4, 0.5, 0.7);
        for (let i = 0; i < this.m_count - 1; ++i) {
            draw.DrawSegment(this.m_ps[i], this.m_ps[i + 1], c);
        }
    }
}
///
b2Rope.s_d = new b2Vec2();
b2Rope.s_d1 = new b2Vec2();
b2Rope.s_d2 = new b2Vec2();
b2Rope.s_Jd1 = new b2Vec2();
b2Rope.s_Jd2 = new b2Vec2();
b2Rope.s_J1 = new b2Vec2();
b2Rope.s_J2 = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJSb3BlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL3JvcGUvYjJSb3BlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLEtBQUssRUFBRSxRQUFRLEVBQUUsaUJBQWlCLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUMxRSxPQUFPLEVBQUUsT0FBTyxFQUFFLE1BQU0sRUFBRSxNQUFNLGtCQUFrQixDQUFDO0FBQ25ELE9BQU8sRUFBRSxPQUFPLEVBQVUsTUFBTSxrQkFBa0IsQ0FBQztBQUVuRCxHQUFHO0FBQ0gsTUFBTSxPQUFPLFNBQVM7SUFBdEI7UUFDRSxHQUFHO1FBQ0gsYUFBUSxHQUFhLEVBQUUsQ0FBQztRQUV4QixHQUFHO1FBQ0gsVUFBSyxHQUFHLENBQUMsQ0FBQztRQUVWLEdBQUc7UUFDSCxXQUFNLEdBQWEsRUFBRSxDQUFDO1FBRXRCLEdBQUc7UUFDTSxZQUFPLEdBQVcsSUFBSSxNQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRTVDLEdBQUc7UUFDSCxZQUFPLEdBQUcsR0FBRyxDQUFDO1FBRWQsd0JBQXdCO1FBQ3hCLE9BQUUsR0FBRyxHQUFHLENBQUM7UUFFVCx3RUFBd0U7UUFDeEUsT0FBRSxHQUFHLEdBQUcsQ0FBQztJQUNYLENBQUM7Q0FBQTtBQUVELEdBQUc7QUFDSCxNQUFNLE9BQU8sTUFBTTtJQUFuQjtRQUNFLFlBQU8sR0FBRyxDQUFDLENBQUM7UUFDWixTQUFJLEdBQWEsRUFBRSxDQUFDO1FBQ3BCLFVBQUssR0FBYSxFQUFFLENBQUM7UUFDckIsU0FBSSxHQUFhLEVBQUUsQ0FBQztRQUVwQixVQUFLLEdBQWEsRUFBRSxDQUFDO1FBRXJCLFNBQUksR0FBYSxFQUFFLENBQUM7UUFDcEIsU0FBSSxHQUFhLEVBQUUsQ0FBQztRQUVYLGNBQVMsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQzFDLGNBQVMsR0FB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O1lBRWhDLHFCQUFxQjtZQUNyQix5QkFBeUI7WUFDekIseUJBQXlCO1NBQzFCO0lBQ0gsQ0FBQztJQUVELElBQUksQ0FBQyxJQUFZO1FBQ2YsTUFBTSxDQUFDLEdBQVksSUFBSSxPQUFPLENBQUMsR0FBRyxFQUFFLEdBQUcsRUFBRSxHQUFHLENBQUMsQ0FBQztRQUU5QyxLQUFLLElBQUksQ0FBQyxHQUFHLENBQUMsRUFBRSxDQUFDLEdBQUcsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLEVBQUUsRUFBRSxDQUFDLEVBQUU7WUFDekMsSUFBSSxDQUFDLFdBQVcsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQyxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1NBQ3JEO0lBQ0gsQ0FBQzs7QUFySEQsR0FBRztBQUNZLFVBQUcsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBcUNuQixXQUFJLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNwQixXQUFJLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNwQixZQUFLLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNyQixZQUFLLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNyQixXQUFJLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNwQixXQUFJLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAxMSBFcmluIENhdHRvIGh0dHA6Ly93d3cuYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7IGIyX3BpLCBiMkFzc2VydCwgYjJNYWtlTnVtYmVyQXJyYXkgfSBmcm9tICcuLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7IGIyQXRhbjIsIGIyVmVjMiB9IGZyb20gJy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMkNvbG9yLCBiMkRyYXcgfSBmcm9tICcuLi9jb21tb24vYjJEcmF3JztcclxuXHJcbi8vL1xyXG5leHBvcnQgY2xhc3MgYjJSb3BlRGVmIHtcclxuICAvLy9cclxuICB2ZXJ0aWNlczogYjJWZWMyW10gPSBbXTtcclxuXHJcbiAgLy8vXHJcbiAgY291bnQgPSAwO1xyXG5cclxuICAvLy9cclxuICBtYXNzZXM6IG51bWJlcltdID0gW107XHJcblxyXG4gIC8vL1xyXG4gIHJlYWRvbmx5IGdyYXZpdHk6IGIyVmVjMiA9IG5ldyBiMlZlYzIoMCwgMCk7XHJcblxyXG4gIC8vL1xyXG4gIGRhbXBpbmcgPSAwLjE7XHJcblxyXG4gIC8vLyBTdHJldGNoaW5nIHN0aWZmbmVzc1xyXG4gIGsyID0gMC45O1xyXG5cclxuICAvLy8gQmVuZGluZyBzdGlmZm5lc3MuIFZhbHVlcyBhYm92ZSAwLjUgY2FuIG1ha2UgdGhlIHNpbXVsYXRpb24gYmxvdyB1cC5cclxuICBrMyA9IDAuMTtcclxufVxyXG5cclxuLy8vXHJcbmV4cG9ydCBjbGFzcyBiMlJvcGUge1xyXG4gIG1fY291bnQgPSAwO1xyXG4gIG1fcHM6IGIyVmVjMltdID0gW107XHJcbiAgbV9wMHM6IGIyVmVjMltdID0gW107XHJcbiAgbV92czogYjJWZWMyW10gPSBbXTtcclxuXHJcbiAgbV9pbXM6IG51bWJlcltdID0gW107XHJcblxyXG4gIG1fTHM6IG51bWJlcltdID0gW107XHJcbiAgbV9hczogbnVtYmVyW10gPSBbXTtcclxuXHJcbiAgcmVhZG9ubHkgbV9ncmF2aXR5OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgbV9kYW1waW5nID0gMDtcclxuXHJcbiAgbV9rMiA9IDE7XHJcbiAgbV9rMyA9IDAuMTtcclxuXHJcbiAgR2V0VmVydGV4Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY291bnQ7XHJcbiAgfVxyXG5cclxuICBHZXRWZXJ0aWNlcygpOiBiMlZlYzJbXSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3BzO1xyXG4gIH1cclxuXHJcbiAgLy8vXHJcbiAgSW5pdGlhbGl6ZShkZWY6IGIyUm9wZURlZik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChkZWYuY291bnQgPj0gMyk7XHJcbiAgICB0aGlzLm1fY291bnQgPSBkZWYuY291bnQ7XHJcbiAgICAvLyB0aGlzLm1fcHMgPSAoYjJWZWMyKiliMkFsbG9jKHRoaXMubV9jb3VudCAqIHNpemVvZihiMlZlYzIpKTtcclxuICAgIHRoaXMubV9wcyA9IGIyVmVjMi5NYWtlQXJyYXkodGhpcy5tX2NvdW50KTtcclxuICAgIC8vIHRoaXMubV9wMHMgPSAoYjJWZWMyKiliMkFsbG9jKHRoaXMubV9jb3VudCAqIHNpemVvZihiMlZlYzIpKTtcclxuICAgIHRoaXMubV9wMHMgPSBiMlZlYzIuTWFrZUFycmF5KHRoaXMubV9jb3VudCk7XHJcbiAgICAvLyB0aGlzLm1fdnMgPSAoYjJWZWMyKiliMkFsbG9jKHRoaXMubV9jb3VudCAqIHNpemVvZihiMlZlYzIpKTtcclxuICAgIHRoaXMubV92cyA9IGIyVmVjMi5NYWtlQXJyYXkodGhpcy5tX2NvdW50KTtcclxuICAgIC8vIHRoaXMubV9pbXMgPSAoZmxvYXQzMiopYjJBbGxvYyh0aGlzLm1fY291bnQgKiBzaXplb2YoZmxvYXQzMikpO1xyXG4gICAgdGhpcy5tX2ltcyA9IGIyTWFrZU51bWJlckFycmF5KHRoaXMubV9jb3VudCk7XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7ICsraSkge1xyXG4gICAgICB0aGlzLm1fcHNbaV0uQ29weShkZWYudmVydGljZXNbaV0pO1xyXG4gICAgICB0aGlzLm1fcDBzW2ldLkNvcHkoZGVmLnZlcnRpY2VzW2ldKTtcclxuICAgICAgdGhpcy5tX3ZzW2ldLlNldFplcm8oKTtcclxuXHJcbiAgICAgIGNvbnN0IG06IG51bWJlciA9IGRlZi5tYXNzZXNbaV07XHJcbiAgICAgIGlmIChtID4gMCkge1xyXG4gICAgICAgIHRoaXMubV9pbXNbaV0gPSAxIC8gbTtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICB0aGlzLm1faW1zW2ldID0gMDtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IGNvdW50MjogbnVtYmVyID0gdGhpcy5tX2NvdW50IC0gMTtcclxuICAgIGNvbnN0IGNvdW50MzogbnVtYmVyID0gdGhpcy5tX2NvdW50IC0gMjtcclxuICAgIC8vIHRoaXMubV9McyA9IChmbG9hdDMyKiliZTJBbGxvYyhjb3VudDIgKiBzaXplb2YoZmxvYXQzMikpO1xyXG4gICAgdGhpcy5tX0xzID0gYjJNYWtlTnVtYmVyQXJyYXkoY291bnQyKTtcclxuICAgIC8vIHRoaXMubV9hcyA9IChmbG9hdDMyKiliMkFsbG9jKGNvdW50MyAqIHNpemVvZihmbG9hdDMyKSk7XHJcbiAgICB0aGlzLm1fYXMgPSBiMk1ha2VOdW1iZXJBcnJheShjb3VudDMpO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgY291bnQyOyArK2kpIHtcclxuICAgICAgY29uc3QgcDE6IGIyVmVjMiA9IHRoaXMubV9wc1tpXTtcclxuICAgICAgY29uc3QgcDI6IGIyVmVjMiA9IHRoaXMubV9wc1tpICsgMV07XHJcbiAgICAgIHRoaXMubV9Mc1tpXSA9IGIyVmVjMi5EaXN0YW5jZVZWKHAxLCBwMik7XHJcbiAgICB9XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBjb3VudDM7ICsraSkge1xyXG4gICAgICBjb25zdCBwMTogYjJWZWMyID0gdGhpcy5tX3BzW2ldO1xyXG4gICAgICBjb25zdCBwMjogYjJWZWMyID0gdGhpcy5tX3BzW2kgKyAxXTtcclxuICAgICAgY29uc3QgcDM6IGIyVmVjMiA9IHRoaXMubV9wc1tpICsgMl07XHJcblxyXG4gICAgICBjb25zdCBkMTogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHAyLCBwMSwgYjJWZWMyLnNfdDApO1xyXG4gICAgICBjb25zdCBkMjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHAzLCBwMiwgYjJWZWMyLnNfdDEpO1xyXG5cclxuICAgICAgY29uc3QgYTogbnVtYmVyID0gYjJWZWMyLkNyb3NzVlYoZDEsIGQyKTtcclxuICAgICAgY29uc3QgYjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGQxLCBkMik7XHJcblxyXG4gICAgICB0aGlzLm1fYXNbaV0gPSBiMkF0YW4yKGEsIGIpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9ncmF2aXR5LkNvcHkoZGVmLmdyYXZpdHkpO1xyXG4gICAgdGhpcy5tX2RhbXBpbmcgPSBkZWYuZGFtcGluZztcclxuICAgIHRoaXMubV9rMiA9IGRlZi5rMjtcclxuICAgIHRoaXMubV9rMyA9IGRlZi5rMztcclxuICB9XHJcblxyXG4gIC8vL1xyXG4gIFN0ZXAoaDogbnVtYmVyLCBpdGVyYXRpb25zOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGlmIChoID09PSAwKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCBkOiBudW1iZXIgPSBNYXRoLmV4cCgtaCAqIHRoaXMubV9kYW1waW5nKTtcclxuXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIHRoaXMubV9wMHNbaV0uQ29weSh0aGlzLm1fcHNbaV0pO1xyXG4gICAgICBpZiAodGhpcy5tX2ltc1tpXSA+IDApIHtcclxuICAgICAgICB0aGlzLm1fdnNbaV0uU2VsZk11bEFkZChoLCB0aGlzLm1fZ3Jhdml0eSk7XHJcbiAgICAgIH1cclxuICAgICAgdGhpcy5tX3ZzW2ldLlNlbGZNdWwoZCk7XHJcbiAgICAgIHRoaXMubV9wc1tpXS5TZWxmTXVsQWRkKGgsIHRoaXMubV92c1tpXSk7XHJcbiAgICB9XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBpdGVyYXRpb25zOyArK2kpIHtcclxuICAgICAgdGhpcy5Tb2x2ZUMyKCk7XHJcbiAgICAgIHRoaXMuU29sdmVDMygpO1xyXG4gICAgICB0aGlzLlNvbHZlQzIoKTtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCBpbnZfaDogbnVtYmVyID0gMSAvIGg7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9jb3VudDsgKytpKSB7XHJcbiAgICAgIGIyVmVjMi5NdWxTVihpbnZfaCwgYjJWZWMyLlN1YlZWKHRoaXMubV9wc1tpXSwgdGhpcy5tX3Awc1tpXSwgYjJWZWMyLnNfdDApLCB0aGlzLm1fdnNbaV0pO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vXHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19kID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZUMyKCk6IHZvaWQge1xyXG4gICAgY29uc3QgY291bnQyOiBudW1iZXIgPSB0aGlzLm1fY291bnQgLSAxO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgY291bnQyOyArK2kpIHtcclxuICAgICAgY29uc3QgcDE6IGIyVmVjMiA9IHRoaXMubV9wc1tpXTtcclxuICAgICAgY29uc3QgcDI6IGIyVmVjMiA9IHRoaXMubV9wc1tpICsgMV07XHJcblxyXG4gICAgICBjb25zdCBkOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYocDIsIHAxLCBiMlJvcGUuc19kKTtcclxuICAgICAgY29uc3QgTDogbnVtYmVyID0gZC5Ob3JtYWxpemUoKTtcclxuXHJcbiAgICAgIGNvbnN0IGltMTogbnVtYmVyID0gdGhpcy5tX2ltc1tpXTtcclxuICAgICAgY29uc3QgaW0yOiBudW1iZXIgPSB0aGlzLm1faW1zW2kgKyAxXTtcclxuXHJcbiAgICAgIGlmIChpbTEgKyBpbTIgPT09IDApIHtcclxuICAgICAgICBjb250aW51ZTtcclxuICAgICAgfVxyXG5cclxuICAgICAgY29uc3QgczE6IG51bWJlciA9IGltMSAvIChpbTEgKyBpbTIpO1xyXG4gICAgICBjb25zdCBzMjogbnVtYmVyID0gaW0yIC8gKGltMSArIGltMik7XHJcblxyXG4gICAgICBwMS5TZWxmTXVsU3ViKHRoaXMubV9rMiAqIHMxICogKHRoaXMubV9Mc1tpXSAtIEwpLCBkKTtcclxuICAgICAgcDIuU2VsZk11bEFkZCh0aGlzLm1fazIgKiBzMiAqICh0aGlzLm1fTHNbaV0gLSBMKSwgZCk7XHJcblxyXG4gICAgICAvLyB0aGlzLm1fcHNbaV0gPSBwMTtcclxuICAgICAgLy8gdGhpcy5tX3BzW2kgKyAxXSA9IHAyO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgU2V0QW5nbGUoYW5nbGU6IG51bWJlcik6IHZvaWQge1xyXG4gICAgY29uc3QgY291bnQzOiBudW1iZXIgPSB0aGlzLm1fY291bnQgLSAyO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBjb3VudDM7ICsraSkge1xyXG4gICAgICB0aGlzLm1fYXNbaV0gPSBhbmdsZTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIHNfZDEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgc19kMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBzX0pkMSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBzX0pkMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBzX0oxID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIHNfSjIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlQzMoKTogdm9pZCB7XHJcbiAgICBjb25zdCBjb3VudDM6IG51bWJlciA9IHRoaXMubV9jb3VudCAtIDI7XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBjb3VudDM7ICsraSkge1xyXG4gICAgICBjb25zdCBwMTogYjJWZWMyID0gdGhpcy5tX3BzW2ldO1xyXG4gICAgICBjb25zdCBwMjogYjJWZWMyID0gdGhpcy5tX3BzW2kgKyAxXTtcclxuICAgICAgY29uc3QgcDM6IGIyVmVjMiA9IHRoaXMubV9wc1tpICsgMl07XHJcblxyXG4gICAgICBjb25zdCBtMTogbnVtYmVyID0gdGhpcy5tX2ltc1tpXTtcclxuICAgICAgY29uc3QgbTI6IG51bWJlciA9IHRoaXMubV9pbXNbaSArIDFdO1xyXG4gICAgICBjb25zdCBtMzogbnVtYmVyID0gdGhpcy5tX2ltc1tpICsgMl07XHJcblxyXG4gICAgICBjb25zdCBkMTogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHAyLCBwMSwgYjJSb3BlLnNfZDEpO1xyXG4gICAgICBjb25zdCBkMjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKHAzLCBwMiwgYjJSb3BlLnNfZDIpO1xyXG5cclxuICAgICAgY29uc3QgTDFzcXI6IG51bWJlciA9IGQxLkxlbmd0aFNxdWFyZWQoKTtcclxuICAgICAgY29uc3QgTDJzcXI6IG51bWJlciA9IGQyLkxlbmd0aFNxdWFyZWQoKTtcclxuXHJcbiAgICAgIGlmIChMMXNxciAqIEwyc3FyID09PSAwKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGNvbnN0IGE6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKGQxLCBkMik7XHJcbiAgICAgIGNvbnN0IGI6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihkMSwgZDIpO1xyXG5cclxuICAgICAgbGV0IGFuZ2xlOiBudW1iZXIgPSBiMkF0YW4yKGEsIGIpO1xyXG5cclxuICAgICAgY29uc3QgSmQxOiBiMlZlYzIgPSBiMlZlYzIuTXVsU1YoLTEgLyBMMXNxciwgZDEuU2VsZlNrZXcoKSwgYjJSb3BlLnNfSmQxKTtcclxuICAgICAgY29uc3QgSmQyOiBiMlZlYzIgPSBiMlZlYzIuTXVsU1YoMSAvIEwyc3FyLCBkMi5TZWxmU2tldygpLCBiMlJvcGUuc19KZDIpO1xyXG5cclxuICAgICAgY29uc3QgSjE6IGIyVmVjMiA9IGIyVmVjMi5OZWdWKEpkMSwgYjJSb3BlLnNfSjEpO1xyXG4gICAgICBjb25zdCBKMjogYjJWZWMyID0gYjJWZWMyLlN1YlZWKEpkMSwgSmQyLCBiMlJvcGUuc19KMik7XHJcbiAgICAgIGNvbnN0IEozOiBiMlZlYzIgPSBKZDI7XHJcblxyXG4gICAgICBsZXQgbWFzczogbnVtYmVyID1cclxuICAgICAgICBtMSAqIGIyVmVjMi5Eb3RWVihKMSwgSjEpICsgbTIgKiBiMlZlYzIuRG90VlYoSjIsIEoyKSArIG0zICogYjJWZWMyLkRvdFZWKEozLCBKMyk7XHJcbiAgICAgIGlmIChtYXNzID09PSAwKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIG1hc3MgPSAxIC8gbWFzcztcclxuXHJcbiAgICAgIGxldCBDOiBudW1iZXIgPSBhbmdsZSAtIHRoaXMubV9hc1tpXTtcclxuXHJcbiAgICAgIHdoaWxlIChDID4gYjJfcGkpIHtcclxuICAgICAgICBhbmdsZSAtPSAyICogYjJfcGk7XHJcbiAgICAgICAgQyA9IGFuZ2xlIC0gdGhpcy5tX2FzW2ldO1xyXG4gICAgICB9XHJcblxyXG4gICAgICB3aGlsZSAoQyA8IC1iMl9waSkge1xyXG4gICAgICAgIGFuZ2xlICs9IDIgKiBiMl9waTtcclxuICAgICAgICBDID0gYW5nbGUgLSB0aGlzLm1fYXNbaV07XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGNvbnN0IGltcHVsc2U6IG51bWJlciA9IC10aGlzLm1fazMgKiBtYXNzICogQztcclxuXHJcbiAgICAgIHAxLlNlbGZNdWxBZGQobTEgKiBpbXB1bHNlLCBKMSk7XHJcbiAgICAgIHAyLlNlbGZNdWxBZGQobTIgKiBpbXB1bHNlLCBKMik7XHJcbiAgICAgIHAzLlNlbGZNdWxBZGQobTMgKiBpbXB1bHNlLCBKMyk7XHJcblxyXG4gICAgICAvLyB0aGlzLm1fcHNbaV0gPSBwMTtcclxuICAgICAgLy8gdGhpcy5tX3BzW2kgKyAxXSA9IHAyO1xyXG4gICAgICAvLyB0aGlzLm1fcHNbaSArIDJdID0gcDM7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBEcmF3KGRyYXc6IGIyRHJhdyk6IHZvaWQge1xyXG4gICAgY29uc3QgYzogYjJDb2xvciA9IG5ldyBiMkNvbG9yKDAuNCwgMC41LCAwLjcpO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50IC0gMTsgKytpKSB7XHJcbiAgICAgIGRyYXcuRHJhd1NlZ21lbnQodGhpcy5tX3BzW2ldLCB0aGlzLm1fcHNbaSArIDFdLCBjKTtcclxuICAgIH1cclxuICB9XHJcbn1cclxuIl19