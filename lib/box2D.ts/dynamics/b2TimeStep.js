/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
import { b2Vec2 } from '../common/b2Math';
/// Profiling data. Times are in milliseconds.
export class b2Profile {
    constructor() {
        this.step = NaN;
        this.collide = NaN;
        this.solve = NaN;
        this.solveInit = NaN;
        this.solveVelocity = NaN;
        this.solvePosition = NaN;
        this.broadphase = NaN;
        this.solveTOI = NaN;
        this.step = 0.0;
        this.collide = 0.0;
        this.solve = 0.0;
        this.solveInit = 0.0;
        this.solveVelocity = 0.0;
        this.solvePosition = 0.0;
        this.broadphase = 0.0;
        this.solveTOI = 0.0;
    }
    Reset() {
        this.step = 0;
        this.collide = 0;
        this.solve = 0;
        this.solveInit = 0;
        this.solveVelocity = 0;
        this.solvePosition = 0;
        this.broadphase = 0;
        this.solveTOI = 0;
        return this;
    }
}
/// This is an internal structure.
export class b2TimeStep {
    constructor() {
        this.dt = NaN; // time step
        this.inv_dt = NaN; // inverse time step (0 if dt == 0).
        this.dtRatio = NaN; // dt * inv_dt0
        this.velocityIterations = 0;
        this.positionIterations = 0;
        // #if B2_ENABLE_PARTICLE
        this.particleIterations = 0;
        // #endif
        this.warmStarting = false;
        this.dt = 0.0;
        this.inv_dt = 0.0;
        this.dtRatio = 0.0;
    }
    Copy(step) {
        this.dt = step.dt;
        this.inv_dt = step.inv_dt;
        this.dtRatio = step.dtRatio;
        this.positionIterations = step.positionIterations;
        this.velocityIterations = step.velocityIterations;
        if (B2_ENABLE_PARTICLE) {
            this.particleIterations = step.particleIterations;
        }
        this.warmStarting = step.warmStarting;
        return this;
    }
}
export class b2Position {
    constructor() {
        this.c = new b2Vec2();
        this.a = NaN;
        this.a = 0.0;
    }
    static MakeArray(length) {
        const arr = [];
        for (let i = 0; i < length; ++i) {
            arr.push(new b2Position());
        }
        return arr;
    }
}
export class b2Velocity {
    constructor() {
        this.v = new b2Vec2();
        this.w = NaN;
        this.w = 0.0;
    }
    static MakeArray(length) {
        const arr = [];
        for (let i = 0; i < length; ++i) {
            arr.push(new b2Velocity());
        }
        return arr;
    }
}
export class b2SolverData {
    constructor() {
        this.step = new b2TimeStep();
        this.positions = [null];
        this.velocities = [null];
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUaW1lU3RlcC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9keW5hbWljcy9iMlRpbWVTdGVwLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLE1BQU0sRUFBRSxNQUFNLGtCQUFrQixDQUFDO0FBRTFDLDhDQUE4QztBQUM5QyxNQUFNLE9BQU8sU0FBUztJQVVwQjtRQVRBLFNBQUksR0FBRyxHQUFHLENBQUM7UUFDWCxZQUFPLEdBQUcsR0FBRyxDQUFDO1FBQ2QsVUFBSyxHQUFHLEdBQUcsQ0FBQztRQUNaLGNBQVMsR0FBRyxHQUFHLENBQUM7UUFDaEIsa0JBQWEsR0FBRyxHQUFHLENBQUM7UUFDcEIsa0JBQWEsR0FBRyxHQUFHLENBQUM7UUFDcEIsZUFBVSxHQUFHLEdBQUcsQ0FBQztRQUNqQixhQUFRLEdBQUcsR0FBRyxDQUFDO1FBR2IsSUFBSSxDQUFDLElBQUksR0FBRyxHQUFHLENBQUM7UUFDaEIsSUFBSSxDQUFDLE9BQU8sR0FBRyxHQUFHLENBQUM7UUFDbkIsSUFBSSxDQUFDLEtBQUssR0FBRyxHQUFHLENBQUM7UUFDakIsSUFBSSxDQUFDLFNBQVMsR0FBRyxHQUFHLENBQUM7UUFDckIsSUFBSSxDQUFDLGFBQWEsR0FBRyxHQUFHLENBQUM7UUFDekIsSUFBSSxDQUFDLGFBQWEsR0FBRyxHQUFHLENBQUM7UUFDekIsSUFBSSxDQUFDLFVBQVUsR0FBRyxHQUFHLENBQUM7UUFDdEIsSUFBSSxDQUFDLFFBQVEsR0FBRyxHQUFHLENBQUM7SUFDdEIsQ0FBQztJQUVELEtBQUs7UUFDSCxJQUFJLENBQUMsSUFBSSxHQUFHLENBQUMsQ0FBQztRQUNkLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxDQUFDO1FBQ2pCLElBQUksQ0FBQyxLQUFLLEdBQUcsQ0FBQyxDQUFDO1FBQ2YsSUFBSSxDQUFDLFNBQVMsR0FBRyxDQUFDLENBQUM7UUFDbkIsSUFBSSxD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RCxlQUFVLEdBQWtCLENBQUMsSUFBSSxDQUE2QixDQUFDO0lBQ2pFLENBQUM7Q0FBQSIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDExIEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJWZWMyIH0gZnJvbSAnLi4vY29tbW9uL2IyTWF0aCc7XHJcblxyXG4vLy8gUHJvZmlsaW5nIGRhdGEuIFRpbWVzIGFyZSBpbiBtaWxsaXNlY29uZHMuXHJcbmV4cG9ydCBjbGFzcyBiMlByb2ZpbGUge1xyXG4gIHN0ZXAgPSBOYU47XHJcbiAgY29sbGlkZSA9IE5hTjtcclxuICBzb2x2ZSA9IE5hTjtcclxuICBzb2x2ZUluaXQgPSBOYU47XHJcbiAgc29sdmVWZWxvY2l0eSA9IE5hTjtcclxuICBzb2x2ZVBvc2l0aW9uID0gTmFOO1xyXG4gIGJyb2FkcGhhc2UgPSBOYU47XHJcbiAgc29sdmVUT0kgPSBOYU47XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5zdGVwID0gMC4wO1xyXG4gICAgdGhpcy5jb2xsaWRlID0gMC4wO1xyXG4gICAgdGhpcy5zb2x2ZSA9IDAuMDtcclxuICAgIHRoaXMuc29sdmVJbml0ID0gMC4wO1xyXG4gICAgdGhpcy5zb2x2ZVZlbG9jaXR5ID0gMC4wO1xyXG4gICAgdGhpcy5zb2x2ZVBvc2l0aW9uID0gMC4wO1xyXG4gICAgdGhpcy5icm9hZHBoYXNlID0gMC4wO1xyXG4gICAgdGhpcy5zb2x2ZVRPSSA9IDAuMDtcclxuICB9XHJcblxyXG4gIFJlc2V0KCkge1xyXG4gICAgdGhpcy5zdGVwID0gMDtcclxuICAgIHRoaXMuY29sbGlkZSA9IDA7XHJcbiAgICB0aGlzLnNvbHZlID0gMDtcclxuICAgIHRoaXMuc29sdmVJbml0ID0gMDtcclxuICAgIHRoaXMuc29sdmVWZWxvY2l0eSA9IDA7XHJcbiAgICB0aGlzLnNvbHZlUG9zaXRpb24gPSAwO1xyXG4gICAgdGhpcy5icm9hZHBoYXNlID0gMDtcclxuICAgIHRoaXMuc29sdmVUT0kgPSAwO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gVGhpcyBpcyBhbiBpbnRlcm5hbCBzdHJ1Y3R1cmUuXHJcbmV4cG9ydCBjbGFzcyBiMlRpbWVTdGVwIHtcclxuICBkdCA9IE5hTjsgLy8gdGltZSBzdGVwXHJcbiAgaW52X2R0ID0gTmFOOyAvLyBpbnZlcnNlIHRpbWUgc3RlcCAoMCBpZiBkdCA9PSAwKS5cclxuICBkdFJhdGlvID0gTmFOOyAvLyBkdCAqIGludl9kdDBcclxuICB2ZWxvY2l0eUl0ZXJhdGlvbnMgPSAwO1xyXG4gIHBvc2l0aW9uSXRlcmF0aW9ucyA9IDA7XHJcbiAgLy8gI2lmIEIyX0VOQUJMRV9QQVJUSUNMRVxyXG4gIHBhcnRpY2xlSXRlcmF0aW9ucyA9IDA7XHJcbiAgLy8gI2VuZGlmXHJcbiAgd2FybVN0YXJ0aW5nID0gZmFsc2U7XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5kdCA9IDAuMDtcclxuICAgIHRoaXMuaW52X2R0ID0gMC4wO1xyXG4gICAgdGhpcy5kdFJhdGlvID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgQ29weShzdGVwOiBiMlRpbWVTdGVwKTogYjJUaW1lU3RlcCB7XHJcbiAgICB0aGlzLmR0ID0gc3RlcC5kdDtcclxuICAgIHRoaXMuaW52X2R0ID0gc3RlcC5pbnZfZHQ7XHJcbiAgICB0aGlzLmR0UmF0aW8gPSBzdGVwLmR0UmF0aW87XHJcbiAgICB0aGlzLnBvc2l0aW9uSXRlcmF0aW9ucyA9IHN0ZXAucG9zaXRpb25JdGVyYXRpb25zO1xyXG4gICAgdGhpcy52ZWxvY2l0eUl0ZXJhdGlvbnMgPSBzdGVwLnZlbG9jaXR5SXRlcmF0aW9ucztcclxuICAgIGlmIChCMl9FTkFCTEVfUEFSVElDTEUpIHtcclxuICAgICAgdGhpcy5wYXJ0aWNsZUl0ZXJhdGlvbnMgPSBzdGVwLnBhcnRpY2xlSXRlcmF0aW9ucztcclxuICAgIH1cclxuICAgIHRoaXMud2FybVN0YXJ0aW5nID0gc3RlcC53YXJtU3RhcnRpbmc7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBvc2l0aW9uIHtcclxuICByZWFkb25seSBjID0gbmV3IGIyVmVjMigpO1xyXG4gIGEgPSBOYU47XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5hID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE1ha2VBcnJheShsZW5ndGg6IG51bWJlcik6IGIyUG9zaXRpb25bXSB7XHJcbiAgICBjb25zdCBhcnIgPSBbXTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbGVuZ3RoOyArK2kpIHtcclxuICAgICAgYXJyLnB1c2gobmV3IGIyUG9zaXRpb24oKSk7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gYXJyO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyVmVsb2NpdHkge1xyXG4gIHJlYWRvbmx5IHYgPSBuZXcgYjJWZWMyKCk7XHJcbiAgdyA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLncgPSAwLjA7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTWFrZUFycmF5KGxlbmd0aDogbnVtYmVyKTogYjJWZWxvY2l0eVtdIHtcclxuICAgIGNvbnN0IGFyciA9IFtdO1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBsZW5ndGg7ICsraSkge1xyXG4gICAgICBhcnIucHVzaChuZXcgYjJWZWxvY2l0eSgpKTtcclxuICAgIH1cclxuICAgIHJldHVybiBhcnI7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJTb2x2ZXJEYXRhIHtcclxuICByZWFkb25seSBzdGVwID0gbmV3IGIyVGltZVN0ZXAoKTtcclxuICBwb3NpdGlvbnM6IGIyUG9zaXRpb25bXSA9IChbbnVsbF0gYXMgdW5rbm93bikgYXMgYjJQb3NpdGlvbltdO1xyXG4gIHZlbG9jaXRpZXM6IGIyVmVsb2NpdHlbXSA9IChbbnVsbF0gYXMgdW5rbm93bikgYXMgYjJWZWxvY2l0eVtdO1xyXG59XHJcbiJdfQ==