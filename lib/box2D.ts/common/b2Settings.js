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
export function b2Assert(condition, message) {
    if (!condition) {
        // debugger;
        throw new Error(message);
    }
}
export function b2Maybe(value, def) {
    return value !== undefined ? value : def;
}
export const b2_maxFloat = 1e37; // FLT_MAX instead of Number.MAX_VALUE;
export const b2_epsilon = 1e-5; // FLT_EPSILON instead of Number.MIN_VALUE;
export const b2_epsilon_sq = b2_epsilon * b2_epsilon;
export const b2_pi = Math.PI;
// export const b2_pi: number = 3.14159265359;
/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///
// Collision
/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
export const b2_maxManifoldPoints = 2;
/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
export const b2_maxPolygonVertices = 8;
/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
export const b2_aabbExtension = 0.1;
/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
export const b2_aabbMultiplier = 2;
/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
export const b2_linearSlop = 0.008; // 0.005;
/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
export const b2_angularSlop = (2 / 180) * b2_pi;
/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
export const b2_polygonRadius = 2 * b2_linearSlop;
/// Maximum number of sub-steps per contact in continuous physics simulation.
export const b2_maxSubSteps = 8;
// Dynamics
/// Maximum number of contacts to be handled to solve a TOI impact.
export const b2_maxTOIContacts = 32;
/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
export const b2_velocityThreshold = 1;
/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
export const b2_maxLinearCorrection = 0.2;
/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
export const b2_maxAngularCorrection = (8 / 180) * b2_pi;
/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
export const b2_maxTranslation = 2;
export const b2_maxTranslationSquared = b2_maxTranslation * b2_maxTranslation;
/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
export const b2_maxRotation = 0.5 * b2_pi;
export const b2_maxRotationSquared = b2_maxRotation * b2_maxRotation;
/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
export const b2_baumgarte = 0.2;
export const b2_toiBaumgarte = 0.75;
// Sleep
/// The time that a body must be still before it will go to sleep.
export const b2_timeToSleep = 0.5;
/// A body cannot sleep if its linear velocity is above this tolerance.
export const b2_linearSleepTolerance = 0.01;
/// A body cannot sleep if its angular velocity is above this tolerance.
export const b2_angularSleepTolerance = (2 / 180) * b2_pi;
// Memory Allocation
/// Implement this function to use your own memory allocator.
export function b2Alloc(size) {
    return null;
}
/// If you implement b2Alloc, you should also implement this function.
// eslint-disable-next-line @typescript-eslint/no-empty-function
export function b2Free(mem) { }
/// Logging function.
export function b2Log(message, ...args) {
    // console.log(message, ...args);
}
/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
export class b2Version {
    constructor(major = 0, minor = 0, revision = 0) {
        this.major = 0; ///< significant changes
        this.minor = 0; ///< incremental changes
        this.revision = 0; ///< bug fixes
        this.major = major;
        this.minor = minor;
        this.revision = revision;
    }
    toString() {
        return this.major + '.' + this.minor + '.' + this.revision;
    }
}
/// Current version.
export const b2_version = new b2Version(2, 3, 2);
export const b2_branch = 'master';
export const b2_commit = 'fbf51801d80fc389d43dc46524520e89043b6faf';
export function b2ParseInt(v) {
    return parseInt(v, 10);
}
export function b2ParseUInt(v) {
    return Math.abs(parseInt(v, 10));
}
export function b2MakeArray(length, init) {
    const a = [];
    for (let i = 0; i < length; ++i) {
        a[i] = init(i);
    }
    return a;
}
export function b2MakeNullArray(length) {
    const a = [null];
    for (let i = 0; i < length; ++i) {
        a[i] = null;
    }
    return a;
}
export function b2MakeIntArray(length) {
    const a = [0];
    for (let i = 0; i < length; ++i) {
        a[i] = 0;
    }
    return a;
}
export function b2MakeNumberArray(length) {
    const a = [NaN];
    for (let i = 0; i < length; ++i) {
        a[i] = 0.0;
    }
    return a;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJTZXR0aW5ncy5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb21tb24vYjJTZXR0aW5ncy50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE1BQU0sVUFBVSxRQUFRLENBQUMsU0FBa0IsRUFBRSxPQUFnQjtJQUMzRCxJQUFJLENBQUMsU0FBUyxFQUFFO1FBQ2QsWUFBWTtRQUNaLE1BQU0sSUFBSSxLQUFLLENBQUMsT0FBTyxDQUFDLENBQUM7S0FDMUI7QUFDSCxDQUFDO0FBRUQsTUFBTSxVQUFVLE9BQU8sQ0FBSSxLQUFvQixFQUFFLEdBQU07SUFDckQsT0FBTyxLQUFLLEtBQUssU0FBUyxDQUFDLENBQUMsQ0FBQyxLQUFLLENBQUMsQ0FBQyxDQUFDLEdBQUcsQ0FBQztBQUMzQyxDQUFDO0FBRUQsTUFBTSxDQUFDLE1BQU0sV0FBVyxHQUFHLElBQUksQ0FBQyxDQUFDLHVDQUF1QztBQUN4RSxNQUFNLENBQUMsTUFBTSxVQUFVLEdBQUcsSUFBSSxDQUFDLENBQUMsMkNBQTJDO0FBQzNFLE1BQU0sQ0FBQyxNQUFNLGFBQWEsR0FBVyxVQUFVLEdBQUcsVUFBVSxDQUFDO0FBQzdELE1BQU0sQ0FBQyxNQUFNLEtBQUssR0FBVyxJQUFJLENBQUMsRUFBRSxDQUFDO0FBQ3JDLDhDQUE4QztBQUU5QyxTQUFTO0FBQ1QsMEVBQTBFO0FBQzFFLEdBQUc7QUFFSCxZQUFZO0FBRVosc0VBQXNFO0FBQ3RFLDBCQUEwQjtBQUMxQixNQUFNLENBQUMsTUFBTSxvQkFBb0IsR0FBRyxDQUFDLENBQUM7QUFFdEMsMkVBQTJFO0FBQzNFLHFFQUFxRTtBQUNyRSxNQUFNLENBQUMsTUFBTSxxQkFBcUIsR0FBRyxDQUFDLENBQUM7QUFFdkMseUVBQXlFO0FBQ3pFLG1FQUFtRTtBQUNuRSxzQkFBc0I7QUFDdEIsTUFBTSxDQUFDLE1BQU0sZ0JBQWdCLEdBQUcsR0FBRyxDQUFDO0FBRXBDLDZFQUE2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QUcsTUFBTSxFQUFFLEVBQUUsQ0FBQyxFQUFFO1FBQy9CLENBQUMsQ0FBQyxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUM7S0FDYjtJQUNELE9BQU8sQ0FBQyxDQUFDO0FBQ1gsQ0FBQztBQUVELE1BQU0sVUFBVSxjQUFjLENBQUMsTUFBYztJQUMzQyxNQUFNLENBQUMsR0FBYSxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ3hCLEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxNQUFNLEVBQUUsRUFBRSxDQUFDLEVBQUU7UUFDL0IsQ0FBQyxDQUFDLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQztLQUNWO0lBQ0QsT0FBTyxDQUFDLENBQUM7QUFDWCxDQUFDO0FBRUQsTUFBTSxVQUFVLGlCQUFpQixDQUFDLE1BQWM7SUFDOUMsTUFBTSxDQUFDLEdBQUcsQ0FBQyxHQUFHLENBQUMsQ0FBQztJQUNoQixLQUFLLElBQUksQ0FBQyxHQUFHLENBQUMsRUFBRSxDQUFDLEdBQUcsTUFBTSxFQUFFLEVBQUUsQ0FBQyxFQUFFO1FBQy9CLENBQUMsQ0FBQyxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7S0FDWjtJQUNELE9BQU8sQ0FBQyxDQUFDO0FBQ1gsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyQXNzZXJ0KGNvbmRpdGlvbjogYm9vbGVhbiwgbWVzc2FnZT86IHN0cmluZyk6IHZvaWQge1xyXG4gIGlmICghY29uZGl0aW9uKSB7XHJcbiAgICAvLyBkZWJ1Z2dlcjtcclxuICAgIHRocm93IG5ldyBFcnJvcihtZXNzYWdlKTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMk1heWJlPFQ+KHZhbHVlOiBUIHwgdW5kZWZpbmVkLCBkZWY6IFQpOiBUIHtcclxuICByZXR1cm4gdmFsdWUgIT09IHVuZGVmaW5lZCA/IHZhbHVlIDogZGVmO1xyXG59XHJcblxyXG5leHBvcnQgY29uc3QgYjJfbWF4RmxvYXQgPSAxZTM3OyAvLyBGTFRfTUFYIGluc3RlYWQgb2YgTnVtYmVyLk1BWF9WQUxVRTtcclxuZXhwb3J0IGNvbnN0IGIyX2Vwc2lsb24gPSAxZS01OyAvLyBGTFRfRVBTSUxPTiBpbnN0ZWFkIG9mIE51bWJlci5NSU5fVkFMVUU7XHJcbmV4cG9ydCBjb25zdCBiMl9lcHNpbG9uX3NxOiBudW1iZXIgPSBiMl9lcHNpbG9uICogYjJfZXBzaWxvbjtcclxuZXhwb3J0IGNvbnN0IGIyX3BpOiBudW1iZXIgPSBNYXRoLlBJO1xyXG4vLyBleHBvcnQgY29uc3QgYjJfcGk6IG51bWJlciA9IDMuMTQxNTkyNjUzNTk7XHJcblxyXG4vLy8gQGZpbGVcclxuLy8vIEdsb2JhbCB0dW5pbmcgY29uc3RhbnRzIGJhc2VkIG9uIG1ldGVycy1raWxvZ3JhbXMtc2Vjb25kcyAoTUtTKSB1bml0cy5cclxuLy8vXHJcblxyXG4vLyBDb2xsaXNpb25cclxuXHJcbi8vLyBUaGUgbWF4aW11bSBudW1iZXIgb2YgY29udGFjdCBwb2ludHMgYmV0d2VlbiB0d28gY29udmV4IHNoYXBlcy4gRG9cclxuLy8vIG5vdCBjaGFuZ2UgdGhpcyB2YWx1ZS5cclxuZXhwb3J0IGNvbnN0IGIyX21heE1hbmlmb2xkUG9pbnRzID0gMjtcclxuXHJcbi8vLyBUaGUgbWF4aW11bSBudW1iZXIgb2YgdmVydGljZXMgb24gYSBjb252ZXggcG9seWdvbi4gWW91IGNhbm5vdCBpbmNyZWFzZVxyXG4vLy8gdGhpcyB0b28gbXVjaCBiZWNhdXNlIGIyQmxvY2tBbGxvY2F0b3IgaGFzIGEgbWF4aW11bSBvYmplY3Qgc2l6ZS5cclxuZXhwb3J0IGNvbnN0IGIyX21heFBvbHlnb25WZXJ0aWNlcyA9IDg7XHJcblxyXG4vLy8gVGhpcyBpcyB1c2VkIHRvIGZhdHRlbiBBQUJCcyBpbiB0aGUgZHluYW1pYyB0cmVlLiBUaGlzIGFsbG93cyBwcm94aWVzXHJcbi8vLyB0byBtb3ZlIGJ5IGEgc21hbGwgYW1vdW50IHdpdGhvdXQgdHJpZ2dlcmluZyBhIHRyZWUgYWRqdXN0bWVudC5cclxuLy8vIFRoaXMgaXMgaW4gbWV0ZXJzLlxyXG5leHBvcnQgY29uc3QgYjJfYWFiYkV4dGVuc2lvbiA9IDAuMTtcclxuXHJcbi8vLyBUaGlzIGlzIHVzZWQgdG8gZmF0dGVuIEFBQkJzIGluIHRoZSBkeW5hbWljIHRyZWUuIFRoaXMgaXMgdXNlZCB0byBwcmVkaWN0XHJcbi8vLyB0aGUgZnV0dXJlIHBvc2l0aW9uIGJhc2VkIG9uIHRoZSBjdXJyZW50IGRpc3BsYWNlbWVudC5cclxuLy8vIFRoaXMgaXMgYSBkaW1lbnNpb25sZXNzIG11bHRpcGxpZXIuXHJcbmV4cG9ydCBjb25zdCBiMl9hYWJiTXVsdGlwbGllciA9IDI7XHJcblxyXG4vLy8gQSBzbWFsbCBsZW5ndGggdXNlZCBhcyBhIGNvbGxpc2lvbiBhbmQgY29uc3RyYWludCB0b2xlcmFuY2UuIFVzdWFsbHkgaXQgaXNcclxuLy8vIGNob3NlbiB0byBiZSBudW1lcmljYWxseSBzaWduaWZpY2FudCwgYnV0IHZpc3VhbGx5IGluc2lnbmlmaWNhbnQuXHJcbmV4cG9ydCBjb25zdCBiMl9saW5lYXJTbG9wID0gMC4wMDg7IC8vIDAuMDA1O1xyXG5cclxuLy8vIEEgc21hbGwgYW5nbGUgdXNlZCBhcyBhIGNvbGxpc2lvbiBhbmQgY29uc3RyYWludCB0b2xlcmFuY2UuIFVzdWFsbHkgaXQgaXNcclxuLy8vIGNob3NlbiB0byBiZSBudW1lcmljYWxseSBzaWduaWZpY2FudCwgYnV0IHZpc3VhbGx5IGluc2lnbmlmaWNhbnQuXHJcbmV4cG9ydCBjb25zdCBiMl9hbmd1bGFyU2xvcDogbnVtYmVyID0gKDIgLyAxODApICogYjJfcGk7XHJcblxyXG4vLy8gVGhlIHJhZGl1cyBvZiB0aGUgcG9seWdvbi9lZGdlIHNoYXBlIHNraW4uIFRoaXMgc2hvdWxkIG5vdCBiZSBtb2RpZmllZC4gTWFraW5nXHJcbi8vLyB0aGlzIHNtYWxsZXIgbWVhbnMgcG9seWdvbnMgd2lsbCBoYXZlIGFuIGluc3VmZmljaWVudCBidWZmZXIgZm9yIGNvbnRpbnVvdXMgY29sbGlzaW9uLlxyXG4vLy8gTWFraW5nIGl0IGxhcmdlciBtYXkgY3JlYXRlIGFydGlmYWN0cyBmb3IgdmVydGV4IGNvbGxpc2lvbi5cclxuZXhwb3J0IGNvbnN0IGIyX3BvbHlnb25SYWRpdXM6IG51bWJlciA9IDIgKiBiMl9saW5lYXJTbG9wO1xyXG5cclxuLy8vIE1heGltdW0gbnVtYmVyIG9mIHN1Yi1zdGVwcyBwZXIgY29udGFjdCBpbiBjb250aW51b3VzIHBoeXNpY3Mgc2ltdWxhdGlvbi5cclxuZXhwb3J0IGNvbnN0IGIyX21heFN1YlN0ZXBzID0gODtcclxuXHJcbi8vIER5bmFtaWNzXHJcblxyXG4vLy8gTWF4aW11bSBudW1iZXIgb2YgY29udGFjdHMgdG8gYmUgaGFuZGxlZCB0byBzb2x2ZSBhIFRPSSBpbXBhY3QuXHJcbmV4cG9ydCBjb25zdCBiMl9tYXhUT0lDb250YWN0cyA9IDMyO1xyXG5cclxuLy8vIEEgdmVsb2NpdHkgdGhyZXNob2xkIGZvciBlbGFzdGljIGNvbGxpc2lvbnMuIEFueSBjb2xsaXNpb24gd2l0aCBhIHJlbGF0aXZlIGxpbmVhclxyXG4vLy8gdmVsb2NpdHkgYmVsb3cgdGhpcyB0aHJlc2hvbGQgd2lsbCBiZSB0cmVhdGVkIGFzIGluZWxhc3RpYy5cclxuZXhwb3J0IGNvbnN0IGIyX3ZlbG9jaXR5VGhyZXNob2xkID0gMTtcclxuXHJcbi8vLyBUaGUgbWF4aW11bSBsaW5lYXIgcG9zaXRpb24gY29ycmVjdGlvbiB1c2VkIHdoZW4gc29sdmluZyBjb25zdHJhaW50cy4gVGhpcyBoZWxwcyB0b1xyXG4vLy8gcHJldmVudCBvdmVyc2hvb3QuXHJcbmV4cG9ydCBjb25zdCBiMl9tYXhMaW5lYXJDb3JyZWN0aW9uID0gMC4yO1xyXG5cclxuLy8vIFRoZSBtYXhpbXVtIGFuZ3VsYXIgcG9zaXRpb24gY29ycmVjdGlvbiB1c2VkIHdoZW4gc29sdmluZyBjb25zdHJhaW50cy4gVGhpcyBoZWxwcyB0b1xyXG4vLy8gcHJldmVudCBvdmVyc2hvb3QuXHJcbmV4cG9ydCBjb25zdCBiMl9tYXhBbmd1bGFyQ29ycmVjdGlvbjogbnVtYmVyID0gKDggLyAxODApICogYjJfcGk7XHJcblxyXG4vLy8gVGhlIG1heGltdW0gbGluZWFyIHZlbG9jaXR5IG9mIGEgYm9keS4gVGhpcyBsaW1pdCBpcyB2ZXJ5IGxhcmdlIGFuZCBpcyB1c2VkXHJcbi8vLyB0byBwcmV2ZW50IG51bWVyaWNhbCBwcm9ibGVtcy4gWW91IHNob3VsZG4ndCBuZWVkIHRvIGFkanVzdCB0aGlzLlxyXG5leHBvcnQgY29uc3QgYjJfbWF4VHJhbnNsYXRpb24gPSAyO1xyXG5leHBvcnQgY29uc3QgYjJfbWF4VHJhbnNsYXRpb25TcXVhcmVkOiBudW1iZXIgPSBiMl9tYXhUcmFuc2xhdGlvbiAqIGIyX21heFRyYW5zbGF0aW9uO1xyXG5cclxuLy8vIFRoZSBtYXhpbXVtIGFuZ3VsYXIgdmVsb2NpdHkgb2YgYSBib2R5LiBUaGlzIGxpbWl0IGlzIHZlcnkgbGFyZ2UgYW5kIGlzIHVzZWRcclxuLy8vIHRvIHByZXZlbnQgbnVtZXJpY2FsIHByb2JsZW1zLiBZb3Ugc2hvdWxkbid0IG5lZWQgdG8gYWRqdXN0IHRoaXMuXHJcbmV4cG9ydCBjb25zdCBiMl9tYXhSb3RhdGlvbjogbnVtYmVyID0gMC41ICogYjJfcGk7XHJcbmV4cG9ydCBjb25zdCBiMl9tYXhSb3RhdGlvblNxdWFyZWQ6IG51bWJlciA9IGIyX21heFJvdGF0aW9uICogYjJfbWF4Um90YXRpb247XHJcblxyXG4vLy8gVGhpcyBzY2FsZSBmYWN0b3IgY29udHJvbHMgaG93IGZhc3Qgb3ZlcmxhcCBpcyByZXNvbHZlZC4gSWRlYWxseSB0aGlzIHdvdWxkIGJlIDEgc29cclxuLy8vIHRoYXQgb3ZlcmxhcCBpcyByZW1vdmVkIGluIG9uZSB0aW1lIHN0ZXAuIEhvd2V2ZXIgdXNpbmcgdmFsdWVzIGNsb3NlIHRvIDEgb2Z0ZW4gbGVhZFxyXG4vLy8gdG8gb3ZlcnNob290LlxyXG5leHBvcnQgY29uc3QgYjJfYmF1bWdhcnRlID0gMC4yO1xyXG5leHBvcnQgY29uc3QgYjJfdG9pQmF1bWdhcnRlID0gMC43NTtcclxuXHJcbi8vIFNsZWVwXHJcblxyXG4vLy8gVGhlIHRpbWUgdGhhdCBhIGJvZHkgbXVzdCBiZSBzdGlsbCBiZWZvcmUgaXQgd2lsbCBnbyB0byBzbGVlcC5cclxuZXhwb3J0IGNvbnN0IGIyX3RpbWVUb1NsZWVwID0gMC41O1xyXG5cclxuLy8vIEEgYm9keSBjYW5ub3Qgc2xlZXAgaWYgaXRzIGxpbmVhciB2ZWxvY2l0eSBpcyBhYm92ZSB0aGlzIHRvbGVyYW5jZS5cclxuZXhwb3J0IGNvbnN0IGIyX2xpbmVhclNsZWVwVG9sZXJhbmNlID0gMC4wMTtcclxuXHJcbi8vLyBBIGJvZHkgY2Fubm90IHNsZWVwIGlmIGl0cyBhbmd1bGFyIHZlbG9jaXR5IGlzIGFib3ZlIHRoaXMgdG9sZXJhbmNlLlxyXG5leHBvcnQgY29uc3QgYjJfYW5ndWxhclNsZWVwVG9sZXJhbmNlOiBudW1iZXIgPSAoMiAvIDE4MCkgKiBiMl9waTtcclxuXHJcbi8vIE1lbW9yeSBBbGxvY2F0aW9uXHJcblxyXG4vLy8gSW1wbGVtZW50IHRoaXMgZnVuY3Rpb24gdG8gdXNlIHlvdXIgb3duIG1lbW9yeSBhbGxvY2F0b3IuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkFsbG9jKHNpemU6IG51bWJlcik6IGFueSB7XHJcbiAgcmV0dXJuIG51bGw7XHJcbn1cclxuXHJcbi8vLyBJZiB5b3UgaW1wbGVtZW50IGIyQWxsb2MsIHlvdSBzaG91bGQgYWxzbyBpbXBsZW1lbnQgdGhpcyBmdW5jdGlvbi5cclxuLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIEB0eXBlc2NyaXB0LWVzbGludC9uby1lbXB0eS1mdW5jdGlvblxyXG5leHBvcnQgZnVuY3Rpb24gYjJGcmVlKG1lbTogYW55KTogdm9pZCB7fVxyXG5cclxuLy8vIExvZ2dpbmcgZnVuY3Rpb24uXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkxvZyhtZXNzYWdlOiBzdHJpbmcsIC4uLmFyZ3M6IGFueVtdKTogdm9pZCB7XHJcbiAgLy8gY29uc29sZS5sb2cobWVzc2FnZSwgLi4uYXJncyk7XHJcbn1cclxuXHJcbi8vLyBWZXJzaW9uIG51bWJlcmluZyBzY2hlbWUuXHJcbi8vLyBTZWUgaHR0cDovL2VuLndpa2lwZWRpYS5vcmcvd2lraS9Tb2Z0d2FyZV92ZXJzaW9uaW5nXHJcbmV4cG9ydCBjbGFzcyBiMlZlcnNpb24ge1xyXG4gIG1ham9yID0gMDsgLy8vPCBzaWduaWZpY2FudCBjaGFuZ2VzXHJcbiAgbWlub3IgPSAwOyAvLy88IGluY3JlbWVudGFsIGNoYW5nZXNcclxuICByZXZpc2lvbiA9IDA7IC8vLzwgYnVnIGZpeGVzXHJcblxyXG4gIGNvbnN0cnVjdG9yKG1ham9yID0gMCwgbWlub3IgPSAwLCByZXZpc2lvbiA9IDApIHtcclxuICAgIHRoaXMubWFqb3IgPSBtYWpvcjtcclxuICAgIHRoaXMubWlub3IgPSBtaW5vcjtcclxuICAgIHRoaXMucmV2aXNpb24gPSByZXZpc2lvbjtcclxuICB9XHJcblxyXG4gIHRvU3RyaW5nKCk6IHN0cmluZyB7XHJcbiAgICByZXR1cm4gdGhpcy5tYWpvciArICcuJyArIHRoaXMubWlub3IgKyAnLicgKyB0aGlzLnJldmlzaW9uO1xyXG4gIH1cclxufVxyXG5cclxuLy8vIEN1cnJlbnQgdmVyc2lvbi5cclxuZXhwb3J0IGNvbnN0IGIyX3ZlcnNpb246IGIyVmVyc2lvbiA9IG5ldyBiMlZlcnNpb24oMiwgMywgMik7XHJcblxyXG5leHBvcnQgY29uc3QgYjJfYnJhbmNoID0gJ21hc3Rlcic7XHJcbmV4cG9ydCBjb25zdCBiMl9jb21taXQgPSAnZmJmNTE4MDFkODBmYzM4OWQ0M2RjNDY1MjQ1MjBlODkwNDNiNmZhZic7XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJQYXJzZUludCh2OiBzdHJpbmcpOiBudW1iZXIge1xyXG4gIHJldHVybiBwYXJzZUludCh2LCAxMCk7XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMlBhcnNlVUludCh2OiBzdHJpbmcpOiBudW1iZXIge1xyXG4gIHJldHVybiBNYXRoLmFicyhwYXJzZUludCh2LCAxMCkpO1xyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJNYWtlQXJyYXk8VD4obGVuZ3RoOiBudW1iZXIsIGluaXQ6IChpOiBudW1iZXIpID0+IFQpOiBUW10ge1xyXG4gIGNvbnN0IGE6IFRbXSA9IFtdO1xyXG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbGVuZ3RoOyArK2kpIHtcclxuICAgIGFbaV0gPSBpbml0KGkpO1xyXG4gIH1cclxuICByZXR1cm4gYTtcclxufVxyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyTWFrZU51bGxBcnJheTxUPihsZW5ndGg6IG51bWJlcik6IEFycmF5PFQgfCBudWxsPiB7XHJcbiAgY29uc3QgYTogQXJyYXk8VCB8IG51bGw+ID0gW251bGxdO1xyXG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbGVuZ3RoOyArK2kpIHtcclxuICAgIGFbaV0gPSBudWxsO1xyXG4gIH1cclxuICByZXR1cm4gYTtcclxufVxyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyTWFrZUludEFycmF5KGxlbmd0aDogbnVtYmVyKTogbnVtYmVyW10ge1xyXG4gIGNvbnN0IGE6IG51bWJlcltdID0gWzBdO1xyXG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbGVuZ3RoOyArK2kpIHtcclxuICAgIGFbaV0gPSAwO1xyXG4gIH1cclxuICByZXR1cm4gYTtcclxufVxyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyTWFrZU51bWJlckFycmF5KGxlbmd0aDogbnVtYmVyKTogbnVtYmVyW10ge1xyXG4gIGNvbnN0IGEgPSBbTmFOXTtcclxuICBmb3IgKGxldCBpID0gMDsgaSA8IGxlbmd0aDsgKytpKSB7XHJcbiAgICBhW2ldID0gMC4wO1xyXG4gIH1cclxuICByZXR1cm4gYTtcclxufVxyXG4iXX0=