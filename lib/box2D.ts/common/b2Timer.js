/*
 * Copyright (c) 2011 Erin Catto http://box2d.org
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
/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
export class b2Timer {
    constructor() {
        this.m_start = NaN;
        if (B2_ENABLE_PROFILER) {
            this.m_start = performance.now();
        }
    }
    /// Reset the timer.
    Reset() {
        if (B2_ENABLE_PROFILER) {
            this.m_start = performance.now();
        }
        return this;
    }
    /// Get the time since construction or the last reset.
    GetMilliseconds() {
        if (B2_ENABLE_PROFILER) {
            return performance.now() - this.m_start;
        }
        else {
            return 0;
        }
    }
}
export class b2Counter {
    constructor() {
        this.m_count = 0;
        this.m_min_count = 0;
        this.m_max_count = 0;
    }
    GetCount() {
        return this.m_count;
    }
    GetMinCount() {
        return this.m_min_count;
    }
    GetMaxCount() {
        return this.m_max_count;
    }
    ResetCount() {
        const count = this.m_count;
        this.m_count = 0;
        return count;
    }
    ResetMinCount() {
        this.m_min_count = 0;
    }
    ResetMaxCount() {
        this.m_max_count = 0;
    }
    Increment() {
        ++this.m_count;
        if (this.m_max_count < this.m_count) {
            this.m_max_count = this.m_count;
        }
    }
    Decrement() {
        --this.m_count;
        if (this.m_min_count > this.m_count) {
            this.m_min_count = this.m_count;
        }
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUaW1lci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb21tb24vYjJUaW1lci50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILGdFQUFnRTtBQUNoRSwrQkFBK0I7QUFDL0IsTUFBTSxPQUFPLE9BQU87SUFHbEI7UUFGQSxZQUFPLEdBQUcsR0FBRyxDQUFDO1FBR1osSUFBSSxrQkFBa0IsRUFBRTtZQUN0QixJQUFJLENBQUMsT0FBTyxHQUFHLFdBQVcsQ0FBQyxHQUFHLEVBQUUsQ0FBQztTQUNsQztJQUNILENBQUM7SUFFRCxvQkFBb0I7SUFDcEIsS0FBSztRQUNILElBQUksa0JBQWtCLEVBQUU7WUFDdEIsSUFBSSxDQUFDLE9BQU8sR0FBRyxXQUFXLENBQUMsR0FBRyxFQUFFLENBQUM7U0FDbEM7UUFDRCxPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7SUFFRCxzREFBc0Q7SUFDdEQsZUFBZTtRQUNiLElBQUksa0JBQWtCLEVBQUU7WUFDdEIsT0FBTyxXQUFXLENBQUMsR0FBRyxFQUFFLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQztTQUN6QzthQUFNO1lBQ0wsT0FBTyxDQUFDLENBQUM7U0FDVjtJQUNILENBQUM7Q0FDRjtBQUVELE1BQU0sT0FBTyxTQUFTO0lBQXRCO1FBQ0UsWUFBTyxHQUFHLENBQUMsQ0FBQztRQUNaLGdCQUFXLEdBQUcsQ0FBQyxDQUFDO1FBQ2hCLGdCQUFXLEdBQUcsQ0FBQyxDQUFDO0lBMkNsQixDQUFDO0lBekNDLFFBQVE7UUFDTixPQUFPLElBQUksQ0FBQyxPQUFPLENBQUM7SUFDdEIsQ0FBQztJQUVELFdBQVc7UUFDVCxPQUFPLElBQUksQ0FBQyxXQUFXLENBQUM7SUFDMUIsQ0FBQztJQUVELFdBQVc7UUFDVCxPQUFPLElBQUksQ0FBQyxXQUFXLENBQUM7SUFDMUIsQ0FBQztJQUVELFVBQVU7UUFDUixNQUFNLEtBQUssR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDO1FBQzNCLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxDQUFDO1FBQ2pCLE9BQU8sS0FBSyxDQUFDO0lBQ2YsQ0FBQztJQUVELGFBQWE7UUFDWCxJQUFJLENBQUMsV0FBVyxHQUFHLENBQUMsQ0FBQztJQUN2QixDQUFDO0lBRUQsYUFBYTtRQUNYLElBQUksQ0FBQyxXQUFXLEdBQUcsQ0FBQyxDQUFDO0lBQ3ZCLENBQUM7SUFFRCxTQUFTO1FBQ1AsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDO1FBRWYsSUFBSSxJQUFJLENBQUMsV0FBVyxHQUFHLElBQUksQ0FBQyxPQUFPLEVBQUU7WUFDbkMsSUFBSSxDQUFDLFdBQVcsR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDO1NBQ2pDO0lBQ0gsQ0FBQztJQUVELFNBQVM7UUFDUCxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUM7UUFFZixJQUFJLElBQUksQ0FBQyxXQUFXLEdBQUcsSUFBSSxDQUFDLE9BQU8sRUFBRTtZQUNuQyxJQUFJLENBQUMsV0FBVyxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUM7U0FDakM7SUFDSCxDQUFDO0NBQ0YiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMTEgRXJpbiBDYXR0byBodHRwOi8vYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbi8vLyBUaW1lciBmb3IgcHJvZmlsaW5nLiBUaGlzIGhhcyBwbGF0Zm9ybSBzcGVjaWZpYyBjb2RlIGFuZCBtYXlcclxuLy8vIG5vdCB3b3JrIG9uIGV2ZXJ5IHBsYXRmb3JtLlxyXG5leHBvcnQgY2xhc3MgYjJUaW1lciB7XHJcbiAgbV9zdGFydCA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICBpZiAoQjJfRU5BQkxFX1BST0ZJTEVSKSB7XHJcbiAgICAgIHRoaXMubV9zdGFydCA9IHBlcmZvcm1hbmNlLm5vdygpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vIFJlc2V0IHRoZSB0aW1lci5cclxuICBSZXNldCgpOiBiMlRpbWVyIHtcclxuICAgIGlmIChCMl9FTkFCTEVfUFJPRklMRVIpIHtcclxuICAgICAgdGhpcy5tX3N0YXJ0ID0gcGVyZm9ybWFuY2Uubm93KCk7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHRpbWUgc2luY2UgY29uc3RydWN0aW9uIG9yIHRoZSBsYXN0IHJlc2V0LlxyXG4gIEdldE1pbGxpc2Vjb25kcygpOiBudW1iZXIge1xyXG4gICAgaWYgKEIyX0VOQUJMRV9QUk9GSUxFUikge1xyXG4gICAgICByZXR1cm4gcGVyZm9ybWFuY2Uubm93KCkgLSB0aGlzLm1fc3RhcnQ7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICByZXR1cm4gMDtcclxuICAgIH1cclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMkNvdW50ZXIge1xyXG4gIG1fY291bnQgPSAwO1xyXG4gIG1fbWluX2NvdW50ID0gMDtcclxuICBtX21heF9jb3VudCA9IDA7XHJcblxyXG4gIEdldENvdW50KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2NvdW50O1xyXG4gIH1cclxuXHJcbiAgR2V0TWluQ291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fbWluX2NvdW50O1xyXG4gIH1cclxuXHJcbiAgR2V0TWF4Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fbWF4X2NvdW50O1xyXG4gIH1cclxuXHJcbiAgUmVzZXRDb3VudCgpOiBudW1iZXIge1xyXG4gICAgY29uc3QgY291bnQgPSB0aGlzLm1fY291bnQ7XHJcbiAgICB0aGlzLm1fY291bnQgPSAwO1xyXG4gICAgcmV0dXJuIGNvdW50O1xyXG4gIH1cclxuXHJcbiAgUmVzZXRNaW5Db3VudCgpOiB2b2lkIHtcclxuICAgIHRoaXMubV9taW5fY291bnQgPSAwO1xyXG4gIH1cclxuXHJcbiAgUmVzZXRNYXhDb3VudCgpOiB2b2lkIHtcclxuICAgIHRoaXMubV9tYXhfY291bnQgPSAwO1xyXG4gIH1cclxuXHJcbiAgSW5jcmVtZW50KCk6IHZvaWQge1xyXG4gICAgKyt0aGlzLm1fY291bnQ7XHJcblxyXG4gICAgaWYgKHRoaXMubV9tYXhfY291bnQgPCB0aGlzLm1fY291bnQpIHtcclxuICAgICAgdGhpcy5tX21heF9jb3VudCA9IHRoaXMubV9jb3VudDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIERlY3JlbWVudCgpOiB2b2lkIHtcclxuICAgIC0tdGhpcy5tX2NvdW50O1xyXG5cclxuICAgIGlmICh0aGlzLm1fbWluX2NvdW50ID4gdGhpcy5tX2NvdW50KSB7XHJcbiAgICAgIHRoaXMubV9taW5fY291bnQgPSB0aGlzLm1fY291bnQ7XHJcbiAgICB9XHJcbiAgfVxyXG59XHJcbiJdfQ==