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
import { b2DynamicTree } from './b2DynamicTree';
function std_iter_swap(array, a, b) {
    const tmp = array[a];
    array[a] = array[b];
    array[b] = tmp;
}
function default_compare(a, b) {
    return a < b;
}
function std_sort(array, first = 0, len = array.length - first, cmp = default_compare) {
    let left = first;
    const stack = [];
    let pos = 0;
    for (;;) {
        /* outer loop */
        for (; left + 1 < len; len++) {
            /* sort left to len-1 */
            const pivot = array[left + Math.floor(Math.random() * (len - left))]; /* pick random pivot */
            stack[pos++] = len; /* sort right part later */
            for (let right = left - 1;;) {
                /* inner loop: partitioning */
                // eslint-disable-next-line no-empty
                while (cmp(array[++right], pivot)) { } /* look for greater element */
                // eslint-disable-next-line no-empty
                while (cmp(pivot, array[--len])) { } /* look for smaller element */
                if (right >= len) {
                    break;
                } /* partition point found? */
                std_iter_swap(array, right, len); /* the only swap */
            } /* partitioned, continue left part */
        }
        if (pos === 0) {
            break;
        } /* stack empty? */
        left = len; /* left to right is sorted */
        len = stack[--pos]; /* get next range to sort */
    }
    return array;
}
export class b2Pair {
    constructor(proxyA, proxyB) {
        this.proxyA = proxyA;
        this.proxyB = proxyB;
    }
}
/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
export class b2BroadPhase {
    constructor() {
        this._queryCache = [];
        this.m_tree = new b2DynamicTree();
        this.m_proxyCount = 0;
        // m_moveCapacity: number = 16;
        this.m_moveCount = 0;
        this.m_moveBuffer = [];
        // m_pairCapacity: number = 16;
        this.m_pairCount = 0;
        this.m_pairBuffer = [];
    }
    // m_queryProxyId: number = 0;
    /// Create a proxy with an initial AABB. Pairs are not reported until
    /// UpdatePairs is called.
    CreateProxy(aabb, userData) {
        const proxy = this.m_tree.CreateProxy(aabb, userData);
        ++this.m_proxyCount;
        this.BufferMove(proxy);
        return proxy;
    }
    /// Destroy a proxy. It is up to the client to remove any pairs.
    DestroyProxy(proxy) {
        this.UnBufferMove(proxy);
        --this.m_proxyCount;
        this.m_tree.DestroyProxy(proxy);
    }
    /// Call MoveProxy as many times as you like, then when you are done
    /// call UpdatePairs to finalized the proxy pairs (for your time step).
    MoveProxy(proxy, aabb, displacement) {
        const buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
        if (buffer) {
            this.BufferMove(proxy);
        }
    }
    /// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
    TouchProxy(proxy) {
        this.BufferMove(proxy);
    }
    /// Get the fat AABB for a proxy.
    // GetFatAABB(proxy: b2TreeNode<T>): b2AABB {
    //   return this.m_tree.GetFatAABB(proxy);
    // }
    /// Get user data from a proxy. Returns NULL if the id is invalid.
    // GetUserData(proxy: b2TreeNode<T>): T {
    //   return this.m_tree.GetUserData(proxy);
    // }
    /// Test overlap of fat AABBs.
    // TestOverlap(proxyA: b2TreeNode<T>, proxyB: b2TreeNode<T>): boolean {
    //   const aabbA: b2AABB = this.m_tree.GetFatAABB(proxyA);
    //   const aabbB: b2AABB = this.m_tree.GetFatAABB(proxyB);
    //   return b2TestOverlapAABB(aabbA, aabbB);
    // }
    /// Get the number of proxies.
    GetProxyCount() {
        return this.m_proxyCount;
    }
    /// Update the pairs. This results in pair callbacks. This can only add pairs.
    // UpdatePairs(callback: (a: T, b: T) => void): void {
    //   // Reset pair buffer
    //   this.m_pairCount = 0;
    //
    //   // Perform tree queries for all moving proxies.
    //   for (let i = 0; i < this.m_moveCount; ++i) {
    //     const queryProxy: b2TreeNode<T> | null = this.m_moveBuffer[i];
    //     if (queryProxy === null) {
    //       continue;
    //     }
    //
    //     // This is called from box2d.b2DynamicTree::Query when we are gathering pairs.
    //     // boolean b2BroadPhase::QueryCallback(int32 proxyId);
    //
    //     // We have to query the tree with the fat AABB so that
    //     // we don't fail to create a pair that may touch later.
    //     const fatAABB: b2AABB = queryProxy.aabb; // this.m_tree.GetFatAABB(queryProxy);
    //
    //     // Query tree, create pairs and add them pair buffer.
    //     this.m_tree.Query(fatAABB, (proxy: b2TreeNode<T>): boolean => {
    //       // A proxy cannot form a pair with itself.
    //       if (proxy.m_id === queryProxy.m_id) {
    //         return true;
    //       }
    //
    //       // const proxyA = proxy < queryProxy ? proxy : queryProxy;
    //       // const proxyB = proxy >= queryProxy ? proxy : queryProxy;
    //       let proxyA: b2TreeNode<T>;
    //       let proxyB: b2TreeNode<T>;
    //       if (proxy.m_id < queryProxy.m_id) {
    //         proxyA = proxy;
    //         proxyB = queryProxy;
    //       } else {
    //         proxyA = queryProxy;
    //         proxyB = proxy;
    //       }
    //
    //       // Grow the pair buffer as needed.
    //       if (this.m_pairCount === this.m_pairBuffer.length) {
    //         this.m_pairBuffer[this.m_pairCount] = new b2Pair(proxyA, proxyB);
    //       } else {
    //         const pair: b2Pair<T> = this.m_pairBuffer[this.m_pairCount];
    //         pair.proxyA = proxyA;
    //         pair.proxyB = proxyB;
    //       }
    //
    //       ++this.m_pairCount;
    //
    //       return true;
    //     });
    //   }
    //
    //   // Reset move buffer
    //   this.m_moveCount = 0;
    //
    //   // Sort the pair buffer to expose duplicates.
    //   std_sort(this.m_pairBuffer, 0, this.m_pairCount, b2PairLessThan);
    //
    //   // Send the pairs back to the client.
    //   let i: number = 0;
    //   while (i < this.m_pairCount) {
    //     const primaryPair: b2Pair<T> = this.m_pairBuffer[i];
    //     const userDataA: T = primaryPair.proxyA.userData; // this.m_tree.GetUserData(primaryPair.proxyA);
    //     const userDataB: T = primaryPair.proxyB.userData; // this.m_tree.GetUserData(primaryPair.proxyB);
    //
    //     callback(userDataA, userDataB);
    //     ++i;
    //
    //     // Skip any duplicate pairs.
    //     while (i < this.m_pairCount) {
    //       const pair: b2Pair<T> = this.m_pairBuffer[i];
    //       if (pair.proxyA.m_id !== primaryPair.proxyA.m_id || pair.proxyB.m_id !== primaryPair.proxyB.m_id) {
    //         break;
    //       }
    //       ++i;
    //     }
    //   }
    //
    //   // Try to keep the tree balanced.
    //   // this.m_tree.Rebalance(4);
    // }
    /// Update the pairs. This results in pair callbacks. This can only add pairs.
    UpdatePairs_(a, b) {
        // Reset pair buffer
        this.m_pairCount = 0;
        // Perform tree queries for all moving proxies.
        this._PerformTreeQueriesForMovingProxies();
        // Reset move buffer
        this.m_moveCount = 0;
        // Sort the pair buffer to expose duplicates.
        std_sort(this.m_pairBuffer, 0, this.m_pairCount, b2PairLessThan);
        // Send the pairs back to the client.
        this._SendPairsBackToClient(a, b);
        // Try to keep the tree balanced.
        // this.m_tree.Rebalance(4);
    }
    _PerformTreeQueriesForMovingProxies() {
        // Perform tree queries for all moving proxies.
        for (let i = 0; i < this.m_moveCount; ++i) {
            const queryProxy = this.m_moveBuffer[i];
            if (queryProxy === null) {
                continue;
            }
            // This is called from box2d.b2DynamicTree::Query when we are gathering pairs.
            // boolean b2BroadPhase::QueryCallback(int32 proxyId);
            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            const fatAABB = queryProxy.aabb; // this.m_tree.GetFatAABB(queryProxy);
            this._queryCache.length = 0;
            // Query tree, create pairs and add them pair buffer.
            this.m_tree.Query_(fatAABB, this._queryCache);
            for (let j = 0; j < this._queryCache.length; ++j) {
                const proxy = this._queryCache[j];
                // A proxy cannot form a pair with itself.
                if (proxy.m_id === queryProxy.m_id) {
                    continue;
                }
                // const proxyA = proxy < queryProxy ? proxy : queryProxy;
                // const proxyB = proxy >= queryProxy ? proxy : queryProxy;
                let proxyA;
                let proxyB;
                if (proxy.m_id < queryProxy.m_id) {
                    proxyA = proxy;
                    proxyB = queryProxy;
                }
                else {
                    proxyA = queryProxy;
                    proxyB = proxy;
                }
                // Grow the pair buffer as needed.
                if (this.m_pairCount === this.m_pairBuffer.length) {
                    this.m_pairBuffer[this.m_pairCount] = new b2Pair(proxyA, proxyB);
                }
                else {
                    const pair = this.m_pairBuffer[this.m_pairCount];
                    pair.proxyA = proxyA;
                    pair.proxyB = proxyB;
                }
                ++this.m_pairCount;
            }
        }
    }
    _SendPairsBackToClient(a, b) {
        const count = this.m_pairCount;
        let i = 0;
        let j = 0;
        while (i < count) {
            const primaryPair = this.m_pairBuffer[i];
            a[j] = primaryPair.proxyA.userData; // this.m_tree.GetUserData(primaryPair.proxyA);
            b[j] = primaryPair.proxyB.userData; // this.m_tree.GetUserData(primaryPair.proxyB);
            ++j;
            ++i;
            // Skip any duplicate pairs.
            while (i < count) {
                const pair = this.m_pairBuffer[i];
                if (pair.proxyA.m_id !== primaryPair.proxyA.m_id ||
                    pair.proxyB.m_id !== primaryPair.proxyB.m_id) {
                    break;
                }
                ++i;
            }
        }
    }
    /// Query an AABB for overlapping proxies. The callback class
    /// is called for each proxy that overlaps the supplied AABB.
    Query(aabb, callback) {
        this.m_tree.Query(aabb, callback);
    }
    QueryPoint(point, callback) {
        this.m_tree.QueryPoint(point, callback);
    }
    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback a callback class that is called for each proxy that is hit by the ray.
    RayCast(input, callback) {
        this.m_tree.RayCast(input, callback);
    }
    /// Get the height of the embedded tree.
    GetTreeHeight() {
        return this.m_tree.GetHeight();
    }
    /// Get the balance of the embedded tree.
    GetTreeBalance() {
        return this.m_tree.GetMaxBalance();
    }
    /// Get the quality metric of the embedded tree.
    GetTreeQuality() {
        return this.m_tree.GetAreaRatio();
    }
    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    ShiftOrigin(newOrigin) {
        this.m_tree.ShiftOrigin(newOrigin);
    }
    BufferMove(proxy) {
        this.m_moveBuffer[this.m_moveCount] = proxy;
        ++this.m_moveCount;
    }
    UnBufferMove(proxy) {
        const i = this.m_moveBuffer.indexOf(proxy);
        if (i >= 0) {
            this.m_moveBuffer[i] = null;
        }
    }
}
/// This is used to sort pairs.
function b2PairLessThan(pair1, pair2) {
    if (pair1.proxyA.m_id < pair2.proxyA.m_id) {
        return true;
    }
    if (pair1.proxyA.m_id === pair2.proxyA.m_id) {
        return pair1.proxyB.m_id < pair2.proxyB.m_id;
    }
    return false;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJCcm9hZFBoYXNlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2NvbGxpc2lvbi9iMkJyb2FkUGhhc2UudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFJSCxPQUFPLEVBQUUsYUFBYSxFQUFjLE1BQU0saUJBQWlCLENBQUM7QUFFNUQsU0FBUyxhQUFhLENBQUksS0FBVSxFQUFFLENBQVMsRUFBRSxDQUFTO0lBQ3hELE1BQU0sR0FBRyxHQUFNLEtBQUssQ0FBQyxDQUFDLENBQUMsQ0FBQztJQUN4QixLQUFLLENBQUMsQ0FBQyxDQUFDLEdBQUcsS0FBSyxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ3BCLEtBQUssQ0FBQyxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7QUFDakIsQ0FBQztBQUVELFNBQVMsZUFBZSxDQUFJLENBQUksRUFBRSxDQUFJO0lBQ3BDLE9BQU8sQ0FBQyxHQUFHLENBQUMsQ0FBQztBQUNmLENBQUM7QUFFRCxTQUFTLFFBQVEsQ0FDZixLQUFVLEVBQ1YsS0FBSyxHQUFHLENBQUMsRUFDVCxNQUFjLEtBQUssQ0FBQyxNQUFNLEdBQUcsS0FBSyxFQUNsQyxNQUErQixlQUFlO0lBRTlDLElBQUksSUFBSSxHQUFHLEtBQUssQ0FBQztJQUNqQixNQUFNLEtBQUssR0FBYSxFQUFFLENBQUM7SUFDM0IsSUFBSSxHQUFHLEdBQUcsQ0FBQyxDQUFDO0lBRVosU0FBUztRQUNQLGdCQUFnQjtRQUNoQixPQUFPLElBQUksR0FBRyxDQUFDLEdBQUcsR0FBRyxFQUFFLEdBQUcsRUFBRSxFQUFFO1lBQzVCLHdCQUF3QjtZQUN4QixNQUFNLEtBQUssR0FBRyxLQUFLLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLE1BQU0sRUFBRSxHQUFHLENBQUMsR0FBRyxHQUFHLElBQUksQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLHVCQUF1QjtZQUM3RixL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Q0FBQyxXQUFXLENBQUM7SUFDckIsQ0FBQztJQUVELFlBQVksQ0FBQyxLQUFvQjtRQUMvQixNQUFNLENBQUMsR0FBVyxJQUFJLENBQUMsWUFBWSxDQUFDLE9BQU8sQ0FBQyxLQUFLLENBQUMsQ0FBQztRQUNuRCxJQUFJLENBQUMsSUFBSSxDQUFDLEVBQUU7WUFDVixJQUFJLENBQUMsWUFBWSxDQUFDLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQztTQUM3QjtJQUNILENBQUM7Q0FDRjtBQUVELCtCQUErQjtBQUMvQixTQUFTLGNBQWMsQ0FBSSxLQUFnQixFQUFFLEtBQWdCO0lBQzNELElBQUksS0FBSyxDQUFDLE1BQU0sQ0FBQyxJQUFJLEdBQUcsS0FBSyxDQUFDLE1BQU0sQ0FBQyxJQUFJLEVBQUU7UUFDekMsT0FBTyxJQUFJLENBQUM7S0FDYjtJQUVELElBQUksS0FBSyxDQUFDLE1BQU0sQ0FBQyxJQUFJLEtBQUssS0FBSyxDQUFDLE1BQU0sQ0FBQyxJQUFJLEVBQUU7UUFDM0MsT0FBTyxLQUFLLENBQUMsTUFBTSxDQUFDLElBQUksR0FBRyxLQUFLLENBQUMsTUFBTSxDQUFDLElBQUksQ0FBQztLQUM5QztJQUVELE9BQU8sS0FBSyxDQUFDO0FBQ2YsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJWZWMyLCBYWSB9IGZyb20gJy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMkFBQkIsIGIyUmF5Q2FzdElucHV0IH0gZnJvbSAnLi9iMkNvbGxpc2lvbic7XHJcbmltcG9ydCB7IGIyRHluYW1pY1RyZWUsIGIyVHJlZU5vZGUgfSBmcm9tICcuL2IyRHluYW1pY1RyZWUnO1xyXG5cclxuZnVuY3Rpb24gc3RkX2l0ZXJfc3dhcDxUPihhcnJheTogVFtdLCBhOiBudW1iZXIsIGI6IG51bWJlcik6IHZvaWQge1xyXG4gIGNvbnN0IHRtcDogVCA9IGFycmF5W2FdO1xyXG4gIGFycmF5W2FdID0gYXJyYXlbYl07XHJcbiAgYXJyYXlbYl0gPSB0bXA7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGRlZmF1bHRfY29tcGFyZTxUPihhOiBULCBiOiBUKTogYm9vbGVhbiB7XHJcbiAgcmV0dXJuIGEgPCBiO1xyXG59XHJcblxyXG5mdW5jdGlvbiBzdGRfc29ydDxUPihcclxuICBhcnJheTogVFtdLFxyXG4gIGZpcnN0ID0gMCxcclxuICBsZW46IG51bWJlciA9IGFycmF5Lmxlbmd0aCAtIGZpcnN0LFxyXG4gIGNtcDogKGE6IFQsIGI6IFQpID0+IGJvb2xlYW4gPSBkZWZhdWx0X2NvbXBhcmUsXHJcbik6IFRbXSB7XHJcbiAgbGV0IGxlZnQgPSBmaXJzdDtcclxuICBjb25zdCBzdGFjazogbnVtYmVyW10gPSBbXTtcclxuICBsZXQgcG9zID0gMDtcclxuXHJcbiAgZm9yICg7Oykge1xyXG4gICAgLyogb3V0ZXIgbG9vcCAqL1xyXG4gICAgZm9yICg7IGxlZnQgKyAxIDwgbGVuOyBsZW4rKykge1xyXG4gICAgICAvKiBzb3J0IGxlZnQgdG8gbGVuLTEgKi9cclxuICAgICAgY29uc3QgcGl2b3QgPSBhcnJheVtsZWZ0ICsgTWF0aC5mbG9vcihNYXRoLnJhbmRvbSgpICogKGxlbiAtIGxlZnQpKV07IC8qIHBpY2sgcmFuZG9tIHBpdm90ICovXHJcbiAgICAgIHN0YWNrW3BvcysrXSA9IGxlbjsgLyogc29ydCByaWdodCBwYXJ0IGxhdGVyICovXHJcbiAgICAgIGZvciAobGV0IHJpZ2h0ID0gbGVmdCAtIDE7IDsgKSB7XHJcbiAgICAgICAgLyogaW5uZXIgbG9vcDogcGFydGl0aW9uaW5nICovXHJcbiAgICAgICAgLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIG5vLWVtcHR5XHJcbiAgICAgICAgd2hpbGUgKGNtcChhcnJheVsrK3JpZ2h0XSwgcGl2b3QpKSB7fSAvKiBsb29rIGZvciBncmVhdGVyIGVsZW1lbnQgKi9cclxuICAgICAgICAvLyBlc2xpbnQtZGlzYWJsZS1uZXh0LWxpbmUgbm8tZW1wdHlcclxuICAgICAgICB3aGlsZSAoY21wKHBpdm90LCBhcnJheVstLWxlbl0pKSB7fSAvKiBsb29rIGZvciBzbWFsbGVyIGVsZW1lbnQgKi9cclxuICAgICAgICBpZiAocmlnaHQgPj0gbGVuKSB7XHJcbiAgICAgICAgICBicmVhaztcclxuICAgICAgICB9IC8qIHBhcnRpdGlvbiBwb2ludCBmb3VuZD8gKi9cclxuICAgICAgICBzdGRfaXRlcl9zd2FwKGFycmF5LCByaWdodCwgbGVuKTsgLyogdGhlIG9ubHkgc3dhcCAqL1xyXG4gICAgICB9IC8qIHBhcnRpdGlvbmVkLCBjb250aW51ZSBsZWZ0IHBhcnQgKi9cclxuICAgIH1cclxuICAgIGlmIChwb3MgPT09IDApIHtcclxuICAgICAgYnJlYWs7XHJcbiAgICB9IC8qIHN0YWNrIGVtcHR5PyAqL1xyXG4gICAgbGVmdCA9IGxlbjsgLyogbGVmdCB0byByaWdodCBpcyBzb3J0ZWQgKi9cclxuICAgIGxlbiA9IHN0YWNrWy0tcG9zXTsgLyogZ2V0IG5leHQgcmFuZ2UgdG8gc29ydCAqL1xyXG4gIH1cclxuXHJcbiAgcmV0dXJuIGFycmF5O1xyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJQYWlyPFQ+IHtcclxuICBjb25zdHJ1Y3RvcihwdWJsaWMgcHJveHlBOiBiMlRyZWVOb2RlPFQ+LCBwdWJsaWMgcHJveHlCOiBiMlRyZWVOb2RlPFQ+KSB7fVxyXG59XHJcblxyXG4vLy8gVGhlIGJyb2FkLXBoYXNlIGlzIHVzZWQgZm9yIGNvbXB1dGluZyBwYWlycyBhbmQgcGVyZm9ybWluZyB2b2x1bWUgcXVlcmllcyBhbmQgcmF5IGNhc3RzLlxyXG4vLy8gVGhpcyBicm9hZC1waGFzZSBkb2VzIG5vdCBwZXJzaXN0IHBhaXJzLiBJbnN0ZWFkLCB0aGlzIHJlcG9ydHMgcG90ZW50aWFsbHkgbmV3IHBhaXJzLlxyXG4vLy8gSXQgaXMgdXAgdG8gdGhlIGNsaWVudCB0byBjb25zdW1lIHRoZSBuZXcgcGFpcnMgYW5kIHRvIHRyYWNrIHN1YnNlcXVlbnQgb3ZlcmxhcC5cclxuZXhwb3J0IGNsYXNzIGIyQnJvYWRQaGFzZTxUPiB7XHJcbiAgcHJpdmF0ZSBfcXVlcnlDYWNoZTogYjJUcmVlTm9kZTxUPltdID0gW107XHJcblxyXG4gIHJlYWRvbmx5IG1fdHJlZTogYjJEeW5hbWljVHJlZTxUPiA9IG5ldyBiMkR5bmFtaWNUcmVlPFQ+KCk7XHJcbiAgbV9wcm94eUNvdW50ID0gMDtcclxuICAvLyBtX21vdmVDYXBhY2l0eTogbnVtYmVyID0gMTY7XHJcbiAgbV9tb3ZlQ291bnQgPSAwO1xyXG4gIHJlYWRvbmx5IG1fbW92ZUJ1ZmZlcjogQXJyYXk8YjJUcmVlTm9kZTxUPiB8IG51bGw+ID0gW107XHJcbiAgLy8gbV9wYWlyQ2FwYWNpdHk6IG51bWJlciA9IDE2O1xyXG4gIG1fcGFpckNvdW50ID0gMDtcclxuICByZWFkb25seSBtX3BhaXJCdWZmZXI6IEFycmF5PGIyUGFpcjxUPj4gPSBbXTtcclxuICAvLyBtX3F1ZXJ5UHJveHlJZDogbnVtYmVyID0gMDtcclxuXHJcbiAgLy8vIENyZWF0ZSBhIHByb3h5IHdpdGggYW4gaW5pdGlhbCBBQUJCLiBQYWlycyBhcmUgbm90IHJlcG9ydGVkIHVudGlsXHJcbiAgLy8vIFVwZGF0ZVBhaXJzIGlzIGNhbGxlZC5cclxuICBDcmVhdGVQcm94eShhYWJiOiBiMkFBQkIsIHVzZXJEYXRhOiBUKTogYjJUcmVlTm9kZTxUPiB7XHJcbiAgICBjb25zdCBwcm94eTogYjJUcmVlTm9kZTxUPiA9IHRoaXMubV90cmVlLkNyZWF0ZVByb3h5KGFhYmIsIHVzZXJEYXRhKTtcclxuICAgICsrdGhpcy5tX3Byb3h5Q291bnQ7XHJcbiAgICB0aGlzLkJ1ZmZlck1vdmUocHJveHkpO1xyXG4gICAgcmV0dXJuIHByb3h5O1xyXG4gIH1cclxuXHJcbiAgLy8vIERlc3Ryb3kgYSBwcm94eS4gSXQgaXMgdXAgdG8gdGhlIGNsaWVudCB0byByZW1vdmUgYW55IHBhaXJzLlxyXG4gIERlc3Ryb3lQcm94eShwcm94eTogYjJUcmVlTm9kZTxUPik6IHZvaWQge1xyXG4gICAgdGhpcy5VbkJ1ZmZlck1vdmUocHJveHkpO1xyXG4gICAgLS10aGlzLm1fcHJveHlDb3VudDtcclxuICAgIHRoaXMubV90cmVlLkRlc3Ryb3lQcm94eShwcm94eSk7XHJcbiAgfVxyXG5cclxuICAvLy8gQ2FsbCBNb3ZlUHJveHkgYXMgbWFueSB0aW1lcyBhcyB5b3UgbGlrZSwgdGhlbiB3aGVuIHlvdSBhcmUgZG9uZVxyXG4gIC8vLyBjYWxsIFVwZGF0ZVBhaXJzIHRvIGZpbmFsaXplZCB0aGUgcHJveHkgcGFpcnMgKGZvciB5b3VyIHRpbWUgc3RlcCkuXHJcbiAgTW92ZVByb3h5KHByb3h5OiBiMlRyZWVOb2RlPFQ+LCBhYWJiOiBiMkFBQkIsIGRpc3BsYWNlbWVudDogYjJWZWMyKTogdm9pZCB7XHJcbiAgICBjb25zdCBidWZmZXI6IGJvb2xlYW4gPSB0aGlzLm1fdHJlZS5Nb3ZlUHJveHkocHJveHksIGFhYmIsIGRpc3BsYWNlbWVudCk7XHJcbiAgICBpZiAoYnVmZmVyKSB7XHJcbiAgICAgIHRoaXMuQnVmZmVyTW92ZShwcm94eSk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLy8gQ2FsbCB0byB0cmlnZ2VyIGEgcmUtcHJvY2Vzc2luZyBvZiBpdCdzIHBhaXJzIG9uIHRoZSBuZXh0IGNhbGwgdG8gVXBkYXRlUGFpcnMuXHJcbiAgVG91Y2hQcm94eShwcm94eTogYjJUcmVlTm9kZTxUPik6IHZvaWQge1xyXG4gICAgdGhpcy5CdWZmZXJNb3ZlKHByb3h5KTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGZhdCBBQUJCIGZvciBhIHByb3h5LlxyXG4gIC8vIEdldEZhdEFBQkIocHJveHk6IGIyVHJlZU5vZGU8VD4pOiBiMkFBQkIge1xyXG4gIC8vICAgcmV0dXJuIHRoaXMubV90cmVlLkdldEZhdEFBQkIocHJveHkpO1xyXG4gIC8vIH1cclxuXHJcbiAgLy8vIEdldCB1c2VyIGRhdGEgZnJvbSBhIHByb3h5LiBSZXR1cm5zIE5VTEwgaWYgdGhlIGlkIGlzIGludmFsaWQuXHJcbiAgLy8gR2V0VXNlckRhdGEocHJveHk6IGIyVHJlZU5vZGU8VD4pOiBUIHtcclxuICAvLyAgIHJldHVybiB0aGlzLm1fdHJlZS5HZXRVc2VyRGF0YShwcm94eSk7XHJcbiAgLy8gfVxyXG5cclxuICAvLy8gVGVzdCBvdmVybGFwIG9mIGZhdCBBQUJCcy5cclxuICAvLyBUZXN0T3ZlcmxhcChwcm94eUE6IGIyVHJlZU5vZGU8VD4sIHByb3h5QjogYjJUcmVlTm9kZTxUPik6IGJvb2xlYW4ge1xyXG4gIC8vICAgY29uc3QgYWFiYkE6IGIyQUFCQiA9IHRoaXMubV90cmVlLkdldEZhdEFBQkIocHJveHlBKTtcclxuICAvLyAgIGNvbnN0IGFhYmJCOiBiMkFBQkIgPSB0aGlzLm1fdHJlZS5HZXRGYXRBQUJCKHByb3h5Qik7XHJcbiAgLy8gICByZXR1cm4gYjJUZXN0T3ZlcmxhcEFBQkIoYWFiYkEsIGFhYmJCKTtcclxuICAvLyB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIG51bWJlciBvZiBwcm94aWVzLlxyXG4gIEdldFByb3h5Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fcHJveHlDb3VudDtcclxuICB9XHJcblxyXG4gIC8vLyBVcGRhdGUgdGhlIHBhaXJzLiBUaGlzIHJlc3VsdHMgaW4gcGFpciBjYWxsYmFja3MuIFRoaXMgY2FuIG9ubHkgYWRkIHBhaXJzLlxyXG4gIC8vIFVwZGF0ZVBhaXJzKGNhbGxiYWNrOiAoYTogVCwgYjogVCkgPT4gdm9pZCk6IHZvaWQge1xyXG4gIC8vICAgLy8gUmVzZXQgcGFpciBidWZmZXJcclxuICAvLyAgIHRoaXMubV9wYWlyQ291bnQgPSAwO1xyXG4gIC8vXHJcbiAgLy8gICAvLyBQZXJmb3JtIHRyZWUgcXVlcmllcyBmb3IgYWxsIG1vdmluZyBwcm94aWVzLlxyXG4gIC8vICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fbW92ZUNvdW50OyArK2kpIHtcclxuICAvLyAgICAgY29uc3QgcXVlcnlQcm94eTogYjJUcmVlTm9kZTxUPiB8IG51bGwgPSB0aGlzLm1fbW92ZUJ1ZmZlcltpXTtcclxuICAvLyAgICAgaWYgKHF1ZXJ5UHJveHkgPT09IG51bGwpIHtcclxuICAvLyAgICAgICBjb250aW51ZTtcclxuICAvLyAgICAgfVxyXG4gIC8vXHJcbiAgLy8gICAgIC8vIFRoaXMgaXMgY2FsbGVkIGZyb20gYm94MmQuYjJEeW5hbWljVHJlZTo6UXVlcnkgd2hlbiB3ZSBhcmUgZ2F0aGVyaW5nIHBhaXJzLlxyXG4gIC8vICAgICAvLyBib29sZWFuIGIyQnJvYWRQaGFzZTo6UXVlcnlDYWxsYmFjayhpbnQzMiBwcm94eUlkKTtcclxuICAvL1xyXG4gIC8vICAgICAvLyBXZSBoYXZlIHRvIHF1ZXJ5IHRoZSB0cmVlIHdpdGggdGhlIGZhdCBBQUJCIHNvIHRoYXRcclxuICAvLyAgICAgLy8gd2UgZG9uJ3QgZmFpbCB0byBjcmVhdGUgYSBwYWlyIHRoYXQgbWF5IHRvdWNoIGxhdGVyLlxyXG4gIC8vICAgICBjb25zdCBmYXRBQUJCOiBiMkFBQkIgPSBxdWVyeVByb3h5LmFhYmI7IC8vIHRoaXMubV90cmVlLkdldEZhdEFBQkIocXVlcnlQcm94eSk7XHJcbiAgLy9cclxuICAvLyAgICAgLy8gUXVlcnkgdHJlZSwgY3JlYXRlIHBhaXJzIGFuZCBhZGQgdGhlbSBwYWlyIGJ1ZmZlci5cclxuICAvLyAgICAgdGhpcy5tX3RyZWUuUXVlcnkoZmF0QUFCQiwgKHByb3h5OiBiMlRyZWVOb2RlPFQ+KTogYm9vbGVhbiA9PiB7XHJcbiAgLy8gICAgICAgLy8gQSBwcm94eSBjYW5ub3QgZm9ybSBhIHBhaXIgd2l0aCBpdHNlbGYuXHJcbiAgLy8gICAgICAgaWYgKHByb3h5Lm1faWQgPT09IHF1ZXJ5UHJveHkubV9pZCkge1xyXG4gIC8vICAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgLy8gICAgICAgfVxyXG4gIC8vXHJcbiAgLy8gICAgICAgLy8gY29uc3QgcHJveHlBID0gcHJveHkgPCBxdWVyeVByb3h5ID8gcHJveHkgOiBxdWVyeVByb3h5O1xyXG4gIC8vICAgICAgIC8vIGNvbnN0IHByb3h5QiA9IHByb3h5ID49IHF1ZXJ5UHJveHkgPyBwcm94eSA6IHF1ZXJ5UHJveHk7XHJcbiAgLy8gICAgICAgbGV0IHByb3h5QTogYjJUcmVlTm9kZTxUPjtcclxuICAvLyAgICAgICBsZXQgcHJveHlCOiBiMlRyZWVOb2RlPFQ+O1xyXG4gIC8vICAgICAgIGlmIChwcm94eS5tX2lkIDwgcXVlcnlQcm94eS5tX2lkKSB7XHJcbiAgLy8gICAgICAgICBwcm94eUEgPSBwcm94eTtcclxuICAvLyAgICAgICAgIHByb3h5QiA9IHF1ZXJ5UHJveHk7XHJcbiAgLy8gICAgICAgfSBlbHNlIHtcclxuICAvLyAgICAgICAgIHByb3h5QSA9IHF1ZXJ5UHJveHk7XHJcbiAgLy8gICAgICAgICBwcm94eUIgPSBwcm94eTtcclxuICAvLyAgICAgICB9XHJcbiAgLy9cclxuICAvLyAgICAgICAvLyBHcm93IHRoZSBwYWlyIGJ1ZmZlciBhcyBuZWVkZWQuXHJcbiAgLy8gICAgICAgaWYgKHRoaXMubV9wYWlyQ291bnQgPT09IHRoaXMubV9wYWlyQnVmZmVyLmxlbmd0aCkge1xyXG4gIC8vICAgICAgICAgdGhpcy5tX3BhaXJCdWZmZXJbdGhpcy5tX3BhaXJDb3VudF0gPSBuZXcgYjJQYWlyKHByb3h5QSwgcHJveHlCKTtcclxuICAvLyAgICAgICB9IGVsc2Uge1xyXG4gIC8vICAgICAgICAgY29uc3QgcGFpcjogYjJQYWlyPFQ+ID0gdGhpcy5tX3BhaXJCdWZmZXJbdGhpcy5tX3BhaXJDb3VudF07XHJcbiAgLy8gICAgICAgICBwYWlyLnByb3h5QSA9IHByb3h5QTtcclxuICAvLyAgICAgICAgIHBhaXIucHJveHlCID0gcHJveHlCO1xyXG4gIC8vICAgICAgIH1cclxuICAvL1xyXG4gIC8vICAgICAgICsrdGhpcy5tX3BhaXJDb3VudDtcclxuICAvL1xyXG4gIC8vICAgICAgIHJldHVybiB0cnVlO1xyXG4gIC8vICAgICB9KTtcclxuICAvLyAgIH1cclxuICAvL1xyXG4gIC8vICAgLy8gUmVzZXQgbW92ZSBidWZmZXJcclxuICAvLyAgIHRoaXMubV9tb3ZlQ291bnQgPSAwO1xyXG4gIC8vXHJcbiAgLy8gICAvLyBTb3J0IHRoZSBwYWlyIGJ1ZmZlciB0byBleHBvc2UgZHVwbGljYXRlcy5cclxuICAvLyAgIHN0ZF9zb3J0KHRoaXMubV9wYWlyQnVmZmVyLCAwLCB0aGlzLm1fcGFpckNvdW50LCBiMlBhaXJMZXNzVGhhbik7XHJcbiAgLy9cclxuICAvLyAgIC8vIFNlbmQgdGhlIHBhaXJzIGJhY2sgdG8gdGhlIGNsaWVudC5cclxuICAvLyAgIGxldCBpOiBudW1iZXIgPSAwO1xyXG4gIC8vICAgd2hpbGUgKGkgPCB0aGlzLm1fcGFpckNvdW50KSB7XHJcbiAgLy8gICAgIGNvbnN0IHByaW1hcnlQYWlyOiBiMlBhaXI8VD4gPSB0aGlzLm1fcGFpckJ1ZmZlcltpXTtcclxuICAvLyAgICAgY29uc3QgdXNlckRhdGFBOiBUID0gcHJpbWFyeVBhaXIucHJveHlBLnVzZXJEYXRhOyAvLyB0aGlzLm1fdHJlZS5HZXRVc2VyRGF0YShwcmltYXJ5UGFpci5wcm94eUEpO1xyXG4gIC8vICAgICBjb25zdCB1c2VyRGF0YUI6IFQgPSBwcmltYXJ5UGFpci5wcm94eUIudXNlckRhdGE7IC8vIHRoaXMubV90cmVlLkdldFVzZXJEYXRhKHByaW1hcnlQYWlyLnByb3h5Qik7XHJcbiAgLy9cclxuICAvLyAgICAgY2FsbGJhY2sodXNlckRhdGFBLCB1c2VyRGF0YUIpO1xyXG4gIC8vICAgICArK2k7XHJcbiAgLy9cclxuICAvLyAgICAgLy8gU2tpcCBhbnkgZHVwbGljYXRlIHBhaXJzLlxyXG4gIC8vICAgICB3aGlsZSAoaSA8IHRoaXMubV9wYWlyQ291bnQpIHtcclxuICAvLyAgICAgICBjb25zdCBwYWlyOiBiMlBhaXI8VD4gPSB0aGlzLm1fcGFpckJ1ZmZlcltpXTtcclxuICAvLyAgICAgICBpZiAocGFpci5wcm94eUEubV9pZCAhPT0gcHJpbWFyeVBhaXIucHJveHlBLm1faWQgfHwgcGFpci5wcm94eUIubV9pZCAhPT0gcHJpbWFyeVBhaXIucHJveHlCLm1faWQpIHtcclxuICAvLyAgICAgICAgIGJyZWFrO1xyXG4gIC8vICAgICAgIH1cclxuICAvLyAgICAgICArK2k7XHJcbiAgLy8gICAgIH1cclxuICAvLyAgIH1cclxuICAvL1xyXG4gIC8vICAgLy8gVHJ5IHRvIGtlZXAgdGhlIHRyZWUgYmFsYW5jZWQuXHJcbiAgLy8gICAvLyB0aGlzLm1fdHJlZS5SZWJhbGFuY2UoNCk7XHJcbiAgLy8gfVxyXG5cclxuICAvLy8gVXBkYXRlIHRoZSBwYWlycy4gVGhpcyByZXN1bHRzIGluIHBhaXIgY2FsbGJhY2tzLiBUaGlzIGNhbiBvbmx5IGFkZCBwYWlycy5cclxuICBVcGRhdGVQYWlyc18oYTogVFtdLCBiOiBUW10pOiB2b2lkIHtcclxuICAgIC8vIFJlc2V0IHBhaXIgYnVmZmVyXHJcbiAgICB0aGlzLm1fcGFpckNvdW50ID0gMDtcclxuXHJcbiAgICAvLyBQZXJmb3JtIHRyZWUgcXVlcmllcyBmb3IgYWxsIG1vdmluZyBwcm94aWVzLlxyXG4gICAgdGhpcy5fUGVyZm9ybVRyZWVRdWVyaWVzRm9yTW92aW5nUHJveGllcygpO1xyXG5cclxuICAgIC8vIFJlc2V0IG1vdmUgYnVmZmVyXHJcbiAgICB0aGlzLm1fbW92ZUNvdW50ID0gMDtcclxuXHJcbiAgICAvLyBTb3J0IHRoZSBwYWlyIGJ1ZmZlciB0byBleHBvc2UgZHVwbGljYXRlcy5cclxuICAgIHN0ZF9zb3J0KHRoaXMubV9wYWlyQnVmZmVyLCAwLCB0aGlzLm1fcGFpckNvdW50LCBiMlBhaXJMZXNzVGhhbik7XHJcblxyXG4gICAgLy8gU2VuZCB0aGUgcGFpcnMgYmFjayB0byB0aGUgY2xpZW50LlxyXG4gICAgdGhpcy5fU2VuZFBhaXJzQmFja1RvQ2xpZW50KGEsIGIpO1xyXG5cclxuICAgIC8vIFRyeSB0byBrZWVwIHRoZSB0cmVlIGJhbGFuY2VkLlxyXG4gICAgLy8gdGhpcy5tX3RyZWUuUmViYWxhbmNlKDQpO1xyXG4gIH1cclxuXHJcbiAgX1BlcmZvcm1UcmVlUXVlcmllc0Zvck1vdmluZ1Byb3hpZXMoKTogdm9pZCB7XHJcbiAgICAvLyBQZXJmb3JtIHRyZWUgcXVlcmllcyBmb3IgYWxsIG1vdmluZyBwcm94aWVzLlxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fbW92ZUNvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgcXVlcnlQcm94eTogYjJUcmVlTm9kZTxUPiB8IG51bGwgPSB0aGlzLm1fbW92ZUJ1ZmZlcltpXTtcclxuICAgICAgaWYgKHF1ZXJ5UHJveHkgPT09IG51bGwpIHtcclxuICAgICAgICBjb250aW51ZTtcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gVGhpcyBpcyBjYWxsZWQgZnJvbSBib3gyZC5iMkR5bmFtaWNUcmVlOjpRdWVyeSB3aGVuIHdlIGFyZSBnYXRoZXJpbmcgcGFpcnMuXHJcbiAgICAgIC8vIGJvb2xlYW4gYjJCcm9hZFBoYXNlOjpRdWVyeUNhbGxiYWNrKGludDMyIHByb3h5SWQpO1xyXG5cclxuICAgICAgLy8gV2UgaGF2ZSB0byBxdWVyeSB0aGUgdHJlZSB3aXRoIHRoZSBmYXQgQUFCQiBzbyB0aGF0XHJcbiAgICAgIC8vIHdlIGRvbid0IGZhaWwgdG8gY3JlYXRlIGEgcGFpciB0aGF0IG1heSB0b3VjaCBsYXRlci5cclxuICAgICAgY29uc3QgZmF0QUFCQjogYjJBQUJCID0gcXVlcnlQcm94eS5hYWJiOyAvLyB0aGlzLm1fdHJlZS5HZXRGYXRBQUJCKHF1ZXJ5UHJveHkpO1xyXG5cclxuICAgICAgdGhpcy5fcXVlcnlDYWNoZS5sZW5ndGggPSAwO1xyXG4gICAgICAvLyBRdWVyeSB0cmVlLCBjcmVhdGUgcGFpcnMgYW5kIGFkZCB0aGVtIHBhaXIgYnVmZmVyLlxyXG4gICAgICB0aGlzLm1fdHJlZS5RdWVyeV8oZmF0QUFCQiwgdGhpcy5fcXVlcnlDYWNoZSk7XHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5fcXVlcnlDYWNoZS5sZW5ndGg7ICsraikge1xyXG4gICAgICAgIGNvbnN0IHByb3h5OiBiMlRyZWVOb2RlPFQ+ID0gdGhpcy5fcXVlcnlDYWNoZVtqXTtcclxuICAgICAgICAvLyBBIHByb3h5IGNhbm5vdCBmb3JtIGEgcGFpciB3aXRoIGl0c2VsZi5cclxuICAgICAgICBpZiAocHJveHkubV9pZCA9PT0gcXVlcnlQcm94eS5tX2lkKSB7XHJcbiAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIC8vIGNvbnN0IHByb3h5QSA9IHByb3h5IDwgcXVlcnlQcm94eSA/IHByb3h5IDogcXVlcnlQcm94eTtcclxuICAgICAgICAvLyBjb25zdCBwcm94eUIgPSBwcm94eSA+PSBxdWVyeVByb3h5ID8gcHJveHkgOiBxdWVyeVByb3h5O1xyXG4gICAgICAgIGxldCBwcm94eUE6IGIyVHJlZU5vZGU8VD47XHJcbiAgICAgICAgbGV0IHByb3h5QjogYjJUcmVlTm9kZTxUPjtcclxuICAgICAgICBpZiAocHJveHkubV9pZCA8IHF1ZXJ5UHJveHkubV9pZCkge1xyXG4gICAgICAgICAgcHJveHlBID0gcHJveHk7XHJcbiAgICAgICAgICBwcm94eUIgPSBxdWVyeVByb3h5O1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICBwcm94eUEgPSBxdWVyeVByb3h5O1xyXG4gICAgICAgICAgcHJveHlCID0gcHJveHk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBHcm93IHRoZSBwYWlyIGJ1ZmZlciBhcyBuZWVkZWQuXHJcbiAgICAgICAgaWYgKHRoaXMubV9wYWlyQ291bnQgPT09IHRoaXMubV9wYWlyQnVmZmVyLmxlbmd0aCkge1xyXG4gICAgICAgICAgdGhpcy5tX3BhaXJCdWZmZXJbdGhpcy5tX3BhaXJDb3VudF0gPSBuZXcgYjJQYWlyKHByb3h5QSwgcHJveHlCKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgY29uc3QgcGFpcjogYjJQYWlyPFQ+ID0gdGhpcy5tX3BhaXJCdWZmZXJbdGhpcy5tX3BhaXJDb3VudF07XHJcbiAgICAgICAgICBwYWlyLnByb3h5QSA9IHByb3h5QTtcclxuICAgICAgICAgIHBhaXIucHJveHlCID0gcHJveHlCO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgKyt0aGlzLm1fcGFpckNvdW50O1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBfU2VuZFBhaXJzQmFja1RvQ2xpZW50KGE6IFRbXSwgYjogVFtdKSB7XHJcbiAgICBjb25zdCBjb3VudCA9IHRoaXMubV9wYWlyQ291bnQ7XHJcbiAgICBsZXQgaSA9IDA7XHJcbiAgICBsZXQgaiA9IDA7XHJcbiAgICB3aGlsZSAoaSA8IGNvdW50KSB7XHJcbiAgICAgIGNvbnN0IHByaW1hcnlQYWlyOiBiMlBhaXI8VD4gPSB0aGlzLm1fcGFpckJ1ZmZlcltpXTtcclxuICAgICAgYVtqXSA9IHByaW1hcnlQYWlyLnByb3h5QS51c2VyRGF0YTsgLy8gdGhpcy5tX3RyZWUuR2V0VXNlckRhdGEocHJpbWFyeVBhaXIucHJveHlBKTtcclxuICAgICAgYltqXSA9IHByaW1hcnlQYWlyLnByb3h5Qi51c2VyRGF0YTsgLy8gdGhpcy5tX3RyZWUuR2V0VXNlckRhdGEocHJpbWFyeVBhaXIucHJveHlCKTtcclxuICAgICAgKytqO1xyXG4gICAgICArK2k7XHJcblxyXG4gICAgICAvLyBTa2lwIGFueSBkdXBsaWNhdGUgcGFpcnMuXHJcbiAgICAgIHdoaWxlIChpIDwgY291bnQpIHtcclxuICAgICAgICBjb25zdCBwYWlyOiBiMlBhaXI8VD4gPSB0aGlzLm1fcGFpckJ1ZmZlcltpXTtcclxuICAgICAgICBpZiAoXHJcbiAgICAgICAgICBwYWlyLnByb3h5QS5tX2lkICE9PSBwcmltYXJ5UGFpci5wcm94eUEubV9pZCB8fFxyXG4gICAgICAgICAgcGFpci5wcm94eUIubV9pZCAhPT0gcHJpbWFyeVBhaXIucHJveHlCLm1faWRcclxuICAgICAgICApIHtcclxuICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuICAgICAgICArK2k7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBRdWVyeSBhbiBBQUJCIGZvciBvdmVybGFwcGluZyBwcm94aWVzLiBUaGUgY2FsbGJhY2sgY2xhc3NcclxuICAvLy8gaXMgY2FsbGVkIGZvciBlYWNoIHByb3h5IHRoYXQgb3ZlcmxhcHMgdGhlIHN1cHBsaWVkIEFBQkIuXHJcbiAgUXVlcnkoYWFiYjogYjJBQUJCLCBjYWxsYmFjazogKG5vZGU6IGIyVHJlZU5vZGU8VD4pID0+IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIHRoaXMubV90cmVlLlF1ZXJ5KGFhYmIsIGNhbGxiYWNrKTtcclxuICB9XHJcblxyXG4gIFF1ZXJ5UG9pbnQocG9pbnQ6IFhZLCBjYWxsYmFjazogKG5vZGU6IGIyVHJlZU5vZGU8VD4pID0+IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIHRoaXMubV90cmVlLlF1ZXJ5UG9pbnQocG9pbnQsIGNhbGxiYWNrKTtcclxuICB9XHJcblxyXG4gIC8vLyBSYXktY2FzdCBhZ2FpbnN0IHRoZSBwcm94aWVzIGluIHRoZSB0cmVlLiBUaGlzIHJlbGllcyBvbiB0aGUgY2FsbGJhY2tcclxuICAvLy8gdG8gcGVyZm9ybSBhIGV4YWN0IHJheS1jYXN0IGluIHRoZSBjYXNlIHdlcmUgdGhlIHByb3h5IGNvbnRhaW5zIGEgc2hhcGUuXHJcbiAgLy8vIFRoZSBjYWxsYmFjayBhbHNvIHBlcmZvcm1zIHRoZSBhbnkgY29sbGlzaW9uIGZpbHRlcmluZy4gVGhpcyBoYXMgcGVyZm9ybWFuY2VcclxuICAvLy8gcm91Z2hseSBlcXVhbCB0byBrICogbG9nKG4pLCB3aGVyZSBrIGlzIHRoZSBudW1iZXIgb2YgY29sbGlzaW9ucyBhbmQgbiBpcyB0aGVcclxuICAvLy8gbnVtYmVyIG9mIHByb3hpZXMgaW4gdGhlIHRyZWUuXHJcbiAgLy8vIEBwYXJhbSBpbnB1dCB0aGUgcmF5LWNhc3QgaW5wdXQgZGF0YS4gVGhlIHJheSBleHRlbmRzIGZyb20gcDEgdG8gcDEgKyBtYXhGcmFjdGlvbiAqIChwMiAtIHAxKS5cclxuICAvLy8gQHBhcmFtIGNhbGxiYWNrIGEgY2FsbGJhY2sgY2xhc3MgdGhhdCBpcyBjYWxsZWQgZm9yIGVhY2ggcHJveHkgdGhhdCBpcyBoaXQgYnkgdGhlIHJheS5cclxuICBSYXlDYXN0KFxyXG4gICAgaW5wdXQ6IGIyUmF5Q2FzdElucHV0LFxyXG4gICAgY2FsbGJhY2s6IChpbnB1dDogYjJSYXlDYXN0SW5wdXQsIG5vZGU6IGIyVHJlZU5vZGU8VD4pID0+IG51bWJlcixcclxuICApOiB2b2lkIHtcclxuICAgIHRoaXMubV90cmVlLlJheUNhc3QoaW5wdXQsIGNhbGxiYWNrKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGhlaWdodCBvZiB0aGUgZW1iZWRkZWQgdHJlZS5cclxuICBHZXRUcmVlSGVpZ2h0KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3RyZWUuR2V0SGVpZ2h0KCk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBiYWxhbmNlIG9mIHRoZSBlbWJlZGRlZCB0cmVlLlxyXG4gIEdldFRyZWVCYWxhbmNlKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3RyZWUuR2V0TWF4QmFsYW5jZSgpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgcXVhbGl0eSBtZXRyaWMgb2YgdGhlIGVtYmVkZGVkIHRyZWUuXHJcbiAgR2V0VHJlZVF1YWxpdHkoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fdHJlZS5HZXRBcmVhUmF0aW8oKTtcclxuICB9XHJcblxyXG4gIC8vLyBTaGlmdCB0aGUgd29ybGQgb3JpZ2luLiBVc2VmdWwgZm9yIGxhcmdlIHdvcmxkcy5cclxuICAvLy8gVGhlIHNoaWZ0IGZvcm11bGEgaXM6IHBvc2l0aW9uIC09IG5ld09yaWdpblxyXG4gIC8vLyBAcGFyYW0gbmV3T3JpZ2luIHRoZSBuZXcgb3JpZ2luIHdpdGggcmVzcGVjdCB0byB0aGUgb2xkIG9yaWdpblxyXG4gIFNoaWZ0T3JpZ2luKG5ld09yaWdpbjogWFkpOiB2b2lkIHtcclxuICAgIHRoaXMubV90cmVlLlNoaWZ0T3JpZ2luKG5ld09yaWdpbik7XHJcbiAgfVxyXG5cclxuICBCdWZmZXJNb3ZlKHByb3h5OiBiMlRyZWVOb2RlPFQ+KTogdm9pZCB7XHJcbiAgICB0aGlzLm1fbW92ZUJ1ZmZlclt0aGlzLm1fbW92ZUNvdW50XSA9IHByb3h5O1xyXG4gICAgKyt0aGlzLm1fbW92ZUNvdW50O1xyXG4gIH1cclxuXHJcbiAgVW5CdWZmZXJNb3ZlKHByb3h5OiBiMlRyZWVOb2RlPFQ+KTogdm9pZCB7XHJcbiAgICBjb25zdCBpOiBudW1iZXIgPSB0aGlzLm1fbW92ZUJ1ZmZlci5pbmRleE9mKHByb3h5KTtcclxuICAgIGlmIChpID49IDApIHtcclxuICAgICAgdGhpcy5tX21vdmVCdWZmZXJbaV0gPSBudWxsO1xyXG4gICAgfVxyXG4gIH1cclxufVxyXG5cclxuLy8vIFRoaXMgaXMgdXNlZCB0byBzb3J0IHBhaXJzLlxyXG5mdW5jdGlvbiBiMlBhaXJMZXNzVGhhbjxUPihwYWlyMTogYjJQYWlyPFQ+LCBwYWlyMjogYjJQYWlyPFQ+KTogYm9vbGVhbiB7XHJcbiAgaWYgKHBhaXIxLnByb3h5QS5tX2lkIDwgcGFpcjIucHJveHlBLm1faWQpIHtcclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgaWYgKHBhaXIxLnByb3h5QS5tX2lkID09PSBwYWlyMi5wcm94eUEubV9pZCkge1xyXG4gICAgcmV0dXJuIHBhaXIxLnByb3h5Qi5tX2lkIDwgcGFpcjIucHJveHlCLm1faWQ7XHJcbiAgfVxyXG5cclxuICByZXR1cm4gZmFsc2U7XHJcbn1cclxuIl19