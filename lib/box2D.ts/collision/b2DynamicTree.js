/*
 * Copyright (c) 2009 Erin Catto http://www.box2d.org
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
import { b2_aabbExtension, b2_aabbMultiplier, b2Assert } from '../common/b2Settings';
import { b2Abs, b2AbsInt, b2Max, b2MaxInt, b2Min, b2Vec2 } from '../common/b2Math';
import { b2GrowableStack } from '../common/b2GrowableStack';
import { b2AABB, b2RayCastInput, b2TestOverlapAABB } from './b2Collision';
function verify(value) {
    if (value === null) {
        throw new Error();
    }
    return value;
}
/// A node in the dynamic tree. The client does not interact with this directly.
export class b2TreeNode {
    constructor(m_id) {
        this.m_id = m_id;
        this.aabb = new b2AABB();
        this.parent = null; // or next
        this.child1 = null;
        this.child2 = null;
        this.height = 0; // leaf = 0, free node = -1
        this._userData = null;
    }
    Reset() {
        this._userData = null;
    }
    IsLeaf() {
        return this.child1 === null;
    }
    get userData() {
        if (this._userData === null) {
            throw new Error();
        }
        return this._userData;
    }
    set userData(value) {
        if (this._userData !== null) {
            throw new Error();
        }
        this._userData = value;
    }
}
export class b2DynamicTree {
    constructor() {
        this.m_root = null;
        // b2TreeNode* m_nodes;
        // int32 m_nodeCount;
        // int32 m_nodeCapacity;
        this.m_freeList = null;
        this.m_insertionCount = 0;
        this.m_stack = new b2GrowableStack(256);
    }
    // GetUserData(node: b2TreeNode<T>): T {
    //   !!B2_DEBUG && b2Assert(node !== null);
    //   return node.userData;
    // }
    // GetFatAABB(node: b2TreeNode<T>): b2AABB {
    //   !!B2_DEBUG && b2Assert(node !== null);
    //   return node.aabb;
    // }
    Query(aabb, callback) {
        const stack = this.m_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            const node = stack.Pop();
            if (node === null) {
                continue;
            }
            if (node.aabb.TestOverlap(aabb)) {
                if (node.IsLeaf()) {
                    const proceed = callback(node);
                    if (!proceed) {
                        return;
                    }
                }
                else {
                    stack.Push(node.child1);
                    stack.Push(node.child2);
                }
            }
        }
    }
    Query_(aabb, out) {
        const stack = this.m_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            const node = stack.Pop();
            if (node === null) {
                continue;
            }
            if (node.aabb.TestOverlap(aabb)) {
                if (node.IsLeaf()) {
                    out.push(node);
                }
                else {
                    stack.Push(node.child1);
                    stack.Push(node.child2);
                }
            }
        }
    }
    QueryPoint(point, callback) {
        const stack = this.m_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            const node = stack.Pop();
            if (node === null) {
                continue;
            }
            if (node.aabb.TestContain(point)) {
                if (node.IsLeaf()) {
                    const proceed = callback(node);
                    if (!proceed) {
                        return;
                    }
                }
                else {
                    stack.Push(node.child1);
                    stack.Push(node.child2);
                }
            }
        }
    }
    RayCast(input, callback) {
        const p1 = input.p1;
        const p2 = input.p2;
        const r = b2Vec2.SubVV(p2, p1, b2DynamicTree.s_r);
        !!B2_DEBUG && b2Assert(r.LengthSquared() > 0);
        r.Normalize();
        // v is perpendicular to the segment.
        const v = b2Vec2.CrossOneV(r, b2DynamicTree.s_v);
        const abs_v = b2Vec2.AbsV(v, b2DynamicTree.s_abs_v);
        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        let maxFraction = input.maxFraction;
        // Build a bounding box for the segment.
        const segmentAABB = b2DynamicTree.s_segmentAABB;
        let t_x = p1.x + maxFraction * (p2.x - p1.x);
        let t_y = p1.y + maxFraction * (p2.y - p1.y);
        segmentAABB.lowerBound.x = b2Min(p1.x, t_x);
        segmentAABB.lowerBound.y = b2Min(p1.y, t_y);
        segmentAABB.upperBound.x = b2Max(p1.x, t_x);
        segmentAABB.upperBound.y = b2Max(p1.y, t_y);
        const stack = this.m_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            const node = stack.Pop();
            if (node === null) {
                continue;
            }
            if (!b2TestOverlapAABB(node.aabb, segmentAABB)) {
                continue;
            }
            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            const c = node.aabb.GetCenter();
            const h = node.aabb.GetExtents();
            const separation = b2Abs(b2Vec2.DotVV(v, b2Vec2.SubVV(p1, c, b2Vec2.s_t0))) - b2Vec2.DotVV(abs_v, h);
            if (separation > 0) {
                continue;
            }
            if (node.IsLeaf()) {
                const subInput = b2DynamicTree.s_subInput;
                subInput.p1.Copy(input.p1);
                subInput.p2.Copy(input.p2);
                subInput.maxFraction = maxFraction;
                const value = callback(subInput, node);
                if (value === 0) {
                    // The client has terminated the ray cast.
                    return;
                }
                if (value > 0) {
                    // Update segment bounding box.
                    maxFraction = value;
                    t_x = p1.x + maxFraction * (p2.x - p1.x);
                    t_y = p1.y + maxFraction * (p2.y - p1.y);
                    segmentAABB.lowerBound.x = b2Min(p1.x, t_x);
                    segmentAABB.lowerBound.y = b2Min(p1.y, t_y);
                    segmentAABB.upperBound.x = b2Max(p1.x, t_x);
                    segmentAABB.upperBound.y = b2Max(p1.y, t_y);
                }
            }
            else {
                stack.Push(node.child1);
                stack.Push(node.child2);
            }
        }
    }
    AllocateNode() {
        // Expand the node pool as needed.
        if (this.m_freeList !== null) {
            const node = this.m_freeList;
            this.m_freeList = node.parent; // this.m_freeList = node.next;
            node.parent = null;
            node.child1 = null;
            node.child2 = null;
            node.height = 0;
            return node;
        }
        return new b2TreeNode(b2DynamicTree.s_node_id++);
    }
    FreeNode(node) {
        node.parent = this.m_freeList; // node.next = this.m_freeList;
        node.child1 = null;
        node.child2 = null;
        node.height = -1;
        node.Reset();
        this.m_freeList = node;
    }
    CreateProxy(aabb, userData) {
        const node = this.AllocateNode();
        // Fatten the aabb.
        const r_x = b2_aabbExtension;
        const r_y = b2_aabbExtension;
        node.aabb.lowerBound.x = aabb.lowerBound.x - r_x;
        node.aabb.lowerBound.y = aabb.lowerBound.y - r_y;
        node.aabb.upperBound.x = aabb.upperBound.x + r_x;
        node.aabb.upperBound.y = aabb.upperBound.y + r_y;
        node.userData = userData;
        node.height = 0;
        this.InsertLeaf(node);
        return node;
    }
    DestroyProxy(node) {
        !!B2_DEBUG && b2Assert(node.IsLeaf());
        this.RemoveLeaf(node);
        this.FreeNode(node);
    }
    MoveProxy(node, aabb, displacement) {
        !!B2_DEBUG && b2Assert(node.IsLeaf());
        if (node.aabb.Contains(aabb)) {
            return false;
        }
        this.RemoveLeaf(node);
        // Extend AABB.
        const r_x = b2_aabbExtension;
        const r_y = b2_aabbExtension;
        node.aabb.lowerBound.x = aabb.lowerBound.x - r_x;
        node.aabb.lowerBound.y = aabb.lowerBound.y - r_y;
        node.aabb.upperBound.x = aabb.upperBound.x + r_x;
        node.aabb.upperBound.y = aabb.upperBound.y + r_y;
        // Predict AABB displacement.
        const d_x = b2_aabbMultiplier * displacement.x;
        const d_y = b2_aabbMultiplier * displacement.y;
        if (d_x < 0.0) {
            node.aabb.lowerBound.x += d_x;
        }
        else {
            node.aabb.upperBound.x += d_x;
        }
        if (d_y < 0.0) {
            node.aabb.lowerBound.y += d_y;
        }
        else {
            node.aabb.upperBound.y += d_y;
        }
        this.InsertLeaf(node);
        return true;
    }
    InsertLeaf(leaf) {
        ++this.m_insertionCount;
        if (this.m_root === null) {
            this.m_root = leaf;
            this.m_root.parent = null;
            return;
        }
        // Find the best sibling for this node
        const leafAABB = leaf.aabb;
        let sibling = this.m_root;
        while (!sibling.IsLeaf()) {
            const child1 = verify(sibling.child1);
            const child2 = verify(sibling.child2);
            const area = sibling.aabb.GetPerimeter();
            const combinedAABB = b2DynamicTree.s_combinedAABB;
            combinedAABB.Combine2(sibling.aabb, leafAABB);
            const combinedArea = combinedAABB.GetPerimeter();
            // Cost of creating a new parent for this node and the new leaf
            const cost = 2 * combinedArea;
            // Minimum cost of pushing the leaf further down the tree
            const inheritanceCost = 2 * (combinedArea - area);
            // Cost of descending into child1
            let cost1;
            const aabb = b2DynamicTree.s_aabb;
            let oldArea;
            let newArea;
            if (child1.IsLeaf()) {
                aabb.Combine2(leafAABB, child1.aabb);
                cost1 = aabb.GetPerimeter() + inheritanceCost;
            }
            else {
                aabb.Combine2(leafAABB, child1.aabb);
                oldArea = child1.aabb.GetPerimeter();
                newArea = aabb.GetPerimeter();
                cost1 = newArea - oldArea + inheritanceCost;
            }
            // Cost of descending into child2
            let cost2;
            if (child2.IsLeaf()) {
                aabb.Combine2(leafAABB, child2.aabb);
                cost2 = aabb.GetPerimeter() + inheritanceCost;
            }
            else {
                aabb.Combine2(leafAABB, child2.aabb);
                oldArea = child2.aabb.GetPerimeter();
                newArea = aabb.GetPerimeter();
                cost2 = newArea - oldArea + inheritanceCost;
            }
            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2) {
                break;
            }
            // Descend
            if (cost1 < cost2) {
                sibling = child1;
            }
            else {
                sibling = child2;
            }
        }
        // Create a parent for the siblings.
        const oldParent = sibling.parent;
        const newParent = this.AllocateNode();
        newParent.parent = oldParent;
        newParent.aabb.Combine2(leafAABB, sibling.aabb);
        newParent.height = sibling.height + 1;
        if (oldParent !== null) {
            // The sibling was not the root.
            if (oldParent.child1 === sibling) {
                oldParent.child1 = newParent;
            }
            else {
                oldParent.child2 = newParent;
            }
            newParent.child1 = sibling;
            newParent.child2 = leaf;
            sibling.parent = newParent;
            leaf.parent = newParent;
        }
        else {
            // The sibling was the root.
            newParent.child1 = sibling;
            newParent.child2 = leaf;
            sibling.parent = newParent;
            leaf.parent = newParent;
            this.m_root = newParent;
        }
        // Walk back up the tree fixing heights and AABBs
        let node = leaf.parent;
        while (node !== null) {
            const n = this.Balance(node);
            const child1 = verify(n.child1);
            const child2 = verify(n.child2);
            n.height = 1 + b2MaxInt(child1.height, child2.height);
            n.aabb.Combine2(child1.aabb, child2.aabb);
            node = n.parent;
        }
        // this.Validate();
    }
    RemoveLeaf(leaf) {
        if (leaf === this.m_root) {
            this.m_root = null;
            return;
        }
        const parent = verify(leaf.parent);
        const grandParent = parent && parent.parent;
        const sibling = verify(parent.child1 === leaf ? parent.child2 : parent.child1);
        if (grandParent !== null) {
            // Destroy parent and connect sibling to grandParent.
            if (grandParent.child1 === parent) {
                grandParent.child1 = sibling;
            }
            else {
                grandParent.child2 = sibling;
            }
            sibling.parent = grandParent;
            this.FreeNode(parent);
            // Adjust ancestor bounds.
            let index = grandParent;
            while (index !== null) {
                index = this.Balance(index);
                const child1 = verify(index.child1);
                const child2 = verify(index.child2);
                index.aabb.Combine2(child1.aabb, child2.aabb);
                index.height = 1 + b2MaxInt(child1.height, child2.height);
                index = index.parent;
            }
        }
        else {
            this.m_root = sibling;
            sibling.parent = null;
            this.FreeNode(parent);
        }
        // this.Validate();
    }
    Balance(A) {
        !!B2_DEBUG && b2Assert(A !== null);
        if (A.IsLeaf() || A.height < 2) {
            return A;
        }
        const B = verify(A.child1);
        const C = verify(A.child2);
        const balance = C.height - B.height;
        // Rotate C up
        if (balance > 1) {
            return this.Rotate_C_Up(A, B, C);
        }
        // Rotate B up
        else if (balance < -1) {
            return this.Rotate_B_Up(A, B, C);
        }
        return A;
    }
    Rotate_C_Up(A, B, C) {
        const F = verify(C.child1);
        const G = verify(C.child2);
        // Swap A and C
        C.child1 = A;
        C.parent = A.parent;
        A.parent = C;
        // A's old parent should point to C
        if (C.parent !== null) {
            if (C.parent.child1 === A) {
                C.parent.child1 = C;
            }
            else {
                !!B2_DEBUG && b2Assert(C.parent.child2 === A);
                C.parent.child2 = C;
            }
        }
        else {
            this.m_root = C;
        }
        // Rotate
        if (F.height > G.height) {
            C.child2 = F;
            A.child2 = G;
            G.parent = A;
            A.aabb.Combine2(B.aabb, G.aabb);
            C.aabb.Combine2(A.aabb, F.aabb);
            A.height = 1 + b2MaxInt(B.height, G.height);
            C.height = 1 + b2MaxInt(A.height, F.height);
        }
        else {
            C.child2 = G;
            A.child2 = F;
            F.parent = A;
            A.aabb.Combine2(B.aabb, F.aabb);
            C.aabb.Combine2(A.aabb, G.aabb);
            A.height = 1 + b2MaxInt(B.height, F.height);
            C.height = 1 + b2MaxInt(A.height, G.height);
        }
        return C;
    }
    Rotate_B_Up(A, B, C) {
        const D = verify(B.child1);
        const E = verify(B.child2);
        // Swap A and B
        B.child1 = A;
        B.parent = A.parent;
        A.parent = B;
        // A's old parent should point to B
        if (B.parent !== null) {
            if (B.parent.child1 === A) {
                B.parent.child1 = B;
            }
            else {
                !!B2_DEBUG && b2Assert(B.parent.child2 === A);
                B.parent.child2 = B;
            }
        }
        else {
            this.m_root = B;
        }
        // Rotate
        if (D.height > E.height) {
            B.child2 = D;
            A.child1 = E;
            E.parent = A;
            A.aabb.Combine2(C.aabb, E.aabb);
            B.aabb.Combine2(A.aabb, D.aabb);
            A.height = 1 + b2MaxInt(C.height, E.height);
            B.height = 1 + b2MaxInt(A.height, D.height);
        }
        else {
            B.child2 = E;
            A.child1 = D;
            D.parent = A;
            A.aabb.Combine2(C.aabb, D.aabb);
            B.aabb.Combine2(A.aabb, E.aabb);
            A.height = 1 + b2MaxInt(C.height, D.height);
            B.height = 1 + b2MaxInt(A.height, E.height);
        }
        return B;
    }
    GetHeight() {
        if (this.m_root === null) {
            return 0;
        }
        return this.m_root.height;
    }
    static GetAreaNode(node) {
        if (node === null) {
            return 0;
        }
        if (node.IsLeaf()) {
            return 0;
        }
        let area = node.aabb.GetPerimeter();
        area += b2DynamicTree.GetAreaNode(node.child1);
        area += b2DynamicTree.GetAreaNode(node.child2);
        return area;
    }
    GetAreaRatio() {
        if (this.m_root === null) {
            return 0;
        }
        const root = this.m_root;
        const rootArea = root.aabb.GetPerimeter();
        const totalArea = b2DynamicTree.GetAreaNode(this.m_root);
        /*
            float32 totalArea = 0.0;
            for (int32 i = 0; i < m_nodeCapacity; ++i) {
              const b2TreeNode<T>* node = m_nodes + i;
              if (node.height < 0) {
                // Free node in pool
                continue;
              }
    
              totalArea += node.aabb.GetPerimeter();
            }
            */
        return totalArea / rootArea;
    }
    static ComputeHeightNode(node) {
        if (node === null || node.IsLeaf()) {
            return 0;
        }
        const height1 = b2DynamicTree.ComputeHeightNode(node.child1);
        const height2 = b2DynamicTree.ComputeHeightNode(node.child2);
        return 1 + b2MaxInt(height1, height2);
    }
    ComputeHeight() {
        const height = b2DynamicTree.ComputeHeightNode(this.m_root);
        return height;
    }
    ValidateStructure(node) {
        if (node === null) {
            return;
        }
        if (node === this.m_root) {
            !!B2_DEBUG && b2Assert(node.parent === null);
        }
        if (node.IsLeaf()) {
            if (B2_DEBUG) {
                b2Assert(node.child1 === null);
                b2Assert(node.child2 === null);
                b2Assert(node.height === 0);
            }
            return;
        }
        const child1 = verify(node.child1);
        const child2 = verify(node.child2);
        if (B2_DEBUG) {
            // TODO: index
            // b2Assert(child1.parent === index);
            // b2Assert(child2.parent === index);
        }
        this.ValidateStructure(child1);
        this.ValidateStructure(child2);
    }
    ValidateMetrics(node) {
        if (node === null) {
            return;
        }
        if (node.IsLeaf()) {
            if (B2_DEBUG) {
                b2Assert(node.child1 === null);
                b2Assert(node.child2 === null);
                b2Assert(node.height === 0);
            }
            return;
        }
        const child1 = verify(node.child1);
        const child2 = verify(node.child2);
        if (B2_DEBUG) {
            const height1 = child1.height;
            const height2 = child2.height;
            const height = 1 + b2MaxInt(height1, height2);
            b2Assert(node.height === height);
        }
        const aabb = b2DynamicTree.s_aabb;
        aabb.Combine2(child1.aabb, child2.aabb);
        if (B2_DEBUG) {
            b2Assert(aabb.lowerBound === node.aabb.lowerBound);
            b2Assert(aabb.upperBound === node.aabb.upperBound);
        }
        this.ValidateMetrics(child1);
        this.ValidateMetrics(child2);
    }
    Validate() {
        !!B2_DEBUG && this.ValidateStructure(this.m_root);
        !!B2_DEBUG && this.ValidateMetrics(this.m_root);
        // let freeCount: number = 0;
        // let freeIndex: b2TreeNode<T> | null = this.m_freeList;
        // while (freeIndex !== null) {
        //   freeIndex = freeIndex.parent; // freeIndex = freeIndex.next;
        //   ++freeCount;
        // }
        !!B2_DEBUG && b2Assert(this.GetHeight() === this.ComputeHeight());
        // b2Assert(this.m_nodeCount + freeCount === this.m_nodeCapacity);
    }
    static GetMaxBalanceNode(node, maxBalance) {
        if (node === null) {
            return maxBalance;
        }
        if (node.height <= 1) {
            return maxBalance;
        }
        !!B2_DEBUG && b2Assert(!node.IsLeaf());
        const child1 = verify(node.child1);
        const child2 = verify(node.child2);
        const balance = b2AbsInt(child2.height - child1.height);
        return b2MaxInt(maxBalance, balance);
    }
    GetMaxBalance() {
        const maxBalance = b2DynamicTree.GetMaxBalanceNode(this.m_root, 0);
        /*
            int32 maxBalance = 0;
            for (int32 i = 0; i < m_nodeCapacity; ++i) {
              const b2TreeNode<T>* node = m_nodes + i;
              if (node.height <= 1) {
                continue;
              }
    
              b2Assert(!node.IsLeaf());
    
              int32 child1 = node.child1;
              int32 child2 = node.child2;
              int32 balance = b2Abs(m_nodes[child2].height - m_nodes[child1].height);
              maxBalance = b2Max(maxBalance, balance);
            }
            */
        return maxBalance;
    }
    RebuildBottomUp() {
        /*
            int32* nodes = (int32*)b2Alloc(m_nodeCount * sizeof(int32));
            int32 count = 0;
    
            // Build array of leaves. Free the rest.
            for (int32 i = 0; i < m_nodeCapacity; ++i) {
              if (m_nodes[i].height < 0) {
                // free node in pool
                continue;
              }
    
              if (m_nodes[i].IsLeaf()) {
                m_nodes[i].parent = b2_nullNode;
                nodes[count] = i;
                ++count;
              } else {
                FreeNode(i);
              }
            }
    
            while (count > 1) {
              float32 minCost = b2_maxFloat;
              int32 iMin = -1, jMin = -1;
              for (int32 i = 0; i < count; ++i) {
                b2AABB aabbi = m_nodes[nodes[i]].aabb;
    
                for (int32 j = i + 1; j < count; ++j) {
                  b2AABB aabbj = m_nodes[nodes[j]].aabb;
                  b2AABB b;
                  b.Combine(aabbi, aabbj);
                  float32 cost = b.GetPerimeter();
                  if (cost < minCost) {
                    iMin = i;
                    jMin = j;
                    minCost = cost;
                  }
                }
              }
    
              int32 index1 = nodes[iMin];
              int32 index2 = nodes[jMin];
              b2TreeNode<T>* child1 = m_nodes + index1;
              b2TreeNode<T>* child2 = m_nodes + index2;
    
              int32 parentIndex = AllocateNode();
              b2TreeNode<T>* parent = m_nodes + parentIndex;
              parent.child1 = index1;
              parent.child2 = index2;
              parent.height = 1 + b2Max(child1.height, child2.height);
              parent.aabb.Combine(child1.aabb, child2.aabb);
              parent.parent = b2_nullNode;
    
              child1.parent = parentIndex;
              child2.parent = parentIndex;
    
              nodes[jMin] = nodes[count-1];
              nodes[iMin] = parentIndex;
              --count;
            }
    
            m_root = nodes[0];
            b2Free(nodes);
            */
        this.Validate();
    }
    static ShiftOriginNode(node, newOrigin) {
        if (node === null) {
            return;
        }
        if (node.height <= 1) {
            return;
        }
        !!B2_DEBUG && b2Assert(!node.IsLeaf());
        const child1 = node.child1;
        const child2 = node.child2;
        b2DynamicTree.ShiftOriginNode(child1, newOrigin);
        b2DynamicTree.ShiftOriginNode(child2, newOrigin);
        node.aabb.lowerBound.SelfSub(newOrigin);
        node.aabb.upperBound.SelfSub(newOrigin);
    }
    ShiftOrigin(newOrigin) {
        b2DynamicTree.ShiftOriginNode(this.m_root, newOrigin);
        /*
            // Build array of leaves. Free the rest.
            for (int32 i = 0; i < m_nodeCapacity; ++i) {
              m_nodes[i].aabb.lowerBound -= newOrigin;
              m_nodes[i].aabb.upperBound -= newOrigin;
            }
            */
    }
}
b2DynamicTree.s_r = new b2Vec2();
b2DynamicTree.s_v = new b2Vec2();
b2DynamicTree.s_abs_v = new b2Vec2();
b2DynamicTree.s_segmentAABB = new b2AABB();
b2DynamicTree.s_subInput = new b2RayCastInput();
b2DynamicTree.s_combinedAABB = new b2AABB();
b2DynamicTree.s_aabb = new b2AABB();
b2DynamicTree.s_node_id = 0;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJEeW5hbWljVHJlZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb2xsaXNpb24vYjJEeW5hbWljVHJlZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFBRSxnQkFBZ0IsRUFBRSxpQkFBaUIsRUFBRSxRQUFRLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUNyRixPQUFPLEVBQUUsS0FBSyxFQUFFLFFBQVEsRUFBRSxLQUFLLEVBQUUsUUFBUSxFQUFFLEtBQUssRUFBRSxNQUFNLEVBQU0sTUFBTSxrQkFBa0IsQ0FBQztBQUN2RixPQUFPLEVBQUUsZUFBZSxFQUFFLE1BQU0sMkJBQTJCLENBQUM7QUFDNUQsT0FBTyxFQUFFLE1BQU0sRUFBRSxjQUFjLEVBQUUsaUJBQWlCLEVBQUUsTUFBTSxlQUFlLENBQUM7QUFFMUUsU0FBUyxNQUFNLENBQUksS0FBZTtJQUNoQyxJQUFJLEtBQUssS0FBSyxJQUFJLEVBQUU7UUFDbEIsTUFBTSxJQUFJLEtBQUssRUFBRSxDQUFDO0tBQ25CO0lBQ0QsT0FBTyxLQUFNLENBQUM7QUFDaEIsQ0FBQztBQUVELGdGQUFnRjtBQUNoRixNQUFNLE9BQU8sVUFBVTtJQVNyQixZQUFxQixJQUFZO1FBQVosU0FBSSxHQUFKLElBQUksQ0FBUTtRQVJ4QixTQUFJLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUM3QixXQUFNLEdBQXlCLElBQUksQ0FBQyxDQUFDLFVBQVU7UUFDL0MsV0FBTSxHQUF5QixJQUFJLENBQUM7UUFDcEMsV0FBTSxHQUF5QixJQUFJLENBQUM7UUFDcEMsV0FBTSxHQUFHLENBQUMsQ0FBQyxDQUFDLDJCQUEyQjtRQUUvQixjQUFTLEdBQWEsSUFBSSxDQUFDO0lBRUMsQ0FBQztJQUVyQyxLQUFLO1FBQ0gsSUFBSSxDQUFDLFNBQVMsR0FBRyxJQUFJLENBQUM7SUFDeEIsQ0FBQztJQUVELE1BQU07UUFDSixPQUFP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O1NBQ0Y7YUFBTTtZQUNMLElBQUksQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO1NBQ2pCO1FBRUQsU0FBUztRQUNULElBQUksQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUMsTUFBTSxFQUFFO1lBQ3ZCLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO1lBQ2IsQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7WUFDYixDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztZQUNiLENBQUMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxJQUFJLEVBQUUsQ0FBQyxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQ2hDLENBQUMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxJQUFJLEVBQUUsQ0FBQyxDQUFDLElBQUksQ0FBQyxDQUFDO1lBRWhDLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxHQUFHLFFBQVEsQ0FBQyxDQUFDLENBQUMsTUFBTSxFQUFFLENBQUMsQ0FBQyxNQUFNLENBQUMsQ0FBQztZQUM1QyxDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsR0FBRyxRQUFRLENBQUMsQ0FBQyxDQUFDLE1BQU0sRUFBRSxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUM7U0FDN0M7YUFBTTtZQUNMLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO1lBQ2IsQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7WUFDYixDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztZQUNiLENBQUMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxJQUFJLEVBQUUsQ0FBQyxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQ2hDLENBQUMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxJQUFJLEVBQUUsQ0FBQyxDQUFDLElBQUksQ0FBQyxDQUFDO1lBRWhDLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxHQUFHLFFBQVEsQ0FBQyxDQUFDLENBQUMsTUFBTSxFQUFFLENBQUMsQ0FBQyxNQUFNLENBQUMsQ0FBQztZQUM1QyxDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsR0FBRyxRQUFRLENBQUMsQ0FBQyxDQUFDLE1BQU0sRUFBRSxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUM7U0FDN0M7UUFFRCxPQUFPLENBQUMsQ0FBQztJQUNYLENBQUM7SUFFRCxXQUFXLENBQUMsQ0FBZ0IsRUFBRSxDQUFnQixFQUFFLENBQWdCO1FBQzlELE1BQU0sQ0FBQyxHQUFrQixNQUFNLENBQUMsQ0FBQyxDQUFDLE1BQU0sQ0FBQyxDQUFDO1FBQzFDLE1BQU0sQ0FBQyxHQUFrQixNQUFNLENBQUMsQ0FBQyxDQUFDLE1BQU0sQ0FBQyxDQUFDO1FBRTFDLGVBQWU7UUFDZixDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztRQUNiLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDLE1BQU0sQ0FBQztRQUNwQixDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztRQUViLG1DQUFtQztRQUNuQyxJQUFJLENBQUMsQ0FBQyxNQUFNLEtBQUssSUFBSSxFQUFFO1lBQ3JCLElBQUksQ0FBQyxDQUFDLE1BQU0sQ0FBQyxNQUFNLEtBQUssQ0FBQyxFQUFFO2dCQUN6QixDQUFDLENBQUMsTUFBTSxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7YUFDckI7aUJBQU07Z0JBQ0wsQ0FBQyxDQUFDLFFBQVEsSUFBSSxRQUFRLENBQUMsQ0FBQyxDQUFDLE1BQU0sQ0FBQyxNQUFNLEtBQUssQ0FBQyxDQUFDLENBQUM7Z0JBQzlDLENBQUMsQ0FBQyxNQUFNLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQzthQUNyQjtTQUNGO2FBQU07WUFDTCxJQUFJLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztTQUNqQjtRQUVELFNBQVM7UUFDVCxJQUFJLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDLE1BQU0sRUFBRTtZQUN2QixDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztZQUNiLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO1lBQ2IsQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7WUFDYixDQUFDLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUNoQyxDQUFDLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUVoQyxDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsR0FBRyxRQUFRLENBQUMsQ0FBQyxDQUFDLE1BQU0sRUFBRSxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUM7WUFDNUMsQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLEdBQUcsUUFBUSxDQUFDLENBQUMsQ0FBQyxNQUFNLEVBQUUsQ0FBQyxDQUFDLE1BQU0sQ0FBQyxDQUFDO1NBQzdDO2FBQU07WUFDTCxDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztZQUNiLENBQUMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO1lBQ2IsQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7WUFDYixDQUFDLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUNoQyxDQUFDLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxJQUFJLENBQUMsQ0FBQztZQUVoQyxDQUFDLENBQUMsTUFBTSxHQUFHLENBQUMsR0FBRyxRQUFRLENBQUMsQ0FBQyxDQUFDLE1BQU0sRUFBRSxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUM7WUFDNUMsQ0FBQyxDQUFDLE1BQU0sR0FBRyxDQUFDLEdBQUcsUUFBUSxDQUFDLENBQUMsQ0FBQyxNQUFNLEVBQUUsQ0FBQyxDQUFDLE1BQU0sQ0FBQyxDQUFDO1NBQzdDO1FBRUQsT0FBTyxDQUFDLENBQUM7SUFDWCxDQUFDO0lBRUQsU0FB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QUFDLFNBQVMsQ0FBQyxDQUFDO0lBQzFDLENBQUM7SUFFRCxXQUFXLENBQUMsU0FBYTtRQUN2QixhQUFhLENBQUMsZUFBZSxDQUFDLElBQUksQ0FBQyxNQUFNLEVBQUUsU0FBUyxDQUFDLENBQUM7UUFFdEQ7Ozs7OztjQU1NO0lBQ1IsQ0FBQzs7QUEzeUJlLGlCQUFHLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNuQixpQkFBRyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDbkIscUJBQU8sR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ3ZCLDJCQUFhLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUM3Qix3QkFBVSxHQUFHLElBQUksY0FBYyxFQUFFLENBQUM7QUFDbEMsNEJBQWMsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQzlCLG9CQUFNLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQWtLL0IsdUJBQVMsR0FBRyxDQUFDLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMDkgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQgeyBiMl9hYWJiRXh0ZW5zaW9uLCBiMl9hYWJiTXVsdGlwbGllciwgYjJBc3NlcnQgfSBmcm9tICcuLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7IGIyQWJzLCBiMkFic0ludCwgYjJNYXgsIGIyTWF4SW50LCBiMk1pbiwgYjJWZWMyLCBYWSB9IGZyb20gJy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMkdyb3dhYmxlU3RhY2sgfSBmcm9tICcuLi9jb21tb24vYjJHcm93YWJsZVN0YWNrJztcclxuaW1wb3J0IHsgYjJBQUJCLCBiMlJheUNhc3RJbnB1dCwgYjJUZXN0T3ZlcmxhcEFBQkIgfSBmcm9tICcuL2IyQ29sbGlzaW9uJztcclxuXHJcbmZ1bmN0aW9uIHZlcmlmeTxUPih2YWx1ZTogVCB8IG51bGwpOiBUIHtcclxuICBpZiAodmFsdWUgPT09IG51bGwpIHtcclxuICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gIH1cclxuICByZXR1cm4gdmFsdWUhO1xyXG59XHJcblxyXG4vLy8gQSBub2RlIGluIHRoZSBkeW5hbWljIHRyZWUuIFRoZSBjbGllbnQgZG9lcyBub3QgaW50ZXJhY3Qgd2l0aCB0aGlzIGRpcmVjdGx5LlxyXG5leHBvcnQgY2xhc3MgYjJUcmVlTm9kZTxUPiB7XHJcbiAgcmVhZG9ubHkgYWFiYiA9IG5ldyBiMkFBQkIoKTtcclxuICBwYXJlbnQ6IGIyVHJlZU5vZGU8VD4gfCBudWxsID0gbnVsbDsgLy8gb3IgbmV4dFxyXG4gIGNoaWxkMTogYjJUcmVlTm9kZTxUPiB8IG51bGwgPSBudWxsO1xyXG4gIGNoaWxkMjogYjJUcmVlTm9kZTxUPiB8IG51bGwgPSBudWxsO1xyXG4gIGhlaWdodCA9IDA7IC8vIGxlYWYgPSAwLCBmcmVlIG5vZGUgPSAtMVxyXG5cclxuICBwcml2YXRlIF91c2VyRGF0YTogVCB8IG51bGwgPSBudWxsO1xyXG5cclxuICBjb25zdHJ1Y3RvcihyZWFkb25seSBtX2lkOiBudW1iZXIpIHt9XHJcblxyXG4gIFJlc2V0KCk6IHZvaWQge1xyXG4gICAgdGhpcy5fdXNlckRhdGEgPSBudWxsO1xyXG4gIH1cclxuXHJcbiAgSXNMZWFmKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMuY2hpbGQxID09PSBudWxsO1xyXG4gIH1cclxuXHJcbiAgZ2V0IHVzZXJEYXRhKCk6IFQge1xyXG4gICAgaWYgKHRoaXMuX3VzZXJEYXRhID09PSBudWxsKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRoaXMuX3VzZXJEYXRhO1xyXG4gIH1cclxuXHJcbiAgc2V0IHVzZXJEYXRhKHZhbHVlOiBUKSB7XHJcbiAgICBpZiAodGhpcy5fdXNlckRhdGEgIT09IG51bGwpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcbiAgICB0aGlzLl91c2VyRGF0YSA9IHZhbHVlO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyRHluYW1pY1RyZWU8VD4ge1xyXG4gIG1fcm9vdDogYjJUcmVlTm9kZTxUPiB8IG51bGwgPSBudWxsO1xyXG5cclxuICAvLyBiMlRyZWVOb2RlKiBtX25vZGVzO1xyXG4gIC8vIGludDMyIG1fbm9kZUNvdW50O1xyXG4gIC8vIGludDMyIG1fbm9kZUNhcGFjaXR5O1xyXG5cclxuICBtX2ZyZWVMaXN0OiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IG51bGw7XHJcblxyXG4gIG1faW5zZXJ0aW9uQ291bnQgPSAwO1xyXG5cclxuICByZWFkb25seSBtX3N0YWNrID0gbmV3IGIyR3Jvd2FibGVTdGFjazxiMlRyZWVOb2RlPFQ+IHwgbnVsbD4oMjU2KTtcclxuICBzdGF0aWMgcmVhZG9ubHkgc19yID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBzX3YgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHNfYWJzX3YgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHNfc2VnbWVudEFBQkIgPSBuZXcgYjJBQUJCKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHNfc3ViSW5wdXQgPSBuZXcgYjJSYXlDYXN0SW5wdXQoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgc19jb21iaW5lZEFBQkIgPSBuZXcgYjJBQUJCKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHNfYWFiYiA9IG5ldyBiMkFBQkIoKTtcclxuXHJcbiAgLy8gR2V0VXNlckRhdGEobm9kZTogYjJUcmVlTm9kZTxUPik6IFQge1xyXG4gIC8vICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChub2RlICE9PSBudWxsKTtcclxuICAvLyAgIHJldHVybiBub2RlLnVzZXJEYXRhO1xyXG4gIC8vIH1cclxuXHJcbiAgLy8gR2V0RmF0QUFCQihub2RlOiBiMlRyZWVOb2RlPFQ+KTogYjJBQUJCIHtcclxuICAvLyAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQobm9kZSAhPT0gbnVsbCk7XHJcbiAgLy8gICByZXR1cm4gbm9kZS5hYWJiO1xyXG4gIC8vIH1cclxuXHJcbiAgUXVlcnkoYWFiYjogYjJBQUJCLCBjYWxsYmFjazogKG5vZGU6IGIyVHJlZU5vZGU8VD4pID0+IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGNvbnN0IHN0YWNrOiBiMkdyb3dhYmxlU3RhY2s8YjJUcmVlTm9kZTxUPiB8IG51bGw+ID0gdGhpcy5tX3N0YWNrLlJlc2V0KCk7XHJcbiAgICBzdGFjay5QdXNoKHRoaXMubV9yb290KTtcclxuXHJcbiAgICB3aGlsZSAoc3RhY2suR2V0Q291bnQoKSA+IDApIHtcclxuICAgICAgY29uc3Qgbm9kZTogYjJUcmVlTm9kZTxUPiB8IG51bGwgPSBzdGFjay5Qb3AoKTtcclxuICAgICAgaWYgKG5vZGUgPT09IG51bGwpIHtcclxuICAgICAgICBjb250aW51ZTtcclxuICAgICAgfVxyXG5cclxuICAgICAgaWYgKG5vZGUuYWFiYi5UZXN0T3ZlcmxhcChhYWJiKSkge1xyXG4gICAgICAgIGlmIChub2RlLklzTGVhZigpKSB7XHJcbiAgICAgICAgICBjb25zdCBwcm9jZWVkID0gY2FsbGJhY2sobm9kZSk7XHJcbiAgICAgICAgICBpZiAoIXByb2NlZWQpIHtcclxuICAgICAgICAgICAgcmV0dXJuO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICBzdGFjay5QdXNoKG5vZGUuY2hpbGQxKTtcclxuICAgICAgICAgIHN0YWNrLlB1c2gobm9kZS5jaGlsZDIpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgcHVibGljIFF1ZXJ5XyhhYWJiOiBiMkFBQkIsIG91dDogYjJUcmVlTm9kZTxUPltdKTogdm9pZCB7XHJcbiAgICBjb25zdCBzdGFjazogYjJHcm93YWJsZVN0YWNrPGIyVHJlZU5vZGU8VD4gfCBudWxsPiA9IHRoaXMubV9zdGFjay5SZXNldCgpO1xyXG4gICAgc3RhY2suUHVzaCh0aGlzLm1fcm9vdCk7XHJcblxyXG4gICAgd2hpbGUgKHN0YWNrLkdldENvdW50KCkgPiAwKSB7XHJcbiAgICAgIGNvbnN0IG5vZGU6IGIyVHJlZU5vZGU8VD4gfCBudWxsID0gc3RhY2suUG9wKCk7XHJcbiAgICAgIGlmIChub2RlID09PSBudWxsKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGlmIChub2RlLmFhYmIuVGVzdE92ZXJsYXAoYWFiYikpIHtcclxuICAgICAgICBpZiAobm9kZS5Jc0xlYWYoKSkge1xyXG4gICAgICAgICAgb3V0LnB1c2gobm9kZSk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHN0YWNrLlB1c2gobm9kZS5jaGlsZDEpO1xyXG4gICAgICAgICAgc3RhY2suUHVzaChub2RlLmNoaWxkMik7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBRdWVyeVBvaW50KHBvaW50OiBYWSwgY2FsbGJhY2s6IChub2RlOiBiMlRyZWVOb2RlPFQ+KSA9PiBib29sZWFuKTogdm9pZCB7XHJcbiAgICBjb25zdCBzdGFjazogYjJHcm93YWJsZVN0YWNrPGIyVHJlZU5vZGU8VD4gfCBudWxsPiA9IHRoaXMubV9zdGFjay5SZXNldCgpO1xyXG4gICAgc3RhY2suUHVzaCh0aGlzLm1fcm9vdCk7XHJcblxyXG4gICAgd2hpbGUgKHN0YWNrLkdldENvdW50KCkgPiAwKSB7XHJcbiAgICAgIGNvbnN0IG5vZGU6IGIyVHJlZU5vZGU8VD4gfCBudWxsID0gc3RhY2suUG9wKCk7XHJcbiAgICAgIGlmIChub2RlID09PSBudWxsKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGlmIChub2RlLmFhYmIuVGVzdENvbnRhaW4ocG9pbnQpKSB7XHJcbiAgICAgICAgaWYgKG5vZGUuSXNMZWFmKCkpIHtcclxuICAgICAgICAgIGNvbnN0IHByb2NlZWQgPSBjYWxsYmFjayhub2RlKTtcclxuICAgICAgICAgIGlmICghcHJvY2VlZCkge1xyXG4gICAgICAgICAgICByZXR1cm47XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIHN0YWNrLlB1c2gobm9kZS5jaGlsZDEpO1xyXG4gICAgICAgICAgc3RhY2suUHVzaChub2RlLmNoaWxkMik7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBSYXlDYXN0KFxyXG4gICAgaW5wdXQ6IGIyUmF5Q2FzdElucHV0LFxyXG4gICAgY2FsbGJhY2s6IChpbnB1dDogYjJSYXlDYXN0SW5wdXQsIG5vZGU6IGIyVHJlZU5vZGU8VD4pID0+IG51bWJlcixcclxuICApOiB2b2lkIHtcclxuICAgIGNvbnN0IHAxOiBiMlZlYzIgPSBpbnB1dC5wMTtcclxuICAgIGNvbnN0IHAyOiBiMlZlYzIgPSBpbnB1dC5wMjtcclxuICAgIGNvbnN0IHI6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihwMiwgcDEsIGIyRHluYW1pY1RyZWUuc19yKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoci5MZW5ndGhTcXVhcmVkKCkgPiAwKTtcclxuICAgIHIuTm9ybWFsaXplKCk7XHJcblxyXG4gICAgLy8gdiBpcyBwZXJwZW5kaWN1bGFyIHRvIHRoZSBzZWdtZW50LlxyXG4gICAgY29uc3QgdjogYjJWZWMyID0gYjJWZWMyLkNyb3NzT25lVihyLCBiMkR5bmFtaWNUcmVlLnNfdik7XHJcbiAgICBjb25zdCBhYnNfdjogYjJWZWMyID0gYjJWZWMyLkFic1YodiwgYjJEeW5hbWljVHJlZS5zX2Fic192KTtcclxuXHJcbiAgICAvLyBTZXBhcmF0aW5nIGF4aXMgZm9yIHNlZ21lbnQgKEdpbm8sIHA4MCkuXHJcbiAgICAvLyB8ZG90KHYsIHAxIC0gYyl8ID4gZG90KHx2fCwgaClcclxuXHJcbiAgICBsZXQgbWF4RnJhY3Rpb246IG51bWJlciA9IGlucHV0Lm1heEZyYWN0aW9uO1xyXG5cclxuICAgIC8vIEJ1aWxkIGEgYm91bmRpbmcgYm94IGZvciB0aGUgc2VnbWVudC5cclxuICAgIGNvbnN0IHNlZ21lbnRBQUJCOiBiMkFBQkIgPSBiMkR5bmFtaWNUcmVlLnNfc2VnbWVudEFBQkI7XHJcbiAgICBsZXQgdF94OiBudW1iZXIgPSBwMS54ICsgbWF4RnJhY3Rpb24gKiAocDIueCAtIHAxLngpO1xyXG4gICAgbGV0IHRfeTogbnVtYmVyID0gcDEueSArIG1heEZyYWN0aW9uICogKHAyLnkgLSBwMS55KTtcclxuICAgIHNlZ21lbnRBQUJCLmxvd2VyQm91bmQueCA9IGIyTWluKHAxLngsIHRfeCk7XHJcbiAgICBzZWdtZW50QUFCQi5sb3dlckJvdW5kLnkgPSBiMk1pbihwMS55LCB0X3kpO1xyXG4gICAgc2VnbWVudEFBQkIudXBwZXJCb3VuZC54ID0gYjJNYXgocDEueCwgdF94KTtcclxuICAgIHNlZ21lbnRBQUJCLnVwcGVyQm91bmQueSA9IGIyTWF4KHAxLnksIHRfeSk7XHJcblxyXG4gICAgY29uc3Qgc3RhY2s6IGIyR3Jvd2FibGVTdGFjazxiMlRyZWVOb2RlPFQ+IHwgbnVsbD4gPSB0aGlzLm1fc3RhY2suUmVzZXQoKTtcclxuICAgIHN0YWNrLlB1c2godGhpcy5tX3Jvb3QpO1xyXG5cclxuICAgIHdoaWxlIChzdGFjay5HZXRDb3VudCgpID4gMCkge1xyXG4gICAgICBjb25zdCBub2RlOiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IHN0YWNrLlBvcCgpO1xyXG4gICAgICBpZiAobm9kZSA9PT0gbnVsbCkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoIWIyVGVzdE92ZXJsYXBBQUJCKG5vZGUuYWFiYiwgc2VnbWVudEFBQkIpKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIFNlcGFyYXRpbmcgYXhpcyBmb3Igc2VnbWVudCAoR2lubywgcDgwKS5cclxuICAgICAgLy8gfGRvdCh2LCBwMSAtIGMpfCA+IGRvdCh8dnwsIGgpXHJcbiAgICAgIGNvbnN0IGM6IGIyVmVjMiA9IG5vZGUuYWFiYi5HZXRDZW50ZXIoKTtcclxuICAgICAgY29uc3QgaDogYjJWZWMyID0gbm9kZS5hYWJiLkdldEV4dGVudHMoKTtcclxuICAgICAgY29uc3Qgc2VwYXJhdGlvbjogbnVtYmVyID1cclxuICAgICAgICBiMkFicyhiMlZlYzIuRG90VlYodiwgYjJWZWMyLlN1YlZWKHAxLCBjLCBiMlZlYzIuc190MCkpKSAtIGIyVmVjMi5Eb3RWVihhYnNfdiwgaCk7XHJcbiAgICAgIGlmIChzZXBhcmF0aW9uID4gMCkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAobm9kZS5Jc0xlYWYoKSkge1xyXG4gICAgICAgIGNvbnN0IHN1YklucHV0OiBiMlJheUNhc3RJbnB1dCA9IGIyRHluYW1pY1RyZWUuc19zdWJJbnB1dDtcclxuICAgICAgICBzdWJJbnB1dC5wMS5Db3B5KGlucHV0LnAxKTtcclxuICAgICAgICBzdWJJbnB1dC5wMi5Db3B5KGlucHV0LnAyKTtcclxuICAgICAgICBzdWJJbnB1dC5tYXhGcmFjdGlvbiA9IG1heEZyYWN0aW9uO1xyXG5cclxuICAgICAgICBjb25zdCB2YWx1ZTogbnVtYmVyID0gY2FsbGJhY2soc3ViSW5wdXQsIG5vZGUpO1xyXG5cclxuICAgICAgICBpZiAodmFsdWUgPT09IDApIHtcclxuICAgICAgICAgIC8vIFRoZSBjbGllbnQgaGFzIHRlcm1pbmF0ZWQgdGhlIHJheSBjYXN0LlxyXG4gICAgICAgICAgcmV0dXJuO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKHZhbHVlID4gMCkge1xyXG4gICAgICAgICAgLy8gVXBkYXRlIHNlZ21lbnQgYm91bmRpbmcgYm94LlxyXG4gICAgICAgICAgbWF4RnJhY3Rpb24gPSB2YWx1ZTtcclxuICAgICAgICAgIHRfeCA9IHAxLnggKyBtYXhGcmFjdGlvbiAqIChwMi54IC0gcDEueCk7XHJcbiAgICAgICAgICB0X3kgPSBwMS55ICsgbWF4RnJhY3Rpb24gKiAocDIueSAtIHAxLnkpO1xyXG4gICAgICAgICAgc2VnbWVudEFBQkIubG93ZXJCb3VuZC54ID0gYjJNaW4ocDEueCwgdF94KTtcclxuICAgICAgICAgIHNlZ21lbnRBQUJCLmxvd2VyQm91bmQueSA9IGIyTWluKHAxLnksIHRfeSk7XHJcbiAgICAgICAgICBzZWdtZW50QUFCQi51cHBlckJvdW5kLnggPSBiMk1heChwMS54LCB0X3gpO1xyXG4gICAgICAgICAgc2VnbWVudEFBQkIudXBwZXJCb3VuZC55ID0gYjJNYXgocDEueSwgdF95KTtcclxuICAgICAgICB9XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgc3RhY2suUHVzaChub2RlLmNoaWxkMSk7XHJcbiAgICAgICAgc3RhY2suUHVzaChub2RlLmNoaWxkMik7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIHN0YXRpYyBzX25vZGVfaWQgPSAwO1xyXG5cclxuICBBbGxvY2F0ZU5vZGUoKTogYjJUcmVlTm9kZTxUPiB7XHJcbiAgICAvLyBFeHBhbmQgdGhlIG5vZGUgcG9vbCBhcyBuZWVkZWQuXHJcbiAgICBpZiAodGhpcy5tX2ZyZWVMaXN0ICE9PSBudWxsKSB7XHJcbiAgICAgIGNvbnN0IG5vZGU6IGIyVHJlZU5vZGU8VD4gPSB0aGlzLm1fZnJlZUxpc3Q7XHJcbiAgICAgIHRoaXMubV9mcmVlTGlzdCA9IG5vZGUucGFyZW50OyAvLyB0aGlzLm1fZnJlZUxpc3QgPSBub2RlLm5leHQ7XHJcbiAgICAgIG5vZGUucGFyZW50ID0gbnVsbDtcclxuICAgICAgbm9kZS5jaGlsZDEgPSBudWxsO1xyXG4gICAgICBub2RlLmNoaWxkMiA9IG51bGw7XHJcbiAgICAgIG5vZGUuaGVpZ2h0ID0gMDtcclxuICAgICAgcmV0dXJuIG5vZGU7XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIG5ldyBiMlRyZWVOb2RlPFQ+KGIyRHluYW1pY1RyZWUuc19ub2RlX2lkKyspO1xyXG4gIH1cclxuXHJcbiAgRnJlZU5vZGUobm9kZTogYjJUcmVlTm9kZTxUPik6IHZvaWQge1xyXG4gICAgbm9kZS5wYXJlbnQgPSB0aGlzLm1fZnJlZUxpc3Q7IC8vIG5vZGUubmV4dCA9IHRoaXMubV9mcmVlTGlzdDtcclxuICAgIG5vZGUuY2hpbGQxID0gbnVsbDtcclxuICAgIG5vZGUuY2hpbGQyID0gbnVsbDtcclxuICAgIG5vZGUuaGVpZ2h0ID0gLTE7XHJcbiAgICBub2RlLlJlc2V0KCk7XHJcbiAgICB0aGlzLm1fZnJlZUxpc3QgPSBub2RlO1xyXG4gIH1cclxuXHJcbiAgQ3JlYXRlUHJveHkoYWFiYjogYjJBQUJCLCB1c2VyRGF0YTogVCk6IGIyVHJlZU5vZGU8VD4ge1xyXG4gICAgY29uc3Qgbm9kZTogYjJUcmVlTm9kZTxUPiA9IHRoaXMuQWxsb2NhdGVOb2RlKCk7XHJcblxyXG4gICAgLy8gRmF0dGVuIHRoZSBhYWJiLlxyXG4gICAgY29uc3Qgcl94OiBudW1iZXIgPSBiMl9hYWJiRXh0ZW5zaW9uO1xyXG4gICAgY29uc3Qgcl95OiBudW1iZXIgPSBiMl9hYWJiRXh0ZW5zaW9uO1xyXG4gICAgbm9kZS5hYWJiLmxvd2VyQm91bmQueCA9IGFhYmIubG93ZXJCb3VuZC54IC0gcl94O1xyXG4gICAgbm9kZS5hYWJiLmxvd2VyQm91bmQueSA9IGFhYmIubG93ZXJCb3VuZC55IC0gcl95O1xyXG4gICAgbm9kZS5hYWJiLnVwcGVyQm91bmQueCA9IGFhYmIudXBwZXJCb3VuZC54ICsgcl94O1xyXG4gICAgbm9kZS5hYWJiLnVwcGVyQm91bmQueSA9IGFhYmIudXBwZXJCb3VuZC55ICsgcl95O1xyXG4gICAgbm9kZS51c2VyRGF0YSA9IHVzZXJEYXRhO1xyXG4gICAgbm9kZS5oZWlnaHQgPSAwO1xyXG5cclxuICAgIHRoaXMuSW5zZXJ0TGVhZihub2RlKTtcclxuXHJcbiAgICByZXR1cm4gbm9kZTtcclxuICB9XHJcblxyXG4gIERlc3Ryb3lQcm94eShub2RlOiBiMlRyZWVOb2RlPFQ+KTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KG5vZGUuSXNMZWFmKCkpO1xyXG5cclxuICAgIHRoaXMuUmVtb3ZlTGVhZihub2RlKTtcclxuICAgIHRoaXMuRnJlZU5vZGUobm9kZSk7XHJcbiAgfVxyXG5cclxuICBNb3ZlUHJveHkobm9kZTogYjJUcmVlTm9kZTxUPiwgYWFiYjogYjJBQUJCLCBkaXNwbGFjZW1lbnQ6IGIyVmVjMik6IGJvb2xlYW4ge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChub2RlLklzTGVhZigpKTtcclxuXHJcbiAgICBpZiAobm9kZS5hYWJiLkNvbnRhaW5zKGFhYmIpKSB7XHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLlJlbW92ZUxlYWYobm9kZSk7XHJcblxyXG4gICAgLy8gRXh0ZW5kIEFBQkIuXHJcbiAgICBjb25zdCByX3g6IG51bWJlciA9IGIyX2FhYmJFeHRlbnNpb247XHJcbiAgICBjb25zdCByX3k6IG51bWJlciA9IGIyX2FhYmJFeHRlbnNpb247XHJcbiAgICBub2RlLmFhYmIubG93ZXJCb3VuZC54ID0gYWFiYi5sb3dlckJvdW5kLnggLSByX3g7XHJcbiAgICBub2RlLmFhYmIubG93ZXJCb3VuZC55ID0gYWFiYi5sb3dlckJvdW5kLnkgLSByX3k7XHJcbiAgICBub2RlLmFhYmIudXBwZXJCb3VuZC54ID0gYWFiYi51cHBlckJvdW5kLnggKyByX3g7XHJcbiAgICBub2RlLmFhYmIudXBwZXJCb3VuZC55ID0gYWFiYi51cHBlckJvdW5kLnkgKyByX3k7XHJcblxyXG4gICAgLy8gUHJlZGljdCBBQUJCIGRpc3BsYWNlbWVudC5cclxuICAgIGNvbnN0IGRfeDogbnVtYmVyID0gYjJfYWFiYk11bHRpcGxpZXIgKiBkaXNwbGFjZW1lbnQueDtcclxuICAgIGNvbnN0IGRfeTogbnVtYmVyID0gYjJfYWFiYk11bHRpcGxpZXIgKiBkaXNwbGFjZW1lbnQueTtcclxuXHJcbiAgICBpZiAoZF94IDwgMC4wKSB7XHJcbiAgICAgIG5vZGUuYWFiYi5sb3dlckJvdW5kLnggKz0gZF94O1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgbm9kZS5hYWJiLnVwcGVyQm91bmQueCArPSBkX3g7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGRfeSA8IDAuMCkge1xyXG4gICAgICBub2RlLmFhYmIubG93ZXJCb3VuZC55ICs9IGRfeTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIG5vZGUuYWFiYi51cHBlckJvdW5kLnkgKz0gZF95O1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMuSW5zZXJ0TGVhZihub2RlKTtcclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgSW5zZXJ0TGVhZihsZWFmOiBiMlRyZWVOb2RlPFQ+KTogdm9pZCB7XHJcbiAgICArK3RoaXMubV9pbnNlcnRpb25Db3VudDtcclxuXHJcbiAgICBpZiAodGhpcy5tX3Jvb3QgPT09IG51bGwpIHtcclxuICAgICAgdGhpcy5tX3Jvb3QgPSBsZWFmO1xyXG4gICAgICB0aGlzLm1fcm9vdC5wYXJlbnQgPSBudWxsO1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgLy8gRmluZCB0aGUgYmVzdCBzaWJsaW5nIGZvciB0aGlzIG5vZGVcclxuICAgIGNvbnN0IGxlYWZBQUJCOiBiMkFBQkIgPSBsZWFmLmFhYmI7XHJcbiAgICBsZXQgc2libGluZzogYjJUcmVlTm9kZTxUPiA9IHRoaXMubV9yb290O1xyXG4gICAgd2hpbGUgKCFzaWJsaW5nLklzTGVhZigpKSB7XHJcbiAgICAgIGNvbnN0IGNoaWxkMTogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShzaWJsaW5nLmNoaWxkMSk7XHJcbiAgICAgIGNvbnN0IGNoaWxkMjogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShzaWJsaW5nLmNoaWxkMik7XHJcblxyXG4gICAgICBjb25zdCBhcmVhOiBudW1iZXIgPSBzaWJsaW5nLmFhYmIuR2V0UGVyaW1ldGVyKCk7XHJcblxyXG4gICAgICBjb25zdCBjb21iaW5lZEFBQkI6IGIyQUFCQiA9IGIyRHluYW1pY1RyZWUuc19jb21iaW5lZEFBQkI7XHJcbiAgICAgIGNvbWJpbmVkQUFCQi5Db21iaW5lMihzaWJsaW5nLmFhYmIsIGxlYWZBQUJCKTtcclxuICAgICAgY29uc3QgY29tYmluZWRBcmVhOiBudW1iZXIgPSBjb21iaW5lZEFBQkIuR2V0UGVyaW1ldGVyKCk7XHJcblxyXG4gICAgICAvLyBDb3N0IG9mIGNyZWF0aW5nIGEgbmV3IHBhcmVudCBmb3IgdGhpcyBub2RlIGFuZCB0aGUgbmV3IGxlYWZcclxuICAgICAgY29uc3QgY29zdDogbnVtYmVyID0gMiAqIGNvbWJpbmVkQXJlYTtcclxuXHJcbiAgICAgIC8vIE1pbmltdW0gY29zdCBvZiBwdXNoaW5nIHRoZSBsZWFmIGZ1cnRoZXIgZG93biB0aGUgdHJlZVxyXG4gICAgICBjb25zdCBpbmhlcml0YW5jZUNvc3Q6IG51bWJlciA9IDIgKiAoY29tYmluZWRBcmVhIC0gYXJlYSk7XHJcblxyXG4gICAgICAvLyBDb3N0IG9mIGRlc2NlbmRpbmcgaW50byBjaGlsZDFcclxuICAgICAgbGV0IGNvc3QxOiBudW1iZXI7XHJcbiAgICAgIGNvbnN0IGFhYmI6IGIyQUFCQiA9IGIyRHluYW1pY1RyZWUuc19hYWJiO1xyXG4gICAgICBsZXQgb2xkQXJlYTogbnVtYmVyO1xyXG4gICAgICBsZXQgbmV3QXJlYTogbnVtYmVyO1xyXG4gICAgICBpZiAoY2hpbGQxLklzTGVhZigpKSB7XHJcbiAgICAgICAgYWFiYi5Db21iaW5lMihsZWFmQUFCQiwgY2hpbGQxLmFhYmIpO1xyXG4gICAgICAgIGNvc3QxID0gYWFiYi5HZXRQZXJpbWV0ZXIoKSArIGluaGVyaXRhbmNlQ29zdDtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICBhYWJiLkNvbWJpbmUyKGxlYWZBQUJCLCBjaGlsZDEuYWFiYik7XHJcbiAgICAgICAgb2xkQXJlYSA9IGNoaWxkMS5hYWJiLkdldFBlcmltZXRlcigpO1xyXG4gICAgICAgIG5ld0FyZWEgPSBhYWJiLkdldFBlcmltZXRlcigpO1xyXG4gICAgICAgIGNvc3QxID0gbmV3QXJlYSAtIG9sZEFyZWEgKyBpbmhlcml0YW5jZUNvc3Q7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIENvc3Qgb2YgZGVzY2VuZGluZyBpbnRvIGNoaWxkMlxyXG4gICAgICBsZXQgY29zdDI6IG51bWJlcjtcclxuICAgICAgaWYgKGNoaWxkMi5Jc0xlYWYoKSkge1xyXG4gICAgICAgIGFhYmIuQ29tYmluZTIobGVhZkFBQkIsIGNoaWxkMi5hYWJiKTtcclxuICAgICAgICBjb3N0MiA9IGFhYmIuR2V0UGVyaW1ldGVyKCkgKyBpbmhlcml0YW5jZUNvc3Q7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgYWFiYi5Db21iaW5lMihsZWFmQUFCQiwgY2hpbGQyLmFhYmIpO1xyXG4gICAgICAgIG9sZEFyZWEgPSBjaGlsZDIuYWFiYi5HZXRQZXJpbWV0ZXIoKTtcclxuICAgICAgICBuZXdBcmVhID0gYWFiYi5HZXRQZXJpbWV0ZXIoKTtcclxuICAgICAgICBjb3N0MiA9IG5ld0FyZWEgLSBvbGRBcmVhICsgaW5oZXJpdGFuY2VDb3N0O1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBEZXNjZW5kIGFjY29yZGluZyB0byB0aGUgbWluaW11bSBjb3N0LlxyXG4gICAgICBpZiAoY29zdCA8IGNvc3QxICYmIGNvc3QgPCBjb3N0Mikge1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBEZXNjZW5kXHJcbiAgICAgIGlmIChjb3N0MSA8IGNvc3QyKSB7XHJcbiAgICAgICAgc2libGluZyA9IGNoaWxkMTtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICBzaWJsaW5nID0gY2hpbGQyO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gQ3JlYXRlIGEgcGFyZW50IGZvciB0aGUgc2libGluZ3MuXHJcbiAgICBjb25zdCBvbGRQYXJlbnQ6IGIyVHJlZU5vZGU8VD4gfCBudWxsID0gc2libGluZy5wYXJlbnQ7XHJcbiAgICBjb25zdCBuZXdQYXJlbnQ6IGIyVHJlZU5vZGU8VD4gPSB0aGlzLkFsbG9jYXRlTm9kZSgpO1xyXG4gICAgbmV3UGFyZW50LnBhcmVudCA9IG9sZFBhcmVudDtcclxuICAgIG5ld1BhcmVudC5hYWJiLkNvbWJpbmUyKGxlYWZBQUJCLCBzaWJsaW5nLmFhYmIpO1xyXG4gICAgbmV3UGFyZW50LmhlaWdodCA9IHNpYmxpbmcuaGVpZ2h0ICsgMTtcclxuXHJcbiAgICBpZiAob2xkUGFyZW50ICE9PSBudWxsKSB7XHJcbiAgICAgIC8vIFRoZSBzaWJsaW5nIHdhcyBub3QgdGhlIHJvb3QuXHJcbiAgICAgIGlmIChvbGRQYXJlbnQuY2hpbGQxID09PSBzaWJsaW5nKSB7XHJcbiAgICAgICAgb2xkUGFyZW50LmNoaWxkMSA9IG5ld1BhcmVudDtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICBvbGRQYXJlbnQuY2hpbGQyID0gbmV3UGFyZW50O1xyXG4gICAgICB9XHJcblxyXG4gICAgICBuZXdQYXJlbnQuY2hpbGQxID0gc2libGluZztcclxuICAgICAgbmV3UGFyZW50LmNoaWxkMiA9IGxlYWY7XHJcbiAgICAgIHNpYmxpbmcucGFyZW50ID0gbmV3UGFyZW50O1xyXG4gICAgICBsZWFmLnBhcmVudCA9IG5ld1BhcmVudDtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIFRoZSBzaWJsaW5nIHdhcyB0aGUgcm9vdC5cclxuICAgICAgbmV3UGFyZW50LmNoaWxkMSA9IHNpYmxpbmc7XHJcbiAgICAgIG5ld1BhcmVudC5jaGlsZDIgPSBsZWFmO1xyXG4gICAgICBzaWJsaW5nLnBhcmVudCA9IG5ld1BhcmVudDtcclxuICAgICAgbGVhZi5wYXJlbnQgPSBuZXdQYXJlbnQ7XHJcbiAgICAgIHRoaXMubV9yb290ID0gbmV3UGFyZW50O1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFdhbGsgYmFjayB1cCB0aGUgdHJlZSBmaXhpbmcgaGVpZ2h0cyBhbmQgQUFCQnNcclxuICAgIGxldCBub2RlOiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IGxlYWYucGFyZW50O1xyXG4gICAgd2hpbGUgKG5vZGUgIT09IG51bGwpIHtcclxuICAgICAgY29uc3QgbiA9IHRoaXMuQmFsYW5jZShub2RlKTtcclxuXHJcbiAgICAgIGNvbnN0IGNoaWxkMTogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShuLmNoaWxkMSk7XHJcbiAgICAgIGNvbnN0IGNoaWxkMjogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShuLmNoaWxkMik7XHJcblxyXG4gICAgICBuLmhlaWdodCA9IDEgKyBiMk1heEludChjaGlsZDEuaGVpZ2h0LCBjaGlsZDIuaGVpZ2h0KTtcclxuICAgICAgbi5hYWJiLkNvbWJpbmUyKGNoaWxkMS5hYWJiLCBjaGlsZDIuYWFiYik7XHJcblxyXG4gICAgICBub2RlID0gbi5wYXJlbnQ7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gdGhpcy5WYWxpZGF0ZSgpO1xyXG4gIH1cclxuXHJcbiAgUmVtb3ZlTGVhZihsZWFmOiBiMlRyZWVOb2RlPFQ+KTogdm9pZCB7XHJcbiAgICBpZiAobGVhZiA9PT0gdGhpcy5tX3Jvb3QpIHtcclxuICAgICAgdGhpcy5tX3Jvb3QgPSBudWxsO1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgcGFyZW50OiBiMlRyZWVOb2RlPFQ+ID0gdmVyaWZ5KGxlYWYucGFyZW50KTtcclxuICAgIGNvbnN0IGdyYW5kUGFyZW50OiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IHBhcmVudCAmJiBwYXJlbnQucGFyZW50O1xyXG4gICAgY29uc3Qgc2libGluZzogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShwYXJlbnQuY2hpbGQxID09PSBsZWFmID8gcGFyZW50LmNoaWxkMiA6IHBhcmVudC5jaGlsZDEpO1xyXG5cclxuICAgIGlmIChncmFuZFBhcmVudCAhPT0gbnVsbCkge1xyXG4gICAgICAvLyBEZXN0cm95IHBhcmVudCBhbmQgY29ubmVjdCBzaWJsaW5nIHRvIGdyYW5kUGFyZW50LlxyXG4gICAgICBpZiAoZ3JhbmRQYXJlbnQuY2hpbGQxID09PSBwYXJlbnQpIHtcclxuICAgICAgICBncmFuZFBhcmVudC5jaGlsZDEgPSBzaWJsaW5nO1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIGdyYW5kUGFyZW50LmNoaWxkMiA9IHNpYmxpbmc7XHJcbiAgICAgIH1cclxuICAgICAgc2libGluZy5wYXJlbnQgPSBncmFuZFBhcmVudDtcclxuICAgICAgdGhpcy5GcmVlTm9kZShwYXJlbnQpO1xyXG5cclxuICAgICAgLy8gQWRqdXN0IGFuY2VzdG9yIGJvdW5kcy5cclxuICAgICAgbGV0IGluZGV4OiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IGdyYW5kUGFyZW50O1xyXG4gICAgICB3aGlsZSAoaW5kZXggIT09IG51bGwpIHtcclxuICAgICAgICBpbmRleCA9IHRoaXMuQmFsYW5jZShpbmRleCk7XHJcblxyXG4gICAgICAgIGNvbnN0IGNoaWxkMTogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShpbmRleC5jaGlsZDEpO1xyXG4gICAgICAgIGNvbnN0IGNoaWxkMjogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShpbmRleC5jaGlsZDIpO1xyXG5cclxuICAgICAgICBpbmRleC5hYWJiLkNvbWJpbmUyKGNoaWxkMS5hYWJiLCBjaGlsZDIuYWFiYik7XHJcbiAgICAgICAgaW5kZXguaGVpZ2h0ID0gMSArIGIyTWF4SW50KGNoaWxkMS5oZWlnaHQsIGNoaWxkMi5oZWlnaHQpO1xyXG5cclxuICAgICAgICBpbmRleCA9IGluZGV4LnBhcmVudDtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgdGhpcy5tX3Jvb3QgPSBzaWJsaW5nO1xyXG4gICAgICBzaWJsaW5nLnBhcmVudCA9IG51bGw7XHJcbiAgICAgIHRoaXMuRnJlZU5vZGUocGFyZW50KTtcclxuICAgIH1cclxuXHJcbiAgICAvLyB0aGlzLlZhbGlkYXRlKCk7XHJcbiAgfVxyXG5cclxuICBCYWxhbmNlKEE6IGIyVHJlZU5vZGU8VD4pOiBiMlRyZWVOb2RlPFQ+IHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoQSAhPT0gbnVsbCk7XHJcblxyXG4gICAgaWYgKEEuSXNMZWFmKCkgfHwgQS5oZWlnaHQgPCAyKSB7XHJcbiAgICAgIHJldHVybiBBO1xyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IEI6IGIyVHJlZU5vZGU8VD4gPSB2ZXJpZnkoQS5jaGlsZDEpO1xyXG4gICAgY29uc3QgQzogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShBLmNoaWxkMik7XHJcblxyXG4gICAgY29uc3QgYmFsYW5jZTogbnVtYmVyID0gQy5oZWlnaHQgLSBCLmhlaWdodDtcclxuXHJcbiAgICAvLyBSb3RhdGUgQyB1cFxyXG4gICAgaWYgKGJhbGFuY2UgPiAxKSB7XHJcbiAgICAgIHJldHVybiB0aGlzLlJvdGF0ZV9DX1VwKEEsIEIsIEMpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFJvdGF0ZSBCIHVwXHJcbiAgICBlbHNlIGlmIChiYWxhbmNlIDwgLTEpIHtcclxuICAgICAgcmV0dXJuIHRoaXMuUm90YXRlX0JfVXAoQSwgQiwgQyk7XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIEE7XHJcbiAgfVxyXG5cclxuICBSb3RhdGVfQ19VcChBOiBiMlRyZWVOb2RlPFQ+LCBCOiBiMlRyZWVOb2RlPFQ+LCBDOiBiMlRyZWVOb2RlPFQ+KTogYjJUcmVlTm9kZTxUPiB7XHJcbiAgICBjb25zdCBGOiBiMlRyZWVOb2RlPFQ+ID0gdmVyaWZ5KEMuY2hpbGQxKTtcclxuICAgIGNvbnN0IEc6IGIyVHJlZU5vZGU8VD4gPSB2ZXJpZnkoQy5jaGlsZDIpO1xyXG5cclxuICAgIC8vIFN3YXAgQSBhbmQgQ1xyXG4gICAgQy5jaGlsZDEgPSBBO1xyXG4gICAgQy5wYXJlbnQgPSBBLnBhcmVudDtcclxuICAgIEEucGFyZW50ID0gQztcclxuXHJcbiAgICAvLyBBJ3Mgb2xkIHBhcmVudCBzaG91bGQgcG9pbnQgdG8gQ1xyXG4gICAgaWYgKEMucGFyZW50ICE9PSBudWxsKSB7XHJcbiAgICAgIGlmIChDLnBhcmVudC5jaGlsZDEgPT09IEEpIHtcclxuICAgICAgICBDLnBhcmVudC5jaGlsZDEgPSBDO1xyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoQy5wYXJlbnQuY2hpbGQyID09PSBBKTtcclxuICAgICAgICBDLnBhcmVudC5jaGlsZDIgPSBDO1xyXG4gICAgICB9XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1fcm9vdCA9IEM7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gUm90YXRlXHJcbiAgICBpZiAoRi5oZWlnaHQgPiBHLmhlaWdodCkge1xyXG4gICAgICBDLmNoaWxkMiA9IEY7XHJcbiAgICAgIEEuY2hpbGQyID0gRztcclxuICAgICAgRy5wYXJlbnQgPSBBO1xyXG4gICAgICBBLmFhYmIuQ29tYmluZTIoQi5hYWJiLCBHLmFhYmIpO1xyXG4gICAgICBDLmFhYmIuQ29tYmluZTIoQS5hYWJiLCBGLmFhYmIpO1xyXG5cclxuICAgICAgQS5oZWlnaHQgPSAxICsgYjJNYXhJbnQoQi5oZWlnaHQsIEcuaGVpZ2h0KTtcclxuICAgICAgQy5oZWlnaHQgPSAxICsgYjJNYXhJbnQoQS5oZWlnaHQsIEYuaGVpZ2h0KTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIEMuY2hpbGQyID0gRztcclxuICAgICAgQS5jaGlsZDIgPSBGO1xyXG4gICAgICBGLnBhcmVudCA9IEE7XHJcbiAgICAgIEEuYWFiYi5Db21iaW5lMihCLmFhYmIsIEYuYWFiYik7XHJcbiAgICAgIEMuYWFiYi5Db21iaW5lMihBLmFhYmIsIEcuYWFiYik7XHJcblxyXG4gICAgICBBLmhlaWdodCA9IDEgKyBiMk1heEludChCLmhlaWdodCwgRi5oZWlnaHQpO1xyXG4gICAgICBDLmhlaWdodCA9IDEgKyBiMk1heEludChBLmhlaWdodCwgRy5oZWlnaHQpO1xyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBDO1xyXG4gIH1cclxuXHJcbiAgUm90YXRlX0JfVXAoQTogYjJUcmVlTm9kZTxUPiwgQjogYjJUcmVlTm9kZTxUPiwgQzogYjJUcmVlTm9kZTxUPik6IGIyVHJlZU5vZGU8VD4ge1xyXG4gICAgY29uc3QgRDogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShCLmNoaWxkMSk7XHJcbiAgICBjb25zdCBFOiBiMlRyZWVOb2RlPFQ+ID0gdmVyaWZ5KEIuY2hpbGQyKTtcclxuXHJcbiAgICAvLyBTd2FwIEEgYW5kIEJcclxuICAgIEIuY2hpbGQxID0gQTtcclxuICAgIEIucGFyZW50ID0gQS5wYXJlbnQ7XHJcbiAgICBBLnBhcmVudCA9IEI7XHJcblxyXG4gICAgLy8gQSdzIG9sZCBwYXJlbnQgc2hvdWxkIHBvaW50IHRvIEJcclxuICAgIGlmIChCLnBhcmVudCAhPT0gbnVsbCkge1xyXG4gICAgICBpZiAoQi5wYXJlbnQuY2hpbGQxID09PSBBKSB7XHJcbiAgICAgICAgQi5wYXJlbnQuY2hpbGQxID0gQjtcclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KEIucGFyZW50LmNoaWxkMiA9PT0gQSk7XHJcbiAgICAgICAgQi5wYXJlbnQuY2hpbGQyID0gQjtcclxuICAgICAgfVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgdGhpcy5tX3Jvb3QgPSBCO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFJvdGF0ZVxyXG4gICAgaWYgKEQuaGVpZ2h0ID4gRS5oZWlnaHQpIHtcclxuICAgICAgQi5jaGlsZDIgPSBEO1xyXG4gICAgICBBLmNoaWxkMSA9IEU7XHJcbiAgICAgIEUucGFyZW50ID0gQTtcclxuICAgICAgQS5hYWJiLkNvbWJpbmUyKEMuYWFiYiwgRS5hYWJiKTtcclxuICAgICAgQi5hYWJiLkNvbWJpbmUyKEEuYWFiYiwgRC5hYWJiKTtcclxuXHJcbiAgICAgIEEuaGVpZ2h0ID0gMSArIGIyTWF4SW50KEMuaGVpZ2h0LCBFLmhlaWdodCk7XHJcbiAgICAgIEIuaGVpZ2h0ID0gMSArIGIyTWF4SW50KEEuaGVpZ2h0LCBELmhlaWdodCk7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICBCLmNoaWxkMiA9IEU7XHJcbiAgICAgIEEuY2hpbGQxID0gRDtcclxuICAgICAgRC5wYXJlbnQgPSBBO1xyXG4gICAgICBBLmFhYmIuQ29tYmluZTIoQy5hYWJiLCBELmFhYmIpO1xyXG4gICAgICBCLmFhYmIuQ29tYmluZTIoQS5hYWJiLCBFLmFhYmIpO1xyXG5cclxuICAgICAgQS5oZWlnaHQgPSAxICsgYjJNYXhJbnQoQy5oZWlnaHQsIEQuaGVpZ2h0KTtcclxuICAgICAgQi5oZWlnaHQgPSAxICsgYjJNYXhJbnQoQS5oZWlnaHQsIEUuaGVpZ2h0KTtcclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gQjtcclxuICB9XHJcblxyXG4gIEdldEhlaWdodCgpOiBudW1iZXIge1xyXG4gICAgaWYgKHRoaXMubV9yb290ID09PSBudWxsKSB7XHJcbiAgICAgIHJldHVybiAwO1xyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiB0aGlzLm1fcm9vdC5oZWlnaHQ7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBHZXRBcmVhTm9kZTxUPihub2RlOiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCk6IG51bWJlciB7XHJcbiAgICBpZiAobm9kZSA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm4gMDtcclxuICAgIH1cclxuXHJcbiAgICBpZiAobm9kZS5Jc0xlYWYoKSkge1xyXG4gICAgICByZXR1cm4gMDtcclxuICAgIH1cclxuXHJcbiAgICBsZXQgYXJlYTogbnVtYmVyID0gbm9kZS5hYWJiLkdldFBlcmltZXRlcigpO1xyXG4gICAgYXJlYSArPSBiMkR5bmFtaWNUcmVlLkdldEFyZWFOb2RlKG5vZGUuY2hpbGQxKTtcclxuICAgIGFyZWEgKz0gYjJEeW5hbWljVHJlZS5HZXRBcmVhTm9kZShub2RlLmNoaWxkMik7XHJcbiAgICByZXR1cm4gYXJlYTtcclxuICB9XHJcblxyXG4gIEdldEFyZWFSYXRpbygpOiBudW1iZXIge1xyXG4gICAgaWYgKHRoaXMubV9yb290ID09PSBudWxsKSB7XHJcbiAgICAgIHJldHVybiAwO1xyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IHJvb3Q6IGIyVHJlZU5vZGU8VD4gPSB0aGlzLm1fcm9vdDtcclxuICAgIGNvbnN0IHJvb3RBcmVhOiBudW1iZXIgPSByb290LmFhYmIuR2V0UGVyaW1ldGVyKCk7XHJcblxyXG4gICAgY29uc3QgdG90YWxBcmVhOiBudW1iZXIgPSBiMkR5bmFtaWNUcmVlLkdldEFyZWFOb2RlKHRoaXMubV9yb290KTtcclxuXHJcbiAgICAvKlxyXG4gICAgICAgIGZsb2F0MzIgdG90YWxBcmVhID0gMC4wO1xyXG4gICAgICAgIGZvciAoaW50MzIgaSA9IDA7IGkgPCBtX25vZGVDYXBhY2l0eTsgKytpKSB7XHJcbiAgICAgICAgICBjb25zdCBiMlRyZWVOb2RlPFQ+KiBub2RlID0gbV9ub2RlcyArIGk7XHJcbiAgICAgICAgICBpZiAobm9kZS5oZWlnaHQgPCAwKSB7XHJcbiAgICAgICAgICAgIC8vIEZyZWUgbm9kZSBpbiBwb29sXHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIHRvdGFsQXJlYSArPSBub2RlLmFhYmIuR2V0UGVyaW1ldGVyKCk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgICovXHJcblxyXG4gICAgcmV0dXJuIHRvdGFsQXJlYSAvIHJvb3RBcmVhO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIENvbXB1dGVIZWlnaHROb2RlPFQ+KG5vZGU6IGIyVHJlZU5vZGU8VD4gfCBudWxsKTogbnVtYmVyIHtcclxuICAgIGlmIChub2RlID09PSBudWxsIHx8IG5vZGUuSXNMZWFmKCkpIHtcclxuICAgICAgcmV0dXJuIDA7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgaGVpZ2h0MTogbnVtYmVyID0gYjJEeW5hbWljVHJlZS5Db21wdXRlSGVpZ2h0Tm9kZShub2RlLmNoaWxkMSk7XHJcbiAgICBjb25zdCBoZWlnaHQyOiBudW1iZXIgPSBiMkR5bmFtaWNUcmVlLkNvbXB1dGVIZWlnaHROb2RlKG5vZGUuY2hpbGQyKTtcclxuICAgIHJldHVybiAxICsgYjJNYXhJbnQoaGVpZ2h0MSwgaGVpZ2h0Mik7XHJcbiAgfVxyXG5cclxuICBDb21wdXRlSGVpZ2h0KCk6IG51bWJlciB7XHJcbiAgICBjb25zdCBoZWlnaHQ6IG51bWJlciA9IGIyRHluYW1pY1RyZWUuQ29tcHV0ZUhlaWdodE5vZGUodGhpcy5tX3Jvb3QpO1xyXG4gICAgcmV0dXJuIGhlaWdodDtcclxuICB9XHJcblxyXG4gIFZhbGlkYXRlU3RydWN0dXJlKG5vZGU6IGIyVHJlZU5vZGU8VD4gfCBudWxsKTogdm9pZCB7XHJcbiAgICBpZiAobm9kZSA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKG5vZGUgPT09IHRoaXMubV9yb290KSB7XHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQobm9kZS5wYXJlbnQgPT09IG51bGwpO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChub2RlLklzTGVhZigpKSB7XHJcbiAgICAgIGlmIChCMl9ERUJVRykge1xyXG4gICAgICAgIGIyQXNzZXJ0KG5vZGUuY2hpbGQxID09PSBudWxsKTtcclxuICAgICAgICBiMkFzc2VydChub2RlLmNoaWxkMiA9PT0gbnVsbCk7XHJcbiAgICAgICAgYjJBc3NlcnQobm9kZS5oZWlnaHQgPT09IDApO1xyXG4gICAgICB9XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCBjaGlsZDE6IGIyVHJlZU5vZGU8VD4gPSB2ZXJpZnkobm9kZS5jaGlsZDEpO1xyXG4gICAgY29uc3QgY2hpbGQyOiBiMlRyZWVOb2RlPFQ+ID0gdmVyaWZ5KG5vZGUuY2hpbGQyKTtcclxuXHJcbiAgICBpZiAoQjJfREVCVUcpIHtcclxuICAgICAgLy8gVE9ETzogaW5kZXhcclxuICAgICAgLy8gYjJBc3NlcnQoY2hpbGQxLnBhcmVudCA9PT0gaW5kZXgpO1xyXG4gICAgICAvLyBiMkFzc2VydChjaGlsZDIucGFyZW50ID09PSBpbmRleCk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5WYWxpZGF0ZVN0cnVjdHVyZShjaGlsZDEpO1xyXG4gICAgdGhpcy5WYWxpZGF0ZVN0cnVjdHVyZShjaGlsZDIpO1xyXG4gIH1cclxuXHJcbiAgVmFsaWRhdGVNZXRyaWNzKG5vZGU6IGIyVHJlZU5vZGU8VD4gfCBudWxsKTogdm9pZCB7XHJcbiAgICBpZiAobm9kZSA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKG5vZGUuSXNMZWFmKCkpIHtcclxuICAgICAgaWYgKEIyX0RFQlVHKSB7XHJcbiAgICAgICAgYjJBc3NlcnQobm9kZS5jaGlsZDEgPT09IG51bGwpO1xyXG4gICAgICAgIGIyQXNzZXJ0KG5vZGUuY2hpbGQyID09PSBudWxsKTtcclxuICAgICAgICBiMkFzc2VydChub2RlLmhlaWdodCA9PT0gMCk7XHJcbiAgICAgIH1cclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGNvbnN0IGNoaWxkMTogYjJUcmVlTm9kZTxUPiA9IHZlcmlmeShub2RlLmNoaWxkMSk7XHJcbiAgICBjb25zdCBjaGlsZDI6IGIyVHJlZU5vZGU8VD4gPSB2ZXJpZnkobm9kZS5jaGlsZDIpO1xyXG5cclxuICAgIGlmIChCMl9ERUJVRykge1xyXG4gICAgICBjb25zdCBoZWlnaHQxID0gY2hpbGQxLmhlaWdodDtcclxuICAgICAgY29uc3QgaGVpZ2h0MiA9IGNoaWxkMi5oZWlnaHQ7XHJcbiAgICAgIGNvbnN0IGhlaWdodCA9IDEgKyBiMk1heEludChoZWlnaHQxLCBoZWlnaHQyKTtcclxuICAgICAgYjJBc3NlcnQobm9kZS5oZWlnaHQgPT09IGhlaWdodCk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgYWFiYjogYjJBQUJCID0gYjJEeW5hbWljVHJlZS5zX2FhYmI7XHJcbiAgICBhYWJiLkNvbWJpbmUyKGNoaWxkMS5hYWJiLCBjaGlsZDIuYWFiYik7XHJcblxyXG4gICAgaWYgKEIyX0RFQlVHKSB7XHJcbiAgICAgIGIyQXNzZXJ0KGFhYmIubG93ZXJCb3VuZCA9PT0gbm9kZS5hYWJiLmxvd2VyQm91bmQpO1xyXG4gICAgICBiMkFzc2VydChhYWJiLnVwcGVyQm91bmQgPT09IG5vZGUuYWFiYi51cHBlckJvdW5kKTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLlZhbGlkYXRlTWV0cmljcyhjaGlsZDEpO1xyXG4gICAgdGhpcy5WYWxpZGF0ZU1ldHJpY3MoY2hpbGQyKTtcclxuICB9XHJcblxyXG4gIFZhbGlkYXRlKCk6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiB0aGlzLlZhbGlkYXRlU3RydWN0dXJlKHRoaXMubV9yb290KTtcclxuICAgICEhQjJfREVCVUcgJiYgdGhpcy5WYWxpZGF0ZU1ldHJpY3ModGhpcy5tX3Jvb3QpO1xyXG5cclxuICAgIC8vIGxldCBmcmVlQ291bnQ6IG51bWJlciA9IDA7XHJcbiAgICAvLyBsZXQgZnJlZUluZGV4OiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IHRoaXMubV9mcmVlTGlzdDtcclxuICAgIC8vIHdoaWxlIChmcmVlSW5kZXggIT09IG51bGwpIHtcclxuICAgIC8vICAgZnJlZUluZGV4ID0gZnJlZUluZGV4LnBhcmVudDsgLy8gZnJlZUluZGV4ID0gZnJlZUluZGV4Lm5leHQ7XHJcbiAgICAvLyAgICsrZnJlZUNvdW50O1xyXG4gICAgLy8gfVxyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5HZXRIZWlnaHQoKSA9PT0gdGhpcy5Db21wdXRlSGVpZ2h0KCkpO1xyXG5cclxuICAgIC8vIGIyQXNzZXJ0KHRoaXMubV9ub2RlQ291bnQgKyBmcmVlQ291bnQgPT09IHRoaXMubV9ub2RlQ2FwYWNpdHkpO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgR2V0TWF4QmFsYW5jZU5vZGU8VD4obm9kZTogYjJUcmVlTm9kZTxUPiB8IG51bGwsIG1heEJhbGFuY2U6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICBpZiAobm9kZSA9PT0gbnVsbCkge1xyXG4gICAgICByZXR1cm4gbWF4QmFsYW5jZTtcclxuICAgIH1cclxuXHJcbiAgICBpZiAobm9kZS5oZWlnaHQgPD0gMSkge1xyXG4gICAgICByZXR1cm4gbWF4QmFsYW5jZTtcclxuICAgIH1cclxuXHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KCFub2RlLklzTGVhZigpKTtcclxuXHJcbiAgICBjb25zdCBjaGlsZDE6IGIyVHJlZU5vZGU8VD4gPSB2ZXJpZnkobm9kZS5jaGlsZDEpO1xyXG4gICAgY29uc3QgY2hpbGQyOiBiMlRyZWVOb2RlPFQ+ID0gdmVyaWZ5KG5vZGUuY2hpbGQyKTtcclxuICAgIGNvbnN0IGJhbGFuY2U6IG51bWJlciA9IGIyQWJzSW50KGNoaWxkMi5oZWlnaHQgLSBjaGlsZDEuaGVpZ2h0KTtcclxuICAgIHJldHVybiBiMk1heEludChtYXhCYWxhbmNlLCBiYWxhbmNlKTtcclxuICB9XHJcblxyXG4gIEdldE1heEJhbGFuY2UoKTogbnVtYmVyIHtcclxuICAgIGNvbnN0IG1heEJhbGFuY2U6IG51bWJlciA9IGIyRHluYW1pY1RyZWUuR2V0TWF4QmFsYW5jZU5vZGUodGhpcy5tX3Jvb3QsIDApO1xyXG5cclxuICAgIC8qXHJcbiAgICAgICAgaW50MzIgbWF4QmFsYW5jZSA9IDA7XHJcbiAgICAgICAgZm9yIChpbnQzMiBpID0gMDsgaSA8IG1fbm9kZUNhcGFjaXR5OyArK2kpIHtcclxuICAgICAgICAgIGNvbnN0IGIyVHJlZU5vZGU8VD4qIG5vZGUgPSBtX25vZGVzICsgaTtcclxuICAgICAgICAgIGlmIChub2RlLmhlaWdodCA8PSAxKSB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIGIyQXNzZXJ0KCFub2RlLklzTGVhZigpKTtcclxuXHJcbiAgICAgICAgICBpbnQzMiBjaGlsZDEgPSBub2RlLmNoaWxkMTtcclxuICAgICAgICAgIGludDMyIGNoaWxkMiA9IG5vZGUuY2hpbGQyO1xyXG4gICAgICAgICAgaW50MzIgYmFsYW5jZSA9IGIyQWJzKG1fbm9kZXNbY2hpbGQyXS5oZWlnaHQgLSBtX25vZGVzW2NoaWxkMV0uaGVpZ2h0KTtcclxuICAgICAgICAgIG1heEJhbGFuY2UgPSBiMk1heChtYXhCYWxhbmNlLCBiYWxhbmNlKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgKi9cclxuXHJcbiAgICByZXR1cm4gbWF4QmFsYW5jZTtcclxuICB9XHJcblxyXG4gIFJlYnVpbGRCb3R0b21VcCgpOiB2b2lkIHtcclxuICAgIC8qXHJcbiAgICAgICAgaW50MzIqIG5vZGVzID0gKGludDMyKiliMkFsbG9jKG1fbm9kZUNvdW50ICogc2l6ZW9mKGludDMyKSk7XHJcbiAgICAgICAgaW50MzIgY291bnQgPSAwO1xyXG5cclxuICAgICAgICAvLyBCdWlsZCBhcnJheSBvZiBsZWF2ZXMuIEZyZWUgdGhlIHJlc3QuXHJcbiAgICAgICAgZm9yIChpbnQzMiBpID0gMDsgaSA8IG1fbm9kZUNhcGFjaXR5OyArK2kpIHtcclxuICAgICAgICAgIGlmIChtX25vZGVzW2ldLmhlaWdodCA8IDApIHtcclxuICAgICAgICAgICAgLy8gZnJlZSBub2RlIGluIHBvb2xcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgaWYgKG1fbm9kZXNbaV0uSXNMZWFmKCkpIHtcclxuICAgICAgICAgICAgbV9ub2Rlc1tpXS5wYXJlbnQgPSBiMl9udWxsTm9kZTtcclxuICAgICAgICAgICAgbm9kZXNbY291bnRdID0gaTtcclxuICAgICAgICAgICAgKytjb3VudDtcclxuICAgICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICAgIEZyZWVOb2RlKGkpO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgd2hpbGUgKGNvdW50ID4gMSkge1xyXG4gICAgICAgICAgZmxvYXQzMiBtaW5Db3N0ID0gYjJfbWF4RmxvYXQ7XHJcbiAgICAgICAgICBpbnQzMiBpTWluID0gLTEsIGpNaW4gPSAtMTtcclxuICAgICAgICAgIGZvciAoaW50MzIgaSA9IDA7IGkgPCBjb3VudDsgKytpKSB7XHJcbiAgICAgICAgICAgIGIyQUFCQiBhYWJiaSA9IG1fbm9kZXNbbm9kZXNbaV1dLmFhYmI7XHJcblxyXG4gICAgICAgICAgICBmb3IgKGludDMyIGogPSBpICsgMTsgaiA8IGNvdW50OyArK2opIHtcclxuICAgICAgICAgICAgICBiMkFBQkIgYWFiYmogPSBtX25vZGVzW25vZGVzW2pdXS5hYWJiO1xyXG4gICAgICAgICAgICAgIGIyQUFCQiBiO1xyXG4gICAgICAgICAgICAgIGIuQ29tYmluZShhYWJiaSwgYWFiYmopO1xyXG4gICAgICAgICAgICAgIGZsb2F0MzIgY29zdCA9IGIuR2V0UGVyaW1ldGVyKCk7XHJcbiAgICAgICAgICAgICAgaWYgKGNvc3QgPCBtaW5Db3N0KSB7XHJcbiAgICAgICAgICAgICAgICBpTWluID0gaTtcclxuICAgICAgICAgICAgICAgIGpNaW4gPSBqO1xyXG4gICAgICAgICAgICAgICAgbWluQ29zdCA9IGNvc3Q7XHJcbiAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgaW50MzIgaW5kZXgxID0gbm9kZXNbaU1pbl07XHJcbiAgICAgICAgICBpbnQzMiBpbmRleDIgPSBub2Rlc1tqTWluXTtcclxuICAgICAgICAgIGIyVHJlZU5vZGU8VD4qIGNoaWxkMSA9IG1fbm9kZXMgKyBpbmRleDE7XHJcbiAgICAgICAgICBiMlRyZWVOb2RlPFQ+KiBjaGlsZDIgPSBtX25vZGVzICsgaW5kZXgyO1xyXG5cclxuICAgICAgICAgIGludDMyIHBhcmVudEluZGV4ID0gQWxsb2NhdGVOb2RlKCk7XHJcbiAgICAgICAgICBiMlRyZWVOb2RlPFQ+KiBwYXJlbnQgPSBtX25vZGVzICsgcGFyZW50SW5kZXg7XHJcbiAgICAgICAgICBwYXJlbnQuY2hpbGQxID0gaW5kZXgxO1xyXG4gICAgICAgICAgcGFyZW50LmNoaWxkMiA9IGluZGV4MjtcclxuICAgICAgICAgIHBhcmVudC5oZWlnaHQgPSAxICsgYjJNYXgoY2hpbGQxLmhlaWdodCwgY2hpbGQyLmhlaWdodCk7XHJcbiAgICAgICAgICBwYXJlbnQuYWFiYi5Db21iaW5lKGNoaWxkMS5hYWJiLCBjaGlsZDIuYWFiYik7XHJcbiAgICAgICAgICBwYXJlbnQucGFyZW50ID0gYjJfbnVsbE5vZGU7XHJcblxyXG4gICAgICAgICAgY2hpbGQxLnBhcmVudCA9IHBhcmVudEluZGV4O1xyXG4gICAgICAgICAgY2hpbGQyLnBhcmVudCA9IHBhcmVudEluZGV4O1xyXG5cclxuICAgICAgICAgIG5vZGVzW2pNaW5dID0gbm9kZXNbY291bnQtMV07XHJcbiAgICAgICAgICBub2Rlc1tpTWluXSA9IHBhcmVudEluZGV4O1xyXG4gICAgICAgICAgLS1jb3VudDtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIG1fcm9vdCA9IG5vZGVzWzBdO1xyXG4gICAgICAgIGIyRnJlZShub2Rlcyk7XHJcbiAgICAgICAgKi9cclxuXHJcbiAgICB0aGlzLlZhbGlkYXRlKCk7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTaGlmdE9yaWdpbk5vZGU8VD4obm9kZTogYjJUcmVlTm9kZTxUPiB8IG51bGwsIG5ld09yaWdpbjogWFkpOiB2b2lkIHtcclxuICAgIGlmIChub2RlID09PSBudWxsKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAobm9kZS5oZWlnaHQgPD0gMSkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCghbm9kZS5Jc0xlYWYoKSk7XHJcblxyXG4gICAgY29uc3QgY2hpbGQxOiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IG5vZGUuY2hpbGQxO1xyXG4gICAgY29uc3QgY2hpbGQyOiBiMlRyZWVOb2RlPFQ+IHwgbnVsbCA9IG5vZGUuY2hpbGQyO1xyXG4gICAgYjJEeW5hbWljVHJlZS5TaGlmdE9yaWdpbk5vZGUoY2hpbGQxLCBuZXdPcmlnaW4pO1xyXG4gICAgYjJEeW5hbWljVHJlZS5TaGlmdE9yaWdpbk5vZGUoY2hpbGQyLCBuZXdPcmlnaW4pO1xyXG5cclxuICAgIG5vZGUuYWFiYi5sb3dlckJvdW5kLlNlbGZTdWIobmV3T3JpZ2luKTtcclxuICAgIG5vZGUuYWFiYi51cHBlckJvdW5kLlNlbGZTdWIobmV3T3JpZ2luKTtcclxuICB9XHJcblxyXG4gIFNoaWZ0T3JpZ2luKG5ld09yaWdpbjogWFkpOiB2b2lkIHtcclxuICAgIGIyRHluYW1pY1RyZWUuU2hpZnRPcmlnaW5Ob2RlKHRoaXMubV9yb290LCBuZXdPcmlnaW4pO1xyXG5cclxuICAgIC8qXHJcbiAgICAgICAgLy8gQnVpbGQgYXJyYXkgb2YgbGVhdmVzLiBGcmVlIHRoZSByZXN0LlxyXG4gICAgICAgIGZvciAoaW50MzIgaSA9IDA7IGkgPCBtX25vZGVDYXBhY2l0eTsgKytpKSB7XHJcbiAgICAgICAgICBtX25vZGVzW2ldLmFhYmIubG93ZXJCb3VuZCAtPSBuZXdPcmlnaW47XHJcbiAgICAgICAgICBtX25vZGVzW2ldLmFhYmIudXBwZXJCb3VuZCAtPSBuZXdPcmlnaW47XHJcbiAgICAgICAgfVxyXG4gICAgICAgICovXHJcbiAgfVxyXG59XHJcbiJdfQ==