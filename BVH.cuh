#pragma once

#ifndef __BVH__
#define __BVH__

#include "Intersection.cuh"

#define StackSize 64
#define sizeMul 6
#define RestructStackSize 4
#define Ci 1.2


//mul array
__device__ int const mul[] = {10, 100, 1000, 10000, 100000, 1000000};

__device__ float getArea(float3 min, float3 max) {
    float3 len = max - min;
    float dx = len.x;
    float dy = len.y;
    float dz = len.z;

    return 2 * (dx * dy + dx * dz + dy * dz);
}

__device__ float getTotalArea(CylinderNode **leaves, int nLeaves, unsigned char s) {
    float3 lmin, lmax, min = make_float3(FLT_MAX), max = make_float3(-FLT_MAX);
    
    for (int i = 0; i < nLeaves; i++) {
        if ((s >> i) & 1 == 1) {
            lmin = leaves[i]->min;
            lmax = leaves[i]->max;

            min = fminf(min, lmin);
            max = fmaxf(max, lmax);           
        }
    }

    return getArea(min, max);
}

__device__ void propagateAreaCost(CylinderNode *root, CylinderNode **leaves, int numLeaves) {
    CylinderNode *cur;
    CylinderNode lChild, rChild;
    float3 min, max;
    float area;

    for (int i = 0; i < numLeaves; i++) {
        cur = leaves[i];
        cur = cur->parent;

        while (cur != nullptr && cur->parent != nullptr) {
            if (cur->cost == 0.0) {
                if (cur->lchild->cost != 0.0 && cur->rchild->cost != 0.0) {
                    lChild = *cur->lchild;
                    rChild = *cur->rchild;

                    min = fminf(lChild.min, rChild.min);
                    max = fminf(lChild.max, rChild.max);

                    cur->min = min;
                    cur->max = max;
                    
                    area = getArea(min, max);
                    cur->area = area;

                    cur->cost = Ci * area + lChild.cost + rChild.cost;

                } else {
                    // Only one side propagated
                    break;
                }
            }
            cur = cur->parent;
        }
    }

    // Propagate root
    lChild = *root->lchild;
    rChild = *root->rchild;

    min = fminf(lChild.min, rChild.min);
    max = fminf(lChild.max, rChild.max);

    root->min = min;
    root->max = max;
                    
    area = getArea(min, max);
    root->area = area;
    root->cost = Ci * area + lChild.cost + rChild.cost;
}

__device__
bool traverse(CylinderNode *bvh, uint bvhSize, Ray ray, RayIntersection *minIntersect, int *rayHairIntersections) {
    float distance;
    bool intersectionFound = false;
   
    RayIntersection curr = *minIntersect;

    CylinderNode *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    CylinderNode *childL, *childR, *node = &bvh[0], tmp;

    tmp = *node;
    if(tmp.type == AABB) {
        intersectionFound = AABBIntersection(ray, tmp.min, tmp.max);
    } else {
        intersectionFound = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation);
    }
    
    if(!intersectionFound) {
        return false;
    }

    bool result = false;
    bool lIntersection, rIntersection, traverseL, traverseR; 
    while(node != nullptr) {
        lIntersection = rIntersection = traverseL = traverseR = false;

        childL = node->lchild;
        if(childL != nullptr) {
            tmp = *childL;
            if(tmp.type == AABB) {
                lIntersection = AABBIntersection(ray, tmp.min, tmp.max, &distance);
            } else {
                lIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation, &distance);
            }

            if (lIntersection && distance < minIntersect->distance) {
                // Leaf node
                if (childL->shape != nullptr) {
                    (*rayHairIntersections)++;
                    intersectionFound = intersection(ray, &curr, childL->shape);

                    if(intersectionFound && (curr.distance < minIntersect->distance)) {
                        result = true;
                        *minIntersect = curr;
                    }

                } else {
                    traverseL = true;
                }
            }
        }

        childR = node->rchild;
        if(childR != nullptr) {
            tmp = *childR;
            if(tmp.type == AABB) {
                rIntersection = AABBIntersection(ray, tmp.min, tmp.max, &distance);
            } else {
                rIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation, &distance);
            }

            if (rIntersection && distance < minIntersect->distance) {
                // Leaf node
                if (childR->shape != nullptr) {
                    (*rayHairIntersections)++;
                    intersectionFound = intersection(ray, &curr, childR->shape);

                    if(intersectionFound && (curr.distance < minIntersect->distance)) {
                        result = true;
                        *minIntersect = curr;
                    }

                } else {
                    traverseR = true;
                }
            }
        }

        
        if (!traverseL && !traverseR) {
            node = stackNodes[--stackIndex]; // pop

        } else {
            node = (traverseL) ? childL : childR;
            if (traverseL && traverseR) {             
                stackNodes[stackIndex++] = childR; // push
            }
        }
    }

    return result;
}

__device__
bool traverseShadow(CylinderNode *bvh, uint bvhSize, Ray ray) {
    bool intersectionFound = false;

    CylinderNode *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    CylinderNode *childL, *childR, *node = &bvh[0], tmp;

    tmp = *node;
    if(tmp.type == AABB) {
        intersectionFound = AABBIntersection(ray, tmp.min, tmp.max);
    } else {
        intersectionFound = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation);
    }
    
    if(!intersectionFound) {
        return false;
    }

    bool lIntersection, rIntersection, traverseL, traverseR; 
    while(node != nullptr) {
        lIntersection = rIntersection = traverseL = traverseR = false;

        childL = node->lchild;
        if(childL != nullptr) {
            tmp = *childL;
            if(tmp.type == AABB) {
                lIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            } else {
                lIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation);
            }

            if (lIntersection) {
                // Leaf node
                if (childL->shape != nullptr) {
                    intersectionFound = intersection(ray, nullptr, childL->shape);

                    if(intersectionFound) {
                        return true;
                    }

                } else {
                    traverseL = true;
                }
            }
        }

        childR = node->rchild;
        if(childR != nullptr) {
            tmp = *childR;
            if(tmp.type == AABB) {
                rIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            } else {
                rIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation);
            }

            if (rIntersection) {
                // Leaf node
                if (childR->shape != nullptr) {
                    intersectionFound = intersection(ray, nullptr, childR->shape);

                    if(intersectionFound) {
                        return true;
                    }

                } else {
                    traverseR = true;
                }
            }
        }

        
        if (!traverseL && !traverseR) {
            node = stackNodes[--stackIndex]; // pop

        } else {
            node = (traverseL) ? childL : childR;
            if (traverseL && traverseR) {
                stackNodes[stackIndex++] = childR; // push
            }
        }
    }

    return false;
}

inline __device__ int longestCommonPrefix(int i, int j, uint nObjects, CylinderNode *bvhLeaves) {
    if (j >= 0 && j < nObjects) {
        size_t mci = bvhLeaves[i].shape->mortonCode;
        size_t mcj = bvhLeaves[j].shape->mortonCode;

        //Concatenate index to key k' = k | index
        if(mci == mcj) {
            int maxIndex = sizeMul - 1;
            int exp = 0;
            if(i > 10) {
                exp = (int)log10f(i);
            }
            
            if(exp < sizeMul) { 
                mci = mci * mul[exp] + i;

            } else {
                mci *= mul[maxIndex];

                exp -= sizeMul;
                while(exp >= 0) {
                    if(exp < sizeMul) {
                        mci *= mul[exp];
                        break;
                    } else {
                        mci *= mul[maxIndex];
                        exp -= sizeMul;
                    }
                }

                mci += i;
            }

            exp = 0;
            if(j > 10) {
                exp = (int)log10f(j);
            }

            if(exp < sizeMul) { 
                mcj = mcj * mul[exp] + j;

            } else {
                mcj *= mul[maxIndex];

                exp -= sizeMul;
                while(exp >= 0) {
                    if(exp < sizeMul) {
                        mcj *= mul[exp];
                        break;
                    } else {
                        mcj *= mul[maxIndex];
                        exp -= sizeMul;
                    }
                }
                
                mcj += j;
            }

            return __clzll(mci ^ mcj);
        }

        return __clz(mci ^ mcj);

    } else {
        return -1;
    }
}

__device__ void restructTree(CylinderNode *parent, CylinderNode **leaves, CylinderNode **nodes, 
                             unsigned char partition, unsigned char *optimal, int &index, bool left, int numLeaves) {

    PartitionEntry stack[RestructStackSize];
    int topIndex = RestructStackSize;
    PartitionEntry tmp = {partition, left, parent};
    stack[--topIndex] = tmp;

    // Do while stack is not empty
    while (topIndex != RestructStackSize) {
        PartitionEntry *pe = &stack[topIndex++];
        partition = pe->partition;
        left = pe->left;
        parent = pe->parent;

        if (__popc(partition) == 1) {
            // Leaf
            int leafIndex = __ffs(partition) - 1;

            CylinderNode *leaf = leaves[leafIndex];
            if (left) {
                parent->lchild = leaf;
            } else {
                parent->rchild = leaf;
            }
            leaf->parent = parent;

        } else {
            // Internal node
            CylinderNode *node = nodes[index++];

            // Set cost to 0 as a mark
            node->cost = 0.0f;

            if (left) {
                parent->lchild = node;
            } else {
                parent->rchild = node;
            }
            node->parent = parent;

            unsigned char leftPartition = optimal[partition];
            unsigned char rightPartition = (~leftPartition) & partition;


            PartitionEntry tmp1 = {leftPartition, true, node};
            stack[--topIndex] = tmp1;
            PartitionEntry tmp2 = {rightPartition, false, node};
            stack[--topIndex] = tmp2;
        }
    }

    propagateAreaCost(parent, leaves, numLeaves);
}

__device__ void calculateOptimalTreelet(CylinderNode **leaves, int nLeaves, unsigned char *p_opt) {
    int const numSubsets = (1 << nLeaves) - 1;

    float a[128];
    float c_opt[128];

    // Calculate surface area for each subset
    for (unsigned char s = 1; s <= numSubsets; s++) {
        a[s] = getTotalArea(leaves, nLeaves, s);
    }

    // Initialize costs of individual leaves
    for (int i = 0; i <= (nLeaves - 1); i++) {
        c_opt[(1 << i)] = leaves[i]->cost;
    }

    unsigned char p_s, d, p;
    float c_s, c;
    // Optimize every subset of leaves
    for (int k = 2; k <= nLeaves; k++) {
        // Try each way of partitioning the leaves
        for (unsigned char s = 1; s <= numSubsets; s++) {
            if (__popc(s) == k) {
                c_s = FLT_MAX;
                p_s = 0;
                d = (s - 1) & s;
                p = (-d) & s;

                while (p != 0) {
                    c = c_opt[p] + c_opt[s ^ p];
                    if (c < c_s) {
                        c_s = c;
                        p_s = p;
                    }
                    
                    p = (p - d) & s;
                }

                // Calculate final SAH cost
                c_opt[s] = Ci * a[s] + c_s;
                p_opt[s] = p_s;
            }
        }
    }
}

__device__ void treeletOptimize(CylinderNode *root) {
    if (root->shape != nullptr) {
        return;
    }

    // Find a treelet with max number of leaves being 7
    CylinderNode *leaves[7];

    // Max 7 (leaves) - 1 (root doesn't count) - 1
    CylinderNode *nodes[5];

    int counter = 0;
    int nodeCounter = 0;
    float maxArea;
    int maxIndex = 0;
    leaves[counter++] = root->lchild;
    leaves[counter++] = root->rchild;

    CylinderNode *tmp;
    while (counter < 7 && maxIndex != -1) {
        maxIndex = -1;
        maxArea = -1.0;

        for (int i = 0; i < counter; i++) {
            if (!(leaves[i]->shape != nullptr)) {
                float area = leaves[i]->area;
                if (area > maxArea) {
                    maxArea = area;
                    maxIndex = i;
                }
            }
        }

        if (maxIndex != -1) {
            tmp = leaves[maxIndex];

            nodes[nodeCounter++] = tmp;

            leaves[maxIndex] = leaves[counter - 1];
            leaves[counter - 1] = tmp->lchild;
            leaves[counter++] = tmp->rchild;
        }
    }

    unsigned char optimal[128];

    // Call calculateOptimalCost here
    calculateOptimalTreelet(leaves, counter, optimal);


    unsigned char mask = (1 << counter) - 1;    
    int index = 0;                              
    unsigned char leftIndex = mask;
    unsigned char left = optimal[leftIndex];

    restructTree(root, leaves, nodes, left, optimal, index, true, counter);

    unsigned char right = (~left) & mask;
    restructTree(root, leaves, nodes, right, optimal, index, false, counter);


    CylinderNode lChild = *root->lchild;
    CylinderNode rChild = *root->rchild;
    float3 rMin = fminf(lChild.min, rChild.min);
    float3 rMax = fminf(lChild.max, rChild.max);
    float rArea = getArea(rMin, rMax);

    root->min = rMin;
    root->max = rMax;
    root->area = rArea;
    root->cost = Ci * rArea + lChild.cost + rChild.cost;
}

__global__ void buildBVH(CylinderNode *bvh, uint nObjects) {
    CylinderNode *bvhLeaves = &bvh[nObjects - 1];
    uint size = nObjects;

    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= nObjects - 1) {
        return;
    }
    
    // Determine direction of the range (+1 or -1)
    int sign = longestCommonPrefix(i, i + 1, size, bvhLeaves) - longestCommonPrefix(i, i - 1, size, bvhLeaves); 
    
    int d = sign > 0 ? 1 : -1;
    
    // Compute upper bound for the length of the range
    int sigMin = longestCommonPrefix(i, i - d, size, bvhLeaves);
    int lmax = 2;

    while (longestCommonPrefix(i, i + lmax * d, size, bvhLeaves) > sigMin) {
        lmax *= 2;
    }

    // Find the other end using binary search
    int l = 0;
    float divider = 2.0f;
    for (int t = lmax / divider; t >= 1.0f; divider *= 2.0f) {
        if (longestCommonPrefix(i, i + (l + t) * d, size, bvhLeaves) > sigMin) {
            l += t;
        }
        t = lmax / divider;
    }
  
    int j = i + l * d;
  
    // Find the split position using binary search
    int sigNode = longestCommonPrefix(i, j, size, bvhLeaves);
    int s = 0;

    divider = 2.0f;
    for (int t = ceilf(l / divider); t >= 1.0f; divider *= 2.0f) {
        if (longestCommonPrefix(i, i + (s + t) * d, size, bvhLeaves) > sigNode) {
            s += t;
        }
        t = ceilf(l / divider);
    }

    int gamma = i + s * d + imin(d, 0);

    // Output child pointers
    CylinderNode *current = &bvh[i];

    if (imin(i, j) == gamma) {
        current->lchild = &bvhLeaves[gamma];
    } else {
        current->lchild = &bvh[gamma];
    }

    if (imax(i, j) == gamma + 1) {
        current->rchild = &bvhLeaves[gamma + 1];
    } else {
        current->rchild = &bvh[gamma + 1];
    }

    current->lchild->parent = current;
    current->rchild->parent = current;
}

__global__ void computeBVHBB(CylinderNode *bvh, uint nObjects) {
    CylinderNode *bvhLeaves = &bvh[nObjects - 1];
    
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i > nObjects - 1) {
        return;
    }
    CylinderNode *node = &bvhLeaves[i];
    node = node->parent;

    int oldLock = atomicAdd(&node->lock, 1);
    while(1) {
        if(oldLock == 0) {
            return;
        }

        CylinderNode lChild = *node->lchild;
        CylinderNode rChild = *node->rchild;
        node->min = fminf(lChild.min, rChild.min);
        node->max = fmaxf(lChild.max, rChild.max);

        //if root
        if(node->parent == nullptr) {
            return;
        }

        node = node->parent;
        oldLock = atomicAdd(&node->lock, 1);
    }
}

__global__ void computeLeavesOBBs(CylinderNode *bvh, uint nObjects) {
    CylinderNode *bvhLeaves = &bvh[nObjects - 1];
    
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i > nObjects - 1) {
        return;
    }

    CylinderNode *node = &bvhLeaves[i];

    if(node->shape != nullptr && node->type == OBB) {
        float radius = node->shape->radius;
        node->max = make_float3(radius, radius, length(node->shape->top - node->shape->base));
        node->min = make_float3(-radius, -radius, 0);
    }
}

__global__ void optimizeBVH(CylinderNode *bvh, uint nObjects, int *nodeCounter) {
    CylinderNode *bvhLeaves = &bvh[nObjects - 1];
    
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (index >= nObjects) {
        return;
    }

    CylinderNode *leaf = &bvhLeaves[index];

    float area = getArea(leaf->min, leaf->max);
    leaf->area = area;
    leaf->cost = area;

    CylinderNode *current = leaf->parent;
    int currentIndex = current - bvh;
    int res = atomicAdd(&nodeCounter[currentIndex], 1);

    // internal nodes
    while (1) {
        if (res == 0) {
            return;
        }

        treeletOptimize(current);

        // If root
        if (current->parent == nullptr) {
            return;
        }

        current = current->parent;
        currentIndex = current - bvh;
        res = atomicAdd(&nodeCounter[currentIndex], 1);
    }
}

#endif;
