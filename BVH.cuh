#pragma once

#ifndef __BVH__
#define __BVH__

#include "Intersection.cuh"

#define StackSize 64
#define sizeMul 6
#define RestructStackSize 32
#define Ci 1.2
#define TRBVHIterations 7


//mul array
__device__ int const mul[] = {10, 100, 1000, 10000, 100000, 1000000};

template <typename BVHNodeType>
__device__ void computeNodeBB(BVHNodeType *node) {
    float3 lmin, lmax, rmin, rmax;
    lmin = node->lchild->min;
    lmax = node->lchild->max;

    rmin = node->rchild->min;
    rmax = node->rchild->max;

    node->min = fminf(lmin, rmin);
    node->max = fmaxf(lmax, rmax);
}

template <typename BVHNodeType>
__device__ void computeNodeBB(BVHNodeType *node, float3 *rMin, float3 *rMax) {
    float3 lmin, lmax, rmin, rmax;
    lmin = node->lchild->min;
    lmax = node->lchild->max;

    rmin = node->rchild->min;
    rmax = node->rchild->max;

    rmin = fminf(lmin, rmin);
    rmax = fmaxf(lmax, rmax);

    node->min = rmin;
    node->max = rmax;

    *rMin = rmin;
    *rMax = rmax;
}

__device__ float getArea(float3 min, float3 max) {
    float3 len = max - min;
    float dx = len.x;
    float dy = len.y;
    float dz = len.z;

    return 2 * (dx * dy + dx * dz + dy * dz);
}

template <typename BVHNodeType>
__device__ float getTotalArea(BVHNodeType **leaves, int nLeaves, unsigned char s) {
    float3 lmin, lmax, min = make_float3(FLT_MAX), max = make_float3(-FLT_MAX);
    
    for(int i = 0; i < nLeaves; i++) {
        if((s >> i) & 1 == 1) {
            lmin = leaves[i]->min;
            lmax = leaves[i]->max;

            min = fminf(min, lmin);
            max = fmaxf(max, lmax);           
        }
    }

    return getArea(min, max);
}

template <typename BVHNodeType>
__device__ void propagateAreaCost(BVHNodeType *root, BVHNodeType **leaves, int numLeaves, 
                                  BVHNodeType *bvh, float *areaVector, float *costVector) {
    BVHNodeType *cur;
    float3 min, max;
    float area, costL, costR;
    int currIndex, lChildIndex, rChildIndex;

    for(int i = 0; i < numLeaves; i++) {
        cur = leaves[i];
        cur = cur->parent;

        while(cur != nullptr && cur->parent != nullptr) {
            currIndex = cur - bvh;
            lChildIndex = cur->lchild - bvh;
            rChildIndex = cur->rchild - bvh;

            if(costVector[currIndex] == 0.0) {
                if((costL = costVector[lChildIndex]) != 0.0 && (costR = costVector[rChildIndex]) != 0.0) {
                    computeNodeBB(cur, &min, &max);
                    
                    area = getArea(min, max);
                    areaVector[currIndex] = area;

                    costVector[currIndex] = Ci * area + costL + costR;

                } else {
                    // Only one side propagated
                    break;
                }
            }
            cur = cur->parent;
        }
    }

    // Propagate root
    computeNodeBB(root, &min, &max);
              
    currIndex = root - bvh;
    lChildIndex = root->lchild - bvh;
    rChildIndex = root->rchild - bvh;

    costL = costVector[lChildIndex];
    costR = costVector[rChildIndex];

    area = getArea(min, max);
    areaVector[currIndex] = area;
    costVector[currIndex] = Ci * area + costL + costR;
}

/*
 * Traverse BVH and find nearest intersection
 */
template <typename BVHNodeType>
__device__ bool traverseHybridBVH(BVHNodeType *bvh, uint bvhSize, Ray ray, RayIntersection *minIntersect, 
                                  int &rayHairIntersections) {
    float distance;
    bool intersectionFound = false;
   
    RayIntersection curr = *minIntersect;

    BVHNodeType *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    BVHNodeType *childL, *childR, *node = &bvh[0], tmp;

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
                lIntersection = AABBIntersection(ray, tmp.min, tmp.max, distance);
            } else {
                lIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation, distance);
            }

            if (lIntersection && distance < minIntersect->distance) {
                // Leaf node
                if (childL->shape != nullptr) {
                    rayHairIntersections++;
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
                rIntersection = AABBIntersection(ray, tmp.min, tmp.max, distance);
            } else {
                rIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation, distance);
            }

            if (rIntersection && distance < minIntersect->distance) {
                // Leaf node
                if (childR->shape != nullptr) {
                    rayHairIntersections++;
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

template <typename BVHNodeType>
__device__ bool traverse(BVHNodeType *bvh, uint bvhSize, Ray ray, RayIntersection *minIntersect, 
                         int &rayHairIntersections) {
    float distance;
    bool intersectionFound = false;
   
    RayIntersection curr = *minIntersect;

    BVHNodeType *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    BVHNodeType *childL, *childR, *node = &bvh[0], tmp;

    tmp = *node;
    
    intersectionFound = AABBIntersection(ray, tmp.min, tmp.max);
    
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
            
            lIntersection = AABBIntersection(ray, tmp.min, tmp.max, distance);
            
            if (lIntersection && distance < minIntersect->distance) {
                // Leaf node
                if (childL->shape != nullptr) {
                    rayHairIntersections++;
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
            
            rIntersection = AABBIntersection(ray, tmp.min, tmp.max, distance);
            
            if (rIntersection && distance < minIntersect->distance) {
                // Leaf node
                if (childR->shape != nullptr) {
                    rayHairIntersections++;
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

template <typename BVHNodeType>
__device__ bool traverseShadowHybridBVH(BVHNodeType *bvh, uint bvhSize, Ray ray) {
    bool intersectionFound = false;

    BVHNodeType *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    BVHNodeType *childL, *childR, *node = &bvh[0], tmp;

    tmp = *node;
    if(tmp.type == AABB) {
        intersectionFound = AABBIntersection(ray, tmp.min, tmp.max);
    } else {
        intersectionFound = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation);
    }
    
    if(!intersectionFound) {
        return false;
    }

    /*if (node->shape != nullptr) {
        return intersection(ray, nullptr, node->shape);
    }*/

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

template <typename BVHNodeType>
__device__ bool traverseShadow(BVHNodeType *bvh, uint bvhSize, Ray ray) {
    bool intersectionFound = false;

    BVHNodeType *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    BVHNodeType *childL, *childR, *node = &bvh[0], tmp;

    tmp = *node;
    
    intersectionFound = AABBIntersection(ray, tmp.min, tmp.max);
    
    if(!intersectionFound) {
        return false;
    }

    /*if (node->shape != nullptr) {
        return intersection(ray, nullptr, node->shape);
    }*/

    bool lIntersection, rIntersection, traverseL, traverseR; 
    while(node != nullptr) {
        lIntersection = rIntersection = traverseL = traverseR = false;

        childL = node->lchild;
        if(childL != nullptr) {
            tmp = *childL;
                
            lIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            
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
            
            rIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            
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

__device__ 
int longestCommonPrefix(int i, int j, uint nObjects, uint *mortonCodes) {
    if (j >= 0 && j < nObjects) {
        size_t mci = mortonCodes[i];
        size_t mcj = mortonCodes[j];

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

template <typename BVHNodeType>
__device__ void restructTree(BVHNodeType *parent, BVHNodeType **leaves, BVHNodeType **nodes, 
                             unsigned char partition, unsigned char *optimal, int &index, bool left, int numLeaves,
                             BVHNodeType *bvh, float *areaVector, float *costVector) {

    PartitionEntry<BVHNodeType> stack[RestructStackSize];
    int topIndex = RestructStackSize, tmpNodeIndex;
    stack[--topIndex] = PartitionEntry<BVHNodeType>(partition, left, parent);
    BVHNodeType *tmpNode;
    unsigned char leftPartition, rightPartition;

    // Do while stack is not empty
    while(topIndex != RestructStackSize) {
        PartitionEntry<BVHNodeType> *pe = &stack[topIndex++];
        partition = pe->partition;
        left = pe->left;
        parent = pe->parent;

        //process leaf
        if(__popc(partition) == 1) {
            int leafIndex = __ffs(partition) - 1;

            tmpNode = leaves[leafIndex];
            if(left) {
                parent->lchild = tmpNode;
            } else {
                parent->rchild = tmpNode;
            }
            tmpNode->parent = parent;


        } else {
            // process internal node

            //debug
            /*if (index >= TRBVHIterations || index < 0) {
                printf("index out of range\n");
                return;
            }*/

            tmpNode = nodes[index++];

            // Mark node cost with 0
            tmpNodeIndex = tmpNode - bvh; 
            costVector[tmpNodeIndex] = 0.0f;

            if(left) {
                parent->lchild = tmpNode;
            } else {
                parent->rchild = tmpNode;
            }
            tmpNode->parent = parent;

            //debug
            /*if (partition >= 128) {
                printf("partition out of range\n");
                return;
            }*/

            leftPartition = optimal[partition];
            rightPartition = (~leftPartition) & partition;

            //debug
            /*if (topIndex < 2) {
                printf("restructTree stack not big enough. Increase RestructStackSize!\n");
            }*/

            stack[--topIndex] = PartitionEntry<BVHNodeType>(leftPartition, true, tmpNode);
            stack[--topIndex] = PartitionEntry<BVHNodeType>(rightPartition, false, tmpNode);
        }
    }

    propagateAreaCost(parent, leaves, numLeaves, bvh, areaVector, costVector);
}

template <typename BVHNodeType>
__device__ void calculateOptimalTreelet(BVHNodeType **leaves, int nLeaves, unsigned char *p_opt,
                                        BVHNodeType *bvh, float *areaVector, float *costVector) {
    int const numSubsets = (1 << nLeaves) - 1;

    float a[128];
    float c_opt[128];

    // Calculate surface area for each subset
    for (unsigned char s = 1; s <= numSubsets; s++) {
        a[s] = getTotalArea(leaves, nLeaves, s);
    }

    // Initialize costs of individual leaves
    int index;
    for (int i = 0; i < nLeaves; i++) {
        index = leaves[i] - bvh;
        c_opt[(1 << i)] = costVector[index];
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

template <typename BVHNodeType>
__device__ void optimizeTreelet(BVHNodeType *treeletRoot, BVHNodeType *bvh, float *areaVector, float *costVector) {
    if (treeletRoot == nullptr || treeletRoot->shape != nullptr) {
        return;
    }

    BVHNodeType *leaves[TRBVHIterations];
    BVHNodeType *nodes[TRBVHIterations - 2];
    unsigned char optimal[128];

    int nodeIndex;
    int counter = 0;
    int nodeCounter = 0;
    float maxArea;
    int maxIndex = 0;
    leaves[counter++] = treeletRoot->lchild;
    leaves[counter++] = treeletRoot->rchild;

    BVHNodeType *tmp;
    while (counter < TRBVHIterations && maxIndex != -1) {
        maxIndex = -1;
        maxArea = -1.0f;

        for (int i = 0; i < counter; i++) {
            if (!(leaves[i]->shape != nullptr)) {
                nodeIndex = leaves[i] - bvh;
                float area = areaVector[nodeIndex];
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

    calculateOptimalTreelet(leaves, counter, optimal, bvh, areaVector, costVector);


    unsigned char mask = (1 << counter) - 1;    
    int index = 0;                              
    unsigned char leftIndex = mask;
    unsigned char left = optimal[leftIndex];

    restructTree(treeletRoot, leaves, nodes, left, optimal, index, true, counter, bvh, areaVector, costVector);

    unsigned char right = (~left) & mask;
    restructTree(treeletRoot, leaves, nodes, right, optimal, index, false, counter, bvh, areaVector, costVector);

    float3 rMin, rMax;
    computeNodeBB(treeletRoot, &rMin, &rMax);

    float rArea = getArea(rMin, rMax);

    float costL = costVector[treeletRoot->lchild - bvh];

    float costR = costVector[treeletRoot->rchild - bvh];
    
    nodeIndex = treeletRoot - bvh;
    areaVector[nodeIndex] = rArea;
    costVector[nodeIndex] = Ci * rArea + costL + costR;
}

template <typename BVHNodeType>
__global__ void buildBVH(BVHNodeType *bvh, uint nObjects, uint *mortonCodes) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= nObjects - 1) {
        return;
    }
    
    BVHNodeType *bvhLeaves = &bvh[nObjects - 1];

    // Determine direction of the range (+1 or -1)
    int sign = longestCommonPrefix(i, i + 1, nObjects, mortonCodes) - longestCommonPrefix(i, i - 1, nObjects, mortonCodes); 
    
    int d = sign > 0 ? 1 : -1;
    
    // Compute upper bound for the length of the range
    int sigMin = longestCommonPrefix(i, i - d, nObjects, mortonCodes);
    int lmax = 2;

    while (longestCommonPrefix(i, i + lmax * d, nObjects, mortonCodes) > sigMin) {
        lmax *= 2;
    }

    // Find the other end using binary search
    int l = 0;
    float divider = 2.0f;
    for (int t = lmax / divider; t >= 1.0f; divider *= 2.0f) {
        if (longestCommonPrefix(i, i + (l + t) * d, nObjects, mortonCodes) > sigMin) {
            l += t;
        }
        t = lmax / divider;
    }
  
    int j = i + l * d;
  
    // Find the split position using binary search
    int sigNode = longestCommonPrefix(i, j, nObjects, mortonCodes);
    int s = 0;

    divider = 2.0f;
    for (int t = ceilf(l / divider); t >= 1.0f; divider *= 2.0f) {
        if (longestCommonPrefix(i, i + (s + t) * d, nObjects, mortonCodes) > sigNode) {
            s += t;
        }
        t = ceilf(l / divider);
    }

    int gamma = i + s * d + imin(d, 0);

    // Output child pointers
    BVHNodeType *current = &bvh[i];

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


__global__ void computeBVHBB(CylinderNode *bvh, uint nObjects, int *lock, Cylinder *d_shapes,
                             Matrix *d_matrixes, float3 *d_translations, uint *d_OBBIndexes, uint nOBBs) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    CylinderNode *bvhLeaves = &bvh[nObjects - 1];

    if(i < nObjects) {
        uint auxIndex;
        bvhLeaves[i].shape = &d_shapes[i];

        if(i < nOBBs && d_OBBIndexes != nullptr) {
            auxIndex = d_OBBIndexes[i];
            bvhLeaves[auxIndex].matrix = &d_matrixes[i];
            bvhLeaves[auxIndex].translation = &d_translations[i];
        }
    }

    if (i > nObjects - 1) {
        return;
    }

    
    CylinderNode *node = &bvhLeaves[i];
    node = node->parent;
    int index = node - bvh;
    int oldLock = atomicAdd(&lock[index], 1);
    while(1) {
        if(oldLock == 0) {
            return;
        }

        computeNodeBB(node);

        //if root
        if(node->parent == nullptr) {
            return;
        }

        node = node->parent;
        index = node - bvh;
        oldLock = atomicAdd(&lock[index], 1);
    }
}

template <typename BVHNodeType, typename ShapeType>
__global__ void computeBVHBB(BVHNodeType *bvh, uint nObjects, int *lock, ShapeType *d_shapes) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    BVHNodeType *bvhLeaves = &bvh[nObjects - 1];

    if(i < nObjects) {
        bvhLeaves[i].shape = &d_shapes[i];
    }

    if (i > nObjects - 1) {
        return;
    }

    
    BVHNodeType *node = &bvhLeaves[i];
    node = node->parent;
    int index = node - bvh;
    int oldLock = atomicAdd(&lock[index], 1);
    while(1) {
        if(oldLock == 0) {
            return;
        }

        computeNodeBB(node);

        //if root
        if(node->parent == nullptr) {
            return;
        }

        node = node->parent;
        index = node - bvh;
        oldLock = atomicAdd(&lock[index], 1);
    }
}

__global__ void computeLeavesOBBs(CylinderNode *bvh, uint nObjects) {    
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i > nObjects - 1) {
        return;
    }

    CylinderNode *bvhLeaves = &bvh[nObjects - 1];
    CylinderNode *node = &bvhLeaves[i];

    if(node->shape != nullptr && node->type == OBB) {
        float radius = node->shape->radius;
        node->max = make_float3(radius, radius, length(node->shape->top - node->shape->base));
        node->min = make_float3(-radius, -radius, 0);
    }
}

template <typename BVHNodeType>
__global__ void optimizeBVH(BVHNodeType *bvh, uint nObjects, int *nodeCounter, float *areaVector, float *costVector) {
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (index >= nObjects) {
        return;
    }

    BVHNodeType *bvhLeaves = &bvh[nObjects - 1];
    BVHNodeType *leaf = &bvhLeaves[index];
    int currentIndex;

    float area = getArea(leaf->min, leaf->max);

    currentIndex = leaf - bvh;
    areaVector[currentIndex] = area;
    costVector[currentIndex] = area;

    BVHNodeType *current = leaf->parent;
    currentIndex = current - bvh;

    int res = atomicAdd(&nodeCounter[currentIndex], 1);

    // internal nodes
    while (1) {
        if (res == 0) {
            return;
        }

        optimizeTreelet(current, bvh, areaVector, costVector);

        // If root
        if (current == nullptr || current->parent == nullptr) {
            return;
        }

        current = current->parent;
        currentIndex = current - bvh;

        res = atomicAdd(&nodeCounter[currentIndex], 1);
    }
}

#endif;
