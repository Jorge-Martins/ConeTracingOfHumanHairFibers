#pragma once

#ifndef __BVH__
#define __BVH__

#include "Intersection.cuh"

#define StackSize 64
#define sizeMul 6

//mul array
__device__ int const mul[] = {10, 100, 1000, 10000, 100000, 1000000};

__device__
bool traverse(CylinderNode *bvh, uint bvhSize, Ray ray, RayIntersection *minIntersect) {
    bool intersectionFound = false;
    
    int nBBIntersected = 0;
    int nShapesIntersected = 0;

    RayIntersection curr = *minIntersect;

    CylinderNode *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    CylinderNode *childL, *childR, *node = &bvh[0];

    if(node->type == AABB) {
        intersectionFound = AABBIntersection(ray, node->min, node->max);
    } else {
        intersectionFound = OBBIntersection(ray, node->min, node->max, node->matrix, node->translation);
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
            nBBIntersected++;
            if(childL->type == AABB) {
                lIntersection = AABBIntersection(ray, childL->min, childL->max);
            } else {
                lIntersection = OBBIntersection(ray, childL->min, childL->max, childL->matrix, childL->translation);
            }

            if (lIntersection) {
                // Leaf node
                if (childL->shape != nullptr) {
                    nShapesIntersected++;
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
            nBBIntersected++;
            if(childR->type == AABB) {
                rIntersection = AABBIntersection(ray, childR->min, childR->max);
            } else {
                rIntersection = OBBIntersection(ray, childR->min, childR->max, childR->matrix, childR->translation);
            }

            if (rIntersection) {
                // Leaf node
                if (childR->shape != nullptr) {
                    nShapesIntersected++;
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
    
    int nBBIntersected = 0;
    int nShapesIntersected = 0;

    CylinderNode *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    CylinderNode *childL, *childR, *node = &bvh[0];

    if(node->type == AABB) {
        intersectionFound = AABBIntersection(ray, node->min, node->max);
    } else {
        intersectionFound = OBBIntersection(ray, node->min, node->max, node->matrix, node->translation);
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
            nBBIntersected++;
            if(childL->type == AABB) {
                lIntersection = AABBIntersection(ray, childL->min, childL->max);
            } else {
                lIntersection = OBBIntersection(ray, childL->min, childL->max, childL->matrix, childL->translation);
            }

            if (lIntersection) {
                // Leaf node
                if (childL->shape != nullptr) {
                    nShapesIntersected++;
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
            nBBIntersected++;
            if(childR->type == AABB) {
                rIntersection = AABBIntersection(ray, childR->min, childR->max);
            } else {
                rIntersection = OBBIntersection(ray, childR->min, childR->max, childR->matrix, childR->translation);
            }

            if (rIntersection) {
                // Leaf node
                if (childR->shape != nullptr) {
                    nShapesIntersected++;
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

    return result;
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

        node->min = fminf(node->lchild->min, node->rchild->min);
        node->max = fmaxf(node->lchild->max, node->rchild->max);

        //if root
        if(node->parent == nullptr) {
            return;
        }

        node = node->parent;
        oldLock = atomicAdd(&node->lock, 1);
    }
}


#endif;
