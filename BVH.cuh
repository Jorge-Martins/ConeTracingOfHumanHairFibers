#pragma once

#ifndef __BVH__
#define __BVH__

#include "Intersection.cuh"

#define StackSize 32
#define sizeMul 6

//mul array
__device__ int const mul[] = {10, 100, 1000, 10000, 100000, 1000000};

__device__
void traverse(CylinderNode **list, CylinderNode *bvh, uint bvhSize, Ray ray) {
    bool intersection = false;
    CylinderNode *stackNodes[StackSize];
    CylinderNode *stackChildren[StackSize];

    CylinderNode **stackNodesPtr = stackNodes;
    CylinderNode **stackChildrenPtr = stackChildren;

    uint stackIndex = 0;
    uint stackCIndex = 0;
    uint listIndex = 0;

    stackNodes[stackIndex] = stackChildren[stackCIndex] = nullptr;
    
    stackNodes[++stackIndex] = &bvh[0];

    
    CylinderNode *node;

    while((node = stackNodesPtr[stackIndex]) != nullptr) {
        
        if(node->type == AABB) {
            intersection = AABBIntersection(ray, node->min, node->max);
        } else {
            intersection = OBBIntersection(ray, node->min, node->max, node->matrix, node->translation);
        }

        if (intersection) {
            stackIndex--;
            // Leaf node
            if (node->shape != nullptr) {
                list[listIndex++] = node;
                
            // Internal node
            } else {
                stackChildrenPtr[++stackCIndex] = node->lchild;
                stackChildrenPtr[++stackCIndex] = node->rchild;
            }

            if(stackNodes[stackIndex] == nullptr) {
                CylinderNode **tmp;
                int tmpIndex;

                tmp = stackNodesPtr;
                stackNodesPtr = stackChildrenPtr;
                stackChildrenPtr = tmp;

                tmpIndex = stackIndex;
                stackIndex = stackCIndex;
                stackCIndex = tmpIndex;
            }
        }
    }
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
        }

        return __clz(mci ^ mcj);

    } else {
        return -1;
    }
}


//__global__ void buildBVH(CylinderNode *bvh, uint nObjects) {
//    uint size;
//    int i = blockIdx.x * blockDim.x + threadIdx.x;
//
//    if (i >= nObjects) {
//        return;
//    }
//
//    // Run radix tree construction algorithm
//    // Determine direction of the range (+1 or -1)
//    size = nObjects+1;
//    int d = longestCommonPrefix(i, i + 1, size) - longestCommonPrefix(i, i - 1, size) > 0 ? 1 : -1;
//    
//    // Compute upper bound for the length of the range
//    int sigMin = longestCommonPrefix(i, i - d, size);
//    int lmax = 2;
//
//    while (longestCommonPrefix(i, i + lmax * d, size) > sigMin) {
//        lmax *= 2;
//    }
//
//    // Find the other end using binary search
//    int l = 0;
//    int divider = 2;
//    for (int t = lmax / divider; t >= 1; divider *= 2) {
//        if (longestCommonPrefix(i, i + (l + t) * d, size) > sigMin) {
//            l += t;
//        }
//        t = lmax / divider;
//    }
//  
//    int j = i + l * d;
//  
//    // Find the split position using binary search
//    int sigNode = longestCommonPrefix(i, j, size);
//    int s = 0;
//    divider = 2;
//    for (int t = (l + (divider - 1)) / divider; t >= 1; divider *= 2) {
//        if (longestCommonPrefix(i, i + (s + t) * d, size) > sigNode) {
//            s = s + t;
//        }
//        t = (l + (divider - 1)) / divider;
//    }
//
//    int gamma = i + s * d + intMin(d, 0);
//
//    // Output child pointers
//    TreeNode *current = radixTreeNodes + i;
//
//
//    if (intMin(i, j) == gamma) {
//        current->left = radixTreeLeaves + gamma;
//        (radixTreeLeaves + gamma)->parent = current;
//    } else {
//        current->left = radixTreeNodes + gamma;
//        (radixTreeNodes + gamma)->parent = current;
//    }
//
//    if (intMax(i, j) == gamma + 1) {
//        current->right = radixTreeLeaves + gamma + 1;
//        (radixTreeLeaves + gamma + 1)->parent = current;
//    } else {
//        current->right = radixTreeNodes + gamma + 1;
//        (radixTreeNodes + gamma + 1)->parent = current;
//    }
//
//    current->min = intMin(i, j);
//    current->max = intMax(i, j);
//}


#endif;
