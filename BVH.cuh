#pragma once

#ifndef __BVH__
#define __BVH__

#include "Intersection.cuh"

#define StackSize 32

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

inline __device__ int longestCommonPrefix(int i, int j, uint nObjects, CylinderNode *bvh) {
    if (j >= 0 && j < nObjects) {
        return __clz(bvh[i].mortonCode ^ bvh[j].mortonCode);
    } else {
        return -1;
    }
}


#endif;
