
#include "Intersection.cuh"

#define StackSize 32

__device__
void traverse(CylinderNode **list, CylinderNode *bvh, long bvhSize, Ray ray) {
    bool intersection = false;
    int stackNodes[StackSize];
    int stackChildren[StackSize];

    int *stackPtr = stackNodes;
    int *stackChildPtr = stackChildren;

    *stackPtr++ = *stackChildPtr++ = -1;
    *stackPtr = 0;

    int listIndex = 0;
    CylinderNode *node;
    int stackNodeIndex;

    while((stackNodeIndex = *stackPtr) != -1) {
        node = &bvh[stackNodeIndex];
        if(node->type == AABB) {
            intersection = AABBIntersection(ray, node->min, node->max);
        } else {
            intersection = OBBIntersection(ray, node->min, node->max, node->matrix, node->translation);
        }

        if (intersection) {
            --stackPtr;
            // Leaf node
            if (node->shape != nullptr) {
                list[listIndex++] = node;
                
            // Internal node
            } else {
                *stackChildPtr++ = 2 * stackNodeIndex + 1;
                *stackChildPtr++ = 2 * stackNodeIndex + 2;
            }

            if(*stackPtr == -1) {
                int *tmp = stackPtr;

                stackPtr = stackChildPtr;
                stackChildPtr = tmp;
            }
        }
    }
}