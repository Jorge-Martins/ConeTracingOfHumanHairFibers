#pragma once

#ifndef __AOIT__
#define __AOIT__

#include "BVH.cuh"


__device__ void setVectorValue(float4 &vec, int pos, float value) {
    if(pos == 0) {
        vec.x = value;

    } else if(pos == 1) {
        vec.y = value;

    } else if(pos == 2) {
        vec.z = value;

    } else {
        vec.w = value;
    }
}

__device__ float getVectorValue(float4 vec, int pos) {
    if(pos == 0) {
        return vec.x;

    } else if(pos == 1) {
        return vec.y;

    } else if(pos == 2) {
        return vec.z;

    } else {
        return vec.w;
    }
}


// Copyright 2011 Intel Corporation
// All Rights Reserved
//
// Permission is granted to use, copy, distribute and prepare derivative works of this
// software for any purpose and without fee, provided, that the above copyright notice
// and this statement appear in all copies.  Intel makes no representations about the
// suitability of this software for any purpose.  THIS SOFTWARE IS PROVIDED "AS IS."
// INTEL SPECIFICALLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, AND ALL LIABILITY,
// INCLUDING CONSEQUENTIAL AND OTHER INDIRECT DAMAGES, FOR THE USE OF THIS SOFTWARE,
// INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PROPRIETARY RIGHTS, AND INCLUDING THE
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  Intel does not
// assume any responsibility for any errors which may appear in this software nor any
// responsibility to update it.


//////////////////////////////////////////////
// Defines
//////////////////////////////////////////////


#define AOIT_FIRT_NODE_TRANS	(1)
#define AOIT_RT_COUNT			(AOIT_NODE_COUNT / 4)
#define AIOT_EMPTY_NODE_DEPTH	(1E30)

// Forces compression to only work on the second half of the nodes (cheaper and better IQ in most cases)
#define AOIT_DONT_COMPRESS_FIRST_HALF 

//////////////////////////////////////////////
// Structs
//////////////////////////////////////////////

struct AOITData {
    float4 depth[AOIT_RT_COUNT];
    float4 trans[AOIT_RT_COUNT];
};

struct AOITFragment {
    int   index;
    float depthA;
    float transA;
};

__device__ void initAT(AOITData &dataAT) {
    // Initialize AVSM data    
    for(int i = 0; i < AOIT_RT_COUNT; i++) {
        dataAT.depth[i] = make_float4(AIOT_EMPTY_NODE_DEPTH);
        dataAT.trans[i] = make_float4(AOIT_FIRT_NODE_TRANS);
    }
}

//////////////////////////////////////////////////
// Two-level search for AT visibility functions
//////////////////////////////////////////////////
 
__device__ AOITFragment AOITFindFragment(AOITData data, float fragmentDepth) {
    float4 depth, trans;
    float  leftDepth;
    float  leftTrans;
    
    AOITFragment Output;      

    #if AOIT_RT_COUNT > 7    
    if(fragmentDepth > data.depth[6].w) {
        depth        = data.depth[7];
        trans        = data.trans[7];
        leftDepth    = data.depth[6].w;
        leftTrans    = data.trans[6].w;
        Output.index = 28;

    } else
    #endif

    #if AOIT_RT_COUNT > 6    
    if(fragmentDepth > data.depth[5].w) {
        depth        = data.depth[6];
        trans        = data.trans[6];
        leftDepth    = data.depth[5].w;
        leftTrans    = data.trans[5].w;
        Output.index = 24;

    } else
    #endif

    #if AOIT_RT_COUNT > 5    
    if(fragmentDepth > data.depth[4].w) {
        depth        = data.depth[5];
        trans        = data.trans[5];
        leftDepth    = data.depth[4].w;
        leftTrans    = data.trans[4].w;
        Output.index = 20;

    } else
    #endif

    #if AOIT_RT_COUNT > 4    
    if(fragmentDepth > data.depth[3].w) {
        depth        = data.depth[4];
        trans        = data.trans[4];
        leftDepth    = data.depth[3].w;
        leftTrans    = data.trans[3].w;    
        Output.index = 16;

    } else
    #endif

    #if AOIT_RT_COUNT > 3    
    if(fragmentDepth > data.depth[2].w) {
        depth        = data.depth[3];
        trans        = data.trans[3];
        leftDepth    = data.depth[2].w;
        leftTrans    = data.trans[2].w;    
        Output.index = 12;

    } else
    #endif

    #if AOIT_RT_COUNT > 2    
    if(fragmentDepth > data.depth[1].w) {
        depth        = data.depth[2];
        trans        = data.trans[2];
        leftDepth    = data.depth[1].w;
        leftTrans    = data.trans[1].w;          
        Output.index = 8;

    } else
    #endif

    #if AOIT_RT_COUNT > 1    
    if(fragmentDepth > data.depth[0].w) {
        depth        = data.depth[1];
        trans        = data.trans[1];
        leftDepth    = data.depth[0].w;
        leftTrans    = data.trans[0].w;       
        Output.index = 4;

    } else
    #endif

    {    
        depth        = data.depth[0];
        trans        = data.trans[0];
        leftDepth    = data.depth[0].x;
        leftTrans    = data.trans[0].x;      
        Output.index = 0;        
    } 
      
    if(fragmentDepth <= depth.x) {
        Output.depthA = leftDepth;
        Output.transA = leftTrans;

    } else if(fragmentDepth <= depth.y) {
        Output.index += 1;
        Output.depthA = depth.x; 
        Output.transA = trans.x;

    } else if(fragmentDepth <= depth.z) {
        Output.index += 2;
        Output.depthA = depth.y;
        Output.transA = trans.y;

    } else if(fragmentDepth <= depth.w) {
        Output.index += 3;    
        Output.depthA = depth.z;
        Output.transA = trans.z;

    } else {
        Output.index += 4;       
        Output.depthA = depth.w;
        Output.transA = trans.w;
    }
    
    return Output;
}	

////////////////////////////////////////////////////
// Insert a new fragment in the visibility function
////////////////////////////////////////////////////

__device__ void AOITInsertFragment(float fragmentDepth, float fragmentTrans, AOITData &data) {
    // Unpack AOIT data    
    float depth[AOIT_NODE_COUNT + 1];	
    float trans[AOIT_NODE_COUNT + 1];

    for(int i = 0; i < AOIT_RT_COUNT; i++) {
	    for(int j = 0; j < 4; j++) {
		    depth[4 * i + j] = getVectorValue(data.depth[i], j);
		    trans[4 * i + j] = getVectorValue(data.trans[i], j);
	    }
    }	

    // Find insertion index 
    AOITFragment tempFragment = AOITFindFragment(data, fragmentDepth);
    const int index = tempFragment.index;

    // If we are inserting in the first node then use 1.0 as previous transmittance value
    // (we don't store it, but it's implicitly set to 1. This allows us to store one more node)
    const float prevTrans = index != 0 ? tempFragment.transA : 1.0f;

    // Make space for the new fragment. Also composite new fragment with the current curve 
    // (except for the node that represents the new fragment)
    for(int i = AOIT_NODE_COUNT - 1; i >= 0; i--) {
        if(index <= i) {
            depth[i + 1] = depth[i];
            trans[i + 1] = trans[i] * fragmentTrans;
        }
    }
    
    // Insert new fragment
    for(int i = 0; i <= AOIT_NODE_COUNT; i++) {
        if(index == i) {
            depth[i] = fragmentDepth;
            trans[i] = fragmentTrans * prevTrans;
        }
    } 
    
    // pack representation if we have too many nodes
    if(depth[AOIT_NODE_COUNT] != AIOT_EMPTY_NODE_DEPTH) {	                
        
        // That's total number of nodes that can be possibly removed
        const int removalCandidateCount = (AOIT_NODE_COUNT + 1) - 1;

        #ifdef AOIT_DONT_COMPRESS_FIRST_HALF
        // Although to bias our compression scheme in order to favor..
        // .. the closest nodes to the eye we skip the first 50%

		const int startRemovalIdx = removalCandidateCount / 2;
        
        #else

		const int startRemovalIdx = 1;

        #endif

        float nodeUnderError[removalCandidateCount];

        for(int i = startRemovalIdx; i < removalCandidateCount; i++) {
            nodeUnderError[i] = (depth[i] - depth[i - 1]) * (trans[i - 1] - trans[i]);
        }

        // Find the node the generates the smallest removal error
        int smallestErrorIdx;
        float smallestError;

        smallestErrorIdx = startRemovalIdx;
        smallestError = nodeUnderError[smallestErrorIdx];
        

        for(int i = startRemovalIdx + 1; i < removalCandidateCount; i++) {
            if(nodeUnderError[i] < smallestError) {
                smallestError = nodeUnderError[i];
                smallestErrorIdx = i;
            } 
        }

        // Remove that node..
        for(int i = startRemovalIdx; i < AOIT_NODE_COUNT; i++) {
            if(i >= smallestErrorIdx) {
                depth[i] = depth[i + 1];
            }

            if(i - 1 >= smallestErrorIdx - 1) {
                trans[i - 1] = trans[i];
            }
        }
    }
    
    // Pack AOIT data
    for(int i = 0; i < AOIT_RT_COUNT; i++) {
	    for(int j = 0; j < 4; j++) {
		    setVectorValue(data.depth[i], j, depth[4 * i + j]);
		    setVectorValue(data.trans[i], j, trans[4 * i + j]);
	    }
    }	
}

/*
 * Traverse BVH and insert intersected shapes into AT (for shadows)
 */
template <typename BVHNodeType>
__device__ bool traverseHybridBVH(BVHNodeType *bvh, uint bvhSize, Ray ray, 
                                  IntersectionLstItem *shapeIntersectionLst, int &lstIndex, AOITData &dataAT) {
    
    bool intersectionFound = false;
    RayIntersection curr = RayIntersection();

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
    //int aux = 0;
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
                     intersectionFound = intersection(ray, &curr, childL->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        } else {
                            return result;
                        }
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
                   intersectionFound = intersection(ray, &curr, childR->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        } else {
                            return result;
                        }
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

    /*if(aux > 0) {
        printf("Increase INTERSECTION_LST_SIZE from %d to %d\n", INTERSECTION_LST_SIZE, lstIndex + aux);
    }*/

    return result;
}

template <typename BVHNodeType>
__device__ bool traverse(BVHNodeType *bvh, uint bvhSize, Ray ray, 
                         IntersectionLstItem *shapeIntersectionLst, int &lstIndex, AOITData &dataAT) {

    bool intersectionFound = false;
    RayIntersection curr = RayIntersection();

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
    //int aux = 0;
    while(node != nullptr) {
        lIntersection = rIntersection = traverseL = traverseR = false;

        childL = node->lchild;
        if(childL != nullptr) {
            tmp = *childL;
            
            lIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            
            if (lIntersection) {
                // Leaf node
                if (childL->shape != nullptr) {
                    intersectionFound = intersection(ray, &curr, childL->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        } else {
                            return result;
                        }
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
                    intersectionFound = intersection(ray, &curr, childR->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        } else {
                            return result;
                        }
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

   /* if(aux > 0) {
        printf("Increase INTERSECTION_LST_SIZE from %d to %d\n", INTERSECTION_LST_SIZE, lstIndex + aux);
    }*/

    return result;
}

/*
 * Traverse BVH and save all intersections with shapes
 */
template <typename BVHNodeType>
__device__ bool traverseHybridBVH(BVHNodeType *bvh, uint bvhSize, Ray ray, 
                                  RayIntersection *shapeIntersectionLst, int &lstIndex) {
    
    bool intersectionFound = false;
    RayIntersection curr = RayIntersection();

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
                lIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            } else {
                lIntersection = OBBIntersection(ray, tmp.min, tmp.max, tmp.matrix, tmp.translation);
            }

            if (lIntersection) {
                // Leaf node
                if (childL->shape != nullptr) {
                    intersectionFound = intersection(ray, &curr, childL->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex++] = curr;
                            result = true;
                            
                        } else {
                            return result;
                        }
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
                   intersectionFound = intersection(ray, &curr, childR->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex++] = curr;
                            result = true;
                            
                        } else {
                            return result;
                        }
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
__device__ bool traverse(BVHNodeType *bvh, uint bvhSize, Ray ray, 
                         RayIntersection *shapeIntersectionLst, int &lstIndex) {

    bool intersectionFound = false;
    RayIntersection curr = RayIntersection();

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
            
            lIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            
            if (lIntersection) {
                // Leaf node
                if (childL->shape != nullptr) {
                    intersectionFound = intersection(ray, &curr, childL->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex++] = curr;
                            result = true;
                            
                        } else {
                            return result;
                        }
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
                    intersectionFound = intersection(ray, &curr, childR->shape);

                    if(intersectionFound) {
                        if(lstIndex < INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex++] = curr;
                            result = true;
                            
                        } else {
                            return result;
                        }
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

__device__ float3 computeAOITColor(IntersectionLstItem *shapeIntersectionLst, int lstSize, AOITData dataAT) {
    IntersectionLstItem *node;

    float3 color = make_float3(0.0f);
    // Fetch all nodes again and composite them
    float factor = 1.0f / lstSize;
    for(int i = 0; i < lstSize; i++) {
        node = &shapeIntersectionLst[i];

        AOITFragment frag = AOITFindFragment(dataAT, node->distance);

        float vis = frag.index == 0 ? 1.0f : frag.transA;
        color += factor * node->color * node->transparency * vis;
                  
    }

    return color;
}

__device__ float3 computeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    AOITData dataAT;
    bool intersectionFound = false;

    initAT(dataAT);

    IntersectionLstItem shapeIntersectionLst[INTERSECTION_LST_SIZE];
    int lstSize = 0;

    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        if(d_shapeSizes[shapeType] == 0) {
            continue;
        }

        if(shapeType == sphereIndex) {
            SphereNode *bvh = (SphereNode*) d_shapes[shapeType];

            intersectionFound |= traverse(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize, dataAT);
               
        } else if(shapeType == cylinderIndex) {
            CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

            intersectionFound |= traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize, dataAT);

        } else if(shapeType == triangleIndex) {
            TriangleNode *bvh = (TriangleNode*) d_shapes[shapeType];

            intersectionFound |= traverse(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize, dataAT);

        } else if(shapeType == planeIndex) {
            Plane *plane = (Plane*) d_shapes[shapeType];
            bool result;
            RayIntersection curr = RayIntersection();

            result = intersection(feeler, &curr, plane[0]);

            if(result) {
                if(lstSize < INTERSECTION_LST_SIZE) {
                    shapeIntersectionLst[lstSize++].update(curr);
                }
                AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
            }

            intersectionFound |= result;
        }
    }

    float3 color = make_float3(1.0f);
    if(intersectionFound) {
        color = computeAOITColor(shapeIntersectionLst, lstSize, dataAT);
    }

    return color;
}

__device__ float3 cylComputeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];
    AOITData dataAT;
    bool intersectionFound = false;

    initAT(dataAT);

    IntersectionLstItem shapeIntersectionLst[INTERSECTION_LST_SIZE];
    int lstSize = 0;

    intersectionFound = traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize, dataAT);

    float3 color = make_float3(1.0f);
    if(intersectionFound) {
        color = computeAOITColor(shapeIntersectionLst, lstSize, dataAT);
    }

    return color;
}

__device__ bool findIntersections(int **d_shapes, uint *d_shapeSizes, Ray feeler, RayIntersection *shapeIntersectionLst,
                                  int &lstSize) {
    bool intersectionFound = false;

    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        if(d_shapeSizes[shapeType] == 0) {
            continue;
        }

        if(shapeType == sphereIndex) {
            SphereNode *bvh = (SphereNode*) d_shapes[shapeType];

            intersectionFound |= traverse(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize);
               
        } else if(shapeType == cylinderIndex) {
            CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

            intersectionFound |= traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize);

        } else if(shapeType == triangleIndex) {
            TriangleNode *bvh = (TriangleNode*) d_shapes[shapeType];

            intersectionFound |= traverse(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize);

        } else if(shapeType == planeIndex) {
            Plane *plane = (Plane*) d_shapes[shapeType];
            bool result;
            RayIntersection curr = RayIntersection();

            result = intersection(feeler, &curr, plane[0]);

            if(result && lstSize < INTERSECTION_LST_SIZE) {
                shapeIntersectionLst[lstSize++] = curr;
                
            }

            intersectionFound |= result;
        }
    }

    
    return intersectionFound;
}

__device__ bool findCylIntersections(int **d_shapes, uint *d_shapeSizes, Ray feeler, RayIntersection *shapeIntersectionLst,
                                     int &lstSize) {

    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];
    
    bool intersectionFound = traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize);

    return intersectionFound;
}


__device__ float3 computeShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, 
                                 RayIntersection intersect, uint li, float3 feelerDir) {
    Ray feeler = Ray(intersect.point, feelerDir);
    bool result = false;

    #ifndef AT_SHADOWS
    #ifdef GENERAL_INTERSECTION
    bool inShadow = findShadow(d_shapes, d_shapeSizes, feeler);
    #else
    bool inShadow = cylFindShadow(d_shapes, d_shapeSizes, feeler);
    #endif
            
	result = !inShadow;

    #else

    #ifdef GENERAL_INTERSECTION
    float3 transmitance = computeTransparency(d_shapes, d_shapeSizes, feeler);
    #else
    float3 transmitance = cylComputeTransparency(d_shapes, d_shapeSizes, feeler);
    #endif
            
    result = length(transmitance) > 0.01;
    #endif

    if(result) {
        Material *mat = &intersect.shapeMaterial;
        float3 reflectDir = reflect(-feelerDir, intersect.normal);
        float Lspec = powf(fmaxf(dot(reflectDir, -ray.direction), 0.0f), mat->shininess);
        float Ldiff = fmaxf(dot(feelerDir, intersect.normal), 0.0f);

        #ifndef AT_SHADOWS
        return (Ldiff * mat->color * mat->Kdiffuse + Lspec * mat->Kspecular) * lights[li].color;

        #else
        return (Ldiff * mat->color * mat->Kdiffuse + Lspec * mat->Kspecular) * transmitance * lights[li].color;

        #endif
	}

    return make_float3(0.0f);
}

__device__ float3 computeSoftShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, 
                                     RayIntersection intersect, uint li, float3 feelerDir) {
    float3 u, v;
    const float3 xAxis = make_float3(1, 0, 0);
    const float3 yAxis = make_float3(0, 1, 0);

    if (equal(dot(xAxis, feelerDir), 1.0f)) {
		u = cross(feelerDir, yAxis);
	}
	else {
		u = cross(feelerDir, xAxis);
	}
	v = cross(feelerDir, u);
            
    float3 localColor = make_float3(0.0f);
    for (int x = 0; x < LIGHT_SAMPLE_RADIUS; x++) {
		for (int y = 0; y < LIGHT_SAMPLE_RADIUS; y++) {
			float xCoord = LIGHT_SOURCE_SIZE * ((y + 0.5f) * LIGHT_SAMPLE_RADIUS_F - 0.5f);
			float yCoord = LIGHT_SOURCE_SIZE * ((x + 0.5f) * LIGHT_SAMPLE_RADIUS_F - 0.5f);

			feelerDir = normalize((lights[li].position + xCoord*u + yCoord*v) - intersect.point);
                    
            localColor += SUM_FACTOR * computeShadows(d_shapes, d_shapeSizes, lights, ray,
                                                      intersect, li, feelerDir);
		}
	}

    return localColor;
}

__device__ float3 computeAT(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, Ray ray,
                            float3 backColor) {

    RayIntersection intersections[INTERSECTION_LST_SIZE];
    AOITData dataAT;

    int lstSize;
    float3 colorAux;
    float3 black = make_float3(0.0f);
    RayIntersection *node;

    initAT(dataAT);

    #ifdef GENERAL_INTERSECTION 
    bool foundIntersect = findIntersections(d_shapes, d_shapeSizes, ray, intersections, lstSize);
    #else
	bool foundIntersect = findCylIntersections(d_shapes, d_shapeSizes, ray, intersections, lstSize);
    #endif

    if(foundIntersect) {
        for(int i = 0; i < lstSize; i++) {
            // local illumination
            colorAux = black;
	        for(uint li = 0; li < lightSize; li++) {
                #ifndef SOFT_SHADOWS
                colorAux += computeShadows(d_shapes, d_shapeSizes, lights, ray, intersections[i],
                                           li, normalize(lights[li].position - intersections[i].point));
                    
                #else
                colorAux += computeSoftShadows(d_shapes, d_shapeSizes, lights, ray, intersections[i],
                                               li, normalize(lights[li].position - intersections[i].point));
                #endif
	        }

            intersections[i].shapeMaterial.color = colorAux;
            AOITInsertFragment(intersections[i].distance, intersections[i].shapeMaterial.transparency, dataAT);
        }

        float3 color = black;
        // Fetch all nodes again and composite them
        float factor = 1.0f / lstSize;
        for(int i = 0; i < lstSize; i++) {
            node = &intersections[i];

            AOITFragment frag = AOITFindFragment(dataAT, node->distance);

            float vis = frag.index == 0 ? 1.0f : frag.transA;
            color += factor * node->shapeMaterial.color * node->shapeMaterial.transparency * vis;
                  
        }

        return color;

    } else {
        return backColor;
    }
}

#endif;