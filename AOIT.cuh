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

#ifndef AOIT_NODE_COUNT 
#define AOIT_NODE_COUNT			(16)
#endif

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
	    for(int j = 0; j < 4; ++j) {
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
            if(smallestErrorIdx <= i) {
                depth[i] = depth[i + 1];
            }
        }

        for(int i = startRemovalIdx - 1; i < AOIT_NODE_COUNT; i++) {
            if(smallestErrorIdx - 1 <= i) {
                trans[i] = trans[i + 1];
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


__device__ float3 computeAOITColor(IntersectionLstItem *shapeIntersectionLst, int lstSize) {
    AOITData data;
    IntersectionLstItem *node;

    // Initialize AVSM data    
    for(int i = 0; i < AOIT_RT_COUNT; i++) {
        data.depth[i] = make_float4(AIOT_EMPTY_NODE_DEPTH);
        data.trans[i] = make_float4(AOIT_FIRT_NODE_TRANS);
    }
    
    // Fetch all nodes and add them to our visibiity function
    for(int i = 0; i < lstSize; i++) {
        node = &shapeIntersectionLst[i];
        
        AOITInsertFragment(node->distance, node->transparency, data);           
    }

    float3 color = make_float3(0.0f);

    // Fetch all nodes again and composite them
    for(int i = 0; i < lstSize; i++) {
        node = &shapeIntersectionLst[i];

        AOITFragment frag = AOITFindFragment(data, node->distance);

        float vis = frag.index == 0 ? 1.0f : frag.transA;
        color += node->color * node->transparency * vis;
                  
    }

    return color;
}

__device__ float3 computeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    bool intersectionFound = false;

    IntersectionLstItem shapeIntersectionLst[INTERSECTION_LST_SIZE];
    int lstSize = 0;

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

            if(result) {
                shapeIntersectionLst[lstSize++].update(curr);
            }

            intersectionFound |= result;
        }
    }

    float3 color = make_float3(1.0f);
    if(intersectionFound) {
        color = computeAOITColor(shapeIntersectionLst, lstSize);
    }

    return color;
}

__device__
float3 computeATShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, Ray feeler, float3 blackColor, Material mat, 
                      RayIntersection intersect, uint li, float3 feelerDir) {
    feeler.update(intersect.point, feelerDir);
            
   
    float3 transmitance = computeTransparency(d_shapes, d_shapeSizes, feeler);
    
            
	if(length(transmitance) > 0.01) {
        float3 reflectDir = reflect(-feelerDir, intersect.normal);
        float Lspec = powf(fmaxf(dot(reflectDir, -ray.direction), 0.0f), mat.shininess);
        float Ldiff = fmaxf(dot(feelerDir, intersect.normal), 0.0f);

        return (Ldiff * mat.color * mat.Kdiffuse + Lspec * mat.Kspecular) * transmitance * lights[li].color;
	}

    return blackColor;
}

#endif;