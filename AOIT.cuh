#pragma once

#ifndef __AOIT__
#define __AOIT__

#include "tracing.cuh"

#define AOIT_FIRT_NODE_TRANS	(1)
#define AOIT_RT_COUNT			(AOIT_NODE_COUNT / 4)
#define AIOT_EMPTY_NODE_DEPTH	(1E30)

// Forces compression to only work on the second half of the nodes (cheaper and better IQ in most cases)
#define AOIT_DONT_COMPRESS_FIRST_HALF 


__device__ void initAT(AOITData &dataAT) {
    // Initialize AVSM data    
    for(int i = 0; i < AOIT_NODE_COUNT; i++) {
        dataAT.depth[i] = AIOT_EMPTY_NODE_DEPTH;
        dataAT.trans[i] = AOIT_FIRT_NODE_TRANS;
    }
}

__device__ AOITFragment AOITFindFragment(AOITData data, float fragmentDepth) {
    float depth[4];
    float trans[4];
    float  leftDepth;
    float  leftTrans;
    
    AOITFragment Output;      

    #if AOIT_RT_COUNT > 7    
    if(fragmentDepth > data.depth[27]) {
        leftDepth    = data.depth[27];
        leftTrans    = data.trans[27];
        Output.index = 28;

    } else
    #endif

    #if AOIT_RT_COUNT > 6    
    if(fragmentDepth > data.depth[23]) {
        leftDepth    = data.depth[23];
        leftTrans    = data.trans[23];
        Output.index = 24;

    } else
    #endif

    #if AOIT_RT_COUNT > 5    
    if(fragmentDepth > data.depth[19]) {
        leftDepth    = data.depth[19];
        leftTrans    = data.trans[19];
        Output.index = 20;

    } else
    #endif

    #if AOIT_RT_COUNT > 4    
    if(fragmentDepth > data.depth[15]) {
        leftDepth    = data.depth[15];
        leftTrans    = data.trans[15];    
        Output.index = 16;

    } else
    #endif

    #if AOIT_RT_COUNT > 3    
    if(fragmentDepth > data.depth[11]) {
        leftDepth    = data.depth[11];
        leftTrans    = data.trans[11];    
        Output.index = 12;

    } else
    #endif

    #if AOIT_RT_COUNT > 2    
    if(fragmentDepth > data.depth[7]) {
        leftDepth    = data.depth[7];
        leftTrans    = data.trans[7];          
        Output.index = 8;

    } else
    #endif

    #if AOIT_RT_COUNT > 1    
    if(fragmentDepth > data.depth[3]) {
        leftDepth    = data.depth[3];
        leftTrans    = data.trans[3];       
        Output.index = 4;

    } else
    #endif

    {    
        leftDepth    = data.depth[0];
        leftTrans    = data.trans[0];      
        Output.index = 0;        
    } 
    
    for(short i = 0; i < 4; i++) {
        depth[i] = data.depth[Output.index + i];
        trans[i] = data.trans[Output.index + i];
    }

    if(fragmentDepth <= depth[0]) {
        Output.depthA = leftDepth;
        Output.transA = leftTrans;

    } else if(fragmentDepth <= depth[1]) {
        Output.index += 1;
        Output.depthA = depth[0]; 
        Output.transA = trans[0];

    } else if(fragmentDepth <= depth[2]) {
        Output.index += 2;
        Output.depthA = depth[1];
        Output.transA = trans[1];

    } else if(fragmentDepth <= depth[3]) {
        Output.index += 3;    
        Output.depthA = depth[2];
        Output.transA = trans[2];

    } else {
        Output.index += 4;       
        Output.depthA = depth[3];
        Output.transA = trans[3];
    }
    
    return Output;
}	

__device__ void AOITInsertFragment(float fragmentDepth, float fragmentTrans, AOITData &data) {
    // Find insertion index 
    AOITFragment tempFragment = AOITFindFragment(data, fragmentDepth);
    const int index = tempFragment.index;

    // If we are inserting in the first node then use 1.0 as previous transmittance value
    const float prevTrans = index != 0 ? tempFragment.transA : 1.0f;

    // Make space for the new fragment. Also composite new fragment with the current curve 
    for(int i = AOIT_NODE_COUNT - 1; i >= 0; i--) {
        if(index <= i) {
            data.depth[i + 1] = data.depth[i];
            data.trans[i + 1] = data.trans[i] * fragmentTrans;
        }
    }
    
    // Insert new fragment
    for(int i = 0; i <= AOIT_NODE_COUNT; i++) {
        if(index == i) {
            data.depth[i] = fragmentDepth;
            data.trans[i] = fragmentTrans * prevTrans;
        }
    } 
    
    // pack representation if we have too many nodes
    if(data.depth[AOIT_NODE_COUNT] != AIOT_EMPTY_NODE_DEPTH) {	                
        
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
            nodeUnderError[i] = (data.depth[i] - data.depth[i - 1]) * (data.trans[i - 1] - data.trans[i]);
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
                data.depth[i] = data.depth[i + 1];
            }

            if(i - 1 >= smallestErrorIdx - 1) {
                data.trans[i - 1] = data.trans[i];
            }
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
                    if(lstIndex < INTERSECTION_LST_SIZE) {
                        intersectionFound = intersection(ray, &curr, childL->shape);

                        if(intersectionFound) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        } 

                    } else {
                        return result;
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
                    if(lstIndex < INTERSECTION_LST_SIZE) {
                        intersectionFound = intersection(ray, &curr, childR->shape);

                        if(intersectionFound) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        } 

                    } else {
                        return result;
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
                    if(lstIndex < INTERSECTION_LST_SIZE) {
                        intersectionFound = intersection(ray, &curr, childR->shape);

                        if(intersectionFound) {
                            AOITInsertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                            shapeIntersectionLst[lstIndex++].update(curr);
                            result = true;
                            
                        }

                    } else {
                        return result;
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

    float vis, factor = 1.0f / lstSize;
    for(int i = 0; i < lstSize; i++) {
        node = &shapeIntersectionLst[i];

        AOITFragment frag = AOITFindFragment(dataAT, node->distance);

        vis = frag.index == 0 ? 1.0f : frag.transA;
        color += factor * node->color * node->transparency * vis;      
    }

    return color;
}

__device__ float3 computeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler, 
                                      IntersectionLstItem *shapeIntersectionLst) {

    bool intersectionFound = false;
    AOITData dataAT;
    initAT(dataAT);

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
            bool result = false;
            RayIntersection curr = RayIntersection();

            if(lstSize < INTERSECTION_LST_SIZE) {
                result = intersection(feeler, &curr, plane[0]);
            }

            if(result) {
                shapeIntersectionLst[lstSize].update(curr);
                lstSize++;
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

__device__ float3 cylComputeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler,
                                         IntersectionLstItem *shapeIntersectionLst) {
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];
    bool intersectionFound = false;
    AOITData dataAT;
    initAT(dataAT);

    int lstSize = 0;

    intersectionFound = traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, shapeIntersectionLst, lstSize, dataAT);

    float3 color = make_float3(1.0f);
    if(intersectionFound) {
        color = computeAOITColor(shapeIntersectionLst, lstSize, dataAT);
    }

    return color;
}

__device__ float3 computeShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, 
                                 RayIntersection intersect, uint li, float3 feelerDir, 
                                 IntersectionLstItem *intersectionLst) {

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
    float3 transmitance = computeTransparency(d_shapes, d_shapeSizes, feeler, intersectionLst);
    #else
    float3 transmitance = cylComputeTransparency(d_shapes, d_shapeSizes, feeler, intersectionLst);
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
                                     RayIntersection intersect, uint li, float3 feelerDir,
                                     IntersectionLstItem *intersectionLst) {
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
                                                      intersect, li, feelerDir, intersectionLst);
		}
	}

    return localColor;
}


#endif;