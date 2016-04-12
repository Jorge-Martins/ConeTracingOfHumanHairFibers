#pragma once

#ifndef __AOITHAIR__
#define __AOITHAIR__

#define AOIT_FIRT_NODE_TRANS	(1)
#define AIOT_EMPTY_NODE_DEPTH	(1E30)
#define AOIT_HAIR_RT_COUNT (AOIT_HAIR_NODE_COUNT / 4)

//#define AOIT_DONT_COMPRESS_FIRST_HALF

#include "ShadowTransmitance.cuh"

__device__ AOITFragment getFragment(AOITHair data, float fragmentDepth) {
    float depth[4];
    float trans[4];
    float  leftDepth;
    float  leftTrans;
    
    AOITFragment Output;      

    #if AOIT_HAIR_RT_COUNT > 7    
    if(fragmentDepth > data.depth[27]) {
        leftDepth    = data.depth[27];
        leftTrans    = data.trans[27];
        Output.index = 28;

    } else
    #endif

    #if AOIT_HAIR_RT_COUNT > 6    
    if(fragmentDepth > data.depth[23]) {
        leftDepth    = data.depth[23];
        leftTrans    = data.trans[23];
        Output.index = 24;

    } else
    #endif

    #if AOIT_HAIR_RT_COUNT > 5    
    if(fragmentDepth > data.depth[19]) {
        leftDepth    = data.depth[19];
        leftTrans    = data.trans[19];
        Output.index = 20;

    } else
    #endif

    #if AOIT_HAIR_RT_COUNT > 4    
    if(fragmentDepth > data.depth[15]) {
        leftDepth    = data.depth[15];
        leftTrans    = data.trans[15];    
        Output.index = 16;

    } else
    #endif

    #if AOIT_HAIR_RT_COUNT > 3    
    if(fragmentDepth > data.depth[11]) {
        leftDepth    = data.depth[11];
        leftTrans    = data.trans[11];    
        Output.index = 12;

    } else
    #endif

    #if AOIT_HAIR_RT_COUNT > 2    
    if(fragmentDepth > data.depth[7]) {
        leftDepth    = data.depth[7];
        leftTrans    = data.trans[7];          
        Output.index = 8;

    } else
    #endif

    #if AOIT_HAIR_RT_COUNT > 1    
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

__device__ void insertFragment(float fragmentDepth, float fragmentTrans, AOITHair &data) {
    // Find insertion index 
    AOITFragment tempFragment = getFragment(data, fragmentDepth);
    const int index = tempFragment.index;

    // If we are inserting in the first node then use 1.0 as previous transmittance value
    const float prevTrans = index != 0 ? tempFragment.transA : 1.0f;

    // Make space for the new fragment. Also composite new fragment with the current curve 
    for(int i = AOIT_HAIR_NODE_COUNT - 1; i >= 0; i--) {
        if(index <= i) {
            data.depth[i + 1] = data.depth[i];
            data.trans[i + 1] = data.trans[i] * fragmentTrans;
        }
    }
    
    // Insert new fragment
    data.depth[index] = fragmentDepth;
    data.trans[index] = fragmentTrans * prevTrans;

    // pack representation if we have too many nodes
    if(data.depth[AOIT_HAIR_NODE_COUNT] != AIOT_EMPTY_NODE_DEPTH) {	                
        
        // That's total number of nodes that can be possibly removed
        const int removalCandidateCount = (AOIT_HAIR_NODE_COUNT + 1) - 1;

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
        for(int i = startRemovalIdx; i < AOIT_HAIR_NODE_COUNT; i++) {
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
 * Traverse BVH and save all intersections with shapes
 */
template <typename BVHNodeType>
__device__ bool traverseHairHybridBVH(BVHNodeType *bvh, uint bvhSize, Cone cone, 
                                      RayIntersection *shapeIntersectionLst, int &lstIndex,
                                      int &rayHairIntersections, AOITHair &dataAT, 
                                      RayIntersection shadowPoints[][N_SHADOW_POINTS], 
                                      int *nShadowPoints, float3 rayDirections[][N_SHADOW_POINTS]) {
    float distance;
    float minDistance = FLT_MAX;
    bool intersectionFound = false;
    const float factor = 1.13f; 
    RayIntersection curr = RayIntersection();

    BVHNodeType *stackNodes[StackSize];
    
    uint stackIndex = 0;

    stackNodes[stackIndex++] = nullptr;
    
    BVHNodeType *childL, *childR, *node = &bvh[0], tmp;

    tmp = *node;
    if(tmp.type == AABB) {
        intersectionFound = AABBIntersection(cone, tmp.min, tmp.max);
    } else {
        intersectionFound = OBBIntersection(cone, tmp.min, tmp.max, tmp.matrix, tmp.translation);
    }
    
    if(!intersectionFound) {
        return false;
    }

    RayIntersection tempShadowPoints[N_SHADOW_POINTS];
    int tempNShadowPoints;
    float3 tempRayDirections[N_SHADOW_POINTS];

    bool lIntersection, rIntersection, traverseL, traverseR; 

    while(node != nullptr) {
        lIntersection = rIntersection = traverseL = traverseR = false;

        childL = node->lchild;
        if(childL != nullptr) {
            tmp = *childL;
            if(tmp.type == AABB) {
                lIntersection = AABBIntersection(cone, tmp.min, tmp.max, distance);
            } else {
                lIntersection = OBBIntersection(cone, tmp.min, tmp.max, tmp.matrix, tmp.translation, distance);
            }

            if (lIntersection && distance < minDistance) {
                // Leaf node
                if (childL->shape != nullptr) {
                    #ifdef PRINT_N_INTERSECTIONS
                    rayHairIntersections++;
                    #endif

                    intersectionFound = intersection(cone, &curr, childL->shape, tempShadowPoints, 
                                                     tempNShadowPoints, tempRayDirections);

                    if(intersectionFound) {
                        float areaFraction = curr.shapeMaterial.ior;
                        insertFragment(curr.distance, (1.0f - areaFraction) + 
                                                      (areaFraction * curr.shapeMaterial.transparency), dataAT);

                        if(lstIndex < HAIR_INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex] = curr;

                            nShadowPoints[lstIndex] = tempNShadowPoints;
                            //store temp values
                            for(int i = 0; i < tempNShadowPoints; i++) {
                                shadowPoints[lstIndex][i].normal = tempShadowPoints[i].normal;
                                shadowPoints[lstIndex][i].point = tempShadowPoints[i].point;
                                shadowPoints[lstIndex][i].shapeMaterial = tempShadowPoints[i].shapeMaterial;

                                rayDirections[lstIndex][i] = tempRayDirections[i];
                            }

                            lstIndex++;

                        } else {
                            distance = curr.distance;
                            
                            for(int i = 0; i < HAIR_INTERSECTION_LST_SIZE; i++) {
                                if(shapeIntersectionLst[i].distance > distance) {
                                    shapeIntersectionLst[i] = curr;

                                    nShadowPoints[i] = tempNShadowPoints;
                                    //store temp values
                                    for(int j = 0; j < tempNShadowPoints; j++) {
                                        shadowPoints[i][j].normal = tempShadowPoints[j].normal;
                                        shadowPoints[i][j].point = tempShadowPoints[j].point;
                                        shadowPoints[i][j].shapeMaterial = tempShadowPoints[j].shapeMaterial;

                                        rayDirections[i][j] = tempRayDirections[j];
                                    }

                                    break;
                                }
                            }
                        }

                        minDistance = fminf(minDistance, factor * curr.distance);
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
                rIntersection = AABBIntersection(cone, tmp.min, tmp.max, distance);
            } else {
                rIntersection = OBBIntersection(cone, tmp.min, tmp.max, tmp.matrix, tmp.translation, distance);
            }

            if (rIntersection && distance < minDistance) {
                // Leaf node
                if (childR->shape != nullptr) {
                    #ifdef PRINT_N_INTERSECTIONS
                    rayHairIntersections++;
                    #endif

                    intersectionFound = intersection(cone, &curr, childR->shape, tempShadowPoints, 
                                                     tempNShadowPoints, tempRayDirections);

                    if(intersectionFound) {
                        float areaFraction = curr.shapeMaterial.ior;
                        insertFragment(curr.distance, (1.0f - areaFraction) + 
                                                      (areaFraction * curr.shapeMaterial.transparency), dataAT);

                        if(lstIndex < HAIR_INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex] = curr;
                            
                            nShadowPoints[lstIndex] = tempNShadowPoints;
                            //store temp values
                            for(int i = 0; i < tempNShadowPoints; i++) {
                                shadowPoints[lstIndex][i].normal = tempShadowPoints[i].normal;
                                shadowPoints[lstIndex][i].point = tempShadowPoints[i].point;
                                shadowPoints[lstIndex][i].shapeMaterial = tempShadowPoints[i].shapeMaterial;

                                rayDirections[lstIndex][i] = tempRayDirections[i];
                            }

                            lstIndex++;
                        
                        } else {
                            distance = curr.distance;
                            
                            for(int i = 0; i < HAIR_INTERSECTION_LST_SIZE; i++) {
                                if(shapeIntersectionLst[i].distance > distance) {
                                    shapeIntersectionLst[i] = curr;

                                    nShadowPoints[i] = tempNShadowPoints;
                                    //store temp values
                                    for(int j = 0; j < tempNShadowPoints; j++) {
                                        shadowPoints[i][j].normal = tempShadowPoints[j].normal;
                                        shadowPoints[i][j].point = tempShadowPoints[j].point;
                                        shadowPoints[i][j].shapeMaterial = tempShadowPoints[j].shapeMaterial;

                                        rayDirections[i][j] = tempRayDirections[j];
                                    }

                                    break;
                                }
                            }
                        }

                        minDistance = fminf(minDistance, factor * curr.distance);
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

    return lstIndex > 0;
}

template <typename BVHNodeType>
__device__ bool traverseHairHybridBVH(BVHNodeType *bvh, uint bvhSize, Ray ray, 
                                      RayIntersection *shapeIntersectionLst, int &lstIndex,
                                      int &rayHairIntersections, AOITHair &dataAT) {
    float distance;
    float minDistance = FLT_MAX;
    bool intersectionFound = false;
    const float factor = 1.13f; 
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

            if (lIntersection && distance < minDistance) {
                // Leaf node
                if (childL->shape != nullptr) {
                    #ifdef PRINT_N_INTERSECTIONS
                    rayHairIntersections++;
                    #endif

                    intersectionFound = intersection(ray, &curr, childL->shape);

                    if(intersectionFound) {
                        insertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                        
                        if(lstIndex < HAIR_INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex] = curr;
                            lstIndex++;

                        } else {
                            distance = curr.distance;
                            
                            for(int i = 0; i < HAIR_INTERSECTION_LST_SIZE; i++) {
                                if(shapeIntersectionLst[i].distance > distance) {
                                    shapeIntersectionLst[i] = curr;
                                    break;
                                }
                            }
                        }

                        minDistance = fminf(minDistance, factor * curr.distance);
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

            if (rIntersection && distance < minDistance) {
                // Leaf node
                if (childR->shape != nullptr) {
                    #ifdef PRINT_N_INTERSECTIONS
                    rayHairIntersections++;
                    #endif

                    intersectionFound = intersection(ray, &curr, childR->shape);

                    if(intersectionFound) {
                        insertFragment(curr.distance, curr.shapeMaterial.transparency, dataAT);
                        
                        if(lstIndex < HAIR_INTERSECTION_LST_SIZE) {
                            shapeIntersectionLst[lstIndex] = curr;
                            lstIndex++;
                        
                        } else {
                            distance = curr.distance;
                            
                            for(int i = 0; i < HAIR_INTERSECTION_LST_SIZE; i++) {
                                if(shapeIntersectionLst[i].distance > distance) {
                                    shapeIntersectionLst[i] = curr;
                                    break;
                                }
                            }
                        }

                        minDistance = fminf(minDistance, factor * curr.distance);
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

    return lstIndex > 0;
}


__device__ void initAT(AOITHair &dataAT) {
    // Initialize AVSM data    
    for(int i = 0; i < AOIT_HAIR_NODE_COUNT; i++) {
        dataAT.depth[i] = AIOT_EMPTY_NODE_DEPTH;
        dataAT.trans[i] = AOIT_FIRT_NODE_TRANS;
    }
}

__device__ bool findHairIntersections(int **d_shapes, uint *d_shapeSizes, Ray ray, RayIntersection *shapeIntersectionLst,
                                      int &lstSize, int &rayHairIntersections, AOITHair &dataAT) {

    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];
    
    bool intersectionFound = traverseHairHybridBVH(bvh, d_shapeSizes[cylinderIndex], ray, 
                                                   shapeIntersectionLst, lstSize, 
                                                   rayHairIntersections, dataAT);

    return intersectionFound;
}

__device__ bool findHairIntersections(int **d_shapes, uint *d_shapeSizes, Cone cone, RayIntersection *shapeIntersectionLst,
                                      int &lstSize, int &rayHairIntersections, AOITHair &dataAT, RayIntersection shadowPoints[][N_SHADOW_POINTS], 
                                      int *nShadowPoints, float3 rayDirections[][N_SHADOW_POINTS]) {

    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];
    
    bool intersectionFound = traverseHairHybridBVH(bvh, d_shapeSizes[cylinderIndex], cone, 
                                                   shapeIntersectionLst, lstSize, 
                                                   rayHairIntersections, dataAT,
                                                   shadowPoints, nShadowPoints,
                                                   rayDirections);

    return intersectionFound;
}

__device__ float3 computeHairAT(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, Ray ray,
                                RayIntersection *hairIntersections,  float3 backgroundColor, 
                                float backgroundDistance, int &rayHairIntersections) {

    int lstSize = 0;
    float3 colorAux;
    float3 black = make_float3(0.0f);
    RayIntersection *node;

    AOITHair dataAT;
    initAT(dataAT);

    insertFragment(backgroundDistance, 0.0f, dataAT);

    bool foundIntersect = findHairIntersections(d_shapes, d_shapeSizes, ray, hairIntersections, 
                                                lstSize, rayHairIntersections, dataAT);

    if(foundIntersect) {    
        for(int i = 0; i < lstSize; i++) {
            // local illumination
            colorAux = black;
	        for(uint li = 0; li < lightSize; li++) {
                #ifndef SOFT_SHADOWS
                colorAux += computeShadows(d_shapes, d_shapeSizes, lights, ray, hairIntersections[i],
                                           li, normalize(lights[li].position - hairIntersections[i].point));
                    
                #else
                colorAux += computeSoftShadows(d_shapes, d_shapeSizes, lights, ray, hairIntersections[i],
                                               li, normalize(lights[li].position - hairIntersections[i].point));
                #endif
	        }

            //save local color
            hairIntersections[i].shapeMaterial.color = colorAux;
        }
    }

    float3 color = black;
    // Fetch all nodes again and composite them
    float vis;

    //background special case -> areaFraction 1 transparency 0
    AOITFragment frag = getFragment(dataAT, backgroundDistance);
    vis = frag.index == 0 ? 1.0f : frag.transA;
    color += backgroundColor * vis;

    for(int i = 0; i < lstSize; i++) {
        node = &hairIntersections[i];

        AOITFragment frag = getFragment(dataAT, node->distance);

        vis = frag.index == 0 ? 1.0f : frag.transA;
        color += node->shapeMaterial.color * (1.0f - node->shapeMaterial.transparency) * vis;
                  
    }

    return color;
}

__device__ float3 computeHairAT(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, Cone cone,
                                RayIntersection *hairIntersections, float3 backgroundColor, 
                                float backgroundDistance, int &rayHairIntersections, 
                                RayIntersection shadowPoints[][N_SHADOW_POINTS], 
                                int *nShadowPoints, float3 rayDirections[][N_SHADOW_POINTS]) {

    int lstSize = 0;
    float3 colorAux;
    float3 black = make_float3(0.0f);
    RayIntersection *node;

    AOITHair dataAT;
    initAT(dataAT);
    
    insertFragment(backgroundDistance, 0.0f, dataAT);

    bool foundIntersect = findHairIntersections(d_shapes, d_shapeSizes, cone, hairIntersections, 
                                                lstSize, rayHairIntersections, dataAT, shadowPoints, 
                                                nShadowPoints, rayDirections);
    
    if(foundIntersect) {
        Ray ray = Ray();
        float3 shadowPointColor;

        for(int i = 0; i < lstSize; i++) {
            shadowPointColor = black;

            float factor = 1.0f / nShadowPoints[i];

            for(int s = 0; s < nShadowPoints[i]; s++) {
                ray.update(cone.origin, rayDirections[i][s]);

                // local illumination
                colorAux = black;
	            for(uint li = 0; li < lightSize; li++) {
                    #ifndef SOFT_SHADOWS
                    colorAux += computeShadows(d_shapes, d_shapeSizes, lights, ray, shadowPoints[i][s],
                                               li, normalize(lights[li].position - shadowPoints[i][s].point));
                    
                    #else
                    colorAux += computeSoftShadows(d_shapes, d_shapeSizes, lights, ray, shadowPoints[i][s],
                                                   li, normalize(lights[li].position - shadowPoints[i][s].point));
                    #endif
	            }

                shadowPointColor += colorAux;
            }

            hairIntersections[i].shapeMaterial.color = factor * shadowPointColor;

        }
    }

    float3 color = black;
    // Fetch all nodes again and composite them
    float vis;
    
    //background special case -> areaFraction 1 transparency 0
    AOITFragment frag = getFragment(dataAT, backgroundDistance);
    vis = frag.index == 0 ? 1.0f : frag.transA;
    color += backgroundColor * vis;

    for(int i = 0; i < lstSize; i++) {
        node = &hairIntersections[i];

        AOITFragment frag = getFragment(dataAT, node->distance);

        vis = frag.index == 0 ? 1.0f : frag.transA;

        color += node->shapeMaterial.color * node->shapeMaterial.ior * (1.0f - node->shapeMaterial.transparency) * vis;
                  
    }

    return color;
}


#endif