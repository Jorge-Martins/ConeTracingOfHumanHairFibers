#pragma once

#ifndef __SHADOWTRANMITANCE__
#define __SHADOWTRANMITANCE__

#include "tracing.cuh"


/*
 * Traverse BVH and insert intersected shapes into AT (for shadows)
 */
template <typename BVHNodeType>
__device__ bool traverseHybridBVH(BVHNodeType *bvh, uint bvhSize, Ray ray, float3 &color, float &transmitance) {
    
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
                    if(transmitance > TRANSMITANCE_LIMIT) {
                        intersectionFound = intersection(ray, &curr, childL->shape);

                        if(intersectionFound) {
                            transmitance *= curr.shapeMaterial.transparency;
                            color *= curr.shapeMaterial.color;
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
                    if(transmitance > TRANSMITANCE_LIMIT) {
                        intersectionFound = intersection(ray, &curr, childR->shape);

                        if(intersectionFound) {
                            transmitance *= curr.shapeMaterial.transparency;
                            color *= curr.shapeMaterial.color;
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
__device__ bool traverse(BVHNodeType *bvh, uint bvhSize, Ray ray, float3 &color, float &transmitance) {

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
                    if(transmitance > TRANSMITANCE_LIMIT) {
                        intersectionFound = intersection(ray, &curr, childL->shape);

                        if(intersectionFound) {
                            transmitance *= curr.shapeMaterial.transparency;
                            color *= curr.shapeMaterial.color;
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
            
            rIntersection = AABBIntersection(ray, tmp.min, tmp.max);
            
            if (rIntersection) {
                // Leaf node
                if (childR->shape != nullptr) {
                    if(transmitance > TRANSMITANCE_LIMIT) {
                        intersectionFound = intersection(ray, &curr, childR->shape);

                        if(intersectionFound) {
                            transmitance *= curr.shapeMaterial.transparency;
                            color *= curr.shapeMaterial.color;
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


__device__ float3 computeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler, float3 lightColor) {
    bool intersectionFound = false;

    float3 color = lightColor;
    float transmitance = 1.0f;

    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        if(d_shapeSizes[shapeType] == 0) {
            continue;
        }

        if(shapeType == sphereIndex && transmitance > TRANSMITANCE_LIMIT) {
            SphereNode *bvh = (SphereNode*) d_shapes[shapeType];

            intersectionFound |= traverse(bvh, d_shapeSizes[cylinderIndex], feeler, color, transmitance);
               
        } else if(shapeType == cylinderIndex && transmitance > TRANSMITANCE_LIMIT) {
            CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

            intersectionFound |= traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, color, transmitance);

        } else if(shapeType == triangleIndex && transmitance > TRANSMITANCE_LIMIT) {
            TriangleNode *bvh = (TriangleNode*) d_shapes[shapeType];

            intersectionFound |= traverse(bvh, d_shapeSizes[cylinderIndex], feeler, color, transmitance);

        } else if(shapeType == planeIndex && transmitance > TRANSMITANCE_LIMIT) {
            Plane *plane = (Plane*) d_shapes[shapeType];
            bool result = false;
            RayIntersection curr = RayIntersection();

            result = intersection(feeler, &curr, plane[0]);
            
            if(result) {
                transmitance *= curr.shapeMaterial.transparency;
                color *= curr.shapeMaterial.color;
            }

            intersectionFound |= result;
        }
    }

    return color * transmitance;
}

__device__ float3 cylComputeTransparency(int **d_shapes, uint *d_shapeSizes, Ray feeler, float3 lightColor) {
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];

    float3 color = lightColor;
    float transmitance = 1.0f;

    traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler, color, transmitance);

    return color * transmitance;
}

__device__ float3 computeShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, 
                                 RayIntersection intersect, uint li, float3 feelerDir) {

    Ray feeler = Ray(intersect.point, feelerDir);
    bool result = false;

    #ifndef SHADOW_TRANSMITANCE
    #ifdef GENERAL_INTERSECTION
    bool inShadow = findShadow(d_shapes, d_shapeSizes, feeler);

    #else
    bool inShadow = cylFindShadow(d_shapes, d_shapeSizes, feeler);
    #endif
            
	result = !inShadow;

    #else

    #ifdef GENERAL_INTERSECTION
    float3 transmitance = computeTransparency(d_shapes, d_shapeSizes, feeler, lights[li].color);

    #else
    float3 transmitance = cylComputeTransparency(d_shapes, d_shapeSizes, feeler, lights[li].color);
    #endif
            
    result = length(transmitance) > 0.01f;
    #endif

    if(result) {
        Material *mat = &intersect.shapeMaterial;
        float3 reflectDir = reflect(-feelerDir, intersect.normal);
        float Lspec = powf(fmaxf(dot(reflectDir, -ray.direction), 0.0f), mat->shininess);
        float Ldiff = fmaxf(dot(feelerDir, intersect.normal), 0.0f);

        #ifndef SHADOW_TRANSMITANCE
        return (Ldiff * mat->color * mat->Kdiffuse + Lspec * mat->Kspecular) * lights[li].color;

        #else
        return (Ldiff * mat->color * mat->Kdiffuse + Lspec * mat->Kspecular) * transmitance;

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
                    
            localColor += computeShadows(d_shapes, d_shapeSizes, lights, ray,
                                         intersect, li, feelerDir);
		}
	}

    return SUM_FACTOR * localColor;
}


#endif;