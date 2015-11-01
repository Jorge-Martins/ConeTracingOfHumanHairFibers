#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_


#include <cfloat>

#include "BVH.cuh"

//size ray array
__device__ int const raysSize = 2 << (MAX_DEPTH - 1);

//reflection and refraction arrays 
__device__ int const sizeRRArrays = (2 << (MAX_DEPTH - 1)) - 1;

//size local array
__device__ int const raysPerPixel = (2 << MAX_DEPTH) - 1;



__device__
bool findShadow(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    bool intersectionFound = false;
    //tmp debug
    float distance = FLT_MAX;
    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        for (uint i = 0; i < d_shapeSizes[shapeType]; i++) {
            if(shapeType == sphereIndex) {
                SphereNode *sphereNode = (SphereNode*) d_shapes[shapeType];

                SphereNode *node = &sphereNode[i];
                intersectionFound = AABBIntersection(feeler, node->min, node->max, &distance);

                if(intersectionFound) {
                    intersectionFound = intersection(feeler, nullptr, node->shape);
                }
               
            } else if(shapeType == cylinderIndex) {
                CylinderNode *cylinderNode = (CylinderNode*) d_shapes[shapeType];
                uint leafOffset = d_shapeSizes[shapeType] - 1;

                CylinderNode *node = &cylinderNode[leafOffset + i];
                if(node->type == AABB) {
                    intersectionFound = AABBIntersection(feeler, node->min, node->max, &distance);
                } else {
                    intersectionFound = OBBIntersection(feeler, node->min, node->max, 
                                                        node->matrix, node->translation, &distance);
                }

                if(intersectionFound) {
                    intersectionFound = intersection(feeler, nullptr, node->shape);
                }

            } else if(shapeType == triangleIndex) {
                TriangleNode *triangleNode = (TriangleNode*) d_shapes[shapeType];

                TriangleNode *node = &triangleNode[i];
                intersectionFound = AABBIntersection(feeler, node->min, node->max, &distance);

                if(intersectionFound) {
                    intersectionFound = intersection(feeler, nullptr, node->shape);
                }

            } else if(shapeType == planeIndex) {
                Plane *plane = (Plane*) d_shapes[shapeType];
                intersectionFound = intersection(feeler, nullptr, plane[i]);

            } else {
                return false;
            }

            if(intersectionFound) {
                return true;
            } 
	    }
    }

    return intersectionFound;
}

__device__
bool nearestIntersect(int **d_shapes, uint *d_shapeSizes, Ray ray, RayIntersection *out) {
	RayIntersection minIntersect(FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool minIntersectionFound = false, intersectionFound = false;

	RayIntersection curr = minIntersect;
    //tmp debug
    float distance = FLT_MAX;
    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        for (uint i = 0; i < d_shapeSizes[shapeType]; i++) {
            if(shapeType == sphereIndex) {
                SphereNode *sphereNode = (SphereNode*) d_shapes[shapeType];

                SphereNode *node = &sphereNode[i];
                intersectionFound = AABBIntersection(ray, node->min, node->max, &distance);

                if(intersectionFound) {
                    intersectionFound = intersection(ray, &curr, node->shape);
                }
               
            } else if(shapeType == cylinderIndex) {
                CylinderNode *node = nullptr;
                
                CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

                bool bvhHit = traverse(node, bvh, d_shapeSizes[shapeType], ray);

                if(bvhHit) {
                    intersectionFound = intersection(ray, &curr, node->shape);
                }
                
                break;
                //uint leafOffset = d_shapeSizes[shapeType] - 1;

                
                /*CylinderNode *node = &cylinderNode[leafOffset + i];
                if(node->type == AABB) {
                    intersectionFound = AABBIntersection(ray, node->min, node->max);
                } else {
                    intersectionFound = OBBIntersection(ray, node->min, node->max, 
                                                        node->matrix, node->translation);
                }

                if(intersectionFound) {
                    intersectionFound = intersection(ray, &curr, node->shape);
                }*/

            } else if(shapeType == triangleIndex) {
                TriangleNode *triangleNode = (TriangleNode*) d_shapes[shapeType];

                TriangleNode *node = &triangleNode[i];
                intersectionFound = AABBIntersection(ray, node->min, node->max, &distance);

                if(intersectionFound) {
                    intersectionFound = intersection(ray, &curr, node->shape);
                }

            } else if(shapeType == planeIndex) {
                Plane *plane = (Plane*) d_shapes[shapeType];
                intersectionFound = intersection(ray, &curr, plane[i]);

            } else {
                return false;
            }

		    if (intersectionFound) {
                if (curr.distance < minIntersect.distance) {
                    minIntersectionFound = true;
				    minIntersect = curr;
			    }
		    }
	    }
    }
    
	if (minIntersectionFound) {
		*out = minIntersect;
	}
	return minIntersectionFound;
}


__device__
int rayIndex(int index) {
    return index % raysSize;
}

__device__
float3 refract(float3 inDir, float3 normal, float eta) {
    float cosi = dot(-inDir, normal);
    float cost2 = 1.0f - eta * eta * (1.0f - cosi*cosi);
    float3 t = eta*inDir + ((eta*cosi - sqrt(abs(cost2))) * normal);

    if(cost2 > 0) {
        return t;
    } 

    return make_float3(0.0f);
}

__device__
float3 computeTransmissionDir(float3 inDir, float3 normal, float beforeIOR, float afterIOR) {
	return refract(inDir, normal, beforeIOR / afterIOR);
}

__device__
float3 rayTracing(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                 float3 rayOrigin, float3 rayDirection, Ray *ray, float3* locals,
                 float3* reflectionCols, float3* refractionCols, uint offset) {

    uint rayOffset = offset * raysSize;                 
    uint localsOffset = offset * raysPerPixel;
    uint rrOffset = offset * sizeRRArrays;
    Ray feeler = Ray();

    ray[rayOffset].update(rayOrigin, rayDirection);
    
    RayIntersection intersect;
    int level;
    float3 blackColor = make_float3(0.0f);
    
    for(int rayN = 0; rayN < raysPerPixel; rayN++) {
        //skip secundary rays that don't exist
        if(!ray[rayOffset + rayIndex(rayN)].exists) {
            if(rayN < sizeRRArrays) {
                reflectionCols[rrOffset + rayN] = blackColor;
                refractionCols[rrOffset + rayN] = blackColor;
                level = 2 * rayN;
                ray[rayOffset + rayIndex(level + 1)].exists = false;
                ray[rayOffset + rayIndex(level + 2)].exists = false;
            }
            continue;
        }

	    bool foundIntersect = nearestIntersect(d_shapes, d_shapeSizes, ray[rayOffset + rayIndex(rayN)], &intersect);

	    if (!foundIntersect) {
            if(rayN == 0) {
                return backcolor;
            }

            locals[localsOffset + rayN] = backcolor;

            if(rayN < sizeRRArrays) {
                reflectionCols[rrOffset + rayN] = blackColor;
                refractionCols[rrOffset + rayN] = blackColor;
                level = 2 * rayN;
                ray[rayOffset + rayIndex(level + 1)].exists = false;
                ray[rayOffset + rayIndex(level + 2)].exists = false;
            }
            continue;
        } 

        Material mat = intersect.shapeMaterial;
         
        // local illumination
	    locals[localsOffset + rayN] = blackColor;
	    for(uint li = 0; li < lightSize; li++) {
		    float3 feelerDir = normalize(lights[li].position - intersect.point);
            feeler.update(intersect.point, feelerDir);
            
            bool inShadow = findShadow(d_shapes, d_shapeSizes, feeler);
                
		    if(!inShadow) {
                float3 reflectDir = reflect(-feelerDir, intersect.normal);
                float Lspec = powf(fmaxf(dot(reflectDir, -ray[rayOffset + rayIndex(rayN)].direction), 0.0f), 
                                         mat.shininess);
                float Ldiff = fmaxf(dot(feelerDir, intersect.normal), 0.0f);

			    
                locals[localsOffset + rayN] +=  (Ldiff * mat.color * mat.Kdiffuse + mat.color * Lspec * mat.Kspecular) * lights[li].color;
		    }
	    }
    
        if(rayN < sizeRRArrays) {
            reflectionCols[rrOffset + rayN] = blackColor;
            refractionCols[rrOffset + rayN] = blackColor;
            level = 2 * rayN;
            ray[rayOffset + rayIndex(level + 1)].exists = false;
            ray[rayOffset + rayIndex(level + 2)].exists = false;
            // reflection
            level = 2 * rayN + 1;
	        if(mat.Kspecular > 0.0f) {
                float3 reflectDir = reflect(ray[rayOffset + rayIndex(rayN)].direction, intersect.normal);
		        ray[rayOffset + rayIndex(level)].update(intersect.point, reflectDir);
                reflectionCols[rrOffset + rayN] = mat.color * mat.Kspecular;
            }

	        // transmission
            level = 2 * rayN + 2;
	        if(mat.transparency > 0.0f) {
		        float ior1, ior2;
		        if(intersect.isEntering) {
			        ior1 = 1.0f;
			        ior2 = mat.ior;
		        }
		        else {
			        ior1 = mat.ior;
			        ior2 = 1.0f;
		        }
		        float3 refractionDir = computeTransmissionDir(ray[rayOffset + rayIndex(rayN)].direction, intersect.normal, ior1, ior2);
		            
                if (!equal(length(refractionDir), 0.0f)) {
			        ray[rayOffset + rayIndex(level)].update(intersect.point, refractionDir);
                    refractionCols[rrOffset + rayN] = mat.color * mat.transparency;
                }
	        }
        }
    }

    int startLevel = sizeRRArrays - 1;
    int rrLevel = -2;

    if(MAX_DEPTH > 2) {
        rrLevel += 2 << (MAX_DEPTH - 2);
    }

    for(int i = startLevel; i >= 0 && i > rrLevel; i--) {
        level = 2 * i;       
        reflectionCols[rrOffset + i] *= locals[localsOffset + level + 1];
        refractionCols[rrOffset + i] *= locals[localsOffset + level + 2];
    }

    for(int i = rrLevel; i >= 0; i--) {
        level = 2 * i;
        locals[localsOffset + level + 1] += reflectionCols[rrOffset + level + 1] + refractionCols[rrOffset + level + 1];
        locals[localsOffset + level + 2] += reflectionCols[rrOffset + level + 2] + refractionCols[rrOffset + level + 2];
        
        reflectionCols[rrOffset + i] *= locals[localsOffset + level + 1];
        refractionCols[rrOffset + i] *= locals[localsOffset + level + 2];
    }

    return locals[localsOffset] + reflectionCols[rrOffset] + refractionCols[rrOffset];
}


__global__
void drawScene(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, int resX,
               int resY, int res_xy, float width, float height, float atDistance, float3 xe, float3 ye, 
               float3 ze, float3 from, float3 *d_output, Ray* ray, float3* d_locals, 
               float3* d_reflectionCols, float3* d_refractionCols) {

    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    uint index = y * (resX * SUPER_SAMPLING) + x;

    if(index < res_xy * SUPER_SAMPLING * SUPER_SAMPLING) {
        float3 zeFactor = -ze * atDistance; 
        float3 yeFactor = ye * height * ((y + 0.5f) / (float)(resY * SUPER_SAMPLING) - 0.5f);
	    float3 xeFactor = xe * width * ((x + 0.5f) / (float)(resX * SUPER_SAMPLING) - 0.5f);

	    float3 direction = normalize(zeFactor + yeFactor + xeFactor);
	
        float3 color = rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                  ray, d_locals, d_reflectionCols, d_refractionCols, index);
    
        if(SUPER_SAMPLING > 1) {
            index = (uint)(y / (float)SUPER_SAMPLING) * resX + (uint)(x / (float)SUPER_SAMPLING);
            d_output[index] += color; // (SUPER_SAMPLING * SUPER_SAMPLING);
        } else {
            d_output[index] = color;
        }
    }
}

__global__
void clearImage(float3 *d_output, float3 value, int resX, int res) {
    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    uint index = y * resX + x;

    if(index < res) {
        d_output[index] = value;
    }
}

void deviceClearImage(float3 *d_output, float3 value, int resX, int resY, dim3 gridSize, dim3 blockSize) {
    clearImage<<<gridSize, blockSize>>>(d_output, make_float3(0.0f), resX, resX * resY);
}

void deviceDrawScene(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 ssgridSize, dim3 blockSize, Ray* ray,
                     float3* d_locals, float3* d_reflectionCols, float3* d_refractionCols) {
    
    int res_xy = resX * resY;
    drawScene<<<ssgridSize, blockSize>>>(d_shapes, d_shapeSizes, lights, lightSize, backcolor, resX, resY,
                                       res_xy, width, height, atDistance, xe, ye, ze, from, d_output, ray,
                                       d_locals, d_reflectionCols, d_refractionCols);

}


void deviceBuildBVH(CylinderNode *bvh, uint nObjects, dim3 gridSize, dim3 blockSize) {

    buildBVH<<<gridSize, blockSize>>>(bvh, nObjects);

    computeBVHBB<<<gridSize, blockSize>>>(bvh, nObjects);
}
#endif