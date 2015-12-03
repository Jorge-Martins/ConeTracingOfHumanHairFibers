#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_


#include <cfloat>

#include "BVH.cuh"

#define rtStackSize (2 * MAX_DEPTH)

__device__
float3 printRayHairIntersections(int rayHairIntersections, float3 finalColor, int nRays) {
    if(rayHairIntersections == 0) {
        return finalColor;
    }

    if(rayHairIntersections < 5 * nRays) {
        return make_float3(0, 0, 10);

    } else if (rayHairIntersections < 10 * nRays) {
        return make_float3(0, 10, 0);

    } else if(rayHairIntersections < 20 * nRays) {
        return make_float3(10, 10, 0);

    } else if(rayHairIntersections < 40 * nRays) {
        return make_float3(10, 0, 0);

    }

    return make_float3(0.5, 0, 0);
}

__device__
bool findShadow(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    bool intersectionFound = false;
    
    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        for (uint i = 0; i < d_shapeSizes[shapeType]; i++) {
            if(shapeType == sphereIndex) {
                SphereNode *sphereNode = (SphereNode*) d_shapes[shapeType];

                SphereNode *node = &sphereNode[i];
                intersectionFound = AABBIntersection(feeler, node->min, node->max);

                if(intersectionFound) {
                    intersectionFound = intersection(feeler, nullptr, node->shape);
                }
               
            } else if(shapeType == cylinderIndex) {
                CylinderNode *cylinderNode = (CylinderNode*) d_shapes[shapeType];
                uint leafOffset = d_shapeSizes[shapeType] - 1;

                CylinderNode *node = &cylinderNode[leafOffset + i];
                if(node->type == AABB) {
                    intersectionFound = AABBIntersection(feeler, node->min, node->max);
                } else {
                    intersectionFound = OBBIntersection(feeler, node->min, node->max, 
                                                        node->matrix, node->translation);
                }

                if(intersectionFound) {
                    intersectionFound = intersection(feeler, nullptr, node->shape);
                }

            } else if(shapeType == triangleIndex) {
                TriangleNode *triangleNode = (TriangleNode*) d_shapes[shapeType];

                TriangleNode *node = &triangleNode[i];
                intersectionFound = AABBIntersection(feeler, node->min, node->max);

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
bool cylFindShadow(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];

    return traverseShadow(bvh, d_shapeSizes[cylinderIndex], feeler);
}

__device__
bool cylNearestIntersect(int **d_shapes, uint *d_shapeSizes, Ray ray, RayIntersection *out, int *rayHairIntersections) {
	RayIntersection minIntersect(FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool intersectionFound = false;
    
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];

    intersectionFound = traverse(bvh, d_shapeSizes[cylinderIndex], ray, &minIntersect, rayHairIntersections);

    if(intersectionFound) {      
        *out = minIntersect;
	}

    return intersectionFound;
}

__device__
bool nearestIntersect(int **d_shapes, uint *d_shapeSizes, Ray ray, RayIntersection *out, int *rayHairIntersections) {
	RayIntersection minIntersect(FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool minIntersectionFound = false, intersectionFound = false;

	RayIntersection curr = minIntersect;
    
    for(uint shapeType = 0; shapeType < nShapes; shapeType++) {
        for (uint i = 0; i < d_shapeSizes[shapeType]; i++) {
            if(shapeType == sphereIndex) {
                SphereNode *sphereNode = (SphereNode*) d_shapes[shapeType];

                SphereNode *node = &sphereNode[i];
                intersectionFound = AABBIntersection(ray, node->min, node->max);

                if(intersectionFound) {
                    intersectionFound = intersection(ray, &curr, node->shape);
                }
               
            } else if(shapeType == cylinderIndex) {
                CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

                intersectionFound = traverse(bvh, d_shapeSizes[shapeType], ray, &minIntersect, rayHairIntersections);

                if(intersectionFound) {
                    minIntersectionFound = true;
		        }

                break;
                
            } else if(shapeType == triangleIndex) {
                TriangleNode *triangleNode = (TriangleNode*) d_shapes[shapeType];

                TriangleNode *node = &triangleNode[i];
                intersectionFound = AABBIntersection(ray, node->min, node->max);

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
float3 refract(float3 inDir, float3 normal, float eta) {
    float cosi = dot(-inDir, normal);
    float k = 1.0f - eta * eta * (1.0f - cosi*cosi);

    if(k > 0) {
        return eta*inDir + (eta*cosi - sqrtf(k)) * normal;
    } 

    return make_float3(0.0f);
}

__device__
float3 computeTransmissionDir(float3 inDir, float3 normal, float beforeIOR, float afterIOR) {
	return refract(inDir, normal, beforeIOR / afterIOR);
}

__device__
float3 computeShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, Ray feeler, float3 blackColor, Material mat, 
                      RayIntersection intersect, uint li, float3 feelerDir) {
    feeler.update(intersect.point, feelerDir);
            
    //bool inShadow = findShadow(d_shapes, d_shapeSizes, feeler);
    bool inShadow = cylFindShadow(d_shapes, d_shapeSizes, feeler);
            
	if(!inShadow) {
        float3 reflectDir = reflect(-feelerDir, intersect.normal);
        float Lspec = powf(fmaxf(dot(reflectDir, -ray.direction), 0.0f), mat.shininess);
        float Ldiff = fmaxf(dot(feelerDir, intersect.normal), 0.0f);

        return (Ldiff * mat.color * mat.Kdiffuse + mat.color * Lspec * mat.Kspecular) * lights[li].color;
	}

    return blackColor;
}

__device__
float3 computeSoftShadows(int **d_shapes, uint *d_shapeSizes, Light* lights, Ray ray, Ray feeler, float3 blackColor, 
                          Material mat, RayIntersection intersect, uint li, float3 feelerDir) {
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
            
    float3 localColor = blackColor;
    for (int x = 0; x < LIGHT_SAMPLE_RADIUS; x++) {
		for (int y = 0; y < LIGHT_SAMPLE_RADIUS; y++) {
			float xCoord = LIGHT_SOURCE_SIZE * ((y + 0.5f) / LIGHT_SAMPLE_RADIUS - 0.5f);
			float yCoord = LIGHT_SOURCE_SIZE * ((x + 0.5f) / LIGHT_SAMPLE_RADIUS - 0.5f);

			feelerDir = normalize((lights[li].position + xCoord*u + yCoord*v) - intersect.point);
                    
            localColor += SUM_FACTOR * computeShadows(d_shapes, d_shapeSizes, lights, ray, feeler,
                                         blackColor, mat, intersect, li, feelerDir);
		}
	}

    return localColor;
}



__device__
float3 rayTracing(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                 float3 rayOrigin, float3 rayDirection, RayInfo *globalRayInfo, float3 *globalColors, 
                 unsigned char *globalColorContributionType, uint offset) {

    RayInfo *rayInfoStack = &globalRayInfo[offset * rtStackSize];

    float3 *localsStack = &globalColors[3 * offset * rtStackSize];
    float3 *reflectionColsStack = &localsStack[rtStackSize];
    float3 *refractionColsStack = &reflectionColsStack[rtStackSize];

    unsigned char *colorContributionType = &globalColorContributionType[offset * rtStackSize];

    Ray feeler = Ray();
    Ray ray = Ray();

    int rayIndex = 0, colorsIndex = 0, contributionIndex = 0;
    RayInfo info;
    RayIntersection intersect;

    rayInfoStack[rayIndex++].update(rayOrigin, rayDirection);
    float3 blackColor = make_float3(0.0f);
    float3 colorAux;
    bool computeColor = false;

    int rayHairIntersections = 0, nRays = 0;
    
    while(rayIndex > 0) {
        reflectionColsStack[colorsIndex] = blackColor;
        refractionColsStack[colorsIndex] = blackColor;
        nRays++;
        info = rayInfoStack[--rayIndex];
        colorContributionType[contributionIndex++] = info.type;
        ray.update(info.origin, info.direction);

        //bool foundIntersect = nearestIntersect(d_shapes, d_shapeSizes, ray, &intersect, &rayHairIntersections);
	    bool foundIntersect = cylNearestIntersect(d_shapes, d_shapeSizes, ray, &intersect, &rayHairIntersections);
        
	    if (!foundIntersect) {     
            localsStack[colorsIndex] = backcolor;
            computeColor = true;
            
        } else {
            Material mat = intersect.shapeMaterial;

            // local illumination
            colorAux = blackColor;
	        for(uint li = 0; li < lightSize; li++) {
                /*colorAux += computeShadows(d_shapes, d_shapeSizes, lights, ray, feeler, blackColor, mat, intersect,
                                           li, normalize(lights[li].position - intersect.point));*/
                colorAux += computeSoftShadows(d_shapes, d_shapeSizes, lights, ray, feeler, blackColor, mat, intersect,
                                                 li, normalize(lights[li].position - intersect.point));
	        }
            localsStack[colorsIndex] = colorAux;
     
            computeColor = true;
            if(info.depth < MAX_DEPTH) {
                // reflection
                colorAux = blackColor;
	            if(mat.Kspecular > 0.0f) {
                    float3 reflectDir = reflect(ray.direction, intersect.normal);
                    rayInfoStack[rayIndex++].update(intersect.point, reflectDir, REFLECTED, info.depth + 1);
                    colorAux = mat.color * mat.Kspecular;
                    computeColor = false;
                }
                reflectionColsStack[colorsIndex] = colorAux;

	            // transmission
                colorAux = blackColor;
	            if(mat.transparency > 0.0f) {
		            float ior1, ior2;
		            if(intersect.isEntering) {
			            ior1 = 1.0f;
			            ior2 = mat.ior;

		            } else {
			            ior1 = mat.ior;
			            ior2 = 1.0f;
		            }
		            float3 refractionDir = computeTransmissionDir(ray.direction, intersect.normal, ior1, ior2);
		            
                    if (!equal(length(refractionDir), 0.0f)) {
			            rayInfoStack[rayIndex++].update(intersect.point, refractionDir, REFRACTED, info.depth + 1);
                        colorAux = mat.color * mat.transparency;
                        computeColor = false;
                    }
	            }
                refractionColsStack[colorsIndex] = colorAux;

                if(!computeColor) {
                    colorsIndex++;
                }
            }
        }

        if(computeColor && info.type != PRIMARY) {
            if(info.type == REFLECTED) {
                unsigned char type;
                int prevColorsIndex;
                while(1) {
                    colorAux = localsStack[colorsIndex] + reflectionColsStack[colorsIndex] + refractionColsStack[colorsIndex];

                    type = colorContributionType[--contributionIndex];
                    prevColorsIndex = colorsIndex - 1;
                    if(type == REFLECTED) {
                        reflectionColsStack[prevColorsIndex] *= colorAux;
                        
                    } else if(type == REFRACTED) {
                        refractionColsStack[prevColorsIndex] *= colorAux;
                        
                    } 

                    if(prevColorsIndex <= rayIndex) {
                        break;
                    } 

                    colorsIndex--;
                }

            } else {
                colorAux = localsStack[colorsIndex] + reflectionColsStack[colorsIndex] + refractionColsStack[colorsIndex];
                refractionColsStack[colorsIndex - 1] *= colorAux;
                contributionIndex--;
            }
        }

        computeColor = false;
    }

    
    colorAux = localsStack[0] + reflectionColsStack[0] + refractionColsStack[0];

    //return printRayHairIntersections(rayHairIntersections, colorAux, nRays);

    return colorAux;
}

__global__
void drawScene(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, int resX,
               int resY, int res_xy, float width, float height, float atDistance, float3 xe, float3 ye, 
               float3 ze, float3 from, float3 *d_output, RayInfo* rayInfo, float3* d_colors, 
               unsigned char *d_colorContributionType) {

    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    uint index = y * resX + x;

    if(index >= res_xy) {
        return;
    }

    float3 zeFactor = -ze * atDistance; 
    float3 yeFactor, xeFactor, direction, color = make_float3(0.0f);
    for(int sx = 0; sx < SUPER_SAMPLING; sx++) {
        for(int sy = 0; sy < SUPER_SAMPLING; sy++) {

            yeFactor = ye * height * ((y + (sy + 0.5f) * SUPER_SAMPLING_F) / (float)resY - 0.5f);
            xeFactor = xe * width * ((x + (sx + 0.5f) * SUPER_SAMPLING_F) / (float)resX - 0.5f);

            direction = normalize(zeFactor + yeFactor + xeFactor);

            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     rayInfo, d_colors, d_colorContributionType, index);
        }
    }
    
    d_output[index] = color;
    
}

__global__
void clearImage(float3 *d_output, float3 value, int resX, int res) {
    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    uint index = y * resX + x;

    if(index >= res) {
        return;
    }
    d_output[index] = value;

}

void deviceClearImage(float3 *d_output, float3 value, int resX, int resY, dim3 gridSize, dim3 blockSize) {
    clearImage<<<gridSize, blockSize>>>(d_output, make_float3(0.0f), resX, resX * resY);
}

void deviceDrawScene(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize, RayInfo* rayInfo,
                     float3* d_colors, unsigned char *d_colorContributionType) {
    
    int res_xy = resX * resY;
    drawScene<<<gridSize, blockSize>>>(d_shapes, d_shapeSizes, lights, lightSize, backcolor, resX, resY,
                                       res_xy, width, height, atDistance, xe, ye, ze, from, d_output, rayInfo,
                                       d_colors, d_colorContributionType);

}


void deviceBuildBVH(CylinderNode *bvh, uint nObjects, dim3 gridSize, dim3 blockSize, uint *mortonCodes) {
    float *areaVector, *costVector;
    int *lock, *nodeCounter;

    uint size = (2 * nObjects - 1) * sizeof(float); 
    checkCudaErrors(cudaMalloc((void**) &areaVector, size));
    checkCudaErrors(cudaMemset(areaVector, INT_MAX, size));

    checkCudaErrors(cudaMalloc((void**) &costVector, size));
    checkCudaErrors(cudaMemset(costVector, INT_MAX, size));

    size = (2 * nObjects - 1) * sizeof(int); 
    checkCudaErrors(cudaMalloc((void**) &lock, size));
    checkCudaErrors(cudaMemset(lock, 0, size));

    
    buildBVH<<<gridSize, blockSize>>>(bvh, nObjects, mortonCodes);

    computeBVHBB<<<gridSize, blockSize>>>(bvh, nObjects, lock);

    nodeCounter = lock;
    size = nObjects * sizeof(int);
    checkCudaErrors(cudaMemset(nodeCounter, 0, size));
    optimizeBVH<<<gridSize, blockSize>>>(bvh, nObjects, nodeCounter, areaVector, costVector);

    computeLeavesOBBs<<<gridSize, blockSize>>>(bvh, nObjects);

    checkCudaErrors(cudaFree(lock));
    checkCudaErrors(cudaFree(areaVector));
    checkCudaErrors(cudaFree(costVector));
    checkCudaErrors(cudaFree(mortonCodes));
}

#endif