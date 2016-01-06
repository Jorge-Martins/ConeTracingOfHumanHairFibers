#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_

#define GENERAL_INTERSECTION
//#define SOFT_SHADOWS

#include <cfloat>

#include "AOIT.cuh"
#include <thrust/random.h>

#define rtStackSize (2 * MAX_DEPTH)


__device__
float2 cudaRandom(thrust::default_random_engine &rng) {
    return make_float2((float)rng() ,
                       (float)rng());
}

__device__
float haltonSequance(int index, int base) {
       float result = 0;
       float f = 1;
      
       for(int i = index; i > 0; i = floorf(i / base)) {
           f = f / base;
           result = result + f * (i % base);
           
       }

       return result;
}

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
        if(d_shapeSizes[shapeType] == 0) {
            continue;
        }

        if(shapeType == sphereIndex) {
            SphereNode *bvh = (SphereNode*) d_shapes[shapeType];

            intersectionFound |= traverseShadow(bvh, d_shapeSizes[shapeType], feeler);
               
        } else if(shapeType == cylinderIndex) {
            CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

            intersectionFound |= traverseShadowHybridBVH(bvh, d_shapeSizes[shapeType], feeler);

        } else if(shapeType == triangleIndex) {
            TriangleNode *bvh = (TriangleNode*) d_shapes[shapeType];

            intersectionFound |= traverseShadow(bvh, d_shapeSizes[shapeType], feeler);

        } else if(shapeType == planeIndex) {
            Plane *plane = (Plane*) d_shapes[shapeType];

            intersectionFound |= intersection(feeler, nullptr, plane[0]);
        }
    }

    return intersectionFound;
}

__device__
bool cylFindShadow(int **d_shapes, uint *d_shapeSizes, Ray feeler) {
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];

    return traverseShadowHybridBVH(bvh, d_shapeSizes[cylinderIndex], feeler);
}

__device__
bool cylNearestIntersect(int **d_shapes, uint *d_shapeSizes, Ray ray, RayIntersection *out, int *rayHairIntersections) {
	RayIntersection minIntersect(FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool intersectionFound = false;
    
    CylinderNode *bvh = (CylinderNode*) d_shapes[cylinderIndex];

    intersectionFound = traverseHybridBVH(bvh, d_shapeSizes[cylinderIndex], ray, &minIntersect, rayHairIntersections);

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
        if(d_shapeSizes[shapeType] == 0) {
            continue;
        }

        if(shapeType == sphereIndex) {
            SphereNode *bvh = (SphereNode*) d_shapes[shapeType];

            intersectionFound = traverse(bvh, d_shapeSizes[shapeType], ray, &minIntersect, rayHairIntersections);

            if(intersectionFound) {
                minIntersectionFound = true;
		    }
               
        } else if(shapeType == cylinderIndex) {
            CylinderNode *bvh = (CylinderNode*) d_shapes[shapeType];

            intersectionFound = traverseHybridBVH(bvh, d_shapeSizes[shapeType], ray, &minIntersect, rayHairIntersections);

            if(intersectionFound) {
                minIntersectionFound = true;
		    }

        } else if(shapeType == triangleIndex) {
            TriangleNode *bvh = (TriangleNode*) d_shapes[shapeType];

            intersectionFound = traverse(bvh, d_shapeSizes[shapeType], ray, &minIntersect, rayHairIntersections);

            if(intersectionFound) {
                minIntersectionFound = true;
		    }

        } else if(shapeType == planeIndex) {
            Plane *plane = (Plane*) d_shapes[shapeType];
            intersectionFound = intersection(ray, &curr, plane[0]);

            if (intersectionFound && curr.distance < minIntersect.distance) {
                minIntersectionFound = true;
				minIntersect = curr;
			}
		    

        } else {
            return false;
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
            
    #ifdef GENERAL_INTERSECTION
    bool inShadow = findShadow(d_shapes, d_shapeSizes, feeler);
    #else
    bool inShadow = cylFindShadow(d_shapes, d_shapeSizes, feeler);
    #endif
            
	if(!inShadow) {
        float3 reflectDir = reflect(-feelerDir, intersect.normal);
        float Lspec = powf(fmaxf(dot(reflectDir, -ray.direction), 0.0f), mat.shininess);
        float Ldiff = fmaxf(dot(feelerDir, intersect.normal), 0.0f);

        return (Ldiff * mat.color * mat.Kdiffuse + Lspec * mat.Kspecular) * lights[li].color;
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
			float xCoord = LIGHT_SOURCE_SIZE * ((y + 0.5f) * LIGHT_SAMPLE_RADIUS_F - 0.5f);
			float yCoord = LIGHT_SOURCE_SIZE * ((x + 0.5f) * LIGHT_SAMPLE_RADIUS_F - 0.5f);

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

        #ifdef GENERAL_INTERSECTION 
        bool foundIntersect = nearestIntersect(d_shapes, d_shapeSizes, ray, &intersect, &rayHairIntersections);
        #else
	    bool foundIntersect = cylNearestIntersect(d_shapes, d_shapeSizes, ray, &intersect, &rayHairIntersections);
        #endif

	    if (!foundIntersect) {     
            localsStack[colorsIndex] = backcolor;
            computeColor = true;
            
        } else {
            Material mat = intersect.shapeMaterial;

            // local illumination
            colorAux = blackColor;
	        for(uint li = 0; li < lightSize; li++) {
                #ifndef SOFT_SHADOWS
                colorAux += computeShadows(d_shapes, d_shapeSizes, lights, ray, feeler, blackColor, mat, intersect,
                                           li, normalize(lights[li].position - intersect.point));
                #else
                colorAux += computeSoftShadows(d_shapes, d_shapeSizes, lights, ray, feeler, blackColor, mat, intersect,
                                                 li, normalize(lights[li].position - intersect.point));
                #endif
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

__device__
float3 naiveSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                          float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                          unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY) {

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int sx = 0; sx < SUPER_SAMPLING; sx++) {
        for(int sy = 0; sy < SUPER_SAMPLING; sy++) {
            yeFactor = ye * ((y + (sy + 0.5f) * SUPER_SAMPLING_F) / (float)resY - 0.5f);
            xeFactor = xe * ((x + (sx + 0.5f) * SUPER_SAMPLING_F) / (float)resX - 0.5f);

            direction = normalize(zeFactor + yeFactor + xeFactor);

            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     rayInfo, d_colors, d_colorContributionType, index);
        }
    }

    return color;
}

__device__
float3 naiveRdmSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                             float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                             unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                             long seed) {

    thrust::default_random_engine rng(seed + index);
    thrust::uniform_real_distribution<float> uniDist;
    rng.discard(2 * index);

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int sx = 0; sx < SUPER_SAMPLING; sx++) {
        for(int sy = 0; sy < SUPER_SAMPLING; sy++) {
            yeFactor = ye * ((y + (sy + uniDist(rng)) * SUPER_SAMPLING_F) / (float)resY - 0.5f);
            xeFactor = xe * ((x + (sx + uniDist(rng)) * SUPER_SAMPLING_F) / (float)resX - 0.5f);

            direction = normalize(zeFactor + yeFactor + xeFactor);

            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     rayInfo, d_colors, d_colorContributionType, index);
        }
    }

    return color;
}

__device__
float3 stocasticSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                              float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                              unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                              long seed) {

    thrust::default_random_engine rng(seed + index);
    thrust::uniform_real_distribution<float> uniDist;

    rng.discard(2 * index);

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int i = 0; i < SUPER_SAMPLING_2; i++) {
        yeFactor = ye * ((y + uniDist(rng)) / (float)resY - 0.5f);
        xeFactor = xe * ((x + uniDist(rng)) / (float)resX - 0.5f);

        direction = normalize(zeFactor + yeFactor + xeFactor);
        color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                 rayInfo, d_colors, d_colorContributionType, index);
    }

    return color;
}

__device__
float3 stocasticHSSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                                float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                                unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                                long seed) {

    uint hsIndex = index + seed;

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int i = 0; i < SUPER_SAMPLING_2; i++) {
        yeFactor = ye * ((y + haltonSequance(hsIndex + i, 3)) / (float)resY - 0.5f);
        xeFactor = xe * ((x + haltonSequance(hsIndex + i, 2)) / (float)resX - 0.5f);

        direction = normalize(zeFactor + yeFactor + xeFactor);

        color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                 rayInfo, d_colors, d_colorContributionType, index);
    }

    return color;
}

__device__
float3 adaptiveStocasticSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                                      float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                                      unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                                      long seed, int initNSamples) {

    thrust::default_random_engine rng(seed + index);
    thrust::uniform_real_distribution<float> uniDist;

    rng.discard(2 * index);

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor, tmp, ref;
    
    int samplingLimit = initNSamples;
    int step = 2;
    float factor = fmaxf(1.0f / samplingLimit, SUPER_SAMPLING_2_F);
    float difValue = 0.0f, threashold = initNSamples * 0.001f;

    ref = backcolor;

    for(int i = 0; i < SUPER_SAMPLING_2 && i < samplingLimit; i++) {
        yeFactor = ye * ((y + uniDist(rng)) / (float)resY - 0.5f);
        xeFactor = xe * ((x + uniDist(rng)) / (float)resX - 0.5f);

        direction = normalize(zeFactor + yeFactor + xeFactor);

        tmp = rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                         rayInfo, d_colors, d_colorContributionType, index);

        color += factor * tmp;

        difValue += length(ref - tmp);

        if(i == samplingLimit - 1 && samplingLimit * step <= SUPER_SAMPLING_2 && difValue >= threashold) {
            samplingLimit *= step;
            ref = color;
            color /= step;

            factor = 1.0f / samplingLimit;
            threashold *= step;
        }
    }

    //color debug output
    /*if(samplingLimit <= initNSamples) {
        return make_float3(0.76, 1, 0.96);
    } else if (samplingLimit <= 2 * initNSamples) {
        return make_float3(0, 0, 1);
    } else if(samplingLimit <= 4 * initNSamples) {
        return make_float3(0.6, 1, 0.6);
    } else if(samplingLimit <= 8 * initNSamples) {
        return make_float3(0, 1, 0);
    } else if(samplingLimit <= 16 * initNSamples) {
        return make_float3(1, 1, 0);
    } else if(samplingLimit <= 32 * initNSamples) {
        return make_float3(10, 0, 0);
    } else {
        return make_float3(0.5, 0, 0);
    }*/

    return color;
}


__global__
void drawScene(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, int resX,
               int resY, float atDistance, float3 xe, float3 ye, float3 ze, float3 from, float3 *d_output,
               RayInfo* rayInfo, float3* d_colors, unsigned char *d_colorContributionType, long seed) {

    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= resX || y >= resY) {
        return;
    }

    uint index = y * resX + x;

    d_output[index] = naiveSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                         from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                         resY);
    
    /*d_output[index] = naiveRdmSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                            from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                            resY, seed);*/

    /*d_output[index] = stocasticSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                             from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                             resY, seed);*/

    /*d_output[index] = adaptiveStocasticSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                                     from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                                     resY, seed, 16);*/

    /*d_output[index] = stocasticHSSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                               from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                               resY, seed);*/

}


void deviceDrawScene(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize, RayInfo* rayInfo,
                     float3* d_colors, unsigned char *d_colorContributionType, long seed) {
    
    
    ye *= height;
    xe *= width;
    ze = -ze * atDistance;
    drawScene<<<gridSize, blockSize>>>(d_shapes, d_shapeSizes, lights, lightSize, backcolor, resX, resY,
                                       atDistance, xe, ye, ze, from, d_output, rayInfo, d_colors, 
                                       d_colorContributionType, seed);

}


float deviceBuildCylinderBVH(CylinderNode *bvh, uint nObjects, dim3 gridSize, dim3 blockSize, uint *mortonCodes, 
                             cudaEvent_t &c_start, cudaEvent_t &c_end, Cylinder *d_shapes, Matrix *d_matrixes, 
                             float3 *d_translations, uint *d_OBBIndexes, uint nOBBs) {
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

    cudaEventRecord(c_start);
    buildBVH<<<gridSize, blockSize>>>(bvh, nObjects, mortonCodes);

    computeBVHBB<<<gridSize, blockSize>>>(bvh, nObjects, lock, d_shapes, d_matrixes, 
                                          d_translations, d_OBBIndexes, nOBBs);

    nodeCounter = lock;
    size = nObjects * sizeof(int);
    checkCudaErrors(cudaMemset(nodeCounter, 0, size));
    optimizeBVH<<<gridSize, blockSize>>>(bvh, nObjects, nodeCounter, areaVector, costVector);

    computeLeavesOBBs<<<gridSize, blockSize>>>(bvh, nObjects);
    cudaEventRecord(c_end);

    cudaEventSynchronize(c_end);

    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, c_start, c_end);

    checkCudaErrors(cudaFree(lock));
    checkCudaErrors(cudaFree(areaVector));
    checkCudaErrors(cudaFree(costVector));
    checkCudaErrors(cudaFree(mortonCodes));
    if(d_OBBIndexes != nullptr) {
        checkCudaErrors(cudaFree(d_OBBIndexes));
    }

    return milliseconds / 1000.0f;
}

template <typename BVHNodeType, typename ShapeType>
float deviceBuildBVH(BVHNodeType *bvh, uint nObjects, dim3 gridSize, dim3 blockSize, uint *mortonCodes, 
                     cudaEvent_t &c_start, cudaEvent_t &c_end, ShapeType *d_shapes) {
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

    cudaEventRecord(c_start);
    buildBVH<<<gridSize, blockSize>>>(bvh, nObjects, mortonCodes);

    computeBVHBB<<<gridSize, blockSize>>>(bvh, nObjects, lock, d_shapes);

    nodeCounter = lock;
    size = nObjects * sizeof(int);
    checkCudaErrors(cudaMemset(nodeCounter, 0, size));
    optimizeBVH<<<gridSize, blockSize>>>(bvh, nObjects, nodeCounter, areaVector, costVector);

    cudaEventRecord(c_end);

    cudaEventSynchronize(c_end);

    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, c_start, c_end);

    checkCudaErrors(cudaFree(lock));
    checkCudaErrors(cudaFree(areaVector));
    checkCudaErrors(cudaFree(costVector));
    checkCudaErrors(cudaFree(mortonCodes));

    return milliseconds / 1000.0f;
}

float deviceBuildSphereBVH(SphereNode *bvh, uint nObjects, dim3 gridSize, dim3 blockSize, uint *mortonCodes, 
                           cudaEvent_t &c_start, cudaEvent_t &c_end, Sphere *d_shapes) {

    return  deviceBuildBVH(bvh, nObjects, gridSize, blockSize, mortonCodes, c_start, c_end, d_shapes);
}

float deviceBuildTriangleBVH(TriangleNode *bvh, uint nObjects, dim3 gridSize, dim3 blockSize, uint *mortonCodes, 
                            cudaEvent_t &c_start, cudaEvent_t &c_end, Triangle *d_shapes) {

    return  deviceBuildBVH(bvh, nObjects, gridSize, blockSize, mortonCodes, c_start, c_end, d_shapes);
}

#endif