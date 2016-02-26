#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_

#include "AOITHair.cuh"
#include <thrust/random.h>

#define rtStackSize (2 * MAX_DEPTH)
#define STOP_IMPORTANCE 0.04

__device__
float2 cudaRandom(thrust::default_random_engine &rng) {
    return make_float2((float)rng() ,
                       (float)rng());
}

__device__
float haltonSequance(int index, int base) {
       float result = 0.0f;
       float f = 1.0f;
      
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
float3 rayTracing(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                 float3 rayOrigin, float3 rayDirection, uint offset, RayIntersection *globalHairIntersectionLst) {

    RayIntersection *hairIntersectionLst = &globalHairIntersectionLst[HAIR_INTERSECTION_LST_SIZE * offset];

    Ray ray = Ray(rayOrigin, rayDirection);

    float3 blackColor = make_float3(0.0f);
    float3 colorAux;

    int rayHairIntersections = 0;

    colorAux = computeHairAT(d_shapes, d_shapeSizes, lights, lightSize, ray,
                             hairIntersectionLst, backcolor, 100.0f, rayHairIntersections);
    
    #ifdef PRINT_N_INTERSECTIONS
    colorAux = make_float3(rayHairIntersections, 1, 0.0f);
    #endif
    
    return colorAux;
}

__device__
float3 coneTracing(int **d_shapes, uint *d_shapeSizes, Light* lights, uint lightSize, float3 backcolor, 
                   float3 coneOrigin, float3 coneDirection, float coneSpread, int offset, 
                   RayIntersection *globalHairIntersectionLst) {

    RayIntersection *hairIntersectionLst = &globalHairIntersectionLst[HAIR_INTERSECTION_LST_SIZE * offset];

    Cone cone = Cone(coneOrigin, coneDirection, coneSpread);

    float3 blackColor = make_float3(0.0f);
    float3 colorAux;

    int rayHairIntersections = 0;

    colorAux = computeHairAT(d_shapes, d_shapeSizes, lights, lightSize, cone,
                             hairIntersectionLst, backcolor, 100.0f, rayHairIntersections);
        
    #ifdef PRINT_N_INTERSECTIONS
    colorAux = make_float3(rayHairIntersections, 1, 0.0f);
    #endif
    
    return colorAux;
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

    Ray ray = Ray();

    int rayIndex = 0;
    int colorsIndex = 0;
    int contributionIndex = 0;
    RayInfo info;
    RayIntersection intersect;

    rayInfoStack[rayIndex++].update(rayOrigin, rayDirection);
    float3 blackColor = make_float3(0.0f);
    float3 colorAux;

    int rayHairIntersections = 0;

    int nRays = 0;

    bool foundIntersect;
    bool computeColor = false;

    Material mat;

    while(rayIndex > 0) {
        reflectionColsStack[colorsIndex] = blackColor;
        refractionColsStack[colorsIndex] = blackColor;
        nRays++;
        info = rayInfoStack[--rayIndex];
        colorContributionType[contributionIndex++] = info.type;
        ray.update(info.origin, info.direction);
        
        foundIntersect = false;
        #ifdef GENERAL_INTERSECTION 
        foundIntersect = nearestIntersect(d_shapes, d_shapeSizes, ray, &intersect, rayHairIntersections);
        #else
	    foundIntersect = cylNearestIntersect(d_shapes, d_shapeSizes, ray, &intersect, rayHairIntersections);
        #endif

	    if (!foundIntersect) {     
            localsStack[colorsIndex] = backcolor;
            computeColor = true;
            
        } else {      
            mat = intersect.shapeMaterial;

            // local illumination
            colorAux = blackColor;
	        for(uint li = 0; li < lightSize; li++) {
                #ifndef SOFT_SHADOWS
                colorAux += computeShadows(d_shapes, d_shapeSizes, lights, ray, intersect,
                                           li, normalize(lights[li].position - intersect.point));
                    
                #else
                colorAux += computeSoftShadows(d_shapes, d_shapeSizes, lights, ray, intersect,
                                               li, normalize(lights[li].position - intersect.point));
                #endif
	        }
            
            localsStack[colorsIndex] = colorAux;
     
            computeColor = true;
            if(info.depth < MAX_DEPTH) {
                // reflection
                colorAux = blackColor;
	            if(mat.Kspecular > EPSILON && info.importance > STOP_IMPORTANCE) {
                    float3 reflectDir = reflect(ray.direction, intersect.normal);
                    float importance = info.importance * fminf(length(mat.color) * 0.66f, 1.0f) * mat.Kspecular;
                    rayInfoStack[rayIndex++].update(intersect.point, reflectDir, REFLECTED, 
                                                    info.depth + 1, importance);
                    colorAux = mat.color * mat.Kspecular;
                    computeColor = false;
                }
                reflectionColsStack[colorsIndex] = colorAux;

	            // transmission
                colorAux = blackColor;
                if(mat.transparency > EPSILON && info.importance > STOP_IMPORTANCE) {
		            float ior1, ior2, importance;
		            if(intersect.isEntering) {
			            ior1 = 1.0f;
			            ior2 = mat.ior;
                        importance = mat.transparency;

		            } else {
			            ior1 = mat.ior;
			            ior2 = 1.0f;
                        importance = 1.0f;
		            }
		            float3 refractionDir = computeTransmissionDir(ray.direction, intersect.normal, ior1, ior2);
		            
                    if (length(refractionDir) > EPSILON) {
                        colorAux = mat.color * importance;
                        importance *= info.importance * fminf(length(mat.color) * 0.66f, 1.0f);
			            rayInfoStack[rayIndex++].update(intersect.point, refractionDir, REFRACTED, 
                                                        info.depth + 1, importance);
                        computeColor = false;
                    }
	            }
                refractionColsStack[colorsIndex] = colorAux;

                if(!computeColor) {
                    colorsIndex++;
                }
            }
        }

        #ifndef PRINT_N_INTERSECTIONS
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
        #endif
        computeColor = false;

    }

    #ifndef PRINT_N_INTERSECTIONS
    colorAux = localsStack[0] + reflectionColsStack[0] + refractionColsStack[0];
    
    #else
    colorAux = make_float3(rayHairIntersections, nRays, 0.0f);
    
    #endif
    
    return colorAux;
}

__device__
float3 naiveSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                          float3 xe, float3 ye, float3 zeFactor, float3 from, float coneSpread, RayInfo* rayInfo, 
                          float3* d_colors, unsigned char *d_colorContributionType, uint index, uint x, uint y, 
                          int resX, int resY, RayIntersection *d_hairIntersectionLst) {

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    coneSpread *= SUPER_SAMPLING_2_F;
    for(int sx = 0; sx < SUPER_SAMPLING; sx++) {
        for(int sy = 0; sy < SUPER_SAMPLING; sy++) {
            yeFactor = ye * ((y + (sy + 0.5f) * SUPER_SAMPLING_F) / (float)resY - 0.5f);
            xeFactor = xe * ((x + (sx + 0.5f) * SUPER_SAMPLING_F) / (float)resX - 0.5f);

            direction = normalize(zeFactor + yeFactor + xeFactor);

            #ifndef CONE_TRACING
            #ifndef AT_HAIR
            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     rayInfo, d_colors, d_colorContributionType, index);
            #else
            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     index, d_hairIntersectionLst);
            #endif

            #else 
            color += SUPER_SAMPLING_2_F * coneTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                      coneSpread, index, d_hairIntersectionLst);
            #endif
        }
    }

    return color;
}

__device__
float3 naiveRdmSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                             float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                             unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                             int seed, RayIntersection *d_hairIntersectionLst) {

    thrust::default_random_engine rng(seed + index);
    thrust::uniform_real_distribution<float> uniDist;
    rng.discard(2 * index);

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int sx = 0; sx < SUPER_SAMPLING; sx++) {
        for(int sy = 0; sy < SUPER_SAMPLING; sy++) {
            yeFactor = ye * ((y + (sy + uniDist(rng)) * SUPER_SAMPLING_F) / (float)resY - 0.5f);
            xeFactor = xe * ((x + (sx + uniDist(rng)) * SUPER_SAMPLING_F) / (float)resX - 0.5f);

            direction = normalize(zeFactor + yeFactor + xeFactor);

            #ifndef AT_HAIR
            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     rayInfo, d_colors, d_colorContributionType, index);
            #else
            color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                     index, d_hairIntersectionLst);
            #endif
        }
    }

    return color;
}

__device__
float3 stocasticSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                              float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                              unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                              int seed, RayIntersection *d_hairIntersectionLst) {

    thrust::default_random_engine rng(seed + index);
    thrust::uniform_real_distribution<float> uniDist;

    rng.discard(2 * index);

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int i = 0; i < SUPER_SAMPLING_2; i++) {
        yeFactor = ye * ((y + uniDist(rng)) / (float)resY - 0.5f);
        xeFactor = xe * ((x + uniDist(rng)) / (float)resX - 0.5f);

        direction = normalize(zeFactor + yeFactor + xeFactor);

        #ifndef AT_HAIR
        color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                 rayInfo, d_colors, d_colorContributionType, index);
        #else
        color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                 index, d_hairIntersectionLst);
        #endif
    }

    return color;
}

__device__
float3 stocasticHSSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                                float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                                unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                                int seed, RayIntersection *d_hairIntersectionLst) {

    int hsIndex = index + seed;

    float3 direction, color = make_float3(0.0f), yeFactor, xeFactor;
    for(int i = 0; i < SUPER_SAMPLING_2; i++) {
        yeFactor = ye * ((y + haltonSequance(hsIndex + i, 3)) / (float)resY - 0.5f);
        xeFactor = xe * ((x + haltonSequance(hsIndex + i, 2)) / (float)resX - 0.5f);

        direction = normalize(zeFactor + yeFactor + xeFactor);

        #ifndef AT_HAIR
        color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                 rayInfo, d_colors, d_colorContributionType, index);
        #else
        color += SUPER_SAMPLING_2_F * rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                                                 index, d_hairIntersectionLst);
        #endif
    }

    return color;
}

__device__
float3 adaptiveStocasticSupersampling(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                                      float3 xe, float3 ye, float3 zeFactor, float3 from, RayInfo* rayInfo, float3* d_colors, 
                                      unsigned char *d_colorContributionType, uint index, uint x, uint y, int resX, int resY,
                                      int seed, int initNSamples, RayIntersection *d_hairIntersectionLst) {

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

        #ifndef AT_HAIR
        tmp = rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                         rayInfo, d_colors, d_colorContributionType, index);
        #else
        tmp = rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 
                         index, d_hairIntersectionLst);
        #endif

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
               int resY, float coneSpread, float3 xe, float3 ye, float3 ze, float3 from, float3 *d_output,
               RayInfo *rayInfo, float3 *d_colors, unsigned char *d_colorContributionType, int seed,
               RayIntersection *d_hairIntersectionLst) {

    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= resX || y >= resY) {
        return;
    }

    uint index = y * resX + x;


    d_output[index] = naiveSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                         from, coneSpread, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                         resY, d_hairIntersectionLst);
    
    /*d_output[index] = naiveRdmSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                            from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                            resY, seed, d_hairIntersectionLst);*/

    /*d_output[index] = stocasticSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                             from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                             resY, seed, d_hairIntersectionLst);*/

    /*d_output[index] = adaptiveStocasticSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                                     from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                                     resY, seed, 16, d_hairIntersectionLst);*/

    /*d_output[index] = stocasticHSSupersampling(d_shapes, d_shapeSizes, lights, lightSize, backcolor, xe, ye, ze, 
                                               from, rayInfo, d_colors, d_colorContributionType, index, x, y, resX, 
                                               resY, seed, d_hairIntersectionLst);*/

    #ifdef PRINT_N_INTERSECTIONS
    int rayHairIntersections = d_output[index].x;
    int nRays = d_output[index].y;
    d_output[index] = printRayHairIntersections(rayHairIntersections, backcolor, nRays);
    #endif

}


void deviceDrawScene(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize, RayInfo *rayInfo,
                     float3 *d_colors, unsigned char *d_colorContributionType, int seed, 
                     RayIntersection *d_hairIntersectionLst) {
    
    
    ye *= height;
    xe *= width;
    ze = -ze * atDistance;
    float pixelSize = 0.5f * width / (float) resX;
    float coneSpread = atanf(pixelSize / atDistance);
    drawScene<<<gridSize, blockSize>>>(d_shapes, d_shapeSizes, lights, lightSize, backcolor, resX, resY,
                                       coneSpread, xe, ye, ze, from, d_output, rayInfo, d_colors, 
                                       d_colorContributionType, seed, d_hairIntersectionLst);

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