#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_

#include <vector_types.h>
#include <vector_functions.h>
#include <math_functions.h>

#include <cfloat>

#include "Scene.h"

__device__ float EPSILON = 1E-5f;

//size reflection and refraction arrays 
__device__ int const sizeRRArrays = (2 << (MAX_DEPTH - 1)) - 1;

//size local and ray arrays 
__device__ int const raysPerPixel = (2 << MAX_DEPTH) - 1;

__device__
bool equal(float f1, float f2) {
	float diffAbs = abs(f1 - f2);
	return diffAbs < EPSILON;
}


__device__
bool intersection(Ray ray, RayIntersection *out, Sphere sphere) {
    float d_2, r_2, b, root, t;

    float3 s_r = sphere.center - ray.origin;
    
    r_2 = sphere.r * sphere.r;
    d_2 = dot(s_r, s_r);

    if(equal(d_2, r_2)) {
        return false;
    }
    b = dot(ray.direction, s_r);

    if (d_2 > r_2 && b < 0.0f) {
        return false; 
    }

    root = b*b - d_2 + r_2;
    if(root < 0.0f) {
        return false;
    }

    t = min(b - sqrtf(root), b + sqrtf(root));

    if (out != nullptr) {
        out->point = ray.origin + ray.direction * t;
		out->normal = normalize((out->point - sphere.center) / sphere.r);

		bool entering = true;
		if (d_2 < r_2) {
			out->normal *= -1.0f;
			entering = false;
		}
        
        out->point += out->normal * EPSILON;
		out->shape = &sphere;
		out->distance = t;
        out->isEntering = entering;
	}

    return true;
}


__device__
bool nearestIntersect(Sphere *shapes, size_t shapeSize, Ray ray, RayIntersection *out) {
	RayIntersection minIntersect(FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool intersectionFound = false;

	RayIntersection curr = minIntersect;
    for (size_t i = 0; i < shapeSize; i++) {
		if (intersection(ray, &curr, shapes[i])) {
            if (curr.distance < minIntersect.distance) {
				minIntersect = curr;
                intersectionFound = true;
			}
		}
	}
    
	if (intersectionFound) {
		*out = minIntersect;
	}
	return intersectionFound;
}

__device__
void compensatePrecision(Ray &ray) {
	ray.origin += ray.direction * EPSILON;
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
float3 rayTracing(Sphere* shapes, size_t shapeSize, Light* lights, size_t lightSize, float3 backcolor, 
                 float3 rayOrigin, float3 rayDirection, float MediumIor, Ray *ray, float3* locals,
                 float3* reflectionCols, float3* refractionCols, int offset) {

                     
    int rayOffset = offset * raysPerPixel;
    int rrOffset = offset * raysPerPixel;
    Ray feeler = Ray();
    
    ray[rayOffset].update(rayOrigin, rayDirection);

    float ior[raysPerPixel];
    ior[0] = MediumIor;

    RayIntersection intersect;
    int level;
    float3 blackColor = make_float3(0.0f);
    
    for(int rayN = 0; rayN < raysPerPixel; rayN++) {
        //skip secundary rays that don't exist
        while(!ray[rayOffset + rayN].exists && rayN < sizeRRArrays) {
            level = 2 * rayN;
            ray[rayOffset + level + 1].exists = false;
            ray[rayOffset + level + 2].exists = false;
            rayN++;
        }
        
	    bool foundIntersect = nearestIntersect(shapes, shapeSize, ray[rayOffset + rayN], &intersect);

	    if (!foundIntersect) {
            locals[rayOffset + rayN] = backcolor;
            reflectionCols[rrOffset + rayN] = blackColor;
            refractionCols[rrOffset + rayN] = blackColor;

            if(rayN == 0) {
                break;
            }

            if(rayN < sizeRRArrays) {
                level = 2 * rayN;
                ray[rayOffset + level + 1].exists = false;
                ray[rayOffset + level + 2].exists = false;
            }
            
        
        } else {
            Material mat = intersect.shape->material;
    
            // local illumination
	        locals[rayOffset + rayN] = blackColor;
	        for(size_t li = 0; li < lightSize; li++) {
		        float3 feelerDir = normalize(lights[li].position - intersect.point);
                feeler.update(intersect.point, feelerDir);
                compensatePrecision(feeler);

		        bool inShadow = false;
		        for(size_t si = 0; si < shapeSize; si++) {
			        if (intersection(feeler, nullptr, shapes[si])) {
				        inShadow = true;
				        break;
			        }
		        }
                
		        if(!inShadow) {
                    float spec = 0.0f;
                    float diff = fmax(dot(feelerDir, intersect.normal), 0.0f);
                    float3 reflectDir = reflect(-feelerDir, intersect.normal);
                    
                    if(diff > 0.0f) {
                        spec = powf(fmax(dot(reflectDir, ray[rayOffset + rayN].direction), 0.0f), mat.shininess);
                    }

			        float3 seenColor = mat.color * lights[li].color;
			        locals[rayOffset + rayN] += seenColor * (diff * mat.diffuse + spec * mat.specular);
		        }
	        }
    
            level = 2 * rayN;
            ray[rayOffset + level + 1].exists = false;
            ray[rayOffset + level + 2].exists = false;
            reflectionCols[rrOffset + rayN] = blackColor;
            refractionCols[rrOffset + rayN] = blackColor;
            if(rayN < sizeRRArrays) {
                // reflection
                level = 2 * rayN + 1;
	            if(mat.specular > 0.0f) {
		            ray[rayOffset + level].update(intersect.point, reflect(ray[rayOffset + rayN].direction, intersect.normal));
		            compensatePrecision(ray[rayOffset + level]);

                    reflectionCols[rrOffset + rayN] = mat.color * mat.specular;
                    ior[level] = mat.ior;
	        
                }

	            // transmission
                level = 2 * rayN + 2;
	            if(mat.transparency > 0.0f) {
		            float ior1, ior2;
		            if(intersect.isEntering) {
			            ior1 = ior[rayN];
			            ior2 = mat.ior;
		            }
		            else {
			            ior1 = mat.ior;
			            ior2 = ior[rayN];
		            }
		            float3 refractionDir = computeTransmissionDir(ray[rayOffset + rayN].direction, intersect.normal, ior1, ior2);
		            
                    if (!equal(length(refractionDir), 0.0f)) {
			            ray[rayOffset + level].update(intersect.point, refractionDir);
			            compensatePrecision(ray[rayOffset + level]);

                        refractionCols[rrOffset + rayN] = mat.color * mat.transparency;
                        ior[level] = mat.ior;
		        
                    }
	            }
            }
        }
    }

    int startLevel = (2 << (MAX_DEPTH - 1)) - 2;

    for(int i = startLevel; i >= 0; i--) {
        level = 2 * i + 1;
        locals[rayOffset + level] += reflectionCols[rrOffset + level] + refractionCols[rrOffset + level];
        reflectionCols[rrOffset + i] *= locals[rayOffset + level];
        
        level = 2 * i + 2;
        locals[rayOffset + level] += reflectionCols[rrOffset + level] + refractionCols[rrOffset + level];
        refractionCols[rrOffset + i] *= locals[rayOffset + level];
    }

    return locals[rayOffset] + reflectionCols[rrOffset] + refractionCols[rrOffset];
}


__global__
void drawScene(Sphere *shapes, size_t shapeSize, Light* lights, size_t lightSize, float3 backcolor, int resX,
               int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
               float3 ze, float3 from, float3 *d_output, Ray* ray, float3* d_locals, 
               float3* d_reflectionCols, float3* d_refractionCols) {

    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    int index = y * resX + x;

    float3 zeFactor = -ze * atDistance; 
	float3 yeFactor = ye * height * ((y + 0.5f) / (float)resY - 0.5f);
	float3 xeFactor = xe * width * ((x + 0.5f) / (float)resX - 0.5f);

	float3 direction = normalize(zeFactor + yeFactor + xeFactor);
	
    float3 color = rayTracing(shapes, shapeSize, lights, lightSize, backcolor, from, direction, 1.0, 
                              ray, d_locals, d_reflectionCols, d_refractionCols, index);

    d_output[index] = color;
}

void deviceDrawScene(Sphere *shapes, size_t shapeSize, Light* lights, size_t lightSize, float3 backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize, Ray* ray,
                     float3* d_locals, float3* d_reflectionCols, float3* d_refractionCols) {

    drawScene<<<gridSize, blockSize>>>(shapes, shapeSize, lights, lightSize, backcolor, resX, resY,
                                       width, height, atDistance, xe, ye, ze, from, d_output, ray,
                                       d_locals, d_reflectionCols, d_refractionCols);


}


#endif