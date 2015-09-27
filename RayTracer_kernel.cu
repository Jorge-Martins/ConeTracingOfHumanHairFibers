#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_

#include <vector_types.h>
#include <vector_functions.h>
#include <math_functions.h>

#include <cfloat>

#include "Scene.h"

__device__ float EPSILON = 1E-5f;

//size ray array
__device__ int const raysSize = 2 << (MAX_DEPTH - 1);

//reflection and refraction arrays 
__device__ int const sizeRRArrays = (2 << (MAX_DEPTH - 1)) - 1;

//size local array
__device__ int const raysPerPixel = (2 << MAX_DEPTH) - 1;

__device__
bool equal(float f1, float f2) {
	float diffAbs = abs(f1 - f2);
	return diffAbs < EPSILON;
}

__device__
bool intersection(Ray ray, RayIntersection *out, Plane plane) {
    float nDOTrdir = dot(plane.normal, ray.direction);

	if (equal(nDOTrdir, 0.0f)) {
		return false;
    }

	float nDOTr0 = dot(plane.normal, ray.origin);
	float t = -((nDOTr0 + plane.distance) / nDOTrdir);

	if (t < 0.0f) {
		return false;
    }

	if (out != nullptr) {
        out->distance = t;
		out->normal = plane.normal;
		out->point = ray.origin + t*ray.direction;
        out->shapeMaterial = plane.material;
		out->isEntering = nDOTrdir < 0.0f;
        
        out->point += out->normal * EPSILON;
	}
	return true;
}

__device__
bool intersection(Ray ray, RayIntersection *out, Triangle tri) {
      float uu = dot(tri.e1, tri.e1);
      float uv = dot(tri.e1, tri.e2);
      float vv = dot(tri.e2, tri.e2);

      float normalDOTray = dot(tri.normal, ray.direction);
      float3 w0 = ray.origin - tri.vertices[0];
      float a = -dot(tri.normal, w0);
          
      float t = a / normalDOTray;
      float3 p = ray.origin + t * ray.direction;
      
      float3 w = p - tri.vertices[0];

      float wu = dot(w, tri.e1);
      float wv = dot(w, tri.e2);

      float inverseD = 1.0f / (uv * uv - uu * vv);

      float u = (uv * wv - vv * wu) * inverseD;

      if (u < 0.0f || u > 1.0f) {
          return false;
      }

      float v = (uv * wu - uu * wv) * inverseD;

      if (v < 0.0f || (u + v) > 1.0f) {
          return false;
      }


	if (out != nullptr) {
		out->distance = t;
		out->normal = tri.normal;
		out->point = p;
        out->shapeMaterial = tri.material;
		out->isEntering = normalDOTray < 0.0f;

        out->point += out->normal * EPSILON;
	}

	return true;
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
		out->shapeMaterial = sphere.material;
		out->distance = t;
        out->isEntering = entering;
	}

    return true;
}

__device__
bool intersection(Ray ray, RayIntersection *out, Cylinder cylinder) {
    //TODO
    return false;
}

__device__
bool findShadow(int **d_shapes, size_t *d_shapeSizes, Ray feeler) {
    bool intersectionFound = false;
    for(size_t shapeType = 0; shapeType < nShapes; shapeType++) {
        for (size_t i = 0; i < d_shapeSizes[shapeType]; i++) {
            if(shapeType == sphereIndex) {
                Sphere *sphere = (Sphere*) d_shapes[shapeType];
                intersectionFound = intersection(feeler, nullptr, sphere[i]);

            } else if(shapeType == cylinderIndex) {
                Cylinder *cylinder = (Cylinder*) d_shapes[shapeType];
                intersectionFound = intersection(feeler, nullptr, cylinder[i]);

            } else if(shapeType == triangleIndex) {
                Triangle *triangle = (Triangle*) d_shapes[shapeType];
                intersectionFound = intersection(feeler, nullptr, triangle[i]);
            
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
bool nearestIntersect(int **d_shapes, size_t *d_shapeSizes, Ray ray, RayIntersection *out) {
	RayIntersection minIntersect(FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool minIntersectionFound = false, intersectionFound = false;

	RayIntersection curr = minIntersect;
    for(size_t shapeType = 0; shapeType < nShapes; shapeType++) {
        for (size_t i = 0; i < d_shapeSizes[shapeType]; i++) {
            if(shapeType == sphereIndex) {
                Sphere *sphere = (Sphere*) d_shapes[shapeType];
                intersectionFound = intersection(ray, &curr, sphere[i]);

            } else if(shapeType == cylinderIndex) {
                Cylinder *cylinder = (Cylinder*) d_shapes[shapeType];
                intersectionFound = intersection(ray, &curr, cylinder[i]);

            } else if(shapeType == triangleIndex) {
                Triangle *triangle = (Triangle*) d_shapes[shapeType];
                intersectionFound = intersection(ray, &curr, triangle[i]);
            
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
void compensatePrecision(Ray &ray) {
	ray.origin += ray.direction * EPSILON;
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
float3 rayTracing(int **d_shapes, size_t *d_shapeSizes, Light* lights, size_t lightSize, float3 backcolor, 
                 float3 rayOrigin, float3 rayDirection, float MediumIor, Ray *ray, float3* locals,
                 float3* reflectionCols, float3* refractionCols, int offset) {

    int rayOffset = offset * raysSize;                 
    int localsOffset = offset * raysPerPixel;
    int rrOffset = offset * sizeRRArrays;
    Ray feeler = Ray();

    ray[rayOffset].update(rayOrigin, rayDirection);

    float ior[raysPerPixel];
    ior[0] = MediumIor;

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
	    for(size_t li = 0; li < lightSize; li++) {
		    float3 feelerDir = normalize(lights[li].position - intersect.point);
            feeler.update(intersect.point, feelerDir);
            compensatePrecision(feeler);

		    
            bool inShadow = findShadow(d_shapes, d_shapeSizes, feeler);
                
		    if(!inShadow) {
                float3 reflectDir = reflect(-feelerDir, intersect.normal);
                float Lspec = powf(fmax(dot(reflectDir, -ray[rayOffset + rayIndex(rayN)].direction), 0.0f), 
                                         mat.shininess);
                float Ldiff = fmax(dot(feelerDir, intersect.normal), 0.0f);

			    
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
		        ray[rayOffset + rayIndex(level)].update(intersect.point, 
                                                        reflect(ray[rayOffset + rayIndex(rayN)].direction, 
                                                                intersect.normal));
		        compensatePrecision(ray[rayOffset + rayIndex(level)]);

                reflectionCols[rrOffset + rayN] = mat.color * mat.Kspecular;
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
		        float3 refractionDir = computeTransmissionDir(ray[rayOffset + rayIndex(rayN)].direction, intersect.normal, ior1, ior2);
		            
                if (!equal(length(refractionDir), 0.0f)) {
			        ray[rayOffset + rayIndex(level)].update(intersect.point, refractionDir);
			        compensatePrecision(ray[rayOffset + rayIndex(level)]);

                    refractionCols[rrOffset + rayN] = mat.color * mat.transparency;
                    ior[level] = mat.ior;
		        
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
void drawScene(int **d_shapes, size_t *d_shapeSizes, Light *lights, size_t lightSize, float3 backcolor, int resX,
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
	
    float3 color = rayTracing(d_shapes, d_shapeSizes, lights, lightSize, backcolor, from, direction, 1.0, 
                              ray, d_locals, d_reflectionCols, d_refractionCols, index);

    d_output[index] = color;
}

void deviceDrawScene(int **d_shapes, size_t *d_shapeSizes, Light* lights, size_t lightSize, float3 backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize, Ray* ray,
                     float3* d_locals, float3* d_reflectionCols, float3* d_refractionCols) {

    drawScene<<<gridSize, blockSize>>>(d_shapes, d_shapeSizes, lights, lightSize, backcolor, resX, resY,
                                       width, height, atDistance, xe, ye, ze, from, d_output, ray,
                                       d_locals, d_reflectionCols, d_refractionCols);


}


#endif