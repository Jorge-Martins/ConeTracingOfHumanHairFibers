#pragma once

#ifndef _RAYTRACER_KERNEL_CU_
#define _RAYTRACER_KERNEL_CU_

#include <vector_types.h>
#include <vector_functions.h>
#include <math_functions.h>

#include <cfloat>

#include "Scene.h"


__device__
bool equal(float f1, float f2) {
	float diffAbs = abs(f1 - f2);
	return diffAbs < FLOAT_EPSILON;
}


__device__
bool Sphere::intersection(Ray ray, RayIntersection *out) {
    float d_2, r_2, b, c, t;

	float3 rayOrig = ray.origin();
	float xs_xr = _x - rayOrig.x;
	float ys_yr = _y - rayOrig.y;
	float zs_zr = _z - rayOrig.z;

	r_2 = _r * _r;
	d_2 = (xs_xr * xs_xr) + (ys_yr * ys_yr) + (zs_zr * zs_zr);

	if (equal(d_2, r_2)) {
		return false;
	}
	else {
		float3 rayDir = ray.direction();
		b = rayDir.x * xs_xr + rayDir.y * ys_yr + rayDir.z * zs_zr;

		if (d_2 > r_2 && b < 0.0f) {		
			return false;
		}

		c = b*b - d_2 + r_2;

		if (c < 0.0f) {
			return false;
		}

		if (d_2 > r_2) {
			t = b - sqrtf(c);
		}
		else {
			t = b + sqrtf(c);
		}
		if (out != nullptr) {
			out->_point = make_float3(rayOrig.x + rayDir.x * t,
				                        rayOrig.y + rayDir.y * t,
				                        rayOrig.z + rayDir.z * t);

			out->_normal = make_float3((out->_point.x - _x) / _r,
				                        (out->_point.y - _y) / _r,
				                        (out->_point.z - _z) / _r);

			bool entering = true;
			if (d_2 < r_2) {
				out->_normal = out->_normal * -1.0f;
				entering = false;
			}
			out->_shape = this;
			out->_distance = t;
			out->_isEntering = entering;
		}
		return true;
	}
}

__device__
bool Cylinder::intersection(Ray ray, RayIntersection *out) {
    //TODO
    return false;
}


__device__
bool nearestIntersect(Sphere *shapes, size_t shapeSize, Ray ray, RayIntersection *out) {
	RayIntersection minIntersect((float)FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool intersectionFound = false;

	RayIntersection curr = minIntersect;
    for (size_t i = 0; i < shapeSize; i++) {
		if (shapes[i].intersection(ray, &curr)) {
            if (curr._distance < minIntersect._distance) {
				intersectionFound = true;
				minIntersect = curr;
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
	ray.origin() += ray.direction() * ACNE_EPSILON;
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
Color rayTracing(Sphere* shapes, size_t shapeSize, Light* lights, size_t lightSize, Color backcolor, Ray ray, float ior) {

    Ray refractedRay = Ray();
    Ray reflectedRay = Ray();
    Ray feeler = Ray();

    float3 locals[MAX_DEPTH];
    float3 reflectionCols[MAX_DEPTH];
    float3 refractionCols[MAX_DEPTH];

    RayIntersection intersect;
    Ray activeRay;

    for(int depth = 0; depth < MAX_DEPTH; depth++) {

	    bool foundIntersect = nearestIntersect(shapes, shapeSize, ray, &intersect);

	    if (!foundIntersect) {
		    return backcolor;
        }

        Material mat = intersect._shape->material();
    
        // local illumination
	    locals[depth] = make_float3(0.0f);
	    for(size_t li = 0; li < lightSize; li++) {
		    float3 feelerDir = normalize(lights[li].position() - intersect._point);
		    feeler.update(intersect._point, feelerDir);
		    compensatePrecision(feeler);

		    bool inShadow = false;
		    for(size_t si = 0; si < shapeSize; si++) {
			    if (shapes[si].intersection(feeler, nullptr)) {
				    inShadow = true;
				    break;
			    }
		    }
		    if(!inShadow) {
			    float diff = fmax(dot(feelerDir, intersect._normal), 0.0f);
			    float3 reflectDir = reflect(-feelerDir, intersect._normal);
			    float spec = powf(fmax(dot(reflectDir, -ray.direction()), 0.0f), mat.shininess());

			    float3 seenColor = mat.color().color() * lights[li].color().color();
			    locals[depth] += seenColor * (diff * mat.diffuse() + spec * mat.specular());
		    }
	    }
    

        if(depth < MAX_DEPTH) {
	        // reflection
	        if(mat.specular() > 0.0f) {
		        reflectedRay.update(intersect._point, reflect(ray.direction(), intersect._normal));
		        compensatePrecision(reflectedRay);

                reflectionCols[depth] = mat.color().color() * mat.specular();
                //rayTracing(shapes, shapeSize, lights, lightSize, backcolor, reflectedRay, depth - 1, mat.refraction());
	        }

	        // transmission
	        if(mat.transparency() > 0.0f) {
		        float ior1, ior2;
		        if(intersect._isEntering) {
			        ior1 = 1.0f;
			        ior2 = mat.refraction();
		        }
		        else {
			        ior1 = mat.refraction();
			        ior2 = 1.0f;
		        }
		        float3 refractionDir = computeTransmissionDir(ray.direction(), intersect._normal, ior1, ior2);
		        if (!equal(length(refractionDir), 0.0f)) {
			        refractedRay.update(intersect._point, refractionDir);
			        compensatePrecision(refractedRay);
			        
                    refractionCols[depth] = mat.color().color() * mat.transparency();
                    //rayTracing(shapes, shapeSize, lights, lightSize, backcolor, refractedRay, depth - 1, mat.refraction());
		        }
	        }

        } else {
            reflectionCols[depth] = refractionCols[depth] = make_float3(0.0f);
        }
    }

    int i = MAX_DEPTH - 1;
    //locals[i] += reflectionCols[i] + refractionCols[i];

    for(int i = MAX_DEPTH - 2; i >= 0; i--) {
        locals[i] += reflectionCols[i] + refractionCols[i];
        locals[i] += locals[i+1];
    }

	return Color(locals[0]);
}


__global__
void drawScene(Sphere *shapes, size_t shapeSize, Light* lights, size_t lightSize, Color backcolor, int resX,
               int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
               float3 ze, float3 from, float3 *d_output) {

    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

    float zeFactor = atDistance;
	float yeFactor = height * ((y + 0.5f) / resY - 0.5f);
	float xeFactor = width * ((x + 0.5f) / resX - 0.5f);

	float3 direction = normalize(zeFactor * ze + yeFactor * ye + xeFactor * xe);
	Ray ray = Ray(from, direction);

    Color color = rayTracing(shapes, shapeSize, lights, lightSize, backcolor, ray, 1.0);

    d_output[y * resX + x] = normalize(color.color());

}

void deviceDrawScene(Sphere *shapes, size_t shapeSize, Light* lights, size_t lightSize, Color backcolor, 
                     int resX, int resY, float width, float height, float atDistance, float3 xe, float3 ye, 
                     float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize) {

    drawScene<<<gridSize, blockSize>>>(shapes, shapeSize, lights, lightSize, backcolor, resX, resY,
                                       width, height, atDistance, xe, ye, ze, from, d_output);


}


#endif