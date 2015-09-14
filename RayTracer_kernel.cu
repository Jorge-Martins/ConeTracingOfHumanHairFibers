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
float getVectorPos(float3 v, int pos) {
    switch(pos) {
    case 0:
        return v.x;
    case 1:
        return v.y;
    default:
        return v.z;
    }
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
bool Plane::intersection(Ray ray, RayIntersection *out) {
    float nDOTrdir = dot(_normal, ray.direction());

	if (equal(nDOTrdir, 0.0f))
		return false;

	float nDOTr0 = dot(_normal, ray.origin());
	float t = -((nDOTr0 + _d) / nDOTrdir);

	if (t < 0.0f)
		return false;

	if (out != nullptr) {
		bool entering = nDOTrdir < 0.0f;
		*out = RayIntersection(t, ray.origin() + t*ray.direction(), _normal, entering, this);
	}
	return true;
}

__device__
bool Triangle::intersection(Ray ray, RayIntersection *out) {
    int i0, i1, i2;
	float2 uv0, uv1, uv2;
	float alpha, beta;
	float normalDOTray = dot(_normal, ray.direction());

	if (equal(normalDOTray, 0.0f))
		return false;

	float normalDOTr0 = dot(_normal, ray.origin());
	float t = -((normalDOTr0 + _d) / normalDOTray);

	if (t < 0.0f)
		return false;

	calculateIndexes(&i0, &i1, &i2);

	float3 P = ray.origin() + (t * ray.direction());
	
	uv0.x = getVectorPos(P, i1) - getVectorPos(_vertices[0], i1);
	uv0.y = getVectorPos(P, i2) - getVectorPos(_vertices[0], i2);

	uv1.x = getVectorPos(_vertices[1], i1) - getVectorPos(_vertices[0], i1);
	uv1.y = getVectorPos(_vertices[1], i2) - getVectorPos(_vertices[0], i2);

	uv2.x = getVectorPos(_vertices[2], i1) - getVectorPos(_vertices[0], i1);
	uv2.y = getVectorPos(_vertices[2], i2) - getVectorPos(_vertices[0], i2);

	if (equal(uv1.x, 0.0f)) {
		beta = uv0.x / uv2.x;

		if (beta > 0.0f && beta < 1.0f) {
			alpha = (uv0.x - beta * uv2.y) / uv1.y;
		}
		else return false;
	}
	else {
		beta = (uv0.y * uv1.x - uv0.x * uv1.y) / (uv2.y * uv1.x - uv2.x * uv1.y); 
		
		if (beta > 0.0f && beta < 1.0f){
			alpha = (uv0.x - beta * uv2.x) / uv1.x;
		}
		else return false;
	}

	if (out != nullptr) {
		out->_distance = t;
		out->_normal = _normal;
		out->_point = P;
		out->_shape = this;
		out->_isEntering = normalDOTray < 0.0f;
	}

	return alpha >= 0.0f && beta >= 0.0f && (alpha + beta) <= 1.0f;
}
	
__global__
void drawScene(Shape **shapes, size_t shapeSize, Light* lights, size_t lightSize, Color backcolor, int resX,
               int resY, float4 *d_output) {

    float4 pixel = make_float4(1.0f, 0.0f, 0.0f, 1.0f);
    int res = resX * resY;
    for(int i = 0; i < res; i++) {
        d_output[i] = pixel;
    }
}

void deviceDrawScene(Shape **shapes, size_t shapeSize, Light* lights, size_t lightSize, Color backcolor, int resX,
                   int resY, float4 *d_output) {

    drawScene<<<1, 1>>>(shapes, shapeSize, lights, lightSize, backcolor, resX, resY, d_output);
}

__device__
bool nearestIntersect(Shape **shapes, long shapeSize, Ray ray, RayIntersection *out) {
	RayIntersection minIntersect((float)FLT_MAX, make_float3(0.0f), make_float3(0.0f));
	bool intersectionFound = false;

	RayIntersection curr = minIntersect;
    for (long i = 0; i < shapeSize; i++) {
		if (shapes[i]->intersection(ray, &curr)) {
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
Color rayTracing(Shape** shapes, long shapeSize, Light* lights, long lightSize, Color backcolor, Ray ray, int depth, float ior) {

    RayIntersection intersect;

	bool foundIntersect = nearestIntersect(shapes, shapeSize, ray, &intersect);

	if (!foundIntersect)
		return backcolor;

	Material mat = intersect._shape->material();

	// local illumination
	float3 local = make_float3(0.0f);
	for(long li = 0; li < lightSize; li++) {
		float3 feelerDir = normalize(lights[li].position() - intersect._point);
		Ray feeler(intersect._point, feelerDir);
		compensatePrecision(feeler);

		bool inShadow = false;
		for(long si = 0; si < shapeSize; si++) {
			if (shapes[si]->intersection(feeler, nullptr)) {
				inShadow = true;
				break;
			}
		}
		if(!inShadow) {
			float diff = fmax(dot(feelerDir, intersect._normal), 0.0f);
			float3 reflectDir = reflect(-feelerDir, intersect._normal);
			float spec = powf(fmax(dot(reflectDir, -ray.direction()), 0.0f), mat.shininess());

			float3 seenColor = mat.color().color() * lights[li].color().color();
			local += seenColor * (diff * mat.diffuse() + spec * mat.specular());
		}
	}

	// reflection
	Color reflectionCol;
	if(mat.specular() > 0.0f && depth > 0) {
		Ray reflectedRay = Ray(intersect._point, reflect(ray.direction(), intersect._normal));
		compensatePrecision(reflectedRay);
		reflectionCol = rayTracing(shapes, shapeSize, lights, lightSize, backcolor, reflectedRay, depth - 1, mat.refraction()) * mat.color() * mat.specular();
	}

	// transmission
	Color refractionCol;
	if(mat.transparency() > 0.0f && depth > 0) {
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
			Ray refractedRay(intersect._point, refractionDir);
			compensatePrecision(refractedRay);
			refractionCol = rayTracing(shapes, shapeSize, lights, lightSize, backcolor, refractedRay, depth - 1, mat.refraction()) * mat.color() * mat.transparency();
		}
	}


	return Color(local) + reflectionCol + refractionCol;
}





#endif