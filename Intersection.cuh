#pragma once

#ifndef _INTERSECTION_
#define _INTERSECTION_

#include "Primitives.h"

__device__
bool equal(float f1, float f2) {
	float diffAbs = abs(f1 - f2);
	return diffAbs < EPSILON;
}

__device__
bool AABBIntersection(Ray ray, float3 min, float3 max) {
    switch (ray.classification) {
	    case NNN:	
            return !((ray.origin.x < min.x) || (ray.origin.y < min.y) || (ray.origin.z < min.z)
			        || (ray.y_x * min.x - max.y + ray.c_xy > 0)
			        || (ray.x_y * min.y - max.x + ray.c_yx > 0)
			        || (ray.y_z * min.z - max.y + ray.c_zy > 0)
			        || (ray.z_y * min.y - max.z + ray.c_yz > 0)
			        || (ray.z_x * min.x - max.z + ray.c_xz > 0)
			        || (ray.x_z * min.z - max.x + ray.c_zx > 0));
			
	    case NNP:	
		    return !((ray.origin.x < min.x) || (ray.origin.y < min.y) || (ray.origin.z > max.z)
			        || (ray.y_x * min.x - max.y + ray.c_xy > 0)
			        || (ray.x_y * min.y - max.x + ray.c_yx > 0)
			        || (ray.y_z * max.z - max.y + ray.c_zy > 0)
			        || (ray.z_y * min.y - min.z + ray.c_yz < 0)
			        || (ray.z_x * min.x - min.z + ray.c_xz < 0)
			        || (ray.x_z * max.z - max.x + ray.c_zx > 0));

	    case NPN:	
		    return !((ray.origin.x < min.x) || (ray.origin.y > max.y) || (ray.origin.z < min.z)
			        || (ray.y_x * min.x - min.y + ray.c_xy < 0) 
			        || (ray.x_y * max.y - max.x + ray.c_yx > 0)
			        || (ray.y_z * min.z - min.y + ray.c_zy < 0) 
			        || (ray.z_y * max.y - max.z + ray.c_yz > 0)
			        || (ray.z_x * min.x - max.z + ray.c_xz > 0)
			        || (ray.x_z * min.z - max.x + ray.c_zx > 0));
			
	    case NPP:
		    return !((ray.origin.x < min.x) || (ray.origin.y > max.y) || (ray.origin.z > max.z)
			        || (ray.y_x * min.x - min.y + ray.c_xy < 0) 
			        || (ray.x_y * max.y - max.x + ray.c_yx > 0)
			        || (ray.y_z * max.z - min.y + ray.c_zy < 0)
			        || (ray.z_y * max.y - min.z + ray.c_yz < 0)
			        || (ray.z_x * min.x - min.z + ray.c_xz < 0)
			        || (ray.x_z * max.z - max.x + ray.c_zx > 0));
			
	    case PNN:
		    return !((ray.origin.x > max.x) || (ray.origin.y < min.y) || (ray.origin.z < min.z)
			        || (ray.y_x * max.x - max.y + ray.c_xy > 0)
			        || (ray.x_y * min.y - min.x + ray.c_yx < 0)
			        || (ray.y_z * min.z - max.y + ray.c_zy > 0)
			        || (ray.z_y * min.y - max.z + ray.c_yz > 0)
			        || (ray.z_x * max.x - max.z + ray.c_xz > 0)
			        || (ray.x_z * min.z - min.x + ray.c_zx < 0));
			
	    case PNP:
		    return !((ray.origin.x > max.x) || (ray.origin.y < min.y) || (ray.origin.z > max.z)
			        || (ray.y_x * max.x - max.y + ray.c_xy > 0)
			        || (ray.x_y * min.y - min.x + ray.c_yx < 0)
			        || (ray.y_z * max.z - max.y + ray.c_zy > 0)
			        || (ray.z_y * min.y - min.z + ray.c_yz < 0)
			        || (ray.z_x * max.x - min.z + ray.c_xz < 0)
			        || (ray.x_z * max.z - min.x + ray.c_zx < 0));

	    case PPN:
		    return !((ray.origin.x > max.x) || (ray.origin.y > max.y) || (ray.origin.z < min.z)
			        || (ray.y_x * max.x - min.y + ray.c_xy < 0)
			        || (ray.x_y * max.y - min.x + ray.c_yx < 0)
			        || (ray.y_z * min.z - min.y + ray.c_zy < 0) 
			        || (ray.z_y * max.y - max.z + ray.c_yz > 0)
			        || (ray.z_x * max.x - max.z + ray.c_xz > 0)
			        || (ray.x_z * min.z - min.x + ray.c_zx < 0));
			
	    case PPP:
		    return !((ray.origin.x > max.x) || (ray.origin.y > max.y) || (ray.origin.z > max.z)
			        || (ray.y_x * max.x - min.y + ray.c_xy < 0)
			        || (ray.x_y * max.y - min.x + ray.c_yx < 0)
			        || (ray.y_z * max.z - min.y + ray.c_zy < 0)
			        || (ray.z_y * max.y - min.z + ray.c_yz < 0)
			        || (ray.z_x * max.x - min.z + ray.c_xz < 0)
			        || (ray.x_z * max.z - min.x + ray.c_zx < 0));
			
	    case ONN:
		    return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.y < min.y) || (ray.origin.z < min.z)
			        || (ray.y_z * min.z - max.y + ray.c_zy > 0)
			        || (ray.z_y * min.y - max.z + ray.c_yz > 0));
			
	    case ONP:
		    return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.y < min.y) || (ray.origin.z > max.z)
			        || (ray.y_z * max.z - max.y + ray.c_zy > 0)
			        || (ray.z_y * min.y - min.z + ray.c_yz < 0));
			
	    case OPN:
		    return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.y > max.y) || (ray.origin.z < min.z)
			        || (ray.y_z * min.z - min.y + ray.c_zy < 0) 
			        || (ray.z_y * max.y - max.z + ray.c_yz > 0));
			
	    case OPP:
		    return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.y > max.y) || (ray.origin.z > max.z)
			        || (ray.y_z * max.z - min.y + ray.c_zy < 0)
			        || (ray.z_y * max.y - min.z + ray.c_yz < 0));
			
	    case NON:
		    return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			        || (ray.origin.x < min.x) || (ray.origin.z < min.z) 
			        || (ray.z_x * min.x - max.z + ray.c_xz > 0)
			        || (ray.x_z * min.z - max.x + ray.c_zx > 0));
			
	    case NOP:
		    return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			        || (ray.origin.x < min.x) || (ray.origin.z > max.z) 
			        || (ray.z_x * min.x - min.z + ray.c_xz < 0)
			        || (ray.x_z * max.z - max.x + ray.c_zx > 0));
			
	    case PON:
		    return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			        || (ray.origin.x > max.x) || (ray.origin.z < min.z)
			        || (ray.z_x * max.x - max.z + ray.c_xz > 0)
			        || (ray.x_z * min.z - min.x + ray.c_zx < 0));
			
	    case POP:
		    return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			        || (ray.origin.x > max.x) || (ray.origin.z > max.z)
			        || (ray.z_x * max.x - min.z + ray.c_xz < 0)
			        || (ray.x_z * max.z - min.x + ray.c_zx < 0));
			
	    case NNO:
		    return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			        || (ray.origin.x < min.x) || (ray.origin.y < min.y) 
			        || (ray.y_x * min.x - max.y + ray.c_xy > 0)
			        || (ray.x_y * min.y - max.x + ray.c_yx > 0));
			
	    case NPO:
		    return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			        || (ray.origin.x < min.x) || (ray.origin.y > max.y) 
			        || (ray.y_x * min.x - min.y + ray.c_xy < 0) 
			        || (ray.x_y * max.y - max.x + ray.c_yx > 0));
			
	    case PNO:
		    return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			        || (ray.origin.x > max.x) || (ray.origin.y < min.y) 
			        || (ray.y_x * max.x - max.y + ray.c_xy > 0)
			        || (ray.x_y * min.y - min.x + ray.c_yx < 0));
			
	    case PPO:
		    return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			        || (ray.origin.x > max.x) || (ray.origin.y > max.y)
			        || (ray.y_x * max.x - min.y + ray.c_xy < 0)
			        || (ray.x_y * max.y - min.x + ray.c_yx < 0));
			
	    case NOO:
		    return !((ray.origin.x < min.x)
			        || (ray.origin.y < min.y) || (ray.origin.y > max.y)
			        || (ray.origin.z < min.z) || (ray.origin.z > max.z));
			
	    case POO:
		    return !((ray.origin.x > max.x)
			        || (ray.origin.y < min.y) || (ray.origin.y > max.y)
			        || (ray.origin.z < min.z) || (ray.origin.z > max.z));
			
	    case ONO:
		    return !((ray.origin.y < min.y)
			        || (ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.z < min.z) || (ray.origin.z > max.z));
			
	    case OPO:
		    return !((ray.origin.y > max.y)
			        || (ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.z < min.z) || (ray.origin.z > max.z));
			
	    case OON:
		    return !((ray.origin.z < min.z)
			        || (ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.y < min.y) || (ray.origin.y > max.y));
			
	    case OOP:
		    return !((ray.origin.z > max.z)
			        || (ray.origin.x < min.x) || (ray.origin.x > max.x)
			        || (ray.origin.y < min.y) || (ray.origin.y > max.y));
			
	
	}

	return false;
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
bool intersection(Ray ray, RayIntersection *out, Triangle *tri) {
    float normalDOTray = dot(tri->normal, ray.direction); 
    
    float3 h = cross(ray.direction, tri->e2);
	float a = dot(tri->e1, h);

    if (a > -EPSILON && a < EPSILON) {
		return false;
    }
	float f = 1.0f / a;
	float3 s = ray.origin - tri->vertices[0];
	float u = f * dot(s, h);

	if (u < 0.0 || u > 1.0) {
		return false;
    }
	float3 q = cross(s, tri->e1);
	float v = f * dot(ray.direction, q);

	if (v < 0.0 || u + v > 1.0) {
		return false;
    }
	
	float t = f * dot(tri->e2, q);

	if (t < 0) {
		return false;
    }

	if (out != nullptr) {
		out->distance = t;
		out->normal = tri->normal;
		out->point = ray.origin + t * ray.direction;
        out->shapeMaterial = tri->material;
		out->isEntering = normalDOTray < 0.0f;

        out->point += out->normal * EPSILON;
	}

	return true;
}

__device__
bool intersection(Ray ray, RayIntersection *out, Sphere *sphere) {
    float d_2, r_2, b, root, t;

    float3 s_r = sphere->center - ray.origin;
    
    r_2 = sphere->radius * sphere->radius;
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
		out->normal = normalize((out->point - sphere->center) / sphere->radius);

		bool entering = true;
		if (d_2 < r_2) {
			out->normal *= -1.0f;
			entering = false;
		}
        
        out->point += out->normal * EPSILON;
		out->shapeMaterial = sphere->material;
		out->distance = t;
        out->isEntering = entering;
	}

    return true;
}

__device__
bool infiniteCylinderIntersection(Ray ray, RayIntersection *out, Cylinder *cylinder, float3 axis, float *inD, float *outD) {
    float3 r_c = ray.origin - cylinder->base;
    float r_2 = cylinder->radius * cylinder->radius;
    float3 n = cross(ray.direction, axis);

    float ln = length(n);

    // check if is parallel
    if(equal(ln, 0.0f)) {
        *inD = -1.0e21;
	    *outD = 1.0e21;
        return length(r_c - dot(r_c, axis) * axis) <= cylinder->radius;
    }
    n = normalize(n);

    float d = fabs(dot(r_c, n));

    if (d <= cylinder->radius) {
        float3 O = cross(r_c, axis);
    
        float t = -dot(O, n) / ln;
    
        O = normalize(cross(n, axis));

        float s = fabs(sqrtf(r_2 - d*d) / dot(ray.direction, O));

        *inD = t - s;
        *outD = t + s;

        return true;
    }

	return false;
}

__device__
bool intersection(Ray ray, RayIntersection *out, Cylinder *cylinder) {
    float3 axis = normalize(cylinder->top - cylinder->base);
    float3 normal, point; 

    float baseDistance = -dot(-axis, cylinder->base);
    float topDistance = -dot(axis, cylinder->top);

    float dc, dw, t;
	float inD, outD;		/* Object  intersection dists.	*/
    //0 top, 1 side, 2 base
    short sideIn;
    short sideOut;

    if(!infiniteCylinderIntersection(ray, out, cylinder, axis, &inD, &outD)) {
        return false;
    }
    
    sideIn = sideOut = 1;

    /*	Intersect the ray with the bottom end-cap plane.		*/

	dc = dot(-axis, ray.direction);
    dw = dot(-axis, ray.origin) + baseDistance;

    if(dc == 0.0f) {		/* If parallel to bottom plane	*/
        if(dw >= 0.0f) {
            return false;
        }
    } else {
        t  = -dw / dc;
        if(dc >= 0.0f) {			    /* If far plane	*/
            if(t > inD && t < outD) {
                outD = t;
                sideOut = 2;
            }
            if(t < inD) {
                return false;
            }
        } else {				    /* If near plane	*/
            if(t > inD && t < outD) {
                inD	= t;
                sideIn = 2;
                
            }
            if(t > outD) {
                return false;
            }
        }
    }

/*	Intersect the ray with the top end-cap plane.			*/

    dc = dot(axis, ray.direction);
    dw = dot(axis, ray.origin) + topDistance;

	if(dc == 0.0f) {		/* If parallel to top plane	*/
	    if(dw >= 0.0f) {
            return false;
        }
	} else {
	    t  = - dw/dc;
	    if	(dc >= 0.0f) {			    /* If far plane	*/
		    if(t > inD && t < outD) {
                outD = t;
                sideOut = 0;
            }
		    if(t < inD) {
                return false;
            }
	    } else {				    /* If near plane	*/
		    if(t > inD && t < outD) {
                inD	= t;
                sideIn = 0;
                
            }
		    if(t > outD) {
                return false;
	        }
	    } 
    }

    if (inD < 0 && outD < 0) {
		return false;
    }

	if (inD < outD && inD > 0) {
		t = inD;
        point = ray.origin + t * ray.direction;

        if(sideIn == 0) {
            normal = axis;
        } else if(sideIn == 1) {
            float3 v1 = point - cylinder->base;
	        float3 v2 = dot(v1, axis) * axis;
            normal = normalize(v1 - v2);
        } else {
            normal = -axis;
        }
        
    } else if (outD > 0) {
		t = outD;

        point = ray.origin + t * ray.direction;

        if(sideOut == 0) {
            normal = -axis;
        } else if(sideOut == 1) {
            float3 v1 = point - cylinder->base;
	        float3 v2 = dot(v1, axis) * axis;
	        normal = normalize(v2 - v1);
        } else {
            normal = axis;
        }
        
    } else {
        return false;
    }

    if (out != nullptr) {
        out->isEntering = dot(normal, ray.direction) < 0.0f;
        out->shapeMaterial = cylinder->material;
        out->distance = t;
        out->point = point;
        out->normal = normal;

        out->point += normal * EPSILON;
	}

    return true;
}

#endif