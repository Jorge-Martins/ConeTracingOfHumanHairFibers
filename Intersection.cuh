#pragma once

#ifndef _INTERSECTION_
#define _INTERSECTION_

#include "Primitives.h"

__device__
bool equal(float f1, float f2) {
	float diffAbs = fabsf(f1 - f2);
	return diffAbs < EPSILON;
}

__device__
float3 getTransformed(Matrix *m, float3 *translation, float3 v) {
    return *m * (v + *translation);
}

__device__
bool nnn(Ray ray, float3 min, float3 max) {	
    return !((ray.origin.x < min.x) || (ray.origin.y < min.y) || (ray.origin.z < min.z)
			|| (ray.y_x * min.x - max.y + ray.c_xy > 0)
			|| (ray.x_y * min.y - max.x + ray.c_yx > 0)
			|| (ray.y_z * min.z - max.y + ray.c_zy > 0)
			|| (ray.z_y * min.y - max.z + ray.c_yz > 0)
			|| (ray.z_x * min.x - max.z + ray.c_xz > 0)
			|| (ray.x_z * min.z - max.x + ray.c_zx > 0));
}

__device__
bool nnp(Ray ray, float3 min, float3 max) {	
	return !((ray.origin.x < min.x) || (ray.origin.y < min.y) || (ray.origin.z > max.z)
			|| (ray.y_x * min.x - max.y + ray.c_xy > 0)
			|| (ray.x_y * min.y - max.x + ray.c_yx > 0)
			|| (ray.y_z * max.z - max.y + ray.c_zy > 0)
			|| (ray.z_y * min.y - min.z + ray.c_yz < 0)
			|| (ray.z_x * min.x - min.z + ray.c_xz < 0)
			|| (ray.x_z * max.z - max.x + ray.c_zx > 0));
}

__device__
bool npn(Ray ray, float3 min, float3 max) {	
	return !((ray.origin.x < min.x) || (ray.origin.y > max.y) || (ray.origin.z < min.z)
			|| (ray.y_x * min.x - min.y + ray.c_xy < 0) 
			|| (ray.x_y * max.y - max.x + ray.c_yx > 0)
			|| (ray.y_z * min.z - min.y + ray.c_zy < 0) 
			|| (ray.z_y * max.y - max.z + ray.c_yz > 0)
			|| (ray.z_x * min.x - max.z + ray.c_xz > 0)
			|| (ray.x_z * min.z - max.x + ray.c_zx > 0));
}

__device__
bool npp(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x < min.x) || (ray.origin.y > max.y) || (ray.origin.z > max.z)
			|| (ray.y_x * min.x - min.y + ray.c_xy < 0) 
			|| (ray.x_y * max.y - max.x + ray.c_yx > 0)
			|| (ray.y_z * max.z - min.y + ray.c_zy < 0)
			|| (ray.z_y * max.y - min.z + ray.c_yz < 0)
			|| (ray.z_x * min.x - min.z + ray.c_xz < 0)
			|| (ray.x_z * max.z - max.x + ray.c_zx > 0));
}

__device__
bool pnn(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x > max.x) || (ray.origin.y < min.y) || (ray.origin.z < min.z)
			|| (ray.y_x * max.x - max.y + ray.c_xy > 0)
			|| (ray.x_y * min.y - min.x + ray.c_yx < 0)
			|| (ray.y_z * min.z - max.y + ray.c_zy > 0)
			|| (ray.z_y * min.y - max.z + ray.c_yz > 0)
			|| (ray.z_x * max.x - max.z + ray.c_xz > 0)
			|| (ray.x_z * min.z - min.x + ray.c_zx < 0));
}

__device__
bool pnp(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x > max.x) || (ray.origin.y < min.y) || (ray.origin.z > max.z)
			|| (ray.y_x * max.x - max.y + ray.c_xy > 0)
			|| (ray.x_y * min.y - min.x + ray.c_yx < 0)
			|| (ray.y_z * max.z - max.y + ray.c_zy > 0)
			|| (ray.z_y * min.y - min.z + ray.c_yz < 0)
			|| (ray.z_x * max.x - min.z + ray.c_xz < 0)
			|| (ray.x_z * max.z - min.x + ray.c_zx < 0));
}

__device__
bool ppn(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x > max.x) || (ray.origin.y > max.y) || (ray.origin.z < min.z)
			|| (ray.y_x * max.x - min.y + ray.c_xy < 0)
			|| (ray.x_y * max.y - min.x + ray.c_yx < 0)
			|| (ray.y_z * min.z - min.y + ray.c_zy < 0) 
			|| (ray.z_y * max.y - max.z + ray.c_yz > 0)
			|| (ray.z_x * max.x - max.z + ray.c_xz > 0)
			|| (ray.x_z * min.z - min.x + ray.c_zx < 0));
}

__device__
bool ppp(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x > max.x) || (ray.origin.y > max.y) || (ray.origin.z > max.z)
			|| (ray.y_x * max.x - min.y + ray.c_xy < 0)
			|| (ray.x_y * max.y - min.x + ray.c_yx < 0)
			|| (ray.y_z * max.z - min.y + ray.c_zy < 0)
			|| (ray.z_y * max.y - min.z + ray.c_yz < 0)
			|| (ray.z_x * max.x - min.z + ray.c_xz < 0)
			|| (ray.x_z * max.z - min.x + ray.c_zx < 0));
}

__device__
bool onn(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.y < min.y) || (ray.origin.z < min.z)
			|| (ray.y_z * min.z - max.y + ray.c_zy > 0)
			|| (ray.z_y * min.y - max.z + ray.c_yz > 0));
}

__device__
bool onp(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.y < min.y) || (ray.origin.z > max.z)
			|| (ray.y_z * max.z - max.y + ray.c_zy > 0)
			|| (ray.z_y * min.y - min.z + ray.c_yz < 0));
}

__device__
bool opn(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.y > max.y) || (ray.origin.z < min.z)
			|| (ray.y_z * min.z - min.y + ray.c_zy < 0) 
			|| (ray.z_y * max.y - max.z + ray.c_yz > 0));
}

__device__
bool opp(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.y > max.y) || (ray.origin.z > max.z)
			|| (ray.y_z * max.z - min.y + ray.c_zy < 0)
			|| (ray.z_y * max.y - min.z + ray.c_yz < 0));
}

__device__
bool non(Ray ray, float3 min, float3 max) {
	return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			|| (ray.origin.x < min.x) || (ray.origin.z < min.z) 
			|| (ray.z_x * min.x - max.z + ray.c_xz > 0)
			|| (ray.x_z * min.z - max.x + ray.c_zx > 0));
}

__device__
bool nop(Ray ray, float3 min, float3 max) {
	return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			|| (ray.origin.x < min.x) || (ray.origin.z > max.z) 
			|| (ray.z_x * min.x - min.z + ray.c_xz < 0)
			|| (ray.x_z * max.z - max.x + ray.c_zx > 0));
}

__device__
bool pon(Ray ray, float3 min, float3 max) {
	return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			|| (ray.origin.x > max.x) || (ray.origin.z < min.z)
			|| (ray.z_x * max.x - max.z + ray.c_xz > 0)
			|| (ray.x_z * min.z - min.x + ray.c_zx < 0));
}

__device__
bool pop(Ray ray, float3 min, float3 max) {
	return !((ray.origin.y < min.y) || (ray.origin.y > max.y)
			|| (ray.origin.x > max.x) || (ray.origin.z > max.z)
			|| (ray.z_x * max.x - min.z + ray.c_xz < 0)
			|| (ray.x_z * max.z - min.x + ray.c_zx < 0));
}

__device__
bool nno(Ray ray, float3 min, float3 max) {
	return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			|| (ray.origin.x < min.x) || (ray.origin.y < min.y) 
			|| (ray.y_x * min.x - max.y + ray.c_xy > 0)
			|| (ray.x_y * min.y - max.x + ray.c_yx > 0));
}

__device__
bool npo(Ray ray, float3 min, float3 max) {
	return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			|| (ray.origin.x < min.x) || (ray.origin.y > max.y) 
			|| (ray.y_x * min.x - min.y + ray.c_xy < 0) 
			|| (ray.x_y * max.y - max.x + ray.c_yx > 0));
}

__device__
bool pno(Ray ray, float3 min, float3 max) {
	return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			|| (ray.origin.x > max.x) || (ray.origin.y < min.y) 
			|| (ray.y_x * max.x - max.y + ray.c_xy > 0)
			|| (ray.x_y * min.y - min.x + ray.c_yx < 0));
}

__device__
bool ppo(Ray ray, float3 min, float3 max) {
	return !((ray.origin.z < min.z) || (ray.origin.z > max.z)
			|| (ray.origin.x > max.x) || (ray.origin.y > max.y)
			|| (ray.y_x * max.x - min.y + ray.c_xy < 0)
			|| (ray.x_y * max.y - min.x + ray.c_yx < 0));
}

__device__
bool noo(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x < min.x)
			|| (ray.origin.y < min.y) || (ray.origin.y > max.y)
			|| (ray.origin.z < min.z) || (ray.origin.z > max.z));
}

__device__
bool poo(Ray ray, float3 min, float3 max) {
	return !((ray.origin.x > max.x)
			|| (ray.origin.y < min.y) || (ray.origin.y > max.y)
			|| (ray.origin.z < min.z) || (ray.origin.z > max.z));
}

__device__
bool ono(Ray ray, float3 min, float3 max) {
	return !((ray.origin.y < min.y)
			|| (ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.z < min.z) || (ray.origin.z > max.z));
}

__device__
bool opo(Ray ray, float3 min, float3 max) {
	return !((ray.origin.y > max.y)
			|| (ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.z < min.z) || (ray.origin.z > max.z));
}

__device__
bool oon(Ray ray, float3 min, float3 max) {
	return !((ray.origin.z < min.z)
			|| (ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.y < min.y) || (ray.origin.y > max.y));
}

__device__
bool oop(Ray ray, float3 min, float3 max) {
	return !((ray.origin.z > max.z)
			|| (ray.origin.x < min.x) || (ray.origin.x > max.x)
			|| (ray.origin.y < min.y) || (ray.origin.y > max.y));
}

/* AABB intersection with ray slopes */
__device__
bool AABBIntersection(Ray ray, float3 min, float3 max) {
    switch (ray.classification) {
	    case NNN:	
            return nnn(ray, min, max);
			
	    case NNP:	
		    return nnp(ray, min, max);

	    case NPN:	
		    return npn(ray, min, max);
			
	    case NPP:
		    return npp(ray, min, max);
			
	    case PNN:
		    return pnn(ray, min, max);
			
	    case PNP:
		    return pnp(ray, min, max);

	    case PPN:
		    return ppn(ray, min, max);
			
	    case PPP:
		    return ppp(ray, min, max);
			
	    case ONN:
		    return onn(ray, min, max);
			
	    case ONP:
		    return onp(ray, min, max);
			
	    case OPN:
		    return opn(ray, min, max);
			
	    case OPP:
		    return opp(ray, min, max);
			
	    case NON:
		    return non(ray, min, max);
			
	    case NOP:
		    return nop(ray, min, max);
			
	    case PON:
		    return pon(ray, min, max);
			
	    case POP:
		    return pop(ray, min, max);
			
	    case NNO:
		    return nno(ray, min, max);
			
	    case NPO:
		    return npo(ray, min, max);
			
	    case PNO:
		    return pno(ray, min, max);
			
	    case PPO:
		    return ppo(ray, min, max);
			
	    case NOO:
		    return noo(ray, min, max);
			
	    case POO:
		    return poo(ray, min, max);
			
	    case ONO:
		    return ono(ray, min, max);
			
	    case OPO:
		    return opo(ray, min, max);
			
	    case OON:
		    return oon(ray, min, max);
			
	    case OOP:
		    return oop(ray, min, max);
			
	}

	return false;
}

__device__
bool AABBIntersection(Ray ray, float3 min, float3 max, float &distance) {
    float t1, t2;
    switch (ray.classification) {
	    case NNN:	
            if(nnn(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
			    t1 = (max.y - ray.origin.y) * ray.invDirection.y;
			    if(t1 > distance) {
				    distance = t1;
                }
			    t2 = (max.z - ray.origin.z) * ray.invDirection.z;
			    if(t2 > distance) {
				    distance = t2;
                }

			    return true;
            }
			break;

	    case NNP:	
		    if(nnp(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
			    t1 = (max.y - ray.origin.y) * ray.invDirection.y;
			    if(t1 > distance) {
				    distance = t1;
                }
			    t2 = (min.z - ray.origin.z) * ray.invDirection.z;
			    if(t2 > distance) {
				    distance = t2;
                }

			    return true;
            }
            break;

	    case NPN:	
		    if(npn(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
		        t1 = (min.y - ray.origin.y) * ray.invDirection.y;
		        if(t1 > distance) {
			        distance = t1;
                }
		        t2 = (max.z - ray.origin.z) * ray.invDirection.z;
		        if(t2 > distance) {
			        distance = t2;
                }

		        return true;
            }
			break;

	    case NPP:
		    if(npp(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
                t1 = (min.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case PNN:
		    if(pnn(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t1 = (max.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }
                t2 = (max.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case PNP:
		    if(pnp(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t1 = (max.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
            break;

	    case PPN:
		    if(ppn(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t1 = (min.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }
                t2 = (max.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case PPP:
		    if(ppp(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t1 = (min.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case ONN:
		    if(onn(ray, min, max)) {
                distance = (max.y - ray.origin.y) * ray.invDirection.y;
                t2 = (max.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;

            }
			break;

	    case ONP:
		    if(onp(ray, min, max)) {
                distance = (max.y - ray.origin.y) * ray.invDirection.y;
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;

            }
			break;

	    case OPN:
		    if(opn(ray, min, max)) {
                distance = (min.y - ray.origin.y) * ray.invDirection.y;		
                t2 = (max.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case OPP:
		    if(opp(ray, min, max)) {
                distance = (min.y - ray.origin.y) * ray.invDirection.y;		
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case NON:
		    if(non(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
                t2 = (max.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case NOP:
		    if(nop(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case PON:
		    if(pon(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t2 = (max.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case POP:
		    if(pop(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t2 = (min.z - ray.origin.z) * ray.invDirection.z;
                if(t2 > distance) {
                    distance = t2;
                }

                return true;
            }
			break;

	    case NNO:
		    if(nno(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
                t1 = (max.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }

                return true;
            }
			break;

	    case NPO:
		    if(npo(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
                t1 = (min.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }

                return true;
            }
			break;

	    case PNO:
		    if(pno(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t1 = (max.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }

                return true;
            }
			break;

	    case PPO:
		    if(ppo(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                t1 = (min.y - ray.origin.y) * ray.invDirection.y;
                if(t1 > distance) {
                    distance = t1;
                }

                return true;
            }
			
	    case NOO:
		    if(noo(ray, min, max)) {
                distance = (max.x - ray.origin.x) * ray.invDirection.x;
                return true;
            }
			break;

	    case POO:
		    if(poo(ray, min, max)) {
                distance = (min.x - ray.origin.x) * ray.invDirection.x;
                return true;
            }
			break;

	    case ONO:
		    if(ono(ray, min, max)) {
                distance = (max.y - ray.origin.y) * ray.invDirection.y;
                return true;
            }
			break;

	    case OPO:
		    if(opo(ray, min, max)) {
                distance = (min.y - ray.origin.y) * ray.invDirection.y;
                return true;
            }
			break;

	    case OON:
		    if(oon(ray, min, max)) {
                distance = (max.z - ray.origin.z) * ray.invDirection.z;
                return true;
            }
			break;

	    case OOP:
		    if(oop(ray, min, max)) {
                distance = (min.z - ray.origin.z) * ray.invDirection.z;
                return true;
            }
			break;

	}

	return false;
}

__device__
bool OBBIntersection(Ray ray, float3 min, float3 max, Matrix *m, float3 *translation) {
    ray.update(*m * (ray.origin + *translation), *m * ray.direction);
    
    return AABBIntersection(ray, min, max);
}

__device__
bool OBBIntersection(Ray ray, float3 min, float3 max, Matrix *m, float3 *translation, float &distance) {
    ray.update(*m * (ray.origin + *translation), *m * ray.direction);
    
    return AABBIntersection(ray, min, max, distance);
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

    float sRoot = sqrtf(root);
    t = fminf(b - sRoot, b + sRoot);

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
        *inD = -1.0e21f;
	    *outD = 1.0e21f;
        return length(r_c - dot(r_c, axis) * axis) <= cylinder->radius;
    }
    n = normalize(n);

    float d = fabsf(dot(r_c, n));

    if (d <= cylinder->radius) {
        float3 O = cross(r_c, axis);
    
        float t = -dot(O, n) / ln;
    
        O = normalize(cross(n, axis));

        float s = fabsf(sqrtf(r_2 - d*d) / dot(ray.direction, O));

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
    unsigned char sideIn;
    unsigned char sideOut;

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

    bool entering = true;
    unsigned char side = sideIn;
	if (inD < outD && inD > 0) {
		t = inD;
        point = ray.origin + t * ray.direction;

    } else if (outD > 0) {
		t = outD;

        point = ray.origin + t * ray.direction;

        side = sideOut;
        entering = false;
        
    } else {
        return false;
    }

    if(side == 0) {
        normal = axis;
    } else if(side == 1) {
        float3 v1 = point - cylinder->base;
        normal = normalize(v1 - projectVector(v1, axis));
    } else {
        normal = -axis;
    }

    if (out != nullptr) {
        if(!entering) {
            normal *= -1.0f;
        }

        out->isEntering = entering;
        out->shapeMaterial = cylinder->material;
        out->distance = t;
        out->point = point;
        out->normal = normal;

        out->point += normal * EPSILON;
        
	}

    return true;
}

//---------- Cones ---------------------------------------

__device__
bool AABBIntersection(Cone cone, float3 min, float3 max) {
    float3 boxCenter = (max + min) * 0.5f;
    float3 boxExtent = (max - min) * 0.5f;
    
    // Quick-rejection test for boxes below the supporting plane of the cone.
    float3 boxConeVec = boxCenter - cone.origin;
    float ConeDirDOTboxConeVec = dot(cone.direction, boxConeVec);
    float radius =  dot(boxExtent, fabsf(cone.direction));

    if(ConeDirDOTboxConeVec + radius <= 0.0f) {
        // The box is in the halfspace below the supporting plane of the cone.
        return false;
    }

    // Determine the box faces that are visible to the cone vertex.
    int type = 0;
    type += (boxConeVec.x < -boxExtent.x ? 2 : (boxConeVec.x > boxExtent.x ? 0 : 1));
    type += 3 * (boxConeVec.y < -boxExtent.y ? 2 : (boxConeVec.y > boxExtent.y ? 0 : 1));
    type += 9 * (boxConeVec.z < -boxExtent.z ? 2 : (boxConeVec.z > boxExtent.z ? 0 : 1));
    
    //if cone vertex inside the box
    if(type == 13) {
        // The cone vertex is in the box.
        return true;
    }
    
    AABBPolygon polygon = AABBPolygon(type);

    // Test polygon points.
    float3 X[8], PmV[8];
    float coneDirDOTPmV[8], sqrConeDirDOTPmV[8], sqrLenPmV[8], q;
    int iMax = -1, jMax = -1;
    float coneCosAngle_2 = cos(cone.spread) * cos(cone.spread);

    for(unsigned char i = 0; i < polygon.nPoints; i++) {
        int j = polygon.point[i];
        X[j].x = (j & 1 ? boxExtent.x : -boxExtent.x);
        X[j].y = (j & 2 ? boxExtent.y : -boxExtent.y);
        X[j].z = (j & 4 ? boxExtent.z : -boxExtent.z);
        coneDirDOTPmV[j] = dot(cone.direction, X[j]) + ConeDirDOTboxConeVec;

        if(coneDirDOTPmV[j] > 0.0f) {
            PmV[j] = X[j] + boxConeVec;
            sqrConeDirDOTPmV[j] = coneDirDOTPmV[j] * coneDirDOTPmV[j];
            sqrLenPmV[j] = dot(PmV[j], PmV[j]);
            q = sqrConeDirDOTPmV[j] - coneCosAngle_2 * sqrLenPmV[j];

            if(q > 0.0f) {
                return true;
            }

            // Keep track of the maximum in case we must process box edges.
            // This supports the gradient ascent search.
            if (iMax == -1 || sqrConeDirDOTPmV[j] * sqrLenPmV[jMax] > sqrConeDirDOTPmV[jMax] * sqrLenPmV[j]) {
                iMax = i;
                jMax = j;
            }
        }
    }

    if(iMax == -1) {
        return false;
    }

    // Start the gradient ascent search at index jMax.
    float maxSqrLenPmV = sqrLenPmV[jMax];
    float maxConeDirDOTPmV = coneDirDOTPmV[jMax];
    float3 maxX = X[jMax];
    float maxPmV[] = {PmV[jMax].x, PmV[jMax].y, PmV[jMax].z};
    int k0, k1, k2, jDiff;
    float s, fder, numer, denom, DdMmV, det;
    float3 MmV;
    float coneDirection[] = {cone.direction.x, cone.direction.y, cone.direction.z};

    // Search the counterclockwise edge <corner[jMax],corner[jNext]>.
    int iNext = (iMax < polygon.nPoints - 1 ? iMax + 1 : 0);
    int jNext = polygon.point[iNext];
    jDiff = jNext - jMax;
    s = (jDiff > 0 ? 1.0f : -1.0f);
    k0 = abs(jDiff) >> 1;
    fder = s * (coneDirection[k0] * maxSqrLenPmV - maxConeDirDOTPmV * maxPmV[k0]);

    if(fder > 0.0f) {
        // The edge has an interior local maximum in F because
        // F(K[j0]) >= F(K[j1]) and the directional derivative of F at K0
        // is positive.  Compute the local maximum point.
        k1 = (k0 + 1) % 3;
        k2 = (k1 + 1) % 3;
        numer = maxPmV[k1] * maxPmV[k1] + maxPmV[k2] * maxPmV[k2];
        denom = coneDirection[k1] * maxPmV[k1] + coneDirection[k2] * maxPmV[k2];

        setVectorValue(MmV, k0, numer * coneDirection[k0]);
        setVectorValue(MmV, k1, denom * (getVectorValue(maxX, k1) + getVectorValue(boxConeVec, k1)));
        setVectorValue(MmV, k2, denom * (getVectorValue(maxX, k2) + getVectorValue(boxConeVec, k2)));

        // Theoretically, DdMmV > 0, so there is no need to test positivity.
        DdMmV = dot(cone.direction, MmV);
        q = DdMmV * DdMmV - coneCosAngle_2 * dot(MmV, MmV);
        
        if(q > 0.0f) {
            return true;
        }

        // Determine on which side of the spherical arc coneDirection lives on.  
        // If in the polygon side, then there is an intersection.
        det = s * (coneDirection[k1] * maxPmV[k2] - coneDirection[k2] * maxPmV[k1]);
        
        return (det <= 0.0f);
        
    }

    // Search the clockwise edge <corner[jMax],corner[jPrev]>.
    int iPrev = (iMax > 0 ? iMax - 1 : polygon.nPoints - 1);
    int jPrev = polygon.point[iPrev];
    jDiff = jMax - jPrev;
    s = (jDiff > 0 ? 1.0f : -1.0f);
    k0 = abs(jDiff) >> 1;
    fder = -s * (coneDirection[k0] * maxSqrLenPmV - maxConeDirDOTPmV * maxPmV[k0]);
    
    if(fder > 0.0f) {
        // The edge has an interior local maximum in F because
        // F(K[j0]) >= F(K[j1]) and the directional derivative of F at K0
        // is positive.  Compute the local maximum point.
        k1 = (k0 + 1) % 3;
        k2 = (k1 + 1) % 3;
        numer = maxPmV[k1] * maxPmV[k1] + maxPmV[k2] * maxPmV[k2];
        denom = coneDirection[k1] * maxPmV[k1] + coneDirection[k2] * maxPmV[k2];

        setVectorValue(MmV, k0, numer * coneDirection[k0]);
        setVectorValue(MmV, k1, denom * (getVectorValue(maxX, k1) + getVectorValue(boxConeVec, k1)));
        setVectorValue(MmV, k2, denom * (getVectorValue(maxX, k2) + getVectorValue(boxConeVec, k2)));

        // Theoretically, DdMmV > 0, so there is no need to test positivity.
        DdMmV = dot(cone.direction, MmV);
        q = DdMmV * DdMmV - coneCosAngle_2 * dot(MmV, MmV);
        
        if(q > 0.0f) {
            return true;
        }

        // Determine on which side of the spherical arc coneDirection lives on.  
        // If in the polygon side, then there is an intersection.
        det = s * (coneDirection[k1] * maxPmV[k2] - coneDirection[k2] * maxPmV[k1]);

        return (det <= 0.0f);
    }

    return false;
}

__device__
bool AABBIntersection(Cone cone, float3 min, float3 max, float &distance) {
   distance = 0.0f;
   return AABBIntersection(cone, min, max);
}

__device__
bool OBBIntersection(Cone cone, float3 min, float3 max, Matrix *m, float3 *translation) {
    cone.origin = *m * (cone.origin + *translation);
    cone.direction = *m * cone.direction;
    
    return AABBIntersection(cone, min, max);
}

__device__
bool OBBIntersection(Cone cone, float3 min, float3 max, Matrix *m, float3 *translation, float &distance) {
    cone.origin = *m * (cone.origin + *translation);
    cone.direction = *m * cone.direction;

    return AABBIntersection(cone, min, max, distance);
}


__device__
float2 cangle(float2 s, float2 t) {
	return make_float2(dot(s,t),s.x*t.y-s.y*t.x);
}

__device__
float2 cmul(float2 s, float2 t) {
	return make_float2(s.x*t.x-s.y*t.y,s.x*t.y+s.y*t.x);
}

//this ignores length
__device__
float2 csqrt(float2 a) {
	float lg = length(a);
	float r = lg + a.x;
	float c = sqrtf((lg - a.x) * r) * (float)(((int)(a.y) & 0x80000000) + 0x3f800000);
	
    return make_float2(r,c);
}

__device__
float quadCircleIntersectionArea(float2 *quad, float coneCircleR) {
    float area = 0.0f;
    float2 cc = make_float2(1.0f, 0.0f);
    float2 A, B, D;
    float sqA, sqD;
    float sqR = sqrtf(coneCircleR);
    float isqR = 1.0f / sqR;
    size_t has0, has1;

    for(int i = 0; i < 4; i++) {
	    A = quad[(i + 3) % 4];
	    B = quad[i];
	    D = B - A;

        sqA = length(A);
	    sqD = length(D);
	    float ndt = -dot(A, D);

	    float delta = sqrtf(ndt) - (sqA - sqR) * sqD;
	    float2 c = make_float2(1.0f, 0.0f);

	    if(delta > 0.0f) {
		    float isqD = 1.0f / sqD;
		    float dside = sqrtf(delta);
		    float t0 = (ndt - dside) * isqD;
		    float t1 = (ndt + dside) * isqD;
		    float2 xt0 = t0 * D + A;
		    float2 xt1 = t1 * D + A;
		    has0 = ((size_t)((int)(t0)) <= 0x3f800000u);
		    has1 = ((size_t)((int)(t1)) <= 0x3f800000u);
		
            if(has0 > 0) {			
			    c = cmul(c, cangle(A, xt0));
			    A = xt0;
            }

		    if(has1 > 0) {			
			    c = cmul(c, cangle(xt1, B));
			    B = xt1;
            }

        } else {
		    has0 = 0;
		    has1 = 0;
        }

	    if(!(has0 | has1) && sqA > sqR) {
		    c = cmul(c, cangle(A, B));

        } else {
		    area += length(cross(make_float3(A.x, A.y, 0.0f), make_float3(B.x, B.y, 0.0f))) * isqR;
        }
	
        cc=cmul(cc,csqrt(c));
        area += atan2f(cc.y, cc.x) * 2.0f;
    }

    return area;
}

__device__
float SignedPolygonArea(float2 *polygon, int nPoints) {
    float area = 0.0f;
    int j = nPoints - 1;  // The last vertex is the 'previous' one to the first

  for(int i = 0; i<nPoints; i++) { 
      area = area +  (polygon[j].x+polygon[i].x) * (polygon[j].y-polygon[i].y); 
      j = i;  //j is previous vertex to i
    }

  return area * 0.5f;
}


//aproximate cone cylinder intersection
__device__
bool intersection(Cone cone, RayIntersection *out, Cylinder *cylinder) {
    float3 coneCircleV = cone.origin + cone.direction * (length((fmaxf(cylinder->base, cylinder->top) + cylinder->radius) - 
                                                         cone.origin));
    float coneCircleR = length(coneCircleV - cone.origin) * tanf(cone.spread);

    //find projection plane
    float3 planeNormal = -cone.direction;
    float planeDistance = -dot(planeNormal, coneCircleV);

    //compute plane coordenate system and convert the problem to 2D
    float3 planeXAxis = normalize(cross(make_float3(0.0f, 0.0f, 1.0f), planeNormal));
    float3 planeYAxis = normalize(cross(planeNormal, planeXAxis));

    /*approximation through rays
    Idea shoot rays on the border of the circle and estimate the area (polygon area)
       _x_
     x/   \x
    x|  x  |x
     x\_x_/x
        
    */

    int nPoints = 0;
    float2 polygon[(int)N_POLYGONS_POINTS];
    float theta = 0.0f;
    float thetaStep = 2 * PI / N_POLYGONS_POINTS;
    float3 direction;

    float3 normals = make_float3(0.0f);
    float distances = FLT_MAX;
    float3 points;

    for(int i = 0; i < N_POLYGONS_POINTS; i++) {
        float3 point = coneCircleV + (planeYAxis * coneCircleR * sinf(theta) - planeXAxis * coneCircleR * cosf(theta));

        direction = normalize(point - cone.origin);
        if(intersection(Ray(cone.origin, direction), out, cylinder)) {                  
            normals += out->normal;
                    
            if(distances > out->distance) {
                distances = out->distance;
                points = out->point;
            }

            polygon[nPoints++] = make_float2(dot(point, planeXAxis), dot(point, planeYAxis));
        }

        theta += thetaStep;
    }

    //If less than half of the points test the circle vertex
    if(nPoints < (0.5f * N_POLYGONS_POINTS)) {
        if(intersection(Ray(cone.origin, cone.direction), out, cylinder)) {
            normals += out->normal;
                    
            if(distances > out->distance) {
                distances = out->distance;
                points = out->point;
            }

            //add circle vertex to polygon
            polygon[nPoints++] = make_float2(dot(coneCircleV, planeXAxis), dot(coneCircleV, planeYAxis));
        }
    }

    if(nPoints <= 2) {
        return false;
    }

    float area = SignedPolygonArea(polygon, nPoints);

    if(area > 0.0f && out != nullptr) {
        float maxArea = 0.5f * coneCircleR * coneCircleR * N_POLYGONS_POINTS * sinf(thetaStep); 
         
        out->normal = normalize(normals / nPoints);
        out->distance = distances;
        out->point = points;
        out->shapeMaterial = cylinder->material;

        //update area fraction
        out->shapeMaterial.ior = area / maxArea;

        return true;
        
    }

    return false;
}

//Proximate through projection into plane, quad creation, circle-quad intersection
__device__
bool intersection2(Cone cone, RayIntersection *out, Cylinder *cylinder) { 
    float3 quad[4];
    float3 coneCircleV = cone.origin + cone.direction * (length((fmaxf(cylinder->base, cylinder->top) + cylinder->radius) - 
                                                         cone.origin));
    float coneCircleR = length(coneCircleV - cone.origin) * tanf(cone.spread);

    //find projection plane
    float3 planeNormal = -cone.direction;
    float planeDistance = -dot(planeNormal, coneCircleV);

    //compute cylinder coordenate system
    float3 axis = normalize(cylinder->top - cylinder->base);
    float3 xx = normalize(cross(axis, cone.direction));
    float3 yy = normalize(cross(axis, xx));
    float cylinderBaseRadius[2];
    
    float3 point = cylinder->base;

     /*Quad 3D
        2---3
       /   /
      /   /
     0---1
    */

    for(int i = 0; i < 2; i++) {
        //compute 4 points, 2 on the base and 2 on the top and project to the plane
        quad[i * 2] = projectToPlane(point + cylinder->radius * xx, planeNormal, planeDistance);
        quad[i * 2 + 1] = projectToPlane(point - cylinder->radius * xx, planeNormal, planeDistance);

        cylinderBaseRadius[i] = 0.5f * length(projectToPlane(point + cylinder->radius * yy, planeNormal, planeDistance) -
                                              projectToPlane(point - cylinder->radius * yy, planeNormal, planeDistance));
    
        point = cylinder->top;
    }

    float3 projectedAxis = normalize(projectToPlane(cylinder->top, planeNormal, planeDistance) -
                                     projectToPlane(cylinder->base, planeNormal, planeDistance));
    
    //extend the quad according to the cylinder base diameters in the direction of the projected axis
    //base cylinder
    quad[0] -= projectedAxis * cylinderBaseRadius[0];
    quad[1] -= projectedAxis * cylinderBaseRadius[0];

    //top cylinder
    quad[2] += projectedAxis * cylinderBaseRadius[1];
    quad[3] += projectedAxis * cylinderBaseRadius[1];

    //compute plane coordenate system and convert the problem to 2D
    float3 planeXAxis = normalize(cross(make_float3(0.0f, 0.0f, 1.0f), planeNormal));
    float3 planeYAxis = normalize(cross(planeNormal, planeXAxis));

    
    float2 coneCircle2DV = make_float2(dot(coneCircleV, planeXAxis), dot(coneCircleV, planeYAxis));

    float2 quad2D[4];
    for(int i = 0; i < 4; i++) {
        quad2D[i].x = dot(quad[i], planeXAxis);
        quad2D[i].y = dot(quad[i], planeYAxis);

        quad2D[i] -= coneCircle2DV;

        if(i == 3) {
            float2 temp = quad2D[2];
            quad2D[2] = quad2D[3];
            quad2D[3] = temp;
        }
    }

    /*Quad 2D
        3---2
       /   /
      /   /
     0---1
    */
    
    float area = quadCircleIntersectionArea(quad2D, coneCircleR);

    if(area > 0.0f && out != nullptr) {
        float3 normals = make_float3(0.0f);
        float distances = FLT_MAX;
        float3 points;

        int nIntersections = 0;

        float3 rayDirections[5];
        rayDirections[0] = cone.direction;
        rayDirections[1] = normalize((coneCircleV + planeXAxis * coneCircleR) - cone.origin);
        rayDirections[2] = normalize((coneCircleV - planeXAxis * coneCircleR) - cone.origin);
        rayDirections[3] = normalize((coneCircleV + planeYAxis * coneCircleR) - cone.origin);
        rayDirections[4] = normalize((coneCircleV - planeYAxis * coneCircleR) - cone.origin);

        for(int i = 0; i < 5; i++) {
            if(intersection(Ray(cone.origin, rayDirections[i]), out, cylinder)) {
                normals += out->normal;
                    
                if(distances > out->distance) {
                    distances = out->distance;
                    points = out->point;
                }

                nIntersections++;
            }
        }
        
        out->normal = normalize(normals / nIntersections);
        out->distance = distances;
        out->point = points;
        out->shapeMaterial = cylinder->material;

        //update area fraction
        //out->shapeMaterial.ior = area / (PI * coneCircleR * coneCircleR);

        return true;
        
    }

    return false;
}

#endif