#pragma once

#ifndef _PRIMITIVES_
#define _PRIMITIVES_

#include <vector>

#include <iostream>
#include <string>
#include <sstream>

#include "MathUtil.h"

enum BBType {
    AABB,
    OBB
};

/* N - Negative
 * O - zero
 * P - Positive
 */
enum RayClassification {
    NNN, NNO, NNP, NON, NOO, NOP, NPN, NPO, NPP,
    ONN, ONO, ONP, OON, OOO, OOP, OPN, OPO, OPP,
    PNN, PNO, PNP, PON, POO, POP, PPN, PPO, PPP
        
};

struct Ray {
    float3 origin;
    float3 direction;
    
    bool exists;
    short classification;

    //slope
    float x_y, y_x, y_z, z_y, x_z, z_x; 
	float c_xy, c_xz, c_yx, c_yz, c_zx, c_zy;

    __host__ __device__
    void computeSlopes() {
        float3 invDirection = 1.0f / direction;
        x_y = direction.x * invDirection.y;
	    y_x = direction.y * invDirection.x;
	    y_z = direction.y * invDirection.z;
	    z_y = direction.z * invDirection.y;
	    x_z = direction.x * invDirection.z;
	    z_x = direction.z * invDirection.x;

	    c_xy = origin.y - y_x * origin.x;
	    c_xz = origin.z - z_x * origin.x;
	    c_yx = origin.x - x_y * origin.y;
	    c_yz = origin.z - z_y * origin.y;
	    c_zx = origin.x - x_z * origin.z;
	    c_zy = origin.y - y_z * origin.z;

        
        if(direction.x < 0) {
			classification = NNN;

		} else if(direction.x > 0){
			classification = PNN;

		} else {
			classification = ONN;
		}

        if(direction.y < 0) {
			//ignore
		} else if(direction.y > 0){
			classification += 6;

		} else {
			classification += 3;
		}

        if(direction.z < 0) {
			//ignore
		} else if(direction.z > 0){
			classification += 2;

		} else {
			classification += 1;
		}
    }

    __host__ __device__ 
    Ray() {
        origin = make_float3(0.0f);
        direction = make_float3(0.0f, 0.0f, 1.0f);
        exists = true;
    }

    __device__ 
    Ray(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        this->exists = true;

        computeSlopes();
    }

    __device__
    void update(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        this->exists = true;

        computeSlopes();
    }
};

struct Material {
    float3 color;
    float Kdiffuse;
    float Kspecular;
    float shininess;
    float transparency;
    float ior;

    __host__ __device__
    Material() {
        color = make_float3(0.0f);
        Kdiffuse = Kspecular = shininess = transparency = ior = 0.0f;
    }

    __host__ __device__
    Material(float3 color, float Kdiffuse, float Kspecular, float shininess, float transparency, float ior) {
        this->color = color;
        this->Kdiffuse = Kdiffuse;
        this->Kspecular = Kspecular;
        this->shininess = shininess;
        this->transparency = transparency;
        this->ior = ior;
    }
};

struct RayIntersection {
	float distance;
	float3 point;
	float3 normal;
	Material shapeMaterial;
	bool isEntering;

    __device__
    RayIntersection() {
        distance = 0.0f;
        point = make_float3(0.0f);
        normal = make_float3(0.0f);
        isEntering = true;
    }
	
    __device__
    RayIntersection(float distance, float3 point, float3 normal) {
        this->distance = distance;
        this->point = point;
        this->normal = normal;
        isEntering = true;
    }

    __device__
    RayIntersection(float distance, float3 point, float3 normal, bool isEntering, Material shapeMaterial) {
        this->distance = distance;
        this->point = point;
        this->normal = normal;
        this->shapeMaterial = shapeMaterial;
        this->isEntering = isEntering;
    }
};



struct Light {
    float3 color;
    float3 position;

    __host__
    Light() {
        color = position = make_float3(0.0f);
        color = make_float3(1.0f);
    }

    __host__
    Light(float3 position) {
        this->position = position;
        color = make_float3(1.0f);
    }

    __host__
    Light(float3 position, float3 color) {
        this->position = position;
        this->color = color;
    }

    __host__
    std::string print() {
        std::ostringstream os;
        os << "Color: " << color.x << " " << color.y << " " << color.z << std::endl <<
            "Position: " << position.x << " " << position.y << " " << position.z;
        
        return os.str();
    }
};


struct Sphere {
	float3 center;
    float radius;
    Material material;

    __host__
	Sphere() {
        center = make_float3(0.0f);
        radius = 1.0f;
    }

    __host__
	Sphere(float x, float y, float z, float radius) {
        center = make_float3(x, y, z);
        this->radius = radius;
    }
    
    __host__
    std::string print() {
        std::ostringstream os;

        os << "Sphere: " << center.x << " " << center.y << " " << center.z << " " << radius << std::endl;

        return os.str();
    }
};

struct Cylinder {
	float3 base, top;
    float radius;
    Material material;
    uint mortonCode;
    
    __host__
	Cylinder() {}

    __host__
	Cylinder(float3 base, float3 top, float radius) {
        this->base = base;
        this->top = top;
        this->radius = radius;
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Cylinder: Base" << base.x << " " << base.y << " " << base.z << std::endl <<
            "Top" << top.x << " " << top.y << " " << top.z << std::endl <<
            " Radius " << radius << std::endl;

        return os.str();
    }

};

struct Plane {
	float3 normal;
    //distance to origin (0,0,0)
	float distance;
    Material material;

    __host__
	Plane() {}    

    __host__
	Plane(float3 normal, float distance) {
        this->normal = normal;
        this->distance = distance;
    }

    __host__
	Plane(float3 v0, float3 v1, float3 v2) {
        float3 e1 = v1 - v0;
        float3 e2 = v2 - v1;
	    normal = normalize(cross(e1, e2));
	    float nDOTapoint = dot(normal, v0);
	    distance = -nDOTapoint;
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Plane: " << normal.x << " " << normal.y << " " << normal.z << " " << distance << std::endl;

        return os.str();
    }
};

struct Triangle {
	float3 vertices[3];
	float3 e1, e2, normal;
    Material material;

    __host__
	Triangle() {}

    __host__
	Triangle(std::vector<float3> vertices) {
        this->vertices[0] = vertices[0];
        this->vertices[1] = vertices[1];
        this->vertices[2] = vertices[2];

	    e1 = vertices[1] - vertices[0];
	    e2 = vertices[2] - vertices[0];
	    normal = normalize(cross(e1, e2));
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Triangle: " << std::endl <<
            "v1" << vertices[0].x << " " << vertices[0].y << " " << vertices[0].z << std::endl <<
            "v2" << vertices[1].x << " " << vertices[1].y << " " << vertices[1].z << std::endl <<
            "v3" << vertices[2].x << " " << vertices[2].y << " " << vertices[2].z << std::endl;

        return os.str();
    }
};


struct CylinderNode {
    float3 max;
    float3 min;
    short type;
    int lock;
    Cylinder *shape;
    Matrix *matrix;
    float3 *translation;
    
    CylinderNode *lchild, *rchild, *parent;

    __host__
    CylinderNode() {
        type = AABB;
        shape = nullptr;
        matrix = nullptr;
        translation = nullptr;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;
        lock = 0;

        min = make_float3(FLT_MAX);
        max = make_float3(-FLT_MAX);
    }

    __host__
    CylinderNode(short type, Cylinder *shape) {
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;
        lock = 0;
        this->type = type;
        this->shape = shape;

        if(type == AABB) {
            min = fminf(shape->base, shape->top) - shape->radius;
            max = fmaxf(shape->base, shape->top) + shape->radius;
            matrix = nullptr;
            translation = nullptr;

        } else {
            translation = new float3(-shape->base);
            float3 B = make_float3(0.0f, 0.0f, 1.0f);
            float3 A = shape->top - shape->base;

            float height = length(A);

            A = normalize(A);
            
            float dotAB = dot(A, B);
            float3 AxB = cross(A, B);

            /*Matrix R = Matrix(dotAB, -length(AxB), 0.0f,
                              length(AxB), dotAB,  0.0f,
                              0.0f, 0.0f, 1.0f);

            Matrix Fi = Matrix(A, (B - dotAB * A) / length(B - dotAB * A), -AxB);
            
            matrix = new Matrix((Fi * R * Fi.inverse()).M);*/

           
            matrix = new Matrix(AxB, cross(AxB, A), A);  

            //debug
            B = *matrix * (shape->top + *translation);

            max = make_float3(shape->radius, shape->radius, height);
            min = make_float3(-shape->radius, -shape->radius, 0);

        }
    }

    __host__
    CylinderNode(Cylinder *shape) {
        type = AABB;
        this->shape = shape;
        matrix = nullptr;
        translation = nullptr;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;
        lock = 0;

        min = fminf(shape->base, shape->top) - shape->radius;
        max = fmaxf(shape->base, shape->top) + shape->radius;
    }

    __host__ __device__
    float volume() {
        float3 len = max - min;

        return abs(len.x * len.y * len.z);
    }
};

struct SphereNode{
    float3 max;
    float3 min;
    Sphere *shape;

    __host__
    SphereNode() {
        shape = nullptr;
    }

    __host__
    SphereNode(Sphere *shape) {
        this->shape = shape;
        max = shape->center + shape->radius;
        min = shape->center - shape->radius;
    }
};

struct TriangleNode {
    float3 max;
    float3 min;
    Triangle *shape;

    __host__
    TriangleNode() {
        shape = nullptr;
    }

    __host__
    TriangleNode(Triangle *shape) {
        this->shape = shape;
        max = fmaxf(fmaxf(shape->vertices[0], shape->vertices[1]), shape->vertices[2]);
        min = fminf(fminf(shape->vertices[0], shape->vertices[1]), shape->vertices[2]);
    }
};

#endif