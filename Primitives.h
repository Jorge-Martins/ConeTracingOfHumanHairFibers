#pragma once

#ifndef _PRIMITIVES_
#define _PRIMITIVES_

#include <vector>

#include <iostream>
#include <string>
#include <sstream>

#include "MathUtil.h"
#include <glm.hpp>
#include <gtc/quaternion.hpp>
#include <gtc/matrix_transform.hpp>

#define AABB 0
#define OBB  1

#define PRIMARY 0
#define REFLECTED 1
#define REFRACTED 2

/* N - Negative
 * O - zero
 * P - Positive
 */
enum RayClassification {
    NNN, NNO, NNP, NON, NOO, NOP, NPN, NPO, NPP,
    ONN, ONO, ONP, OON, OOO, OOP, OPN, OPO, OPP,
    PNN, PNO, PNP, PON, POO, POP, PPN, PPO, PPP
        
};

struct RayInfo {
    float3 origin;
    float3 direction;
    unsigned char type;
    unsigned char depth;
    float importance;

    __host__ __device__ 
    RayInfo() {
        origin = make_float3(0.0f);
        direction = make_float3(0.0f, 0.0f, 1.0f);
        type = PRIMARY;
        depth = 0;
        importance = 1.0f;
    }

    __device__
    void update(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        type = PRIMARY;
        depth = 0;
        importance = 1.0f;
    }

    __device__
    void update(float3 origin, float3 direction, unsigned char type, unsigned char depth, float importance) {
        this->origin = origin;
        this->direction = direction;
        this->type = type;
        this->depth = depth;
        this->importance = importance; 
    }
};

struct Ray {
    float3 origin;
    float3 direction;
    float3 invDirection;

    /*williams implementation
    int sign[3];
    

    __host__ __device__
    void computeSign() {
        invDirection = 1.0f / direction;

        sign[0] = (invDirection.x < 0);
        sign[1] = (invDirection.y < 0);
        sign[2] = (invDirection.z < 0);
    }*/

    //slope
    short classification;
    float x_y, y_x, y_z, z_y, x_z, z_x; 
	float c_xy, c_xz, c_yx, c_yz, c_zx, c_zy;

    __host__ __device__
    void computeSlopes() {
        invDirection = 1.0f / direction;
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
        
        //computeSign();
        computeSlopes();
    }

    __device__ 
    Ray(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        
        //computeSign();
        computeSlopes();
    }

    __device__
    void update(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        
        //computeSign();
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
    unsigned char type;
    Cylinder *shape;
    Matrix *matrix;
    float3 *translation;
    
    CylinderNode *lchild, *rchild, *parent;

    __host__
    void computeOBB() {
        float3 v = make_float3(0.0f, 0.0f, 1.0f);
        float3 u = shape->top - shape->base;

        float normU = length(shape->top - shape->base);
        float real_part = normU + dot(u, v);
        float3 w;

        /* If u and v are exactly opposite, rotate 180 degrees
         * around an arbitrary orthogonal axis. Axis normalisation
         * can happen later, when we normalise the quaternion. */
        if (real_part < 1.e-6f * normU) {
            real_part = 0.0f;
            w = fabsf(u.x) > fabsf(u.z) ? make_float3(-u.y, u.x, 0.f) : make_float3(0.f, -u.z, u.y);

        } else {
            w = cross(u, v);
        }

        glm::mat4 RotationMatrix = glm::mat4_cast(normalize(glm::quat(real_part, w.x, w.y, w.z)));
        glm::mat4 TranslationMatrix = glm::translate(glm::mat4(), glm::vec3(-shape->base.x, -shape->base.y, -shape->base.z));
		glm::mat4 ModelMatrix = TranslationMatrix * RotationMatrix;

        glm::vec4 c0(ModelMatrix[0]);
        glm::vec4 c1(ModelMatrix[1]);
        glm::vec4 c2(ModelMatrix[2]);
        glm::vec4 c3(ModelMatrix[3]);

        translation = new float3(make_float3(c3.x, c3.y, c3.z));

        matrix = new Matrix(c0.x, c1.x, c2.x,
                            c0.y, c1.y, c2.y,
                            c0.z, c1.z, c2.z);
    }

    __host__ __device__
    CylinderNode() {
        type = AABB;
        shape = nullptr;
        matrix = nullptr;
        translation = nullptr;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;

        min = make_float3(FLT_MAX);
        max = make_float3(-FLT_MAX);
    }

    __host__
    CylinderNode(unsigned char type, Cylinder *shape, uint &nOBBs) {
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;
        this->type = type;
        this->shape = shape;

        min = fminf(shape->base, shape->top) - shape->radius;
        max = fmaxf(shape->base, shape->top) + shape->radius;

        if(type == AABB) {
            matrix = nullptr;
            translation = nullptr;

        } else {
            nOBBs++;
            computeOBB();
        }
    }

    __host__
    CylinderNode(Cylinder *shape, uint &nOBBs) {
        type = AABB;
        this->shape = shape;
        matrix = nullptr;
        translation = nullptr;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;

        min = fminf(shape->base, shape->top) - shape->radius;
        max = fmaxf(shape->base, shape->top) + shape->radius;

        float vAABB = volumeAABB();
        float vOBB = volumeOBB();

        if(vOBB * OBB_AABB_EPSILON < vAABB) {
            type = OBB;
            nOBBs++;
            computeOBB();
        }
    }

    __host__
    float volumeAABB() {
        float3 len = max - min;

        return fabsf(len.x * len.y * len.z);
    }

    __host__
    float volumeOBB() {
        if(shape != nullptr) {
            float height = length(shape->top - shape->base);
            float3 len = make_float3(shape->radius, shape->radius, height) - make_float3(-shape->radius, -shape->radius, 0);

            return fabsf(len.x * len.y * len.z);
        }

        return FLT_MAX;
    }
};

struct SphereNode{
    float3 max;
    float3 min;
    Sphere *shape;

    SphereNode *lchild, *rchild, *parent;

    __host__ __device__
    SphereNode() {
        shape = nullptr;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;

        min = make_float3(FLT_MAX);
        max = make_float3(-FLT_MAX);
    }

    __host__
    SphereNode(Sphere *shape) {
        this->shape = shape;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;

        max = shape->center + shape->radius;
        min = shape->center - shape->radius;
    }
};

struct TriangleNode {
    float3 max;
    float3 min;
    Triangle *shape;

    TriangleNode *lchild, *rchild, *parent;

    __host__ __device__
    TriangleNode() {
        shape = nullptr;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;

        min = make_float3(FLT_MAX);
        max = make_float3(-FLT_MAX);
    }

    __host__
    TriangleNode(Triangle *shape) {
        this->shape = shape;
        lchild = nullptr;
        rchild = nullptr;
        parent = nullptr;

        max = fmaxf(fmaxf(shape->vertices[0], shape->vertices[1]), shape->vertices[2]);
        min = fminf(fminf(shape->vertices[0], shape->vertices[1]), shape->vertices[2]);
    }
};

template <typename BVHNodeType>
struct PartitionEntry {
    unsigned char partition;
    bool left;
    BVHNodeType *parent;

    __device__
    PartitionEntry() {}

    __device__
    PartitionEntry(unsigned char partition, bool left, BVHNodeType *parent) {
        this->partition = partition;
        this->left = left;
        this->parent = parent;
    }
};

struct IntersectionLstItem {
    float3 color;
    float transparency;
    float distance;

    __device__
    IntersectionLstItem() {}

    __device__ 
    void update(RayIntersection info) {
        color = info.shapeMaterial.color;
        transparency = info.shapeMaterial.transparency;
        distance = info.distance;
    }
};

struct AOITData {
    float depth[AOIT_NODE_COUNT + 1];
    float trans[AOIT_NODE_COUNT + 1];
};

struct AOITFragment {
    int   index;
    float depthA;
    float transA;
};

#endif