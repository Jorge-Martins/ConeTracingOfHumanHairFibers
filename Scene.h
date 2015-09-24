#pragma once

#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector_types.h>
#include <vector_functions.h>

#include <vector>
#include <helper_math.h>

#include <helper_cuda.h>

#include <iostream>
#include <string>
#include <sstream>

#define SUPER_SAMPLING 1
#define MAX_DEPTH 3

//====================================  device ========================
struct Sphere;

struct Ray{
    float3 origin;
    float3 direction;
    bool exists;

    __host__ __device__ 
    Ray(){
        this->origin = make_float3(0.0f);
        this->direction = make_float3(0.0f, 0.0f, 1.0f);
        this->exists = true;
    };

    __device__ Ray(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        this->exists = true;
    }

    __device__
    void update(float3 origin, float3 direction) {
        this->origin = origin;
        this->direction = direction;
        this->exists = true;
    }
};

struct RayIntersection{
	float distance;
	float3 point;
	float3 normal;
	Sphere *shape;
	bool isEntering;

    __device__
    RayIntersection() {
        this->distance = 0.0f;
        this->point = make_float3(0.0f);
        this->normal = make_float3(0.0f);
        this->shape = nullptr;
        this->isEntering = true;
    }
	
    __device__
    RayIntersection(float distance, float3 point, float3 normal) {
        this->distance = distance;
        this->point = point;
        this->normal = normal;
        this->shape = nullptr;
        this->isEntering = true;
    }

    __device__
    RayIntersection(float distance, float3 point, float3 normal, bool isEntering, Sphere *shape) {
        this->distance = distance;
        this->point = point;
        this->normal = normal;
        this->shape = shape;
        this->isEntering = isEntering;
    }
};

struct Material {
    float3 color;
    float diffuse;
    float specular;
    float shininess;
    float transparency;
    float ior;

    __host__ 
    Material() {
        color = make_float3(0.0f);
        diffuse = specular = shininess = transparency = ior = 0.0f;
    }

    __host__ 
    Material(float3 color, float diffuse, float specular, float shininess, float transparency, float ior) {
        this->color = color;
        this->diffuse = diffuse;
        this->specular = specular;
        this->shininess = shininess;
        this->transparency = transparency;
        this->ior = ior;
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
    float r;
    Material material;

    __host__
	Sphere() {
        center = make_float3(0.0f);
        r = 1.0f;
    }

    __host__
	Sphere(float x, float y, float z, float r) {
        center = make_float3(x, y, z);
        this->r = r;
    }
    
    /*__host__
    std::string print() {
        std::ostringstream os;

        os << "Sphere: " << x << " " << y << " " << z << " " << r << std::endl;

        return os.str();
    }*/
};

struct Cylinder {
	float4 base, apex;
    Material material;

    __host__
	Cylinder(float4 base, float4 apex) {
        this->base = base;
        this->apex = apex;
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Cylinder: Base" << base.x << " " << base.y << " " << base.z << " " << base.w << std::endl <<
            "Top" << apex.x << " " << apex.y << " " << apex.z << " " << apex.w << std::endl;

        return os.str();
    }

};


struct Scene {
private:
    float3 backcolor;
    Material material;
    std::vector<Sphere> h_shapes;
	std::vector<Light> h_lights;

    Sphere *d_shapes;
    Light *d_lights;

    size_t d_shapesSize;
    size_t d_lightsSize;

public:
    __host__
    Scene() {}


    __host__
	~Scene() {
        checkCudaErrors(cudaFree(d_shapes));
        checkCudaErrors(cudaFree(d_lights));
    }

    __host__
    Sphere* getDShapes() {
        return d_shapes;
    }

    __host__
    size_t getDShapesSize() {
        return d_shapesSize;
    }

    __host__
    size_t getDLightsSize() {
        return d_lightsSize;
    }

    __host__
    Light* getDLights() {
        return d_lights;
    }

    __host__
    void addLight(float3 pos) {
	    h_lights.push_back(Light(pos));
    }

    __host__
    bool copyToDevice() {
        size_t size;
        Light *ltVector = new Light[h_lights.size()];
        Sphere *spVector = new Sphere[h_shapes.size()];

        for(long i = 0; i < h_shapes.size(); i++) {
            spVector[i] = h_shapes[i];
        }

        d_shapesSize = h_shapes.size();
        
        for(long i = 0; i < h_lights.size(); i++) {
            ltVector[i] = h_lights[i];
        }

        d_lightsSize = h_lights.size();
        

        size = h_lights.size() * sizeof(Light);
        //std::cout << "Lights size " << size << std::endl;

        checkCudaErrors(cudaMalloc((void**) &d_lights, size));
        checkCudaErrors(cudaMemcpy(d_lights, ltVector, size, cudaMemcpyHostToDevice));
        

        size = h_shapes.size() * sizeof(Sphere);
        //std::cout << "Shapes size " << size << std::endl;

        checkCudaErrors(cudaMalloc((void**) &d_shapes, size));
        checkCudaErrors(cudaMemcpy(d_shapes, spVector, size, cudaMemcpyHostToDevice));

       
        delete[] ltVector;
        delete[] spVector;

        h_shapes.clear();
        h_lights.clear();

        return true;
    }

    __host__
    void addLight(float3 pos, float3 color) {
	    h_lights.push_back(Light(pos, color));
    }

    __host__
    float3 getBackcolor() {
        return backcolor;
    }

    __host__
    void setBackcolor(float3 color) {
        backcolor = color;
    }
    __host__
    void setMaterial(float3 color, float diffuse, float specular, float shine, float trans, float ior) {
        material = Material(color, diffuse, specular, shine, trans, ior);
    }


    __host__
    void addCylinder(float4 base, float4 apex) {
	    //TODO
    }

    __host__
    void addSphere(float3 center, float radius) {
	    Sphere sp = Sphere(center.x, center.y, center.z, radius);
	    sp.material = material;
	    h_shapes.push_back(sp);
    }

    __host__
    void addTriangle(std::vector<float3> verts) {
	    //TODO
    }

    __host__
    void addPolyPatch(int numVerts, std::vector<float3> verts, std::vector<float3> normals) {
	    //TODO
    }

    __host__
    void addPlane(float3 p1, float3 p2, float3 p3) {
	    //TODO
    }
};





#endif