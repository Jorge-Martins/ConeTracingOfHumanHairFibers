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
#define MAX_DEPTH 2

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


struct Color {
private:
    float3 _color;

public:
    __host__ __device__ 
    Color() {
        _color = make_float3(0.0f);
    }

    __host__ __device__
    Color(float3 color) {
        _color = color;
    }

    __host__ __device__ 
    float r() {
        return _color.x;
    }

    __host__ __device__
    float g() {
        return _color.y;
    }

    __host__ __device__
    float b() {
        return _color.z;
    }
    
    __host__ __device__
    float3 color() {
        return _color;
    }

    __host__ __device__
    void setColor(Color color) {
        _color = make_float3(color.r(), color.g(), color.b()); 
    }

    __host__ __device__
    void setColor(float3 color) {
        _color = color;
    }

    __host__ __device__ 
    Color operator+(Color color) {
        return Color(_color + color.color());
    }

    __host__ __device__ 
    Color operator*(Color color) {
        return Color(_color * color.color());
    }

    __host__ __device__ 
    Color operator+=(Color color) {
        _color += color.color();

        return _color;
    }

    __host__ __device__ 
    Color operator*=(Color color) {
        _color *= color.color();

        return _color;
    }

    __host__ __device__ 
    Color operator*(float factor) {
        return Color(_color *factor);
    }

};

struct Material {
    Color color;
    float diffuse;
    float specular;
    float shininess;
    float transparency;
    float ior;

    __host__ __device__ 
    Material() {
        this->color = Color();
        this->diffuse = this->specular = this->shininess = this->transparency = this->ior = 0.0f;
    }

    __host__ __device__ 
    Material(float3 color, float diffuse, float specular, float shininess, float transparency, float ior) {
        this->color = Color(color);
        this->diffuse = diffuse;
        this->specular = specular;
        this->shininess = shininess;
        this->transparency = transparency;
        this->ior = ior;
    }
};

struct Light {
    Color color;
    float3 position;

    __host__
    Light() {
        position = make_float3(0.0f);
        color = Color(make_float3(1.0f));
    }

    __host__
    Light(float3 position) {
        this->position = position;
        color = Color(make_float3(1.0f));
    }

    __host__
    Light(float3 position, float3 color) {
        this->position = position;
        this->color = Color(color);
    }

    __host__
    std::string print() {
        std::ostringstream os;
        os << "Color: " << color.r() << " " << color.g() << " " << color.b() << std::endl <<
            "Position: " << position.x << " " << position.y << " " << position.z;
        
        return os.str();
    }
};


struct Sphere {
	float x, y, z, r;
    Material material;

    __host__
	Sphere() {
        x = y = z = 0.0f;
        r = 1.0f;
    }

    __host__
	Sphere(float x, float y, float z, float r) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->r = r;
    }
    
    __host__
    std::string print() {
        std::ostringstream os;

        os << "Sphere: " << x << " " << y << " " << z << " " << r << std::endl;

        return os.str();
    }

    __device__
	bool intersection(Ray ray, RayIntersection *out);

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

    __device__
	bool intersection(Ray ray, RayIntersection *out);

};


struct Scene {
private:
    Color backcolor;
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
        cudaFree(d_shapes);
        cudaFree(d_lights);
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

    __host__ __device__
    Color getBackcolor() {
        return backcolor;
    }

    __host__
    void setBackcolor(float3 color) {
        backcolor = Color(color);
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