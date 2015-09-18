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


#define FLOAT_EPSILON 4E-3f
#define ACNE_EPSILON 1E-2f
#define SUPER_SAMPLING 2
#define MAX_DEPTH 1

//====================================  device ========================
struct Sphere;

struct Ray{
private:
    float3 _origin;
    float3 _direction;

public:
    __device__ Ray(){
        _origin = make_float3(0.0f);
        _direction = make_float3(0.0f, 0.0f, 1.0f);
    };

    __device__ Ray(float3 origin, float3 direction) {
        _origin = origin;
        _direction = direction;
    }

    __device__ 
    float3 origin() {
        return _origin;
    }

    __device__ 
    float3 direction() {
        return _direction;
    }

    __device__
    void update(float3 origin, float3 direction) {
        _origin = origin;
        _direction = direction;
    }
};

struct RayIntersection{
	float _distance;
	float3 _point;
	float3 _normal;
	Sphere *_shape;
	bool _isEntering;

    __device__
    RayIntersection() {
        _distance = 0.0f;
        _point = make_float3(0.0f);
        _normal = make_float3(0.0f);
        _shape = nullptr;
        _isEntering = true;
    }
	
    __device__
    RayIntersection(float distance, float3 point, float3 normal) {
        _distance = distance;
        _point = point;
        _normal = normal;
        _shape = nullptr;
        _isEntering = true;
    }

    __device__
    RayIntersection(float distance, float3 point, float3 normal, bool isEntering, Sphere *shape) {
        _distance = distance;
        _point = point;
        _normal = normal;
        _shape = shape;
        _isEntering = isEntering;
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
        return Color(make_float3(this->r() + color.r(), this->g() + color.g(), this->b() + color.b()));
    }

    __host__ __device__ 
    Color operator*(Color color) {
        return Color(make_float3(this->r() * color.r(), this->g() * color.g(), this->b() * color.b()));
    }

    __host__ __device__ 
    Color operator*(float factor) {
        return Color(make_float3(this->r() * factor, this->g() * factor, this->b() * factor));
    }

};

struct Material {
private:
    Color _color;
    float _diffuse;
    float _specular;
    float _shininess;
    float _transparency;
    float _ior;

public:
    __host__ __device__ 
    Material() {
        _color = Color();
        _diffuse = _specular = _shininess = _transparency = _ior = 0.0f;
    }

    __host__ __device__ 
    Material(float3 color, float diffuse, float specular, float shininess, float transparency, float ior) {
        _color = Color(color);
        _diffuse = diffuse;
        _specular = specular;
        _shininess = shininess;
        _transparency = transparency;
        _ior = ior;
    }
    
    __host__ __device__ 
    Color color() {
        return _color;
    }

    __host__ __device__ 
    float diffuse() {
        return _diffuse;
    }

    __host__ __device__ 
    float specular() {
        return _specular;
    }

    __host__ __device__ 
    float shininess() {
        return _shininess;
    }

    __host__ __device__ 
    float transparency() {
        return _transparency;
    }

    __host__ __device__ 
    float refraction() {
        return _ior;
    }
};

struct Light {
private:
    Color _color;
    float3 _position;

public:
    __host__
    Light() {
        _position = make_float3(0.0f);
        _color = Color(make_float3(1.0f));
    }

    __host__
    Light(float3 position) {
        _position = position;
        _color = Color(make_float3(1.0f));
    }

    __host__
    Light(float3 position, float3 color) {
        _position = position;
        _color = Color(color);
    }

    __host__ __device__
    float3 position() {
        return _position;
    }

    __host__ __device__
    Color color() {
        return _color;
    }

    __host__
    std::string print() {
        std::ostringstream os;
        os << "Color: " << _color.r() << " " << _color.g() << " " << _color.b() << std::endl <<
            "Position: " << _position.x << " " << _position.y << " " << _position.z;
        
        return os.str();
    }
};


struct Sphere {
private:
	float _x, _y, _z, _r;
    Material _material;

public:
    __host__
	Sphere() {
        _x = _y = _z = 0.0f;
        _r = 1.0f;
    }

    __host__
	Sphere(float x, float y, float z, float r) {
        _x = x;
        _y = y;
        _z = z;
        _r = r;
    }

    __host__ __device__
	Material material() {
        return _material;
    }

    __host__ __device__
	void setMaterial(Material material) {
        _material = material;
    }

    
    __host__
    std::string print() {
        std::ostringstream os;

        os << "Sphere: " << _x << " " << _y << " " << _z << " " << _r << std::endl;

        return os.str();
    }

    __device__
	bool intersection(Ray ray, RayIntersection *out);

};

struct Cylinder {
private:
	float4 _base, _apex;
    Material _material;

public:
    __host__
	Cylinder(float4 base, float4 apex) {
        _base = base;
        _apex = apex;
    }

    __host__ __device__
	Material material() {
        return _material;
    }

    __host__ __device__
	void setMaterial(Material material) {
        _material = material;
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Cylinder: Base" << _base.x << " " << _base.y << " " << _base.z << " " << _base.w << std::endl <<
            "Top" << _apex.x << " " << _apex.y << " " << _apex.z << " " << _apex.w << std::endl;

        return os.str();
    }

    __device__
	bool intersection(Ray ray, RayIntersection *out);

};


struct Scene {
private:
    Color _backcolor;
    Material _material;
    std::vector<Sphere> _h_shapes;
	std::vector<Light> _h_lights;

    Sphere *_d_shapes;
    Light *_d_lights;

    size_t _d_shapesSize;
    size_t _d_lightsSize;

public:
    __host__
    Scene() {}


    __host__
	~Scene() {
        cudaFree(_d_shapes);
        cudaFree(_d_lights);
    }

    __host__
    Sphere* getDShapes() {
        return _d_shapes;
    }

    __host__
    size_t getDShapesSize() {
        return _d_shapesSize;
    }

    __host__
    size_t getDLightsSize() {
        return _d_lightsSize;
    }

    __host__
    Light* getDLights() {
        return _d_lights;
    }

    __host__
    void addLight(float3 pos) {
	    _h_lights.push_back(Light(pos));
    }

    __host__
    bool copyToDevice() {
        size_t size;
        Light *ltVector = new Light[_h_lights.size()];
        Sphere *spVector = new Sphere[_h_shapes.size()];

        for(long i = 0; i < _h_shapes.size(); i++) {
            spVector[i] = _h_shapes[i];
        }

        _d_shapesSize = _h_shapes.size();
        
        for(long i = 0; i < _h_lights.size(); i++) {
            ltVector[i] = _h_lights[i];
        }

        _d_lightsSize = _h_lights.size();
        

        size = _h_lights.size() * sizeof(Light);
        //std::cout << "Lights size " << size << std::endl;

        checkCudaErrors(cudaMalloc((void**) &_d_lights, size));
        checkCudaErrors(cudaMemcpy(_d_lights, ltVector, size, cudaMemcpyHostToDevice));
        

        size = _h_shapes.size() * sizeof(Sphere);
        //std::cout << "Shapes size " << size << std::endl;

        checkCudaErrors(cudaMalloc((void**) &_d_shapes, size));
        checkCudaErrors(cudaMemcpy(_d_shapes, spVector, size, cudaMemcpyHostToDevice));

       
        delete[] ltVector;
        delete[] spVector;

        _h_shapes.clear();
        _h_lights.clear();

        return true;
    }

    __host__
    void addLight(float3 pos, float3 color) {
	    _h_lights.push_back(Light(pos, color));
    }

    __host__ __device__
    Color backcolor() {
        return _backcolor;
    }

    __host__
    void setBackcolor(float3 color) {
        _backcolor = Color(color);
    }
    __host__
    void setMaterial(float3 color, float diffuse, float specular, float shine, float trans, float ior) {
        _material = Material(color, diffuse, specular, shine, trans, ior);
    }


    __host__
    void addCylinder(float4 base, float4 apex) {
	    //TODO
    }

    __host__
    void addSphere(float3 center, float radius) {
	    Sphere sp = Sphere(center.x, center.y, center.z, radius);
	    sp.setMaterial(_material);
	    _h_shapes.push_back(sp);
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