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


#define FLOAT_EPSILON 1E-3f
#define ACNE_EPSILON 1E-2f


//====================================  device ========================
struct Shape;

struct Ray{
private:
    float3 _origin;
    float3 _direction;
    int _depth;

public:
    __device__ Ray(){
        _origin = make_float3(0.0f);
        _direction = make_float3(0.0f, 0.0f, 1.0f);
    };

    __device__ Ray(float3 origin, float3 direction) {
        _origin = origin;
        _direction = direction;
        _depth = 0;
    }

    __device__ float3 origin() {
        return _origin;
    }

    __device__ float3 direction() {
        return _direction;
    }
};

struct RayIntersection{
	float _distance;
	float3 _point;
	float3 _normal;
	Shape *_shape;
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
    RayIntersection(float distance, float3 point, float3 normal, bool isEntering, Shape *shape) {
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

struct Shape{
private:
    Material _material;

public:
    __host__ __device__
	Material &material() {
        return _material;
    }

    __host__ __device__
	void setMaterial(Material material) {
        _material = material;
    }

    __host__
    virtual std::string print() {
        std::ostringstream os;

        os << "Shape:" << std::endl;

        return os.str();
    }

    __device__
	virtual bool intersection(Ray ray, RayIntersection *out) = 0;
};

struct Sphere : public Shape {
private:
	float _x, _y, _z, _r;

public:
    __host__
	Sphere(float x, float y, float z, float r) {
        _x = x;
        _y = y;
        _z = z;
        _r = r;
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

struct Cylinder : public Shape {
private:
	float4 _base, _apex;

public:
    __host__
	Cylinder(float4 base, float4 apex) {
        _base = base;
        _apex = apex;
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

struct Plane : public Shape {
	float3 _normal;
	float _d;

public:
    __host__
	Plane(float3 normal, float d) {
        _normal = normal;
        _d = d;
    }

    __host__ __device__
	Plane(float3 v1, float3 v2, float3 v3) {
        float3 v12 = v2 - v1;
        float3 v23 = v3 - v2;
	    _normal = normalize(cross(v12, v23));
	    float nDOTapoint = dot(_normal, v1);
	    _d = -nDOTapoint;
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Plane: " << _normal.x << " " << _normal.y << " " << _normal.z << " " << _d << std::endl;

        return os.str();
    }

    __device__
	bool intersection(Ray ray, RayIntersection *out);

};

struct Triangle : public Shape{
private:
	float3 _vertices[3];
	float3 _v1, _v2, _normal;
	float _d;

    __device__
	int maxNormal() {
        if (_normal.x > _normal.y) {
		    if (_normal.x > _normal.z) {
			    return 0;
		    }
		    return 2;
	    }
	    else if (_normal.y > _normal.z) {
		    return 1;
	    }
	    return 2;
    }

    __device__
	void calculateIndexes(int *i0, int *i1, int *i2) {
        int mn = maxNormal();
	    *i0 = mn;

	    if (mn == 0) {
		    *i1 = 1;
		    *i2 = 2;
	    }
	    else if (mn == 1) {
		    *i1 = 0;
		    *i2 = 2;
	    }
	    else {
		    *i1 = 0;
		    *i2 = 1;
	    }
    }

public:
    __host__
	Triangle(std::vector<float3> vertices) {
        _vertices[0] = vertices[0];
        _vertices[1] = vertices[1];
        _vertices[2] = vertices[2];

	    _v1 = vertices[1] - vertices[0];
	    _v2 = vertices[2] - vertices[0];
	    _normal = normalize(cross(_v1, _v2));

	    float normalDOTopoint = dot(_normal, vertices[0]);
	    _d = -normalDOTopoint;
    }

    __host__
    std::string print() {
        std::ostringstream os;

        os << "Triangle: " << std::endl <<
            "v1" << _vertices[0].x << " " << _vertices[0].y << " " << _vertices[0].z << std::endl <<
            "v2" << _vertices[1].x << " " << _vertices[1].y << " " << _vertices[1].z << std::endl <<
            "v3" << _vertices[2].x << " " << _vertices[2].y << " " << _vertices[2].z << std::endl;

        return os.str();
    }

    __device__
	bool intersection(Ray ray, RayIntersection *out);

};


struct Scene {
private:
    Color _backcolor;
    Material _material;
    std::vector<Shape*> _h_shapes;
	std::vector<Light> _h_lights;

    Shape **_d_shapes;
    Light *_d_lights;

    size_t _d_shapesSize;
    size_t _d_lightsSize;

public:
    __host__
    Scene() {}


    __host__
	~Scene() {
        for (int i =0; i < _h_shapes.size(); i++) {
            delete (_h_shapes[i]);
        } 
        _h_shapes.clear();

        cudaFree(_d_shapes);
        cudaFree(_d_lights);
    }

    __host__
    Shape** getDShapes() {
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
        Shape **spVector = new Shape*[_h_shapes.size()];

        //Light *res = new Light[_h_lights.size()];
        //Shape **res = new Shape*[_h_shapes.size()];

        
        for(long i = 0; i < _h_shapes.size(); i++) {
            spVector[i] = _h_shapes[i];

            //std::cout << i << std::endl << spVector[i]->print() << std::endl;
        }

        _d_shapesSize = _h_shapes.size();

        //std::cout << "prov" << std::endl;
        for(long i = 0; i < _h_lights.size(); i++) {
            ltVector[i] = _h_lights[i];

            /*std::cout << i << std::endl;
            std::cout << _h_lights[i].print() << std::endl;*/
        }

        _d_lightsSize = _h_lights.size();

        size = _h_lights.size() * sizeof(Light);
        checkCudaErrors(cudaMalloc((void**) &_d_lights, size));
        checkCudaErrors(cudaMemcpy(_d_lights, ltVector, size, cudaMemcpyHostToDevice));
        
        
        //cudaMemcpy(res, _d_lights, size, cudaMemcpyDeviceToHost);

        /*for(int i = 0; i < _h_lights.size(); i++) {
            std::cout << i << std::endl;
            std::cout << res[i].print() << std::endl;
        }*/
        delete[] ltVector;

        size = _h_shapes.size() * sizeof(Shape *);
        checkCudaErrors(cudaMalloc((void**) &_d_shapes, size));
        checkCudaErrors(cudaMemcpy(_d_shapes, spVector, size, cudaMemcpyHostToDevice));
        
        //cudaMemcpy(res, _d_shapes, size, cudaMemcpyDeviceToHost);

        /*for(int i = 0; i < _h_shapes.size(); i++) {
            std::cout << i << std::endl << res[i]->print() << std::endl;
        }*/
        delete[] spVector;

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
	    Cylinder *sp = new Cylinder(base, apex);
        sp->setMaterial(_material);
        _h_shapes.push_back(sp);
    }

    __host__
    void addSphere(float3 center, float radius) {
	    Sphere *sp = new Sphere(center.x, center.y, center.z, radius);
	    sp->setMaterial(_material);
	    _h_shapes.push_back(sp);
    }

    __host__
    void addTriangle(std::vector<float3> verts) {
	    Triangle *sp = new Triangle(verts);
	    sp->setMaterial(_material);
	    _h_shapes.push_back(sp);
    }

    __host__
    void addPolyPatch(int numVerts, std::vector<float3> verts, std::vector<float3> normals) {
	    //TODO
    }

    __host__
    void addPlane(float3 p1, float3 p2, float3 p3) {
	    Plane *pl = new Plane(p1, p2, p3);
	    pl->setMaterial(_material);
	    _h_shapes.push_back(pl);
    }
};





#endif