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
#define SUPER_SAMPLING_2 (SUPER_SAMPLING * SUPER_SAMPLING);
#define MAX_DEPTH 2

#define KB 1000
#define MB (1000 * KB)
#define GB (1000 * MB)

#define sphereIndex 0
#define cylinderIndex 1
#define triangleIndex 2
#define planeIndex 3
#define nShapes 4

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
    }

    __device__ 
    Ray(float3 origin, float3 direction) {
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

struct RayIntersection{
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
    
    __host__
    std::string print() {
        std::ostringstream os;

        os << "Sphere: " << center.x << " " << center.y << " " << center.z << " " << r << std::endl;

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


struct Scene {
private:
    float3 backcolor;
    Material material;
    std::vector<Sphere> h_spheres;
    std::vector<Cylinder> h_cylinders;
    std::vector<Triangle> h_triangles;
    std::vector<Plane> h_planes;
	std::vector<Light> h_lights;
    
    size_t *h_shapeSizes;
    
    Sphere *d_spheres;
    Cylinder *d_cylinders;
    Triangle *d_triangles;
    Plane *d_planes;

    Light *d_lights;

    int **d_shapes;
    size_t *d_shapeSizes;
    size_t d_lightsSize;

public:
    __host__
    Scene() {
        d_shapeSizes = nullptr;
        d_shapes = nullptr;

        d_spheres = nullptr;
        d_cylinders = nullptr;
        d_triangles = nullptr;
        d_planes = nullptr; 
        
        h_shapeSizes = new size_t[nShapes];
        
        d_lights = nullptr;

        for(int i = 0; i < nShapes; i++) {
            h_shapeSizes[i] = 0;
        }
    }


    __host__
	~Scene() {
        if(d_spheres != nullptr) {
            checkCudaErrors(cudaFree(d_spheres));
        }
        if(d_cylinders != nullptr) {
            checkCudaErrors(cudaFree(d_cylinders));
        }
        if(d_triangles != nullptr) {
            checkCudaErrors(cudaFree(d_triangles));
        }
        if(d_planes != nullptr) {
            checkCudaErrors(cudaFree(d_planes));
        }

        if(d_lights != nullptr) {
            checkCudaErrors(cudaFree(d_lights));
        }

        if(d_shapes != nullptr) {
             checkCudaErrors(cudaFree(d_shapes));
        }

        if(d_shapeSizes != nullptr) {
             checkCudaErrors(cudaFree(d_shapeSizes));
        }
    }

    __host__
    int** getDShapes() {
        return d_shapes;
    }

    __host__
    size_t* getDShapesSize() {
        return d_shapeSizes;
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
    std::string printSize(size_t size) {
        std::ostringstream os;
        float out;
        std::string s;

        if(size > MB) {
            if(size > GB) {
                out = (float) size / (float)GB;
                s = "GB";
            } else {
                out = (float) size / (float)MB;
                s = "MB";
            }
        } else {
            out = (float) size / (float)KB;
            s = "KB";
        }

        os << out << s;
        return os.str();
    }

    __host__
    bool copyToDevice() {
        size_t size, sceneSize = 0;

        Light *ltVector = new Light[h_lights.size()];

        for(size_t i = 0; i < h_lights.size(); i++) {
            ltVector[i] = h_lights[i];
        }

        d_lightsSize = h_lights.size();
        

        size = h_lights.size() * sizeof(Light);
        sceneSize += size;

        checkCudaErrors(cudaMalloc((void**) &d_lights, size));
        checkCudaErrors(cudaMemcpy(d_lights, ltVector, size, cudaMemcpyHostToDevice));

        delete[] ltVector;
        h_lights.clear();

        //Spheres
        size = h_spheres.size();
        if(size > 0) {
            Sphere *sphVector = new Sphere[size];
            h_shapeSizes[sphereIndex] = size;

            for(size_t i = 0; i < size; i++) {
                sphVector[i] = h_spheres[i];
            }

            size *= sizeof(Sphere);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_spheres, size));
            checkCudaErrors(cudaMemcpy(d_spheres, sphVector, size, cudaMemcpyHostToDevice));

            delete[] sphVector;
            h_spheres.clear();
        }

        //cylinders
        size = h_cylinders.size();
        if(size > 0) {
            Cylinder *cylVector = new Cylinder[size];
            h_shapeSizes[cylinderIndex] = size;

            for(size_t i = 0; i < size; i++) {
                cylVector[i] = h_cylinders[i];
            }

            size *= sizeof(Cylinder);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_cylinders, size));
            checkCudaErrors(cudaMemcpy(d_cylinders, cylVector, size, cudaMemcpyHostToDevice));

            delete[] cylVector;
            h_cylinders.clear();
        }

        //triangles
        size = h_triangles.size();
        if(size > 0) {
            Triangle *triVector = new Triangle[size];
            h_shapeSizes[triangleIndex] = size;

            for(size_t i = 0; i < size; i++) {
                triVector[i] = h_triangles[i];
            }

            size *= sizeof(Triangle);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_triangles, size));
            checkCudaErrors(cudaMemcpy(d_triangles, triVector, size, cudaMemcpyHostToDevice));

            delete[] triVector;
            h_triangles.clear();
        }

        //planes
        size = h_planes.size();
        if(size > 0) {
            Plane *plaVector = new Plane[size];
            h_shapeSizes[planeIndex] = size;

            for(size_t i = 0; i < size; i++) {
                plaVector[i] = h_planes[i];
            }

            size *= sizeof(Plane);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_planes, size));
            checkCudaErrors(cudaMemcpy(d_planes, plaVector, size, cudaMemcpyHostToDevice));

            delete[] plaVector;
            h_planes.clear();
        }

        int **shapes = new int*[nShapes];

        shapes[sphereIndex] = (int*)d_spheres;
        shapes[cylinderIndex] = (int*)d_cylinders;
        shapes[triangleIndex] = (int*)d_triangles;
        shapes[planeIndex] = (int*)d_planes;
        
        size = nShapes * sizeof(int*); 
        checkCudaErrors(cudaMalloc((void**) &d_shapes, size));
        checkCudaErrors(cudaMemcpy(d_shapes, shapes, size, cudaMemcpyHostToDevice));
        delete[] shapes;
        

        size = nShapes * sizeof(size_t);
        checkCudaErrors(cudaMalloc((void**) &d_shapeSizes, size));
        checkCudaErrors(cudaMemcpy(d_shapeSizes, h_shapeSizes, size, cudaMemcpyHostToDevice));
        delete[] h_shapeSizes;


        std::cout << "size: " << printSize(sceneSize) << std::endl;
        
        
        
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
    void addCylinder(float3 base, float3 top, float radius) {
	    Cylinder c = Cylinder(base, top, radius);
	    c.material = material;
	    h_cylinders.push_back(c);
    }

    __host__
    void addSphere(float3 center, float radius) {
	    Sphere sp = Sphere(center.x, center.y, center.z, radius);
	    sp.material = material;
	    h_spheres.push_back(sp);
    }

    __host__
    void addTriangle(std::vector<float3> verts) {
	    Triangle t = Triangle(verts);
	    t.material = material;
	    h_triangles.push_back(t);
    }

    __host__
    void addPolyPatch(int numVerts, std::vector<float3> verts, std::vector<float3> normals) {
	    //TODO
    }

    __host__
    void addPlane(float3 p1, float3 p2, float3 p3) {
	    Plane p = Plane(p1, p2, p3);
	    p.material = material;
	    h_planes.push_back(p);
    }
};





#endif