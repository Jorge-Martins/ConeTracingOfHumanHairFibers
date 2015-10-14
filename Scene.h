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

#define KB 1000
#define MB (1000 * KB)
#define GB (1000 * MB)

#define sphereIndex 0
#define cylinderIndex 1
#define triangleIndex 2
#define planeIndex 3
#define nShapes 4

#define EPSILON 1E-4f

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

//====================================  device ========================

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
    Cylinder *shape;

    __host__
    CylinderNode() {
        type = AABB;
        shape = nullptr;
    }

    __host__
    CylinderNode(short type) {
        this->type = type;
        shape = nullptr;
    }

    __host__
    CylinderNode(short type, Cylinder *shape) {
        this->type = type;
        this->shape = shape;

        min = fminf(shape->base, shape->top) - shape->radius;
        max = fmaxf(shape->base, shape->top) + shape->radius;
    }

    __host__
    CylinderNode(Cylinder *shape) {
        type = AABB;
        this->shape = shape;

        min = fminf(shape->base, shape->top) - shape->radius;
        max = fmaxf(shape->base, shape->top) + shape->radius;
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

struct Scene {
private:
    float3 backcolor;
    Material material;
    std::vector<SphereNode> h_spheres;
    std::vector<CylinderNode> h_cylinders;
    std::vector<TriangleNode> h_triangles;
    std::vector<Plane> h_planes;
	std::vector<Light> h_lights;
    
    size_t *h_shapeSizes;
    
    SphereNode *d_spheres;
    CylinderNode *d_cylinders;
    TriangleNode *d_triangles;
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
            SphereNode *sphVector = new SphereNode[size];
            h_shapeSizes[sphereIndex] = size;

            Sphere *h_sphere;
            size_t sizeSphere = sizeof(Sphere);
            for(size_t i = 0; i < size; i++) {
                sphVector[i] = h_spheres[i];

                h_sphere = h_spheres[i].shape;
                if(h_sphere != nullptr) {
                    sceneSize += sizeSphere;
                    checkCudaErrors(cudaMalloc((void**) &sphVector[i].shape, sizeSphere));
                    checkCudaErrors(cudaMemcpy(sphVector[i].shape, h_sphere, sizeSphere, cudaMemcpyHostToDevice));
                }
            }

            size *= sizeof(SphereNode);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_spheres, size));
            checkCudaErrors(cudaMemcpy(d_spheres, sphVector, size, cudaMemcpyHostToDevice));

            delete[] sphVector;
            Sphere *s;
            for(size_t i = 0; i < h_spheres.size(); i++) {
                s = h_spheres[i].shape;

                if(s != nullptr) {
                    delete s;
                }
            }
            h_spheres.clear();
        }

        //cylinders
        size = h_cylinders.size();
        if(size > 0) {
            CylinderNode *cylVector = new CylinderNode[size];
            h_shapeSizes[cylinderIndex] = size;

            Cylinder *h_cylinder;
            size_t cylinderSize = sizeof(Cylinder);
            for(size_t i = 0; i < size; i++) {
                cylVector[i] = h_cylinders[i];

                h_cylinder = h_cylinders[i].shape;
                if(h_cylinder != nullptr) {
                    sceneSize += cylinderSize;
                    checkCudaErrors(cudaMalloc((void**) &cylVector[i].shape, cylinderSize));
                    checkCudaErrors(cudaMemcpy(cylVector[i].shape, h_cylinder, cylinderSize, cudaMemcpyHostToDevice));
                }
            }

            size *= sizeof(CylinderNode);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_cylinders, size));
            checkCudaErrors(cudaMemcpy(d_cylinders, cylVector, size, cudaMemcpyHostToDevice));

            delete[] cylVector;
            Cylinder *c;
            for(size_t i = 0; i < h_cylinders.size(); i++) {
                c = h_cylinders[i].shape;

                if(c != nullptr) {
                    delete c;
                }
            }
            h_cylinders.clear();
        }

        //triangles
        size = h_triangles.size();
        if(size > 0) {
            TriangleNode *triVector = new TriangleNode[size];
            h_shapeSizes[triangleIndex] = size;

            Triangle *h_triangle;
            size_t triangleSize = sizeof(Triangle);
            for(size_t i = 0; i < size; i++) {
                triVector[i] = h_triangles[i];

                h_triangle = h_triangles[i].shape;
                if(h_triangle != nullptr) {
                    sceneSize += triangleSize;
                    checkCudaErrors(cudaMalloc((void**) &triVector[i].shape, triangleSize));
                    checkCudaErrors(cudaMemcpy(triVector[i].shape, h_triangle, triangleSize, cudaMemcpyHostToDevice));
                }
            }

            size *= sizeof(TriangleNode);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_triangles, size));
            checkCudaErrors(cudaMemcpy(d_triangles, triVector, size, cudaMemcpyHostToDevice));

            delete[] triVector;
            Triangle *t;
            for(size_t i = 0; i < h_triangles.size(); i++) {
                t = h_triangles[i].shape;

                if(t != nullptr) {
                    delete t;
                }
            }
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
	    Cylinder *c = new Cylinder(base, top, radius);
	    c->material = material;
        CylinderNode cn = CylinderNode(c);
	    h_cylinders.push_back(cn);
    }

    __host__
    void addSphere(float3 center, float radius) {
	    Sphere *sp = new Sphere(center.x, center.y, center.z, radius);
	    sp->material = material;
        SphereNode spn = SphereNode(sp);
	    h_spheres.push_back(spn);
    }

    __host__
    void addTriangle(std::vector<float3> verts) {
	    Triangle *t = new Triangle(verts);
	    t->material = material;
        TriangleNode tn = TriangleNode(t);
	    h_triangles.push_back(tn);
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