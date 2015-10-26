#pragma once

#ifndef _SCENE_H_
#define _SCENE_H_

#include "Primitives.h"

#define KB 1000
#define MB (1000 * KB)
#define GB (1000 * MB)

struct Scene {
private:
    float3 backcolor;
    Material material;
    std::vector<SphereNode> h_spheres;
    std::vector<CylinderNode> h_cylinders;
    std::vector<TriangleNode> h_triangles;
    std::vector<Plane> h_planes;
	std::vector<Light> h_lights;
    
    long *h_shapeSizes;
    
    SphereNode *d_spheres;
    CylinderNode *d_cylinders;
    TriangleNode *d_triangles;
    Plane *d_planes;

    Light *d_lights;

    int **d_shapes;
    long *d_shapeSizes;
    long d_lightsSize;

public:
    __host__
    Scene() {
        d_shapeSizes = nullptr;
        d_shapes = nullptr;

        d_spheres = nullptr;
        d_cylinders = nullptr;
        d_triangles = nullptr;
        d_planes = nullptr; 
        
        h_shapeSizes = new long[nShapes];
        
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
    long* getDShapesSize() {
        return d_shapeSizes;
    }

    __host__
    long getDLightsSize() {
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
        long size, sceneSize = 0;

        Light *ltVector = new Light[h_lights.size()];

        for(long i = 0; i < h_lights.size(); i++) {
            ltVector[i] = h_lights[i];
        }

        d_lightsSize = (long)h_lights.size();
        

        size = (long)h_lights.size() * sizeof(Light);
        sceneSize += size;

        checkCudaErrors(cudaMalloc((void**) &d_lights, size));
        checkCudaErrors(cudaMemcpy(d_lights, ltVector, size, cudaMemcpyHostToDevice));

        delete[] ltVector;
        h_lights.clear();

        //Spheres
        size = (long)h_spheres.size();
        if(size > 0) {
            SphereNode *sphVector = new SphereNode[size];
            h_shapeSizes[sphereIndex] = size;

            Sphere *h_sphere;
            long sizeSphere = sizeof(Sphere);
            for(long i = 0; i < size; i++) {
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
            for(long i = 0; i < h_spheres.size(); i++) {
                s = h_spheres[i].shape;

                if(s != nullptr) {
                    delete s;
                }
            }
            h_spheres.clear();
        }

        //cylinders
        size = (long)h_cylinders.size();
        if(size > 0) {
            CylinderNode *cylVector = new CylinderNode[size];
            h_shapeSizes[cylinderIndex] = size;

            Cylinder *h_cylinder;
            float3 *h_translation;
            Matrix *h_m;
            long cylinderSize = sizeof(Cylinder);
            for(long i = 0; i < size; i++) {
                cylVector[i] = h_cylinders[i];

                h_cylinder = h_cylinders[i].shape;
                if(h_cylinder != nullptr) {
                    //copy shape
                    sceneSize += cylinderSize;
                    checkCudaErrors(cudaMalloc((void**) &cylVector[i].shape, cylinderSize));
                    checkCudaErrors(cudaMemcpy(cylVector[i].shape, h_cylinder, cylinderSize, cudaMemcpyHostToDevice));
                }

                if(h_cylinders[i].type == OBB) {
                    //copy matrix
                    h_m = h_cylinders[i].matrix;
                    sceneSize += sizeof(Matrix);
                    checkCudaErrors(cudaMalloc((void**) &cylVector[i].matrix, sizeof(Matrix)));
                    checkCudaErrors(cudaMemcpy(cylVector[i].matrix, h_m, sizeof(Matrix), cudaMemcpyHostToDevice));

                    //copy translation
                    h_translation = h_cylinders[i].translation;
                    sceneSize += sizeof(float3);
                    checkCudaErrors(cudaMalloc((void**) &cylVector[i].translation, sizeof(float3)));
                    checkCudaErrors(cudaMemcpy(cylVector[i].translation, h_translation, sizeof(float3), cudaMemcpyHostToDevice));
                }
            }

            size *= sizeof(CylinderNode);
            sceneSize += size;

            checkCudaErrors(cudaMalloc((void**) &d_cylinders, size));
            checkCudaErrors(cudaMemcpy(d_cylinders, cylVector, size, cudaMemcpyHostToDevice));

            delete[] cylVector;
            Cylinder *c;
            for(long i = 0; i < h_cylinders.size(); i++) {
                c = h_cylinders[i].shape;

                if(c != nullptr) {
                    delete c;
                }

                if(h_cylinders[i].type == OBB) {
                    delete h_cylinders[i].matrix;
                    delete h_cylinders[i].translation;
                }
            }
            h_cylinders.clear();
        }

        //triangles
        size = (long)h_triangles.size();
        if(size > 0) {
            TriangleNode *triVector = new TriangleNode[size];
            h_shapeSizes[triangleIndex] = size;

            Triangle *h_triangle;
            long triangleSize = sizeof(Triangle);
            for(long i = 0; i < size; i++) {
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
            for(long i = 0; i < h_triangles.size(); i++) {
                t = h_triangles[i].shape;

                if(t != nullptr) {
                    delete t;
                }
            }
            h_triangles.clear();
        }

        //planes
        size = (long)h_planes.size();
        if(size > 0) {
            Plane *plaVector = new Plane[size];
            h_shapeSizes[planeIndex] = size;

            for(long i = 0; i < size; i++) {
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
        CylinderNode cn = CylinderNode(AABB, c);
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