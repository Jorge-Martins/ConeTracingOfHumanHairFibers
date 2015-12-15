#pragma once

#ifndef _SCENE_H_
#define _SCENE_H_

#include "Primitives.h"
#include <thrust/sort.h>
#include <time.h>

std::string const sSize[] = {"B", "KB", "MB", "GB"};
int const bSize[] = {1, 1000, 1000000, 1000000000};

struct Scene {
private:
    float3 backcolor;
    Material material;
    std::vector<SphereNode> h_spheres;
    std::vector<CylinderNode> h_cylinders;
    std::vector<TriangleNode> h_triangles;
    std::vector<Plane> h_planes;
	std::vector<Light> h_lights;
    
    float3 cmin[nShapes], cmax[nShapes];

    Light *d_lights;

    int **d_shapes;
    uint *d_shapeSizes;
    uint d_lightsSize;

    uint cylinderTransfer(uint size) {
        uint *codes = new uint[size];
        uint *values = new uint[size];

        h_shapeSizes[cylinderIndex] = size;

        uint cylinderSize = sizeof(Cylinder);

        uint code;
        for(uint i = 0; i < size; i++) {
            code = morton3D(computeCenter(cmin[cylinderIndex], cmax[cylinderIndex], h_cylinders[i].min, h_cylinders[i].max));
            codes[i] = code;
            values[i] = i;
        }
            
        std::cout << "Cylinder morton codes sort: " << std::endl;
            
        //sort morton codes
        clock_t start = clock();
        thrust::sort_by_key(codes, codes + size, values);
        clock_t end = clock();

        std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;

        std::cout << "Cylinder data transfer: " << std::endl;
        start = clock();
        uint leafOffset = size - 1;

        uint sizebvh = (2 * size - 1) * sizeof(CylinderNode);
        uint sceneSize = sizebvh;

        checkCudaErrors(cudaMalloc((void**) &d_cylinders, sizebvh));
        
        uint *d_cylMortonCodes;
        checkCudaErrors(cudaMalloc((void**) &d_cylMortonCodes, size * sizeof(uint)));
        checkCudaErrors(cudaMemcpyAsync(d_cylMortonCodes, codes, size * sizeof(uint), cudaMemcpyHostToDevice));

        mortonCodes[cylinderIndex] = (int*)d_cylMortonCodes;
        delete[] codes;

        Cylinder *h_cylinder, *d_cylinder;
        float3 *h_translation, *d_translation;
        Matrix *h_matrix, *d_matrix;
        CylinderNode *node;
        
        for(uint i = 0; i < size; i++) {
            node = &h_cylinders[values[i]];
            
            d_matrix = nullptr;
            d_cylinder = nullptr;
            d_translation = nullptr;

            h_cylinder = node->shape;
            if(h_cylinder != nullptr) {
                //copy shape
                sceneSize += cylinderSize;
                checkCudaErrors(cudaMalloc((void**) &d_cylinder, cylinderSize));
                checkCudaErrors(cudaMemcpyAsync(d_cylinder, h_cylinder, cylinderSize, cudaMemcpyHostToDevice));

                delete h_cylinder;
            }

            if(node->type == OBB) {
                //copy matrix
                h_matrix = node->matrix;
                sceneSize += sizeof(Matrix);
                checkCudaErrors(cudaMalloc((void**) &d_matrix, sizeof(Matrix)));
                checkCudaErrors(cudaMemcpyAsync(d_matrix, h_matrix, sizeof(Matrix), cudaMemcpyHostToDevice));
                delete h_matrix;

                //copy translation
                h_translation = node->translation;
                sceneSize += sizeof(float3);
                checkCudaErrors(cudaMalloc((void**) &d_translation, sizeof(float3)));
                checkCudaErrors(cudaMemcpyAsync(d_translation, h_translation, sizeof(float3), cudaMemcpyHostToDevice));
                delete h_translation;
                
            }

            checkCudaErrors(cudaMemcpyAsync(&d_cylinders[leafOffset + i], node, sizeof(CylinderNode), cudaMemcpyHostToDevice));

            if(d_cylinder) {
                checkCudaErrors(cudaMemcpyAsync(&(d_cylinders[leafOffset + i].shape), &d_cylinder, sizeof(Cylinder*), cudaMemcpyHostToDevice));
            }
            if(d_matrix) {
                checkCudaErrors(cudaMemcpyAsync(&(d_cylinders[leafOffset + i].matrix), &d_matrix, sizeof(Matrix*), cudaMemcpyHostToDevice));
            }
            if(d_translation) {
                checkCudaErrors(cudaMemcpyAsync(&(d_cylinders[leafOffset + i].translation), &d_translation, sizeof(float3*), cudaMemcpyHostToDevice));
            }
        }

        if(leafOffset > 0) {
            cudaMemset(d_cylinders, 0, leafOffset * sizeof(CylinderNode));
        }

        end = clock();
        std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;

        delete[] values;
        
        h_cylinders.erase(h_cylinders.begin(), h_cylinders.end());
        h_cylinders.clear();

        return sceneSize;
    }

public:
    SphereNode *d_spheres;
    CylinderNode *d_cylinders;
    TriangleNode *d_triangles;
    Plane *d_planes;

    uint *h_shapeSizes;

    int **mortonCodes;

    Scene() {
        d_shapeSizes = nullptr;
        d_shapes = nullptr;

        d_spheres = nullptr;
        d_cylinders = nullptr;
        d_triangles = nullptr;
        d_planes = nullptr; 
        
        h_shapeSizes = new uint[nShapes];
        mortonCodes = new int*[nShapes];
        
        d_lights = nullptr;

        for(int i = 0; i < nShapes; i++) {
            h_shapeSizes[i] = 0;
            cmin[i] = make_float3(FLT_MAX);
            cmax[i] = make_float3(-FLT_MAX);
        }
    }

	~Scene() {
        delete[] h_shapeSizes;

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

    int** getDShapes() {
        return d_shapes;
    }

    uint* getDShapesSize() {
        return d_shapeSizes;
    }

    uint getDLightsSize() {
        return d_lightsSize;
    }

    Light* getDLights() {
        return d_lights;
    }

    void addLight(float3 pos) {
	    h_lights.push_back(Light(pos));
    }

    std::string printSize(size_t size) {
        std::ostringstream os;
        
        int index = (int)(log10f((float)size) / 3.0f);
        float out = (float) size / (float)bSize[index];
        std::string s = sSize[index];

        os << out << s;
        return os.str();
    }

    bool copyToDevice() {
        uint size, sceneSize = 0;

        Light *ltVector = new Light[h_lights.size()];

        for(uint i = 0; i < h_lights.size(); i++) {
            ltVector[i] = h_lights[i];
        }

        d_lightsSize = (uint)h_lights.size();
        

        size = (uint)h_lights.size() * sizeof(Light);
        sceneSize += size;

        checkCudaErrors(cudaMalloc((void**) &d_lights, size));
        checkCudaErrors(cudaMemcpy(d_lights, ltVector, size, cudaMemcpyHostToDevice));

        delete[] ltVector;
        h_lights.clear();

        //Spheres
        size = (uint)h_spheres.size();
        if(size > 0) {
            SphereNode *sphVector = new SphereNode[size];
            h_shapeSizes[sphereIndex] = size;

            Sphere *h_sphere;
            uint sizeSphere = sizeof(Sphere);

            SphereNode *node;
            for(uint i = 0; i < size; i++) {
                node = &h_spheres[i];
                sphVector[i] = *node;

                h_sphere = node->shape;
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
            for(uint i = 0; i < h_spheres.size(); i++) {
                s = h_spheres[i].shape;

                if(s != nullptr) {
                    delete s;
                }
            }
            h_spheres.clear();
        }

        //cylinders
        size = (uint)h_cylinders.size();
        if(size > 0) {
            sceneSize += cylinderTransfer(size);
        }

        //triangles
        size = (uint)h_triangles.size();
        if(size > 0) {
            TriangleNode *triVector = new TriangleNode[size];
            h_shapeSizes[triangleIndex] = size;

            Triangle *h_triangle;
            uint triangleSize = sizeof(Triangle);

            TriangleNode *node;
            for(uint i = 0; i < size; i++) {
                node = &h_triangles[i];
                triVector[i] = *node;

                h_triangle = node->shape;
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
            for(uint i = 0; i < h_triangles.size(); i++) {
                t = h_triangles[i].shape;

                if(t != nullptr) {
                    delete t;
                }
            }
            h_triangles.clear();
        }

        //planes
        size = (uint)h_planes.size();
        if(size > 0) {
            Plane *plaVector = new Plane[size];
            h_shapeSizes[planeIndex] = size;

            for(uint i = 0; i < size; i++) {
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

        cmin[cylinderIndex] = fminf(cmin[cylinderIndex], cn.min);
        cmax[cylinderIndex] = fmaxf(cmax[cylinderIndex], cn.max);
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