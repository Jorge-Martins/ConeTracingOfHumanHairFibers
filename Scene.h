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
    std::vector<SphereNode*> h_sphereBVH;
    std::vector<CylinderNode*> h_cylinderBVH;
    std::vector<TriangleNode*> h_triangleBVH;
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
            code = morton3D(computeCenter(cmin[cylinderIndex], cmax[cylinderIndex], h_cylinderBVH[i]->min, h_cylinderBVH[i]->max));
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
        std::cout << "Number of Cylinders = " << size << std::endl;
        std::cout << "Total Cylinders Size: " << printSize(size * (sizeof(CylinderNode) + cylinderSize)) << std::endl;
        start = clock();
        uint leafOffset = size - 1;

        uint sizebvh = (2 * size - 1) * sizeof(CylinderNode);
        uint sceneSize = sizebvh;

        checkCudaErrors(cudaMalloc((void**) &d_cylinderBVH, sizebvh));
        
        uint *d_cylMortonCodes;
        checkCudaErrors(cudaMalloc((void**) &d_cylMortonCodes, size * sizeof(uint)));
        checkCudaErrors(cudaMemcpyAsync(d_cylMortonCodes, codes, size * sizeof(uint), cudaMemcpyHostToDevice));

        mortonCodes[cylinderIndex] = (int*)d_cylMortonCodes;
        delete[] codes;

        checkCudaErrors(cudaMalloc((void**) &d_cylinderShapes, size * cylinderSize));
        sceneSize += size * cylinderSize;

        uint *h_OBBIndexes = nullptr;
        if(nCylOBBs > 0) {
            checkCudaErrors(cudaMalloc((void**) &d_matrixes, nCylOBBs * sizeof(Matrix)));
            checkCudaErrors(cudaMalloc((void**) &d_translations, nCylOBBs * sizeof(float3)));
            checkCudaErrors(cudaMalloc((void**) &d_OBBIndexes, nCylOBBs * sizeof(uint)));

            h_OBBIndexes = new uint[nCylOBBs];

            sceneSize += nCylOBBs * (sizeof(Matrix) + sizeof(float3));
        }

        Cylinder *h_cylinder;
        float3 *h_translation;
        Matrix *h_matrix;
        CylinderNode *node;
        uint obbIndex = 0;

        
        for(uint i = 0; i < size; i++) {
            node = h_cylinderBVH[values[i]];
            
            //copy shape
            h_cylinder = node->shape;
            checkCudaErrors(cudaMemcpyAsync(&d_cylinderShapes[i], h_cylinder, cylinderSize, cudaMemcpyHostToDevice));
            delete h_cylinder;
            

            if(node->type == OBB) {
                //copy matrix
                h_matrix = node->matrix;              
                checkCudaErrors(cudaMemcpyAsync(&d_matrixes[obbIndex], h_matrix, sizeof(Matrix), cudaMemcpyHostToDevice));
                delete h_matrix;

                //copy translation
                h_translation = node->translation;
                checkCudaErrors(cudaMemcpyAsync(&d_translations[obbIndex], h_translation, sizeof(float3), cudaMemcpyHostToDevice));
                delete h_translation;

                h_OBBIndexes[obbIndex++] = i;
                
            }

            checkCudaErrors(cudaMemcpyAsync(&d_cylinderBVH[leafOffset + i], node, sizeof(CylinderNode), cudaMemcpyHostToDevice));
            delete node;
        }

        if(h_OBBIndexes != nullptr) {
            checkCudaErrors(cudaMemcpyAsync(d_OBBIndexes, h_OBBIndexes, nCylOBBs * sizeof(uint), cudaMemcpyHostToDevice));
            delete[] h_OBBIndexes;
        }

        if(leafOffset > 0) {
            cudaMemset(d_cylinderBVH, 0, leafOffset * sizeof(CylinderNode));
        }

        end = clock();
        std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;

        delete[] values;
        h_cylinderBVH.clear();

        return sceneSize;
    }

    template <typename BVHNodeType, typename ShapeType>
    uint shapeTransfer(uint size, int shapeIndex, std::string shapeType, BVHNodeType *d_bvhNodes, 
                       ShapeType *d_shapes, std::vector<BVHNodeType*> h_bvhNodes) {
        uint *codes = new uint[size];
        uint *values = new uint[size];

        h_shapeSizes[shapeIndex] = size;

        uint shapeSize = sizeof(ShapeType);
        uint bvhNodeSize = sizeof(BVHNodeType);

        uint code;
        for(uint i = 0; i < size; i++) {
            code = morton3D(computeCenter(cmin[shapeIndex], cmax[shapeIndex], h_bvhNodes[i]->min, h_bvhNodes[i]->max));
            codes[i] = code;
            values[i] = i;
        }
            
        std::cout << shapeType << " morton codes sort: " << std::endl;
            
        //sort morton codes
        clock_t start = clock();
        thrust::sort_by_key(codes, codes + size, values);
        clock_t end = clock();

        std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;

        std::cout << shapeType << " data transfer: " << std::endl;
        std::cout << "Number of " << shapeType << "s = " << size << std::endl;
        std::cout << "Total " << shapeType << "s Size: " << printSize(size * (bvhNodeSize + shapeSize)) << std::endl;
        start = clock();
        uint leafOffset = size - 1;

        uint sizebvh = (2 * size - 1) * bvhNodeSize;
        uint sceneSize = sizebvh;

        checkCudaErrors(cudaMalloc((void**) &d_bvhNodes, sizebvh));
        checkCudaErrors(cudaMalloc((void**) &d_shapes, size * shapeSize));

        if(shapeType == "Sphere") {
            d_sphereBVH = (SphereNode*)d_bvhNodes;
            d_sphereShapes = (Sphere*)d_shapes;
        } else if(shapeType == "Triangle") {
            d_triangleBVH = (TriangleNode*)d_bvhNodes;
            d_triangleShapes = (Triangle*)d_shapes;
        }
        
        uint *d_mortonCodes;
        checkCudaErrors(cudaMalloc((void**) &d_mortonCodes, size * sizeof(uint)));
        checkCudaErrors(cudaMemcpyAsync(d_mortonCodes, codes, size * sizeof(uint), cudaMemcpyHostToDevice));

        mortonCodes[shapeIndex] = (int*)d_mortonCodes;
        delete[] codes;

        
        sceneSize += size * shapeSize;
        
        ShapeType *h_shape;
        BVHNodeType *node;

        
        for(uint i = 0; i < size; i++) {
            node = h_bvhNodes[values[i]];
            
            //copy shape
            h_shape = node->shape;
            checkCudaErrors(cudaMemcpyAsync(&d_shapes[i], h_shape, shapeSize, cudaMemcpyHostToDevice));
            delete h_shape;
            
            checkCudaErrors(cudaMemcpyAsync(&d_bvhNodes[leafOffset + i], node, bvhNodeSize, cudaMemcpyHostToDevice));
            delete node;
        }

        if(leafOffset > 0) {
            cudaMemset(d_bvhNodes, 0, leafOffset * bvhNodeSize);
        }

        end = clock();
        std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;

        delete[] values;
        h_bvhNodes.clear();

        return sceneSize;
    }

public:
    SphereNode *d_sphereBVH;
    CylinderNode *d_cylinderBVH;
    TriangleNode *d_triangleBVH;
    Plane *d_planes;

    uint *h_shapeSizes;

    int **mortonCodes;

    Cylinder *d_cylinderShapes;
    Sphere *d_sphereShapes;
    Triangle *d_triangleShapes;

    uint nCylOBBs;
    Matrix *d_matrixes;
    float3 *d_translations;
    uint *d_OBBIndexes;

    Scene() {
        d_shapeSizes = nullptr;
        d_shapes = nullptr;

        d_sphereBVH = nullptr;
        d_cylinderBVH = nullptr;
        d_triangleBVH = nullptr;
        d_planes = nullptr; 

        d_cylinderShapes = nullptr;
        d_sphereShapes = nullptr;
        d_triangleShapes = nullptr;

        d_matrixes = nullptr;
        d_translations = nullptr;
        d_OBBIndexes = nullptr;

        nCylOBBs = 0;
        
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

        if(d_sphereBVH != nullptr) {
            checkCudaErrors(cudaFree(d_sphereBVH));
            checkCudaErrors(cudaFree(d_sphereShapes));
        }
        if(d_cylinderBVH != nullptr) {
            checkCudaErrors(cudaFree(d_cylinderBVH));
            checkCudaErrors(cudaFree(d_cylinderShapes));
        }
        if(d_matrixes != nullptr)  {
            checkCudaErrors(cudaFree(d_matrixes));
            checkCudaErrors(cudaFree(d_translations));
        }
        if(d_triangleBVH != nullptr) {
            checkCudaErrors(cudaFree(d_triangleBVH));
            checkCudaErrors(cudaFree(d_triangleShapes));
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
        size = (uint)h_sphereBVH.size();
        if(size > 0) {
            sceneSize += shapeTransfer(size, sphereIndex, "Sphere", d_sphereBVH, d_sphereShapes, h_sphereBVH);
        }

        //cylinders
        size = (uint)h_cylinderBVH.size();
        if(size > 0) {
            sceneSize += cylinderTransfer(size);
        }

        //triangles
        size = (uint)h_triangleBVH.size();
        if(size > 0) {
            sceneSize += shapeTransfer(size, triangleIndex, "Triangle", d_triangleBVH, d_triangleShapes, h_triangleBVH);
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

        shapes[sphereIndex] = (int*)d_sphereBVH;
        shapes[cylinderIndex] = (int*)d_cylinderBVH;
        shapes[triangleIndex] = (int*)d_triangleBVH;
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
        CylinderNode *cn = new CylinderNode(AABB, c, nCylOBBs);

        cmin[cylinderIndex] = fminf(cmin[cylinderIndex], cn->min);
        cmax[cylinderIndex] = fmaxf(cmax[cylinderIndex], cn->max);
	    h_cylinderBVH.push_back(cn);
    }

    __host__
    void addSphere(float3 center, float radius) {
	    Sphere *sp = new Sphere(center.x, center.y, center.z, radius);
	    sp->material = material;
        SphereNode *spn = new SphereNode(sp);

        cmin[sphereIndex] = fminf(cmin[sphereIndex], spn->min);
        cmax[sphereIndex] = fmaxf(cmax[sphereIndex], spn->max);
	    h_sphereBVH.push_back(spn);
    }

    __host__
    void addTriangle(std::vector<float3> verts) {
	    Triangle *t = new Triangle(verts);
	    t->material = material;
        TriangleNode *tn = new TriangleNode(t);

        cmin[triangleIndex] = fminf(cmin[triangleIndex], tn->min);
        cmax[triangleIndex] = fmaxf(cmax[triangleIndex], tn->max);
	    h_triangleBVH.push_back(tn);
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