#pragma once

#ifndef __MCDRIVER_HPP__
#define __MCDRIVER_HPP__ 1

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <vector_types.h>

#define PI 3.14159265359f
#define DEG2RAD (PI/180.0f)
#define RAD2DEG (180.0f/PI)

struct Scene;

namespace MC{

class MC_Driver{
	Scene *_scene;
    float *_initRadius, *_initLongitude, *_initLatitude, *_initFov;
    float3 *_at;

public:
   MC_Driver();
   MC_Driver(Scene *scene);
   MC_Driver(Scene *scene,  float* initRadius, float* initLongitude, float* initLatitude, float* initFov, 
             float3 *at);

   virtual ~MC_Driver();

   bool parse(const char *filename);
  
private:
    void add_view(float3, float3, float3, float, float, int2);
    void add_back_col(float3);
    void add_light(float3);
    void add_light(float3, float3);
    void add_material(float3, float, float, float, float, float);
    void add_cylinder(float4, float4);
    void add_sphere(float3, float);
    void add_poly(int, std::vector<float3>);
    void add_poly_patch(int, std::vector<float3>, std::vector<float3>);
    void add_plane(float3, float3, float3);
	
    void print(std::string msg);
};

} /* end namespace MC */
#endif /* END __MCDRIVER_HPP__ */
