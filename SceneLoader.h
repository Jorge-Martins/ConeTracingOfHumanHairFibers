#pragma once

#ifndef _SCENELOADER_H_
#define _SCENELOADER_H_

#include "Scene.h"
#include <time.h>
#include "parsing/mc_driver.hpp"

#define PI 3.14159265359f
#define DEG2RAD (PI/180.0f)

bool load_nff(std::string filePath, Scene *sc) {
    clock_t start, end;

    filePath += ".nff";
	std::cout << "Loading: " << filePath << std::endl;

    start = clock();
	MC::MC_Driver driver(sc);
	driver.parse(filePath.c_str());
    end = clock();

    std::cout << "Build time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;


    start = clock();
    sc->copyToDevice();
    end = clock();

    std::cout << "Transfer time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl;
	return true;
}

class Camera {
public:
	float3 from, at, up;
	float fov;
	float aspect;
    float width, height;
    float atDistance;
    float3 xe, ye, ze;

	Camera(float3 from, float3 at, float3 up, float fov, float aspect) {
        this->from = from;
        this->at = at;
        this->up = up;
        this->fov = fov;
        this->aspect = aspect;
        computeFrame();
	    computeHitherDimensions();
    }
    //move
    void update(float3 from) {
        this->from = from;
        computeFrame();
	    computeHitherDimensions();
    }
    //reshape
    void update(float aspect) {
        this->aspect = aspect;
	    computeHitherDimensions();
    }

private:
    void computeFrame() {
	    ze = normalize(from - at);
	    xe = normalize(cross(up, ze));
	    ye = cross(ze, xe);
    }

	void computeHitherDimensions() {
	    atDistance = length(at - from);
        height = 2.0f * tanf(fov * DEG2RAD * 0.5f) * atDistance;
	    width = aspect * height;
    }
};


#endif