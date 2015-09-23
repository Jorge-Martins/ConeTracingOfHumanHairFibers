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
private:
	float3 _from, _at, _up;
	float _fov;
	float _width, _height;
    float _aspect;
    float _atDistance;
    float3 _xe, _ye, _ze;

	void computeFrame() {
	    float3 ze = normalize(_from - _at);
	    float3 xe = normalize(cross(_up, ze));
	    float3 ye = cross(ze, xe);

	    _xe = xe;
	    _ye = ye;
	    _ze = -ze;
    }

	
	void computeHitherDimensions() {
	    float atDist = length(_at - _from);
        float h = 2 * tanf(_fov * DEG2RAD * 0.5f) * atDist;
	    float w = _aspect * h;
        
	    _atDistance = atDist;
	    _width = w;
	    _height = h;
    }

public:
	float3 xe() {
        return _xe;
    }

    float3 ye() {
        return _ye;
    }

    float3 ze() {
        return _ze;
    }

    float3 from() {
        return _from;
    }


    float3 up() {
        return _up;
    }

    float width() {
        return _width;
    }

    float height() {
        return _height;
    }

    float atDistance() {
        return _atDistance;
    }

	Camera(float3 from, float3 at, float3 up, float fov, float aspect) {
        _from = from;
        _at = at;
        _up = up;
        _fov = fov;
        _aspect = aspect;
        computeFrame();
	    computeHitherDimensions();
    }
    //move
    void update(float3 from) {
        _from = from;
        computeFrame();
	    computeHitherDimensions();
    }
    //reshape
    void update(float aspect) {
        _aspect = aspect;
	    computeHitherDimensions();
    }

};


#endif