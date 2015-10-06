#pragma once

#ifndef _SCENELOADER_H_
#define _SCENELOADER_H_

#include "Scene.h"
#include <time.h>
#include "parsing/mc_driver.hpp"
#include "parsing/cyHairFile.h"

bool load_nff(std::string filePath, Scene *sc, float *initRadius, float *initLongitude, 
              float *initLatitude, float *initFov, float3 *at, float3 *up) {
    clock_t start, end;

    filePath += ".nff";
	std::cout << "Loading: " << filePath << std::endl;

    start = clock();
	MC::MC_Driver driver(sc, initRadius, initLongitude, initLatitude, initFov, at, up);
	if(!driver.parse(filePath.c_str())) {
        return false;
    }
    end = clock();

    std::cout << "Build time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;

    std::cout << "Transfer" << std::endl;

    start = clock();
    sc->copyToDevice();
    end = clock();

    std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;
	return true;
}


bool load_hair(std::string filePath, Scene *sc, float3 *at, float3 *up) {
    
    filePath += ".hair";
	std::cout << "Loading: " << filePath << std::endl;

    cyHairFile hairfile = cyHairFile();

    // Load the hair model
    int result = hairfile.LoadFromFile(filePath.c_str());

    // Check for errors
    switch(result) {
        case CY_HAIR_FILE_ERROR_CANT_OPEN_FILE:
            printf("Error: Cannot open hair file!\n");
            return false;
        case CY_HAIR_FILE_ERROR_CANT_READ_HEADER:
            printf("Error: Cannot read hair file header!\n");
            return false;
        case CY_HAIR_FILE_ERROR_WRONG_SIGNATURE:
            printf("Error: File has wrong signature!\n");
            return false;
        case CY_HAIR_FILE_ERROR_READING_SEGMENTS:
            printf("Error: Cannot read hair segments!\n");
            return false;
        case CY_HAIR_FILE_ERROR_READING_POINTS:
            printf("Error: Cannot read hair points!\n");
            return false;
        case CY_HAIR_FILE_ERROR_READING_COLORS:
            printf("Error: Cannot read hair colors!\n");
            return false;
        case CY_HAIR_FILE_ERROR_READING_THICKNESS:
            printf("Error: Cannot read hair thickness!\n");
            return false;
        case CY_HAIR_FILE_ERROR_READING_TRANSPARENCY:
            printf("Error: Cannot read hair transparency!\n");
            return false;
        default:
            printf("Hair file \"%s\" loaded.\n", filePath.c_str());
    }

    

    int nSegments = hairfile.GetHeader().hair_count;
    int nPoints = hairfile.GetHeader().point_count;
    printf("Number of hair strands = %d\n", nSegments );
    //printf("Number of hair points = %d\n", nPoints );

    float *colorsArray = hairfile.GetColorsArray();
    float *thicknessArray = hairfile.GetThicknessArray();
    float *transparencyArray = hairfile.GetTransparencyArray();
    float *pointsArray = hairfile.GetPointsArray();
    const unsigned short *segments = hairfile.GetSegmentsArray();

    
    int pointIndex = 0;
    int segmentSize = hairfile.GetHeader().d_segments;
    float hairIor = 1.55f; //paper Light Scattering from Human Hair Fibers
    float Kd = 0.2f; //paper Light Scattering from Human Hair Fibers
    float Ks = 0.4f;  //https://support.solidangle.com/display/NodeRef/hair
    float shininess = 50.0f;
    float transparency = hairfile.GetHeader().d_transparency;
    float thickness = hairfile.GetHeader().d_thickness;

    float3 color, base, top;
    color = make_float3(hairfile.GetHeader().d_color[0], hairfile.GetHeader().d_color[1], hairfile.GetHeader().d_color[2]);

    sc->setBackcolor(make_float3(0.2f, 0.2f, 0.2f));
    sc->addLight(make_float3(1, -4, 4));

    *at = make_float3(pointsArray[0], pointsArray[1], pointsArray[2]);

    if (segments) {
        // If segments array exists
        int index = 3 * (segments[0] - 1);
        float3 lastPoint = make_float3(pointsArray[index], pointsArray[index + 1], pointsArray[index + 2]);
        float3 axis = *at - lastPoint;
        float d = length(axis) / 2.0f;
        axis = normalize(axis);
        *at = lastPoint + d * axis;

        for (int segment = 0; segment < nSegments; segment++ ) {
            segmentSize = segments[segment];

            for(int point = pointIndex; point < pointIndex + segmentSize; point++) {
                int cpIndex = point * 3;

                if(colorsArray) {
                    color = make_float3(colorsArray[cpIndex], colorsArray[cpIndex + 1], colorsArray[cpIndex + 2]);
                } 

                if(transparencyArray) {
                    transparency = transparencyArray[point];
                }

                if(thicknessArray) {
                    thickness = thicknessArray[point];
                }

                sc->setMaterial(color, Kd, Ks, shininess, transparency, hairIor);

                base = make_float3(pointsArray[cpIndex], pointsArray[cpIndex + 1], pointsArray[cpIndex + 2]);
                top = make_float3(pointsArray[cpIndex + 3], pointsArray[cpIndex + 4], pointsArray[cpIndex + 5]);
                sc->addCylinder(base, top, thickness);
            }
            
            pointIndex += segmentSize + 1;
        }
    } else {
        // If segments array does not exist, use default segment count
        int index = 3 * (segmentSize - 1);
        float3 lastPoint = make_float3(pointsArray[index], pointsArray[index + 1], pointsArray[index + 2]);
        float3 axis = *at - lastPoint;
        float d = length(axis) / 2.0f;
        axis = normalize(axis);
        *at = lastPoint + d * axis;

        for (int segment = 0; segment < nSegments; segment++ ) {
            
            for(int point = pointIndex; point < pointIndex + segmentSize; point++) {
                int cpIndex = point * 3;

                if(colorsArray) {
                    color = make_float3(colorsArray[cpIndex], colorsArray[cpIndex + 1], colorsArray[cpIndex + 2]);
                } 

                if(transparencyArray) {
                    transparency = transparencyArray[point];
                }

                if(thicknessArray) {
                    thickness = thicknessArray[point];
                }

                sc->setMaterial(color, Kd, Ks, shininess, transparency, hairIor);

                base = make_float3(pointsArray[cpIndex], pointsArray[cpIndex + 1], pointsArray[cpIndex + 2]);
                top = make_float3(pointsArray[cpIndex + 3], pointsArray[cpIndex + 4], pointsArray[cpIndex + 5]);
                sc->addCylinder(base, top, thickness);
            }

            pointIndex += segmentSize + 1;
        }
    }
    
    sc->copyToDevice();
    
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
    void update(float3 from, float fov) {
        this->fov = fov;
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