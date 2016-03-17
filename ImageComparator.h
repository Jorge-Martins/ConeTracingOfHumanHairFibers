#pragma once

#ifndef _IMAGECOMPARATOR_H_
#define _IMAGECOMPARATOR_H_

#include <FreeImage.h>
#include <string>
#include <iostream>

const std::string resourceDirPath = "../../resources/";

float3* loadImage(int &res, std::string *outFileName) {
    std::string fileName;
    std::cout << "Image load: Type the file name" << std::endl;
    std::cin >> fileName;

    std::string filePath = resourceDirPath + fileName + ".png";
    FIBITMAP *image = FreeImage_Load(FIF_PNG, filePath.c_str(), PNG_DEFAULT);

    if(!image) {
        std::cout << "File not found" << std::endl << std::endl;
        return nullptr;
    }

    image = FreeImage_ConvertTo24Bits(image);

    res = FreeImage_GetWidth(image) * FreeImage_GetHeight(image);

    float3 *imageVector = new float3[res];

    BYTE *pixels = (BYTE*) FreeImage_GetBits(image);

    for(int i = 0; i < res; i++) {
        imageVector[i].x = pixels[i * 3 + 2];
        imageVector[i].y = pixels[i * 3 + 1];
        imageVector[i].z = pixels[i * 3 + 0];
    }

    if(outFileName != nullptr) {
        *outFileName = fileName;
    }

    FreeImage_Unload(image);
    std::cout << "Image loaded!" << std::endl << std::endl;
    return imageVector;
}

float3* computeDiff(float3 *image1, float3 *image2, int res) {
    float3 *output = new float3[res];

    for(int i = 0; i < res; i++) {
        output[i] = fabsf(image1[i] - image2[i]);
    }

    return output;
}

float* computeFloatDiff(float3 *image1, float3 *image2, int res) {
    float *output = new float[res];
    float3 aux;

    for(int i = 0; i < res; i++) {
        aux = fabsf(image1[i] - image2[i]) / 255.0f;
        output[i] = (aux.x + aux.y + aux.z) / 3.0f;
    }

    return output;
}

void saveDiff() {
    std::string fileName1, fileName2;
    float3 *image1 = 0, *image2 = 0;
    int res1, res2, resX, resY;
    float contrastFactor = 5.0f;

    //try again if image not found
    for(int i = 2; i > 0 && !image1; i--) {
        image1 = loadImage(res1, &fileName1);
    }

    //try again if image not found
    for(int i = 2; i > 0 && !image2; i--) {
        image2 = loadImage(res2, &fileName2);
    }

    if(image1 && image2) {
        if(res1 != res2) {
            std::cout << "The two images provided have different resolutions" << std::endl << std::endl;
            
            delete image1;
            delete image2;
            return;
        }

        float3 *imageDif = computeDiff(image1, image2, res1);

        BYTE *imageData = new BYTE[res1 * 3];
        for(int i = 0; i < res1; i++) {
            imageData[i * 3 + 2] = (BYTE) (contrastFactor * imageDif[i].x);
            imageData[i * 3 + 1] = (BYTE) (contrastFactor * imageDif[i].y);
            imageData[i * 3 + 0] = (BYTE) (contrastFactor * imageDif[i].z);
        }

        resX = resY = (int) sqrtf((float)res1);
        std::string newImage = resourceDirPath + "dif_" + fileName1 + "_" + fileName2 + ".png";
        FIBITMAP *image = FreeImage_ConvertFromRawBits(imageData, resX, resY, resX * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
        FreeImage_Save(FIF_PNG, image, newImage.c_str(), 0);

        FreeImage_Unload(image);

        delete imageData;
        delete imageDif;

        std::cout << "Image saved" << std::endl << std::endl;
    
    } else {
        std::cout << "Unable to load images " << fileName1 << " and " << fileName2 << std::endl;
    }

    if(image1) {
        delete image1;
    }

    if(image2) {
        delete image2;
    }
}

float computeRMSError() {
    float3 *image1 = 0, *image2 = 0;
    int res1, res2;
    float rmsr = 0;

    //try again if image not found
    for(int i = 2; i > 0 && !image1; i--) {
        image1 = loadImage(res1, nullptr);
    }

    //try again if image not found
    for(int i = 2; i > 0 && !image2; i--) {
        image2 = loadImage(res2, nullptr);
    }

    if(image1 && image2) {
        if(res1 != res2) {
            std::cout << "The two images provided have different resolutions" << std::endl << std::endl;
            return -1;
        }

        float *imageDiff = computeFloatDiff(image1, image2, res1);

        for(int i = 0; i < res1; i++) {
            rmsr += imageDiff[i] * imageDiff[i];
        }

        rmsr = sqrtf(rmsr / res1);

        delete imageDiff;
    }


    if(image1) {
        delete image1;
    }

    if(image2) {
        delete image2;
    }

    return rmsr;
}

void saveFrame(GLuint tex, std::string sceneName, int resX, int resY) {
    int counter = 1;
    std::stringstream ss;
    FIBITMAP *image = 0;
    std::string file;
    int size = resX * resY * 3;

    ss << resourceDirPath << sceneName;

    #ifndef CONE_TRACING
    ss << "_R_";

    #else
    ss << "_C_";

    #endif


    #ifndef AT_HAIR
    ss << "d" << MAX_DEPTH;

    #else
    ss << "AT" << AOIT_HAIR_NODE_COUNT << "_"  << HAIR_INTERSECTION_LST_SIZE;

    #endif

    ss << "ss" << SUPER_SAMPLING << "_";
		
    file = ss.str() + std::to_string(static_cast<long long>(counter++)) + ".png";
    image = FreeImage_Load(FIF_PNG, file.c_str(), PNG_DEFAULT);

    while(image) {
        FreeImage_Unload(image);
        file = ss.str() + std::to_string(static_cast<long long>(counter++)) + ".png";
        image = FreeImage_Load(FIF_PNG, file.c_str(), PNG_DEFAULT);
    } 

    if(image) { 
        FreeImage_Unload(image);
    }

    BYTE* imageData = new BYTE[size];

    glBindTexture(GL_TEXTURE_2D, tex);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, imageData);
    glBindTexture(GL_TEXTURE_2D, 0);

	image = FreeImage_ConvertFromRawBits(imageData, resX, resY, resX * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
    FreeImage_Save(FIF_PNG, image, file.c_str(), 0);

	FreeImage_Unload(image);
	delete[] imageData;

    #ifndef CONE_TRACING
    #ifndef AT_HAIR
	std::cout << "Snapshot saved as " << sceneName << "_R_" << "d" << MAX_DEPTH << "ss" << SUPER_SAMPLING << "_" <<
                  std::to_string(static_cast<long long>(counter - 1)) << ".png"<< std::endl << std::endl;

    #else
    std::cout << "Snapshot saved as " << sceneName << "_R_" << "AT" << AOIT_HAIR_NODE_COUNT << "_"  << HAIR_INTERSECTION_LST_SIZE << "ss" << SUPER_SAMPLING << "_" <<
                  std::to_string(static_cast<long long>(counter - 1)) << ".png"<< std::endl << std::endl;
    
    #endif

    #else
    #ifndef AT_HAIR
	std::cout << "Snapshot saved as " << sceneName << "_C_" << "d" << MAX_DEPTH << "ss" << SUPER_SAMPLING << "_" <<
                  std::to_string(static_cast<long long>(counter - 1)) << ".png"<< std::endl << std::endl;

    #else
    std::cout << "Snapshot saved as " << sceneName << "_C_" << "AT" << AOIT_HAIR_NODE_COUNT << "_"  << HAIR_INTERSECTION_LST_SIZE << "ss" << SUPER_SAMPLING << "_" <<
                  std::to_string(static_cast<long long>(counter - 1)) << ".png"<< std::endl << std::endl;
    
    #endif
    #endif


}

#endif;