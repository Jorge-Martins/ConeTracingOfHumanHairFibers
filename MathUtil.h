#pragma once

#ifndef _MATHUTIL_H_
#define _MATHUTIL_H_

#include <helper_math.h>
#include <helper_cuda.h>

#define SUPER_SAMPLING 1
#define MAX_DEPTH 2
#define EPSILON 1E-4f

#define sphereIndex 0
#define cylinderIndex 1
#define triangleIndex 2
#define planeIndex 3
#define nShapes 4

struct Matrix {
    float3 M[3];

    __host__ __device__
    Matrix() {
        M[0] = M[1] = M[2] = make_float3(0.0f);
    }

    __host__ __device__
    Matrix(float3 *m) {
        M[0] = m[0];
        M[1] = m[1];
        M[2] = m[2];
    }

    __host__ __device__
    Matrix(float3 a, float3 b, float3 c) {
        M[0] = a;
        M[1] = b;
        M[2] = c;
    }

    __host__ __device__
    Matrix(float a, float b, float c, float d, float e, float f, float g, float h, float i) {
        M[0] = make_float3(a, b, c);
        M[1] = make_float3(d, e, f);
        M[2] = make_float3(g, h, i);
    }

    __host__ __device__
    Matrix transpose() {
        return Matrix(M[0].x, M[1].x, M[2].x, 
                      M[0].y, M[1].y, M[2].y,
                      M[0].z, M[1].z, M[2].z);
    }

    __host__ __device__
    float det(float a, float b, float c, float d) {
        return a * d - c * b;
    }

    __host__ __device__
    float determinant() {
        return M[0].x * det(M[1].y, M[1].z, M[2].y, M[2].z) -
               M[0].y * det(M[1].x, M[1].z, M[2].x, M[2].z) +
               M[0].z * det(M[1].x, M[1].y, M[2].x, M[2].y);
    }

    __host__ __device__
    Matrix inverse() {
        float d = abs(determinant());
        d = 1.0f / d;

        float3 a, b, c;

        a = make_float3(det(M[1].y, M[1].z, M[2].y, M[2].z), -det(M[1].x, M[1].z, M[2].x, M[2].z), det(M[1].x, M[1].y, M[2].x, M[2].y));
        b = make_float3(-det(M[0].y, M[0].z, M[2].y, M[2].z), det(M[0].x, M[0].z, M[2].x, M[2].z), -det(M[0].x, M[0].y, M[2].x, M[2].y));
        c = make_float3(det(M[0].y, M[0].z, M[1].y, M[1].z), -det(M[0].x, M[0].z, M[1].x, M[1].z), det(M[0].x, M[0].y, M[1].x, M[1].y));

        Matrix inv = Matrix(a, b, c).transpose();

        return inv * d;

    }

    __host__ __device__
    Matrix operator*(const float v) {
        return Matrix(M[0] * v, M[1] * v, M[2] * v);
    }

    __host__ __device__
    float3 operator*(const float3 v) {
        float res[3];

        for (int i = 0; i < 3; i++) {
            res[i] = dot(M[i], v);
        }

        return make_float3(res[0], res[1], res[2]);
    }

    __host__ __device__
    Matrix operator*(Matrix m) {
        float temp[9];
        Matrix mt = m.transpose();

        for (int i = 0; i < 3; i++) {
	        for (int j = 0; j < 3; j++) {
			    temp[i*3 + j] = dot(M[i], mt.M[j]);
		    }
        }

        return Matrix(temp[0], temp[1], temp[2],
                      temp[3], temp[4], temp[5],
                      temp[6], temp[7], temp[8]);
    }
};

inline __host__ __device__ float3 operator*(float3 v, Matrix m) {
    Matrix mt = m.transpose();
    float res[3];

    for (int i = 0; i < 3; i++) {
        res[i] = dot(v, mt.M[i]);
    }

    return make_float3(res[0], res[1], res[2]);
}

inline __host__ __device__ Matrix operator*(float f, Matrix m) {
    return m * f;
}

#endif