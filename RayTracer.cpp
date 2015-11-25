#include <stdlib.h> 
#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h> 

#include "SceneLoader.h"
#include <string>

#include <cuda_gl_interop.h>
#include <helper_functions.h>

#include <FreeImage.h>


Scene *scene = 0;
Camera *camera = 0;

RayInfo *d_raysInfo = 0;
float3 *d_locals = 0, *d_reflectionCols = 0, *d_refractionCols = 0;

int fpsCount = 0;
int fpsLimit = 1;        // FPS limit for sampling

int RES_X = 512, RES_Y = 512;
dim3 blockSize(8, 8);
dim3 gridSize;

float horizontalAngle, verticalAngle, radius;
float initHorizontalAngle = 180.0f, initVerticalAngle = 90.0f, initRadius = 20.0f, initFov = 50.0f;//initRadius = 60.0f, initFov = 90.0f;

int xDragStart, yDragStart, dragging, zooming;
float fov;

bool stopRender = false, videoMode = true;

cudaEvent_t c_start, c_end;

// OpenGL pixel buffer object
GLuint pbo;

// OpenGL texture object
GLuint tex = 0;

// CUDA Graphics Resource (to transfer PBO)
struct cudaGraphicsResource *cuda_pbo = 0; 

StopWatchInterface *timer = NULL;

const char* windowTitle = "Msc Ray Tracing";

//std::string sceneName = "rings_low";
//std::string sceneName = "straight"; //initHorizontalAngle = 180.0f  -20z
std::string sceneName = "wCurly"; //initHorizontalAngle = 100.0f

extern void deviceClearImage(float3 *d_output, float3 value, int resX, int resY, dim3 gridSize, dim3 blockSize);

extern void deviceDrawScene(int **d_shapes, uint *d_shapeSizes, Light *lights, uint lightSize, float3 backcolor, 
                            int resX, int resY, float width, float height, float atDistance, float3 xe, 
                            float3 ye, float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize,
                            RayInfo *d_raysInfo, float3 *d_locals, float3 *d_reflectionCols, float3 *d_refractionCols);

extern void deviceBuildBVH(CylinderNode *bvh, uint nObjects, int *d_nodeCounter, dim3 gridSize, dim3 blockSize);

float3 computeFromCoordinates(float3 up){
    float ha, va;

    ha = (float) (horizontalAngle * DEG2RAD);
    va = (float) (verticalAngle * DEG2RAD);

    if(up.x > 0) {
        return make_float3(radius * cos(va), radius * sin(va) * cos(ha), radius * sin(va) * sin(ha));

    } else if(up.y > 0) {
        return make_float3(radius * sin(va) * sin(ha), radius * cos(va), radius * sin(va) * cos(ha));
    }

    return make_float3(radius * sin(va) * cos(ha), radius * sin(va) * sin(ha), radius * cos(va));
}

void cleanup() {
    std::cout << "cleanup" << std::endl;
    if(scene) {
        delete scene;
        std::cout << "scene done" << std::endl;
    }

    if(camera) {
        delete camera;
        std::cout << "camera done" << std::endl;
    }

    if(d_raysInfo) {
        checkCudaErrors(cudaFree(d_raysInfo));
        checkCudaErrors(cudaFree(d_locals));
        checkCudaErrors(cudaFree(d_reflectionCols));
        checkCudaErrors(cudaFree(d_refractionCols));
        std::cout << "rays done" << std::endl;
    }

    sdkDeleteTimer(&timer);

    if (pbo) {
        cudaGraphicsUnregisterResource(cuda_pbo);
        glDeleteBuffersARB(1, &pbo);
        glDeleteTextures(1, &tex);
        std::cout << "pbo done" << std::endl;
    }

    std::cout << "reseting device" << std::endl;
    checkCudaErrors(cudaDeviceReset());
    cudaDeviceReset();
    std::cout << "reseting device done" << std::endl;

}

void computeFPS() {
    fpsCount++;

    if (fpsCount == fpsLimit) {
        char fps[100];
        float ifps = 1.f / (sdkGetAverageTimerValue(&timer) / 1000.f);
        sprintf(fps, "%s: %3.1f fps  HA: %.2f  VA: %.2f  FOV: %.2f", windowTitle, ifps, horizontalAngle, verticalAngle, fov);

        glutSetWindowTitle(fps);
        fpsCount = 0;

        fpsLimit = (int)MAX(1.0f, ifps);
        sdkResetTimer(&timer);
    }
}

void cudaInit() {
    uint size, totalSize = 0;
    clock_t start = clock();
   /* if(d_raysInfo) {
        checkCudaErrors(cudaFree(d_raysInfo));
        checkCudaErrors(cudaFree(d_locals));
        checkCudaErrors(cudaFree(d_reflectionCols));
        checkCudaErrors(cudaFree(d_refractionCols));
    }*/

    //size local array
    size = RES_X * RES_Y;
    uint localsSize = size * ((2 << MAX_DEPTH) - 1);

    //size reflection and refraction arrays 
    uint sizeRRArrays = size * ((1 << MAX_DEPTH) - 1);
    
    uint raysSize = size * (1 << MAX_DEPTH);

    RayInfo *raysInfo = new RayInfo[raysSize]; 
    float3 *colors = new float3[localsSize];

    size = raysSize * sizeof(RayInfo);
    totalSize += size; 
    checkCudaErrors(cudaMalloc((void**) &d_raysInfo, size));
    checkCudaErrors(cudaMemcpyAsync(d_raysInfo, raysInfo, size, cudaMemcpyHostToDevice));

    size = localsSize * sizeof(float3);
    totalSize += size;
    checkCudaErrors(cudaMalloc((void**) &d_locals, size));
    checkCudaErrors(cudaMemcpyAsync(d_locals, colors, size, cudaMemcpyHostToDevice));

    size = sizeRRArrays * sizeof(float3);
    totalSize += size * 2;
    checkCudaErrors(cudaMalloc((void**) &d_reflectionCols, size));
    checkCudaErrors(cudaMemcpyAsync(d_reflectionCols, colors, size, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMalloc((void**) &d_refractionCols, size));
    checkCudaErrors(cudaMemcpyAsync(d_refractionCols, colors, size, cudaMemcpyHostToDevice));

    delete[] raysInfo;
    delete[] colors;
    clock_t end = clock();

    //debug info
    std::cout << "cuda init" << std::endl;
    std::cout << "size: " << scene->printSize(totalSize) << std::endl;
    std::cout << "time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;
}

void initPixelBuffer() {
    if (pbo) {
        // unregister this buffer object from CUDA C
        checkCudaErrors(cudaGraphicsUnregisterResource(cuda_pbo));

        // delete old buffer
        glDeleteBuffersARB(1, &pbo);
        glDeleteTextures(1, &tex);
    }

    // create pixel buffer object for display
    glGenBuffersARB(1, &pbo);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, RES_X * RES_Y * sizeof(float3), 0, GL_STREAM_DRAW_ARB);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

    // register this buffer object with CUDA
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_pbo, pbo, cudaGraphicsMapFlagsWriteDiscard));

    // create texture for display
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, RES_X, RES_Y, 0, GL_RGB, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

//Function that calls the kernel
void render() {
    if(!stopRender || videoMode) {
        float3 *d_output;

        // map PBO to get CUDA device pointer
        checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pbo, 0));
        size_t num_bytes;
        checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void **)&d_output, &num_bytes, cuda_pbo));

        cudaEventRecord(c_start);
        /*if(SUPER_SAMPLING > 1 ) {
            deviceClearImage(d_output, make_float3(0.0f), RES_X, RES_Y, gridSize, blockSize);
        }*/

        // call CUDA kernel, writing results to PBO
        deviceDrawScene(scene->getDShapes(), scene->getDShapesSize(), scene->getDLights(), scene->getDLightsSize(), 
                        scene->getBackcolor(), RES_X, RES_Y, camera->width, camera->height, camera->atDistance, 
                        camera->xe, camera->ye, camera->ze, camera->from, d_output, gridSize, blockSize, 
                        d_raysInfo, d_locals, d_reflectionCols, d_refractionCols);

        cudaError_t error = cudaGetLastError();
        if(error != cudaSuccess) {
            std::cerr << cudaGetErrorString(error) << std::endl;
        }

        if(!videoMode) {
            cudaEventRecord(c_end);
            cudaEventSynchronize(c_end);
            float milliseconds = 0;
            cudaEventElapsedTime(&milliseconds, c_start, c_end);

            //debug info
            std::cout << "render time: " << milliseconds / 1000.0f << "s" << std::endl << std::endl;
            stopRender = true;
        }
        checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pbo, 0));
    }
}

void reshape(int w, int h) {
    //RES_X = w;
    //RES_Y = h;
    
    // calculate new grid size
    //gridSize = dim3(iceil(RES_X, blockSize.x), iceil(RES_Y, blockSize.y));
    //camera->update(RES_X / (float)RES_Y);

    


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    
	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, RES_X, RES_Y);

}

// Draw function by primary ray casting from the eye towards the scene's objects 
void drawScene() {
    sdkStartTimer(&timer);
    
    render();

    // display results
    glClear(GL_COLOR_BUFFER_BIT);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, RES_X, RES_Y, GL_RGB, GL_FLOAT, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

    // draw textured quad
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);

    glTexCoord2f(1, 0);
    glVertex2f(1, 0);

    glTexCoord2f(1, 1);
    glVertex2f(1, 1);

    glTexCoord2f(0, 1);
    glVertex2f(0, 1);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    glutSwapBuffers();
    glutReportErrors();

    sdkStopTimer(&timer);
    computeFPS();
}

void mouseMove(int x, int y) { 	
    float ystep = 0.01f;
    float xstep = 0.01f;
    float zstep = 0.01f;

    if (dragging == 1) {
        horizontalAngle += (-x + xDragStart) * xstep;

        if(horizontalAngle > 360) {
            horizontalAngle -= 360;
        } else if(horizontalAngle < 0) {
            horizontalAngle += 360;
        }

        verticalAngle += (-y + yDragStart) * ystep;

        if(verticalAngle > 179){
            verticalAngle = 179;
        } else if(verticalAngle < 1) {
            verticalAngle = 1;
        }

        //std::cout<< "Ha: " << horizontalAngle << std::endl << "Va: "<< verticalAngle << std::endl;
        camera->update(computeFromCoordinates(camera->up), fov);

    }
    if(zooming == 1) {
        fov += (y - yDragStart) * zstep;

        if(fov > 179){
            fov = 179;
        } else if(fov < 1) {
            fov = 1;
        }

        //std::cout<< "fov: " << fov << std::endl;
        camera->update(computeFromCoordinates(camera->up), fov);
        //std::cout<< "From: " << camera->from().x << " " << camera->from().y << " " << camera->from().z << std::endl;
        //std::cout<< "radius: " << radius << std::endl;
    } 
}

void mousePressed(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) { 
			dragging = 1; 
			xDragStart = x; 
            yDragStart = y; 
		} else  {
			dragging = 0;
		}
	}

   if (button == GLUT_RIGHT_BUTTON){
      if(state == GLUT_DOWN){
         zooming = 1;
         yDragStart = y;
      } else{
         zooming = 0;
      }
   }
}

void initPosition() {
    radius = initRadius;
    verticalAngle = initVerticalAngle;
    horizontalAngle = initHorizontalAngle;
    fov = initFov;
}

void keyboardKey(unsigned char key, int x, int y) {
	if (key == 'c'){
		initPosition();
        camera->update(computeFromCoordinates(camera->up), fov);
	}

	if (key == 'p'){
        int size = RES_X * RES_Y * 3;
		
        std::string file = "../../resources/" + sceneName + ".png";
        BYTE* imageData = new BYTE[size];

        glBindTexture(GL_TEXTURE_2D, tex);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, imageData);
        glBindTexture(GL_TEXTURE_2D, 0);

		FIBITMAP* image = FreeImage_ConvertFromRawBits(imageData, RES_X, RES_Y, RES_X * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
        FreeImage_Save(FIF_PNG, image, file.c_str(), 0);

		FreeImage_Unload(image);
		delete[] imageData;

		std::cout << "Snapshot saved" << std::endl;
	}

    if(key == 'f') {
        std::cout << "rendering new frame" << std::endl << std::endl;
        stopRender = false;
    }

    if(key == 'm') {
        if(videoMode) {
            std::cout << "frame Mode:" << std::endl << std::endl;
        } else {
            std::cout << "video Mode:" << std::endl << std::endl;
        }

        videoMode = !videoMode;
    }
}

void idle() {
    glutPostRedisplay();
}

void buildBVH() {
    uint size = scene->h_shapeSizes[cylinderIndex];

    if(size > 1) {
        int warpSize = 32;
        int *d_nodeCounter;
        uint counterSize =  sizeof(int) * size;

        cudaEventRecord(c_start);
        checkCudaErrors(cudaMalloc((void**) &d_nodeCounter, counterSize));
        checkCudaErrors(cudaMemset(d_nodeCounter, 0, counterSize));

        dim3 grid = dim3(iceil(size, warpSize));
        dim3 vectorBlock = dim3(warpSize);

        deviceBuildBVH(scene->d_cylinders, size, d_nodeCounter, grid, vectorBlock);
        cudaEventRecord(c_end);

        cudaEventSynchronize(c_end);

        float milliseconds = 0;
        cudaEventElapsedTime(&milliseconds, c_start, c_end);

        //debug info
        std::cout << "BVH building time: " << milliseconds / 1000.0f << "s" << std::endl << std::endl;
        cudaFree(d_nodeCounter);
    }
}

int main(int argc, char *argv[]) {
    cudaEventCreate(&c_start);
    cudaEventCreate(&c_end);
    sdkCreateTimer(&timer);
	//std::string path = "../../resources/nffFiles/";
    std::string path = "../../resources/HairModels/";
    scene = new Scene();

    //Explicitly set device 0 
    cudaSetDevice(0); 

    float3 at = make_float3(0.0f);
    float3 up = make_float3(0.0f , 0.0f, 1.0f);

	/*if (!load_nff(path + sceneName, scene, &initRadius, &initVerticalAngle, &initHorizontalAngle, &initFov, &up)) {
        delete scene;

        getchar();
		return -1;
	}*/

    if (!load_hair(path + sceneName, scene)) {
        cleanup();

        getchar();
		return -1;
	}

    initPosition();
    
    float3 from = computeFromCoordinates(up);
    
    buildBVH();
    
    // calculate new grid size
    gridSize = dim3(iceil(RES_X, blockSize.x), iceil(RES_Y, blockSize.y));

    camera = new Camera(from, at, up, fov, (float)RES_X / (float)RES_Y);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	glutInitWindowSize(RES_X, RES_Y);
	glutCreateWindow(windowTitle);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_ARB_pixel_buffer_object")) {
        printf("Required OpenGL extensions missing.");
        cleanup();

        getchar();
		return -1;
    }

    cudaInit();
    initPixelBuffer();

    glDisable(GL_DEPTH_TEST);
	glutReshapeFunc(reshape);
	glutDisplayFunc(drawScene);
	glutMouseFunc(mousePressed);
    glutMotionFunc(mouseMove);
    glutKeyboardFunc(keyboardKey);
    glutIdleFunc(idle);
    glutCloseFunc(cleanup);
	
	glutMainLoop();
    
	return 0;
}