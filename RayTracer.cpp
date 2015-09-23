#include <stdlib.h> 
#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h> 

#include "SceneLoader.h"

#include <cuda_gl_interop.h>
#include <helper_functions.h>

#define iDivUp(a, b) (a % b != 0) ? (a / b + 1) : (a / b)


Scene *scene = 0;
Camera *camera = 0;
Ray *d_rays = 0;
float3 *d_locals = 0, *d_reflectionCols = 0, *d_refractionCols = 0;

int fpsCount = 0;
int fpsLimit = 1;        // FPS limit for sampling

int RES_X = 512, RES_Y = 512;
dim3 blockSize(16, 16);
dim3 gridSize;

float latitude, longitude, radius;
int xDragStart, yDragStart, dragging, zooming;


bool stopRender = false;

// OpenGL pixel buffer object
GLuint pbo;

// OpenGL texture object
GLuint tex = 0;

// CUDA Graphics Resource (to transfer PBO)
struct cudaGraphicsResource *cuda_pbo = 0; 

StopWatchInterface *timer = NULL;

const char* windowTitle = "Msc Ray Tracing";

extern void deviceDrawScene(Sphere* shapes, size_t shapeSize, Light* lights, size_t lightSize, float3 backcolor, 
                            int resX, int resY, float width, float height, float atDistance, float3 xe, 
                            float3 ye, float3 ze, float3 from, float3 *d_output, dim3 gridSize, dim3 blockSize,
                            Ray* d_rays, float3* d_locals, float3* d_reflectionCols, float3* d_refractionCols);


float3 computeFromCoordinates(){
   float phi, theta;

   phi = (float) (latitude * DEG2RAD);
   theta = (float) (longitude * DEG2RAD);

   return make_float3(radius * sin(phi) * cos(theta), radius * sin(phi) * sin(theta), radius * cos(phi));
}

void cleanup() {
    delete scene;
    delete camera;

    checkCudaErrors(cudaFree(d_rays));
    checkCudaErrors(cudaFree(d_locals));
    checkCudaErrors(cudaFree(d_reflectionCols));
    checkCudaErrors(cudaFree(d_refractionCols));


    sdkDeleteTimer(&timer);

    if (pbo) {
        cudaGraphicsUnregisterResource(cuda_pbo);
        glDeleteBuffersARB(1, &pbo);
        glDeleteTextures(1, &tex);
    }

    checkCudaErrors(cudaDeviceReset());

    cudaDeviceReset();
}

void computeFPS() {
    fpsCount++;

    if (fpsCount == fpsLimit) {
        char fps[100];
        float ifps = 1.f / (sdkGetAverageTimerValue(&timer) / 1000.f);
        sprintf(fps, "%s: %3.1f fps", windowTitle, ifps);
        
        glutSetWindowTitle(fps);
        fpsCount = 0;

        fpsLimit = (int)MAX(1.0f, ifps);
        sdkResetTimer(&timer);
    }
}

void cudaInit() {
    clock_t start = clock();
    if(d_rays) {
        checkCudaErrors(cudaFree(d_rays));
        checkCudaErrors(cudaFree(d_locals));
        checkCudaErrors(cudaFree(d_reflectionCols));
        checkCudaErrors(cudaFree(d_refractionCols));
    }

    //size local and ray arrays 
    int raysPerPixel = (2 << MAX_DEPTH) - 1;
    int totalRays = RES_X * RES_Y * raysPerPixel;
    
    //size reflection and refraction arrays 
    int sizeRRArrays =  RES_X * RES_Y * ((2 << (MAX_DEPTH - 1)) - 1);
    

    Ray *rays = new Ray[totalRays]; 
    float3 *colors = new float3[totalRays];

    int size = totalRays * sizeof(Ray);

    checkCudaErrors(cudaMalloc((void**) &d_rays, size));
    checkCudaErrors(cudaMemcpy(d_rays, rays, size, cudaMemcpyHostToDevice));

    size = totalRays * sizeof(float3);
    checkCudaErrors(cudaMalloc((void**) &d_locals, size));
    checkCudaErrors(cudaMemcpy(d_locals, colors, size, cudaMemcpyHostToDevice));

    checkCudaErrors(cudaMalloc((void**) &d_reflectionCols, size));
    checkCudaErrors(cudaMemcpy(d_reflectionCols, colors, size, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMalloc((void**) &d_refractionCols, size));
    checkCudaErrors(cudaMemcpy(d_refractionCols, colors, size, cudaMemcpyHostToDevice));

    delete[] rays;
    delete[] colors;
    clock_t end = clock();
    std::cout << "cuda init time: " << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl;
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
    float3 *d_output;

    // map PBO to get CUDA device pointer
    checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pbo, 0));
    size_t num_bytes;
    checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void **)&d_output, &num_bytes, cuda_pbo));

    // call CUDA kernel, writing results to PBO
    deviceDrawScene(scene->getDShapes(), scene->getDShapesSize(), scene->getDLights(), scene->getDLightsSize(), 
                    scene->getBackcolor(), RES_X, RES_Y, camera->width, camera->height, camera->atDistance, 
                    camera->xe, camera->ye, camera->ze, camera->from, d_output, gridSize, blockSize, 
                    d_rays, d_locals, d_reflectionCols, d_refractionCols);

    cudaError_t error = cudaGetLastError();
    if(error != cudaSuccess) {
        std::cerr << cudaGetErrorString(error) << std::endl;
    }
    
    checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pbo, 0));
    
}

void reshape(int w, int h) {
    RES_X = w;
    RES_Y = h;

    // calculate new grid size
    gridSize = dim3(iDivUp(RES_X, blockSize.x), iDivUp(RES_Y, blockSize.y));
    
    camera->update(RES_X / (float)RES_Y);

    cudaInit();
    initPixelBuffer();
    

    glViewport(0, 0, RES_X, RES_Y);


    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

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

    //glutSwapBuffers();
    glutReportErrors();

    sdkStopTimer(&timer);
    computeFPS();
}

void mouseMove(int x, int y) { 	
    float ystep = 0.002f;
    float xstep = 0.002f;
    float zstep = 0.002f;

    if (dragging == 1) {
        longitude += (-x + xDragStart) * xstep;

        if(longitude > 360) {
            longitude -= 360;
        } else if(longitude < 0) {
            longitude = 360;
        }

        latitude += (-y + yDragStart) * ystep;

        if(latitude > 179){
            latitude = 179;
        } else if(latitude < 1) {
            latitude = 1;
        }

        camera->update(computeFromCoordinates());

    }
    if(zooming == 1) {
        radius += (y - yDragStart) * zstep;

        camera->update(computeFromCoordinates());
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

void idle() {
    glutPostRedisplay();
}

int main(int argc, char *argv[]) {
    sdkCreateTimer(&timer);
	std::string path = "../../resources/nffFiles/";

    scene = new Scene();
    
    radius = 3.0f;//3;
    longitude = 32.f;//132;
    latitude = 55.f;//55;
    
    float3 from = computeFromCoordinates();
    float3 up = make_float3(0.0f , 0.0f, 1.0f);
    float3 at = make_float3(0.0f);
    float fov = 45;

    // calculate new grid size
    gridSize = dim3(iDivUp(RES_X, blockSize.x), iDivUp(RES_Y, blockSize.y));

    camera = new Camera(from, at, up, fov, (float)RES_X / (float)RES_Y);

    //Explicitly set device 0 
    cudaSetDevice(0); 

	if (!load_nff(path + "balls_low", scene)) {
        std::cerr << "Could not find scene file." << std::endl;
		return -1;
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);

	glutInitWindowSize(RES_X, RES_Y);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(windowTitle);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_ARB_pixel_buffer_object")) {
        printf("Required OpenGL extensions missing.");
        exit(EXIT_SUCCESS);
    }

	glutReshapeFunc(reshape);
	glutDisplayFunc(drawScene);
	glutMouseFunc(mousePressed);
    glutMotionFunc(mouseMove);
    glutIdleFunc(idle);
    glutCloseFunc(cleanup);
	glDisable(GL_DEPTH_TEST);

	std::cout << std::endl << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;

	glutMainLoop();

	return 0;
}