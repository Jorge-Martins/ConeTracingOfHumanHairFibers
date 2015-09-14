#include <stdlib.h> 
#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h> 

#include "SceneLoader.h"

#include <cuda_gl_interop.h>
#include <helper_functions.h>

Scene *scene;
Camera *camera;

int fpsCount = 0;
int fpsLimit = 1;        // FPS limit for sampling

int RES_X, RES_Y;

float latitude, longitude, radius;
int xDragStart, yDragStart, dragging, zooming;


bool drawing = false;

std::string balls_complexity = "low";

// OpenGL pixel buffer object
GLuint pbo;

// OpenGL texture object
GLuint tex = 0;

// CUDA Graphics Resource (to transfer PBO)
struct cudaGraphicsResource *cuda_pbo = 0; 

StopWatchInterface *timer = NULL;

const char* windowTitle = "Msc Ray Tracing";

extern void deviceDrawScene(Shape **shapes, size_t shapeSize, Light* lights, size_t lightSize, 
                            Color backcolor, int resX, int resY, float4 *d_output);


float3 computeFromCoordinates(){
   float lat, longi;

   lat = (float) (latitude * DEG2RAD);
   longi = (float) (longitude * DEG2RAD);

   return make_float3(radius * sin(longi) * sin(lat), radius * cos(lat), radius * sin(lat) * cos(longi));
}

void cleanup() {
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
    glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, RES_X * RES_Y * sizeof(float4), 0, GL_STREAM_DRAW_ARB);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

    // register this buffer object with CUDA
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_pbo, pbo, cudaGraphicsMapFlagsWriteDiscard));

    // create texture for display
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, RES_X, RES_Y, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

//Function that calls the kernel
void render() {
    float4 *d_output;

    // map PBO to get CUDA device pointer
    checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pbo, 0));
    size_t num_bytes;
    checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void **)&d_output, &num_bytes, cuda_pbo));

    // call CUDA kernel, writing results to PBO
    deviceDrawScene(scene->getDShapes(), scene->getDShapesSize(), scene->getDLights(), scene->getDLightsSize(), 
                  scene->backcolor(), RES_X, RES_Y, d_output);

    getLastCudaError("render_kernel failed");

    checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pbo, 0));
    
}

void reshape(int w, int h) {
    RES_X = w;
    RES_Y = h;

    camera->update(RES_X, RES_Y);
    initPixelBuffer();

    glViewport(0, 0, w, h);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

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
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, RES_X, RES_Y, GL_RGBA, GL_FLOAT, 0);
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
    float ystep = 0.006f;
    float xstep = 0.008f;
    float zstep = 0.004f;

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

    }
    glutPostRedisplay();
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
   glutPostRedisplay();
}

void idle() {
    glutPostRedisplay();
}

int main(int argc, char *argv[]) {
    sdkCreateTimer(&timer);
	std::string path = "../../resources/nffFiles/";

    scene = new Scene();
    
    radius = 2;
    longitude = 0;
    latitude = 45;

    float3 from = computeFromCoordinates();
    float3 up = make_float3(0,1,0);
    float3 at = make_float3(0.0f);
    float fov = 45;
    RES_X = RES_Y = 512;

    camera = new Camera(from, at, up, fov, RES_X, RES_Y);

    //Explicitly set device 0 
    cudaSetDevice(0); 

	if (!load_nff(path + "balls_" + balls_complexity + ".nff", scene)) {
		std::cout << "Could not find scene files." << std::endl;
		return -1;
	}

	RES_X = camera->winX();
	RES_Y = camera->winY();
	std::cout << "ResX = " << RES_X << std::endl << "ResY = " << RES_Y << std::endl;


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

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


    initPixelBuffer();

	std::cout << std::endl << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;

	glutMainLoop();

	return 0;
}