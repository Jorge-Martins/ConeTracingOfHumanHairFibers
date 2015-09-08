#include <stdlib.h> 
#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h> 
#include <iostream>

#include "SceneLoader.h"

Scene *scene;
Camera *camera;

int fpsCount = 0;
int fpsLimit = 1;        // FPS limit for sampling

int RES_X, RES_Y;
float3 *finalImage;
float3 *d_finalImage;

float latitude, longitude, radius;
int xDragStart, yDragStart, dragging, zooming;

unsigned int frameCount = 0;
unsigned int render = 0;
bool drawing = false;

std::string balls_complexity = "low";

extern void drawScene(Shape **shapes, long shapeSize, Light* lights, long lightSize, Color backcolor, int resX,
                      int resY, float3 *d_finalImage);


float3 computeFromCoordinates(){
   float lat, longi;

   lat = (float) (latitude * DEG2RAD);
   longi = (float) (longitude * DEG2RAD);

   return make_float3(radius * sin(longi) * sin(lat), radius * cos(lat), radius * sin(lat) * cos(longi));
}

void reshape(int w, int h) {
    if(RES_X == w && RES_Y == h) {
        return;
    }

    cudaFree(d_finalImage);
    delete[] finalImage;

    RES_X = w;
    RES_Y = h;

    finalImage = new float3[RES_X * RES_Y];

    int size = RES_X * RES_Y * sizeof(float3);
    checkCudaErrors(cudaMalloc((void**) &d_finalImage, size));
    

    camera->update(RES_X, RES_Y);

	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

    gluOrtho2D(0, RES_X - 1, 0, RES_Y - 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

}

// Draw function by primary ray casting from the eye towards the scene's objects 
void drawScene() {
    int res = RES_X * RES_Y;
    int size = res * sizeof(float3);

    drawScene(scene->getDShapes(), scene->getDShapesSize(), scene->getDLights(), scene->getDLightsSize(), 
              scene->backcolor(), RES_X, RES_Y, d_finalImage);

    checkCudaErrors(cudaMemcpy(finalImage, d_finalImage, size, cudaMemcpyDeviceToHost));

    //Debug
    for(int i = 0; i < res; i++) {
        finalImage[i] = make_float3(1.0f);
    }

    glBegin(GL_POINTS);
	for(int i = 0; i < res; i++) {
        float3 pixel = finalImage[i];
      
        glColor3f(pixel.x, pixel.y, pixel.z);
		glVertex2i(i % RES_X, i / RES_X);
	}
	glEnd();
	glFlush();
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

void computeFPS() {
    frameCount++;
    fpsCount++;

    if (fpsCount == fpsLimit) {
        
    }
}

int main(int argc, char *argv[]) {
	std::string path = "../resources/nffFiles/";

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

	if (!load_nff(path + "balls_" + balls_complexity + ".nff", scene)) {
		std::cout << "Could not find scene files." << std::endl;
		return -1;
	}

	RES_X = camera->winX();
	RES_Y = camera->winY();
	std::cout << "ResX = " << RES_X << std::endl << "ResY = " << RES_Y << std::endl;

    finalImage = new float3[RES_X * RES_Y];

    int size = RES_X * RES_Y * sizeof(float3);
    cudaMalloc((void**) &d_finalImage, size);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);

	glutInitWindowSize(RES_X, RES_Y);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Msc Ray Tracing");
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	glutReshapeFunc(reshape);
	glutDisplayFunc(drawScene);
	glutMouseFunc(mousePressed);
    glutMotionFunc(mouseMove);
	glDisable(GL_DEPTH_TEST);

	std::cout << std::endl << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glutMainLoop();
	return 0;
}