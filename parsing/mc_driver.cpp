#include <cctype>
#include <fstream>
#include <cassert>
#include <sstream>
#include <string>

#include "mc_driver.hpp"
#include "../Scene.h"

MC::MC_Driver::~MC_Driver(){}

MC::MC_Driver::MC_Driver() : _scene(nullptr) {}

MC::MC_Driver::MC_Driver(Scene *scene) : _scene(scene) {}

void MC::MC_Driver::parse( const char *filename ) {
    assert( filename != nullptr );
    std::ifstream in_file( filename );
    if( ! in_file.good() ) exit( EXIT_FAILURE );

    std::string line;
    std::string::size_type sz;
    std::string token;

    while(std::getline(in_file, line)) {
        std::istringstream iss(line);
        
        std::getline(iss, token , ' ');

        if(token == "#") {
            //Ignore the rest of the line
        }

        else if(token == "v") {
            std::string vToken;

            float3 from, at, up;
            float angle, hither;
            int2 resolution;

            for(int i = 0; i < 6; i++) {
                std::getline(in_file, line);
                std::istringstream iss(line);

                std::getline(iss, vToken , ' ');

                if(vToken == "from") {
                    //x y z
                    float data[3];

                    for(int i = 0; i < 3; i++) {
                        std::getline(iss, vToken , ' ');
                        data[i] = std::stof(vToken, &sz);
                    }

                    from = make_float3(data[0], data[1], data[2]);
                } 
                
                else if(vToken == "at") {
                    //x y z
                    float data[3];

                    for(int i = 0; i < 3; i++) {
                        std::getline(iss, vToken , ' ');
                        data[i] = std::stof(vToken, &sz);
                    }

                    at = make_float3(data[0], data[1], data[2]);
                }

                else if(vToken == "up") {
                    //x y z
                    float data[3];

                    for(int i = 0; i < 3; i++) {
                        std::getline(iss, vToken , ' ');
                        data[i] = std::stof(vToken, &sz);
                    }

                    up = make_float3(data[0], data[1], data[2]);
                }

                else if(vToken == "angle") {
                    std::getline(iss, vToken , ' ');
                    angle = std::stof(vToken, &sz);
                }

                else if(vToken == "hither") {
                    std::getline(iss, vToken , ' ');
                    hither = std::stof(vToken, &sz);
                }

                else if(vToken == "resolution") {
                    std::getline(iss, vToken , ' ');
                    int x = std::stoi(vToken, &sz);

                    std::getline(iss, vToken , ' ');
                    int y = std::stoi(vToken, &sz);

                    resolution = make_int2(x, y);
                } 
                
                else {
                    print("token : " + vToken + " was not recognized inside v");
                }
            }

            add_view(from, at, up, angle, hither, resolution);
        }
        
        else if(token == "b") {
            //r g b
            float data[3];

            for(int i = 0; i < 3; i++) {
                std::getline(iss, token , ' ');
                data[i] = std::stof(token, &sz);
            }

            add_back_col(make_float3(data[0], data[1], data[2]));
        }

        else if(token == "l") {
            //x y z [r g b]
            float data[6];

            for(int i = 0; i < 3; i++) {
                std::getline(iss, token , ' ');
                data[i] = std::stof(token, &sz);
            }

            float3 position = make_float3(data[0], data[1], data[2]);

            if(std::getline(iss, token , ' ') && token == "[") {
                for(int i = 3; i < 6; i++) {
                    std::getline(iss, token , ' ');
                    data[i] = std::stof(token, &sz);
                }

                float3 color = make_float3(data[3], data[4], data[5]);

                add_light(position, color);

            } else {
                add_light(position);
            }
        }

        else if(token == "f") {
            // (r, g, b), diffuse, specular, shine, trans, ior
            float data[8];

            for(int i = 0; i < 8; i++) {
                std::getline(iss, token , ' ');
                data[i] = std::stof(token, &sz);
            }

            add_material(make_float3(data[0], data[1], data[2]), data[3], data[4], data[5], data[6], data[7]);
        }

        else if(token == "c") {
            std::string cToken;

            //base top
            float4 cyllinder[2];

            //x y z radius
            float data[4];

            for(int i = 0; i < 2; i++) {
                std::getline(in_file, line);
                std::istringstream iss(line);

                for(int y = 0; i < 4; i++) {
                    std::getline(iss, cToken , ' ');
                    data[y] = std::stof(cToken, &sz);
                }

                cyllinder[i] = make_float4(data[0], data[1], data[2], data[3]);
            }
        }

        else if(token == "s") {
            // x y z radius
            float data[4];

            for(int i = 0; i < 4; i++) {
                std::getline(iss, token , ' ');
                data[i] = std::stof(token, &sz);
            }

            add_sphere(make_float3(data[0], data[1], data[2]), data[3]);
        }

        else if(token == "gpl") {
            float data[9];

            for(int i = 0; i < 9; i++) {
                std::getline(iss, token , ' ');
                data[i] = std::stof(token, &sz);
            }

            add_plane(make_float3(data[0], data[1], data[2]),
                      make_float3(data[3], data[4], data[5]),
                      make_float3(data[6], data[7], data[8]));
        }

        else if(token == "p") {
            float data[3];
            std::string pToken;
            std::vector<float3> poligon;

            std::getline(iss, token , ' ');
            int numberOfTriangles = std::stoi(token, &sz);

            for(int nt = 0; nt < numberOfTriangles; nt++) {
                std::getline(in_file, line);
                std::istringstream iss(line);

                for(int i = 0; i < 3; i++) {
                    std::getline(iss, pToken , ' ');
                    data[i] = std::stof(pToken, &sz);
                }

                poligon.push_back(make_float3(data[0], data[1], data[2]));
            }

            add_poly(numberOfTriangles, poligon);
        }

        else if(token == "pp") {
            float data[3];
            std::string ppToken;
            std::vector<float3> vertsNormals[2];

            std::getline(iss, token , ' ');
            int nVerts = std::stoi(token, &sz);

            for(int nv = 0; nv < nVerts; nv++) {
                std::getline(in_file, line);
                std::istringstream iss(line);

                for(int vIndex = 0; vIndex < 2; vIndex++) {
                    for(int i = 0; i < 3; i++) {
                        std::getline(iss, ppToken , ' ');
                        data[i] = std::stof(ppToken, &sz);
                    }

                    vertsNormals[vIndex].push_back(make_float3(data[0], data[1], data[2]));
                }
            }

            add_poly_patch(nVerts, vertsNormals[0], vertsNormals[1]);
        } 

        else {
            print("token : " + token + " was not recognized");
        }
    }

    in_file.close();
}

void
MC::MC_Driver::add_view(float3 from, float3 at, float3 up, float angle, float hither, int2 res)
{
	//print("Saw a view");
	// _scene->add_view(eye, center, upDir, angle, hither, resolution);
}
void
MC::MC_Driver::add_back_col(float3 color)
{
	//print("Saw background");
	_scene->setBackcolor(color);
}
void
MC::MC_Driver::add_light(float3 position)
{
	//print("Saw light");
	_scene->addLight(position);
}
void
MC::MC_Driver::add_light(float3 position, float3 color)
{
	//print("Saw ligh + col");
	_scene->addLight(position, color);
}
void
MC::MC_Driver::add_material(float3 color, float diff, float spec, float shine, float t, float ior)
{
	//print("Saw material");
	_scene->setMaterial(color, diff, spec, shine, t, ior);
}
void
MC::MC_Driver::add_cylinder(float4 base, float4 top)
{
	//print("Saw cylinder");
	_scene->addCylinder(base,top);
}
void
MC::MC_Driver::add_sphere(float3 center, float radius)
{
	//print("Saw sphere");
	_scene->addSphere(center, radius);
}
void
MC::MC_Driver::add_poly(int nVerts, std::vector<float3> verts)
{
	//print("Saw poly");
	if (nVerts == verts.size()) {
        if(nVerts == 3) {
		    _scene->addTriangle(verts);
        } else {
            //TODO
            print("TODO poly");
        }
	}
	else {
		print("Num verts did not match... :(");
	}
}
void
MC::MC_Driver::add_poly_patch(int nVerts, std::vector<float3> verts, std::vector<float3> normals)
{
	//print("Saw poly patch");
	if (verts.size() == normals.size() && verts.size() == nVerts) {
		_scene->addPolyPatch(nVerts, verts, normals);
	}
	else {
		print("Num verts did not match... :(");
	}
}
void
MC::MC_Driver::add_plane(float3 v1, float3 v2, float3 v3)
{
	//print("Saw plane");
	_scene->addPlane(v1, v2, v3);
}

void
MC::MC_Driver::print(std::string msg)
{
	std::cout << msg << std::endl;
}