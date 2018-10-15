// Justin Kelly
// CPSC 687
// 10034429
// uses Andrew Owens code (as comment shows) as base
// nearly all code from 687 has been removed. only the shell and Vec3f
// are still used here as a base and utility

#include<iostream>
#include<cmath>
#include <string>
#include <cstddef>
#include <vector>
#include <sstream>
#include <iterator>
#include <fstream>
#include <stdexcept>

#include "Vec3f.h"
#include "Camera.h"

using std::cout;
using std::endl;
using std::cerr;
using std::string;

int resolutionM = 400;//256;
float angleM = 0;
float isoValue = 0.0f;
string outputFile = "result.bmp";
Camera cam = Camera(1,1.0f,0,0.0f);

int main( int argc, char** argv )
{

	//look for qualifier arguments to set isoValue, resolution of camera, angle of camera and output name
	int i = 0;
	while(i < argc){
		if (string(argv[i]).compare("-cameraAngle") == 0){
			angleM = atof(argv[i+1]);
			std::cout << "Camera angle set to: ";
			std::cout << angleM;
			std::cout << "\n";
		}
		else if (string(argv[i]).compare("-cameraResolution") == 0){
			resolutionM = atoi(argv[i+1]);
			std::cout << "Camera resol set to: ";
			std::cout << resolutionM;
			std::cout << "\n";
		}
		else if (string(argv[i]).compare("-isoValue") == 0){
			isoValue = atof(argv[i+1]);
			std::cout << "Obj isoValue set to: ";
			std::cout << isoValue;
			std::cout << "\n";
		}
		else if (string(argv[i]).compare("-outFile") == 0){
			outputFile = argv[i+1];
			outputFile = outputFile + ".bmp";
			std::cout << "output file set to:  ";
			std::cout << outputFile;
			std::cout << "\n";
		}
		else if (string(argv[i]).compare("-help") == 0){
			std::cout << "This system takes up to 4 argument pairs: \\n";
			std::cout << "\t-cameraAngle <degrees> \n";
			std::cout << "\t-isovalue <desiredValue>\n";
			std::cout << "\t-outfile <name>\n";
			std::cout << "\t-cameraResolution <even valued integer>\n";
			std::cout << "Or one parameter file:\n";
			std::cout << "\t -paramFile <name>\n";
		}
		else if (string(argv[i]).compare("-paramFile") == 0){
			string param = argv[i+1];
			string line;
			//from http://www.cplusplus.com/doc/tutorial/files/
			std::ifstream myfile(param);
			getline (myfile,line);
			resolutionM = atoi(line.c_str());
			std::cout << "Camera resol set to: ";
			std::cout << resolutionM;
			std::cout << "\n";
			
			getline (myfile,line);
			angleM = atof(line.c_str());
			std::cout << "Camera angle set to: ";
			std::cout << angleM;
			std::cout << "\n";
			
			getline (myfile,line);
			isoValue = atof(line.c_str());
			std::cout << "Obj isoValue set to: ";
			std::cout << isoValue;
			std::cout << "\n";
			
			getline (myfile,line);
			outputFile = line+ ".bmp";
			std::cout << "output file set to:  ";
			std::cout << outputFile;
			std::cout << "\n";
			myfile.close();
		}
		i++;
	}

	//create camera uses bounds for synthetic torus with inner radius 15 and outer radius 35
	cam = Camera(40,100.0f,resolutionM,angleM);
	
	//generate the image of the torus
	cam.produceImage(isoValue, outputFile);
	return 0;
}

