#include "Camera.h"
#include <stdio.h>
#include <math.h>   
#include<iostream>
#include<cmath>
#include <string>
#include <cstring>
#include <cstddef>
#include <vector>
#include <sstream>
#include <iterator>
#include <fstream>
#include <stdexcept>   

#include "Vec3f.h"
#include "Camera.h"
#define off 0.5

using std::string;

Vec3f center;
float circRad = 0;
float angle;
float m_pi = 3.14159265358979323846f;
int resolution = 0;
int heightRes = 0;
std::vector< Vec3f > rayInit;

//initialize
Camera::Camera(float radius, float height, int resol, float ang){
	float radSq = pow(radius, 2);
	circRad = pow (2*radSq, 0.5);
	center = Vec3f(circRad,0,0);
	resolution = resol;
	heightRes = resol/2;
	angle = ((ang+270))*m_pi/180;					//degree in & radian stored
	startTrace(circRad, resol, ang,height);
}

//set up the start points for each ray in the pixel space
void Camera::startTrace(float radius,  int resol, float angle, float height){

	int r = 0;
	int c = 0;
	float spacing = (radius*2.0f)/resol;
	int newH = ceil(height/spacing);
	heightRes = newH;
	if(heightRes%2 == 1){
		heightRes = heightRes-1;
	}
	Vec3f TLCorner = Vec3f(center.x(),(center.y()+((heightRes/2)*spacing))-(spacing/2.0f),(center.z()-((resol/2)*spacing))+(spacing/2.0f));

	
	Vec3f current = Vec3f(TLCorner.x(),TLCorner.y(),TLCorner.z());
	
	while(r < heightRes){

		while(c < resol){
			current = Vec3f(TLCorner.x(),TLCorner.y()-(spacing*r),TLCorner.z()+(spacing*c));
			rayInit.push_back(current);
			c++;
		}
		r++;
		c = 0;
	}
}

//rotate points from initial location to current position
Vec3f Camera::rotatePoint(Vec3f current, float ang){
	Vec3f newP = current;
	newP.x((current.x()*cos(ang))+(current.z()*sin(ang)));
	newP.z((current.z()*cos(ang))-(current.x()*sin(ang)));
	return newP;
}

//get pixel at position r,c on the plane
Vec3f Camera::getRayPoint(int row, int col, float t){
	int offset = (row*resolution)+col;
	Vec3f current = rotatePoint(rayInit[offset], angle);

	Vec3f opposite = rayInit[offset];
	opposite = opposite - (center*2);

	opposite = rotatePoint(opposite, angle);

	Vec3f point = (current)*(1-t) + (opposite)*(t);
	
	return point;
}

//generate the image
void Camera::produceImage(double isoValue, string fName){

	double lightPositionX = 0; // light position initialized
	double lightPositionY = 0; // light position initialized
	double lightPositionZ = 100; // light position initialized
	
	double centerX = 0; // center of torus/ sphere
	double centerY = 0; // center of torus/ sphere
	double centerZ = 0; // center of torus/ sphere

	double radiusSphere = 30; // radius of sphere

	double outerRadiusTorus = 35; // outer radius of torus
	double innerRadiusTorus = 15; // inner radius of torus
	
	double Intensity; 
	FILE *f;
	unsigned char *img = NULL;
	int filesize = 54 + 3*resolution*heightRes;  
	if( img )
		free( img );
	img = (unsigned char *)malloc(3*resolution*heightRes);
	memset(img,0,sizeof(img));
	int x,y,r,g,b; 
	
	//bmp writing based on online resource, modified to fit values used in this program
	for(int i = 0; i < resolution; i++){ // for every pixel in the image plane (from 0......width)
		for(int j = 0; j < heightRes; j++){ // for every pixel in the image plane (from 0......height)

		
			Vec3f start = getRayPoint(j,i, 0);
			Vec3f end = getRayPoint(j,i, 1);
			//std::cout << start;
			//std::cout << "\n";
			double startRayX = start.x();
			double startRayY = start.y();
			double startRayZ = start.z();
			
			double endRayX = end.x();
			double endRayY = end.y();
			double endRayZ = end.z();

			

			Intensity = rayTracer(centerX, centerY, centerZ, startRayX, startRayY, startRayZ, endRayX, endRayY, endRayZ, lightPositionX, lightPositionY, lightPositionZ, outerRadiusTorus, innerRadiusTorus, radiusSphere, isoValue);
			x=i; y=j;//(resolution-1)-j;
			r = Intensity*170;
			g = Intensity*170;
			b = Intensity*170;
			if (r > 255) r=255;
			if (g > 255) g=255;
			if (b > 255) b=255;
			img[(x+y*resolution)*3+0] = (unsigned char)(b);
			img[(x+y*resolution)*3+1] = (unsigned char)(g);
			img[(x+y*resolution)*3+2] = (unsigned char)(r);
		}
	}
	
		unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
	unsigned char bmppad[3] = {0,0,0};

	bmpfileheader[ 2] = (unsigned char)(filesize    );
	bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);

	bmpinfoheader[ 4] = (unsigned char)(       resolution    );
	bmpinfoheader[ 5] = (unsigned char)(       resolution>> 8);
	bmpinfoheader[ 6] = (unsigned char)(       resolution>>16);
	bmpinfoheader[ 7] = (unsigned char)(       resolution>>24);
	bmpinfoheader[ 8] = (unsigned char)(       heightRes    );
	bmpinfoheader[ 9] = (unsigned char)(       heightRes>> 8);
	bmpinfoheader[10] = (unsigned char)(       heightRes>>16);
	bmpinfoheader[11] = (unsigned char)(       heightRes>>24);

	f = fopen(fName.c_str(),"wb");
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	for(int i=0; i<heightRes; i++)
	{
		fwrite(img+(resolution*(heightRes-i-1)*3),3,resolution,f);
		fwrite(bmppad,1,(4-(resolution*3)%4)%4,f);
	}
	fclose(f); 
}

//-----------------------

double Camera::rayTracer(double centerX,double centerY,double centerZ,
	double startRayX,double startRayY,double startRayZ,
	double endRayX, double endRayY, double endRayZ,
	double lightPositionX,double lightPositionY,double lightPositionZ,
	double outerRadiusTorus,double innerRadiusTorus, double radiusSphere, double isoValue){


	std::vector <double> lightPosition(3);
	lightPosition.at(0) = lightPositionX;
	lightPosition.at(1) = lightPositionY;
	lightPosition.at(2) = lightPositionZ;



	double Intensity;
	double param;
	double fx,fy,fz;


	param = bisectionMethod(isoValue, startRayX, startRayY, startRayZ, endRayX, endRayY, endRayZ, outerRadiusTorus, innerRadiusTorus,radiusSphere);
		
				
				fx = getCoord(param, startRayX, endRayX);
				fy = getCoord(param, startRayY, endRayY);
				fz = getCoord(param, startRayZ, endRayZ);

				//normal = sphereNormal(fx,fy,fz);
				Intensity = getIntensity(startRayX, startRayY, startRayZ, lightPositionX, lightPositionY, lightPositionZ, fx, fy, fz,outerRadiusTorus, innerRadiusTorus, radiusSphere);


			return Intensity;
}



double Camera::bisectionMethod(double isoValue, double sx, double sy, double sz, 
	double ex, double ey, double ez,
	double outerRadiusTorus, double innerRadiusTorus, double radiusSphere){

	double a0=0; // initially the ray starts from 0
		double b0 = a0; // b0 is the next step
		double fa0, fb0;
		double step = 0.0005; // gap of each step
		double found = 0; // flag to indicate whether there is root or not

		while(b0<=1){ // until reaches to the last of the ray, which is 1

			b0= b0 + step; // next step calculated
			
			// finding bracket between a0 and b0:
			fa0 = torusFunction( a0,  sx,  sy,  sz,	ex, ey,  ez, outerRadiusTorus,  innerRadiusTorus);
			fb0 = torusFunction( b0,  sx,  sy,  sz,	ex, ey,  ez, outerRadiusTorus,  innerRadiusTorus);

			// if root is on the steps exactly		
			if(fa0== 0){
				
				found = a0;
				break;
			}
			else if(fb0== 0){
				found = b0;
				break;			
			}

			// if one of them results -ve, that means there is a sign difference
			else if(fa0*fb0 < 0){

				// so there must a root between current [a0 and b0]
				// lets call rootfinder() to do the bisection			
				found = rootFinder(a0,b0, sx,  sy,  sz,	 ex,  ey,  ez,outerRadiusTorus,  innerRadiusTorus);
				break;
			}
			else{
				// if nothing found, no chance to get a root, so go for the next step
				a0 = b0;
				found = 0;
			}
		}
	return found;
}


double Camera::rootFinder(double a, double b,  double sx, double sy, double sz,
	double ex, double ey, double ez, double outerRadiusTorus , double innerRadiusTorus) 
{
	
	double f,fc;
	double c;
	double found=0.0;
	c = (a+b)/2; // do the bisection

	// get the function value
	f =  torusFunction( a,  sx,  sy,  sz,	ex, ey,  ez, outerRadiusTorus,  innerRadiusTorus);
	fc = torusFunction( c,  sx,  sy,  sz,	ex, ey,  ez, outerRadiusTorus,  innerRadiusTorus);
	
	if(fc >= 0-off && fc <= 0+off){ // if mid point satisfied to find the value 
		found = c;
	}
	else if(f*fc < 0){ // sign difference found

		b = c; // interchange
		found = rootFinder(a,b, sx,  sy,  sz,	 ex,  ey,  ez,outerRadiusTorus,  innerRadiusTorus);
	}
	else if (f*fc > 0) // sign difference found
	{
		a = c; // interchange
		found = rootFinder(a,b, sx,  sy,  sz,	 ex,  ey,  ez,outerRadiusTorus,  innerRadiusTorus);
	}
	return found;
}

double Camera::torusFunction(double t, double X1, double Y1, double Z1, double X2, double Y2, double Z2, double outerRadiusTorus, double innerRadiusTorus)
{
double f;

	double x = t*X2 + (1-t)*X1;
	double y = t*Y2 + (1-t)*Y1;
	double z = t*Z2 + (1-t)*Z1;


		f = pow( (outerRadiusTorus - sqrt(x*x+y*y)),2)+z*z - innerRadiusTorus*innerRadiusTorus;
	return f;
}


double Camera::getCoord(double t, double start, double end){

	return t*end + (1-t)*start;

}

std::vector<double> Camera::torusNormal(double x, double y, double z, double outerRadiusTorus, double innerRadiusTorus)
{
	std::vector<double> norm(3);
		norm.at(0) = 2*x - (2*outerRadiusTorus*x/sqrt(x*x+y*y));
		norm.at(1) = 2*y - (2*outerRadiusTorus*y/sqrt(x*x+y*y));
		norm.at(2) = 2*z;
	
	return norm;
}

double Camera::getIntensity(double startRayX, double startRayY, double startRayZ,
	double lightPositionX,double lightPositionY,double lightPositionZ,
	double fx, double fy, double fz,
	double outerRadiusTorus, double innerRadiusTorus, double radiusSphere){

	    std::vector <double> cameraPosition(3);
		std::vector <double> normal(3);
		std::vector <double> currentPoint(3);
		std::vector <double> lightPosition(3);

		double thetaLightCamera;
		double thetaLightNormal;
		double Intensity;

		normal = torusNormal(fx,fy,fz, outerRadiusTorus, innerRadiusTorus);

				
		cameraPosition.at(0) = startRayX;
		cameraPosition.at(1) = startRayY;
		cameraPosition.at(2) = startRayZ;
		
		currentPoint.at(0) = fx;
		currentPoint.at(1) = fy;
		currentPoint.at(2) = fz;

		lightPosition.at(0) = lightPositionX;
		lightPosition.at(1) = lightPositionY;
		lightPosition.at(2) = lightPositionZ;

		thetaLightCamera = findAngleLightCamera(currentPoint, lightPosition, cameraPosition);

		thetaLightNormal = findAngleLightNormal(currentPoint, lightPosition, normal);
		Intensity = (0.1 + 0.9*cos(thetaLightCamera) + 0.1*pow(cos(thetaLightNormal),3));

		return Intensity;
}


double Camera::findAngleLightCamera(std::vector<double> currentPoint, std::vector<double> lightPosition, std::vector<double> cameraPosition){
	double theta;

	std::vector <double> xVector(3);
	std::vector <double> yVector(3);


/*currentPoint.at(0) = currentPoint.at(0) - origin.at(0);
	currentPoint.at(1) = currentPoint.at(1) - origin.at(1);
	currentPoint.at(2) = currentPoint.at(2) - origin.at(2);*/

	xVector.at(0) = lightPosition.at(0) - currentPoint.at(0);
	xVector.at(1) = lightPosition.at(1) - currentPoint.at(1);
	xVector.at(2) = lightPosition.at(2) - currentPoint.at(2);

	yVector.at(0) = cameraPosition.at(0) - currentPoint.at(0);
	yVector.at(1) = cameraPosition.at(1) - currentPoint.at(1);
	yVector.at(2) = cameraPosition.at(2) - currentPoint.at(2);

	double a = xVector.at(0)*yVector.at(0) + xVector.at(1)*yVector.at(1) + xVector.at(2)*yVector.at(2);
	double b = (xVector.at(0)*xVector.at(0) + xVector.at(1)*xVector.at(1)+xVector.at(2)*xVector.at(2));
	double c = (yVector.at(0)*yVector.at(0) + yVector.at(1)*yVector.at(1)+yVector.at(2)*yVector.at(2));

	theta = acos (a/sqrt(b*c));
	//theta = a/sqrt(b*c+0.0001);
	return theta;

}


double Camera::findAngleLightNormal(std::vector<double> currentPoint, std::vector<double> lightPosition, std::vector<double> normal){
	double theta;

	std::vector <double> xVector(3);
	std::vector <double> yVector(3);

	/*U.at(0) =   U.at(0) - origin.at(0);
	U.at(1) =	U.at(1) - origin.at(1);
	U.at(2) =   U.at(2) - origin.at(2);
	V.at(0) = V.at(0) - origin.at(0);
	V.at(1) = V.at(1) - origin.at(1);
	V.at(2) = V.at(2) - origin.at(2);

	currentPoint.at(0) = currentPoint.at(0) - origin.at(0);
	currentPoint.at(1) = currentPoint.at(1) - origin.at(1);
	currentPoint.at(2) = currentPoint.at(2) - origin.at(2);*/


	xVector.at(0) = lightPosition.at(0) - currentPoint.at(0);
	xVector.at(1) = lightPosition.at(1) - currentPoint.at(1);
	xVector.at(2) = lightPosition.at(2) - currentPoint.at(2);

	yVector.at(0) = normal.at(0);
	yVector.at(1) = normal.at(1);
	yVector.at(2) = normal.at(2);

	double a = xVector.at(0)*yVector.at(0) + xVector.at(1)*yVector.at(1) + xVector.at(2)*yVector.at(2);
	double b = (xVector.at(0)*xVector.at(0) + xVector.at(1)*xVector.at(1)+xVector.at(2)*xVector.at(2));
	double c = (yVector.at(0)*yVector.at(0) + yVector.at(1)*yVector.at(1)+yVector.at(2)*yVector.at(2));

	theta = acos (a/sqrt(b*c));
	//theta = a/sqrt(b*c+0.0001);
	return theta;

}
