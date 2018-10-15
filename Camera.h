#ifndef CAMERA_H
#define CAMERA_H

#include <cassert>
#include <memory>
#include <initializer_list>
#include <array>
#include <functional>
#include <algorithm>
#include <iterator>
#include <iostream>

#include "Vec3f.h"
#include "Camera.h"

using std::string;

class Camera
{
public:
	//constructor
	explicit Camera();
	Camera(float radius, float height, int resol, float ang);
	
	//prepares the center of each pixel on the camera plane
	void startTrace(float radius, int resol, float angle, float height);
	
	//rotate a point by the given angle
	Vec3f rotatePoint(Vec3f current, float ang);
	
	//find any point on a given ray spanning the bounding volume (0,0 = top left corner of plane)
	Vec3f getRayPoint(int row, int col, float t);
	double getCoord(double t, double start, double end); // converts parameter 't' to X, Y and Z

	/* rayTracer():takes following parameter:
	1. X,Y,Z of the center of the torus or sphere (which is basically (0,0,0))
	2. X,Y,Z of the starting point of the ray
	3. X,Y,Z of the ending point of the ray
	4. X,Y,Z of the light source
	5. outer radius and inner radius of the torus 
	6. radius of the sphere if we want to trace sphere 

	finally the function returns intensity of the intersected point by calculating phong illumination
	Note: this code works on Torus...... sphere can be added later
	And this is the function where you will pass parameters for rotated rays.
	*/
	double rayTracer(double centerX,double centerY,double centerZ,
		double startRayX,double startRayY,double startRayZ,
		double endRayX, double endRayY, double endRayZ,
		double lightPositionX,double lightPositionY,double lightPositionZ,
		double outerRadiusTorus,double innerRadiusTorus, double radiusSphere, double isoValue);

	/*
	bisectionMethod(): returns the parameter of the ray for insected point
	*/

double bisectionMethod(double isoValue, double sx, double sy, double sz, 
	double ex, double ey, double ez,
	double outerRadiusTorus, double innerRadiusTorus, double radiusSphere);

double rootFinder(double a, double b,  double sx, double sy, double sz,
	double ex, double ey, double ez, double outerRadiusTorus , double innerRadiusTorus) ;

	/*
	torusFunction(): Implemeting torus equation
	*/
	double torusFunction(double t,double X1, double Y1, double Z1, double X2, double Y2, double Z2, double outerRadiusTorus, double innerRadiusTorus);
	std::vector<double> torusNormal(double x, double y, double z, double outerRadiusTorus, double innerRadiusTorus);


/*
getIntensity(): Do the Phong illumination for rayTracer()
*/
double getIntensity(double startRayX, double startRayY, double startRayZ,
	double lightPositionX,double lightPositionY,double lightPositionZ,
	double fx, double fy, double fz,
	double outerRadiusTorus, double innerRadiusTorus, double radiusSphere);


/*
findAngleLightCamera() and findAngleLightNormal(): Do angle calculation for Phong illumination at getIntensity()
*/
double findAngleLightCamera(std::vector<double> currentPoint, std::vector<double> lightPosition, std::vector<double> cameraPosition);
double findAngleLightNormal(std::vector<double> currentPoint, std::vector<double> lightPosition, std::vector<double> normal);

void produceImage(double isoValue, string fName);
};

std::ostream & operator<<( std::ostream &, const Camera & mat );

#endif // Camera