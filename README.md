# A very simple raytracer (fall 2014)

- This program takes in several user parameters and produces an offline image of the synthetic torus function.
- Slight shift is that y is height, z is depth and x is width
- Light source is at location (0,0,100), i.e. directly behind torus

Contribution of work:

Justin:
	- Camera creation
	- Ray trace setup
	- Command line arguments and user input

Jubair:
	- Synthetic Function
	- Bisection method
	- Lighting (Phong Illumination)
	- Bitmap writing code found from online resource, exact source unavailable at this time
	
Joint:
	- file output found and intially worked on by Jubair and modified by Justin to fit the code context 


command line arguments:

	-cameraAngle x			->	rotate camera around volume by x degrees. <Default is 0 degrees facing depth>
	-cameraResolution x		-> 	set resolution of the camera width to x. Expects an even value for resolution <default is 400>
	-isoValue x				->  define the isovalue being sought <defaults to 0>
	-outFile x				->  name the resulting image file x.bmp <default file name is result.bmp>. 
	-help					->  provide information on how to use system
	-paramFile x			->  opens the given file x and reads the 4 parameter values from it
									- order of parameters in file are resolution/angle/isoValue/outputFileName
									- only 4 lines in param file with no qualifier, just the values
	please remember that you must provide both a qualifier and a valid value/name after that qualifier for the correct behaviour
	non-invoked arguments are set to their default
	
To run:	
	-> make
	-> ./render [arguments]
	
Citations:
	- Vec3f was provided in CPSC 687 by TA Andrew Owens to simplifyy vector representation and calculation
	- skeletal structure of class organization and makefile format also from this class
	- Bitmap writing code found from online resource, exact source unavailable at this time
	
name of sample output files:

	30deg.bmp
	60deg.bmp
	90deg.bmp
	180deg.bmp
	270deg.bmp
	0deg.bmp

Limitation:
 - Different angle for camera does not generate correct output.
 
How to produce given images:

./render -cameraResolution 1024 -cameraAngle 30 -outFile 30deg
./render -cameraResolution 1024 -cameraAngle 60 -outFile 60deg
./render -cameraResolution 1024 -cameraAngle 90 -outFile 90deg
./render -cameraResolution 1024 -cameraAngle 180 -outFile 180deg
./render -cameraResolution 1024 -cameraAngle 270 -outFile 270deg
./render -cameraResolution 1024 -cameraAngle 0 -outFile 0deg
