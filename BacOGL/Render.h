///
/// @file	Render.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

//#include "DisplayHandler.h"
//#include "PixelTransformer.h"
//#include "SphereGraphic.h"
#include "RotMatrix3D.h"
#include "Spectator.h"
//#include <omp.h>
//using namespace cimg_library;

///
/// @brief  This class acts as intermediate between user controls, GPU Rendering,
/// 		and the physical model in the Spectator class. 
///			In a previous iteration it used to perform the image rendering itself.
///
class Render
{
	//camera turn speed with keyboard controls
	const double CAMERA_STEP = M_PI / 800;

	//The physical model
	Spectator spec;
	//The 3x3 matrices required in the transformation pipeline, saved rowwise
	float* mat1, *mat2, *mat3;
	//The interpolation function representing the path of a 180° fan of lightrays
	float* rasterFun;
	//A factor used to perform speed abberation
	float psiFactor;
	//Key press state variables
	bool forwardKey, backKey, leftKey, rightKey, upKey, downKey;


	
	
public:
	///
	/// @brief Initialized the class
	/// @param rasterResolution How many light rays should be simulated
	/// @param lightStep 		The step size for light ray simulation
	/// @param R 				Event Horizon radius for the black hole
	/// @param surfaceR 		Radius of the texture sphere (which is rendered)
	/// @param rStart 			Starting distance of the observer
	///
	Render(const unsigned int rasterResolution = 2000, const double lightStep = M_PI / 100., const double R = 10, const double surfaceR = 500,
		const double rStart = 25) : spec(rasterResolution, lightStep, Vec3D{ rStart,0.,0. }, R, surfaceR)
	{
		mat1 = new float[9];
		mat2 = new float[9];
		mat3 = new float[9];
		rasterFun = new float[rasterResolution*2];
		psiFactor = 0;
		forwardKey = backKey = leftKey = rightKey = upKey = downKey = false;
		spec.startFreeFall(rStart * 2);
	}

	/// @brief Destructor
	~Render()
	{
		delete[] mat1;
		delete[] mat2;
		delete[] mat3;
		delete[] rasterFun;
	}

	///
	/// @brief Handles keyboard inputs
	///
	void control(const char command)
	{
		Vec2D cameraMove = {0,0};
		switch (command)
		{
		case 'w':
			forwardKey = true;
			break;
		case 's':
			backKey = true;
			break;
		case 'a':
			leftKey = true;
			break;
		case 'd':
			rightKey = true;
			break;
		case 'q':
			upKey = true;
			break;
		case 'e':
			downKey = true;
			break;
		case 't':
			cameraMove[1] = CAMERA_STEP;
			break;
		case 'g':
			cameraMove[1] = -CAMERA_STEP;
			break;
		case 'f':
			cameraMove[0] = CAMERA_STEP;
			break;
		case 'h':
			cameraMove[0] = -CAMERA_STEP;
			break;
		case '1':
			spec.schwarzschildMode();
			break;
		case '2':
			spec.startFreeFall(1e10);
			break;
		case '3':
			spec.startFixedFall();
			break;
		case '4':
			double readL;
			cout << "Enter spin:" << endl;
			cin >> readL;
			spec.startOrbit(readL);
			break;
		case 'z':
			unsigned int nrOfSteps;
			cout << "Enter number of steps:" << endl;
			cin >> nrOfSteps;
			spec.advanceInTime(nrOfSteps);
			break;
		case 'b':
			//printBenchmarks(cout);
			break;
		default:
			;
		
		}
	spec.controlCamera(cameraMove[0], cameraMove[1]);
	}

	///
	/// @brief Handles keys being release for movement inputs
	///
	void releaseButton(const unsigned char command)
	{
		switch (command)
		{
		case 'w':
			forwardKey = false;
			break;
		case 's':
			backKey = false;
			break;
		case 'a':
			leftKey = false;
			break;
		case 'd':
			rightKey = false;
			break;
		case 'q':
			upKey = false;
			break;
		case 'e':
			downKey = false;
			break;
		}
	}

	///
	/// @brief Turns the camera
	/// @param phi Horizontal angle
	/// @param theta Vertical angle
	///
	void moveCamera(const double phi, const double theta)
	{
		spec.controlCamera(phi, theta);
	}

	///
	/// @brief Returns if the system is at a singularity (event horizon or center of black hole)
	///
	bool isSingular() const 
	{
		return spec.isSingular();
	}

	///
	/// @brief prepares all data needed to render a frame and progresses any position simulations
	///
	void prepareData()
	{
		Vec3D controls;
		controls[0] = 0;
		controls[1] = 0;
		controls[2] = 0;
		if (forwardKey)
			controls[0] += 1;
		if (backKey)
			controls[0] += -1;
		if (leftKey)
			controls[1] += 1;
		if (rightKey)
			controls[1] += -1;
		if (upKey)
			controls[2] += 1;
		if (downKey)
			controls[2] += -1;
		spec.prepare(controls);

		double psi;
		spec.outputResults(mat1, mat2, mat3, psi, rasterFun);
		psiFactor = sqrtf((psi - 1) / psi);
	}

	///
	/// @brief Returns a pointer to the first Matrix (Display to movement direction)
	///
	const float* getMat1() const
	{
		return mat1;
	}

	///
	/// @brief Returns a pointer to the second Matrix (Movement direction, to central)
	///
	const float* getMat2() const
	{
		return mat2;
	}

	///
	/// @brief Returns a pointer to the third Matrix (Inverse central to sphere)
	///
	const float* getMat3() const
	{
		return mat3;
	}

	///
	/// @brief Returns a pointer to the interpolation function representing the path of a 180° fan of lightrays
	///
	const float* getRasterFun() const
	{
		return rasterFun;
	}

	///
	/// @brief Returns a pointer to the factor used to perform speed abberation
	/// 
	const float getPsiFactor() const
	{
		return psiFactor;
	}
};


