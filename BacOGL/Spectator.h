///
/// @file	Spectator.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

#include <math.h>
#include "RotMatrix3D.h"
#include "RasterFunction.h"
#include "GeodesicSolver.h"
#include "TransUtilities.h"
#include "Orbit.h"
//#include "Benchmark.h"

///
/// @brief Represents the physical state of the simulation and stores essential calculation results
/// Also responsible for the position of the observer
/// 
class Spectator
{
	//The four possible states for the simulation
	//	SCHWARZSCHILD is an observer with no velocity in space, base to render the other modes (note unintuitive behavior within the event horizon)
	//	FREE_FALLING simulates a frozen snapshot of the observer falling from far away with free choice of position
	//	FIXED_FALLING simulates a fall in real time
	//	FIXED_ORBIT siluates an orbit in real time
	enum SpectatorMode { SCHWARZSCHILD, FREE_FALLING, FIXED_FALLING, FIXED_ORBIT };
	SpectatorMode mode;										//Active Spectator mode
	const double MOVEMENT_STEP = .0501;						//movement speed for user controls in euclidean space
	const double VERTICAL_RESTRICTION = M_PI_2 - 0.01;		//camera constraint
	const double SINGULARITY_BOUND = 1e-10;					//Stop the simulation in a surrounding of singular points

	//Information
	Vec3D position;		//Observer position in polar(r, phi, theta) coordinates 
	Vec3D velocity;		//Velocity of the observer in 4D space-time, restricted to the orbit plane (t, r, phi)
	Vec2D camera; 		//polar direction of camera
	double E;			//Energy of the Observer
	double schwarzR;	//Black hole Schwarzschild radius
	double surfaceR;	//Radius of the texture sphere
	double tau;			//Subjective time
	double timeStep;	//Step size for the simulation
	bool rFalling;		//Wether the observer moves towards the black hole
	bool surfaceBounce;	//Allows to elastically bounce off the texture sphere
	

	//output variables
	//Very involved transformations, explained in Bachelor thesis
	RasterFunction180 geodesics;	//Stores where light rays hit the texture sphere depending on the direction
	RotMatrix3D displayToMovement;	//Rotates from a display oriented system into one aligned with the direction of movement
	RotMatrix3D movementToAxis;		//Further rotates towards the center of the black holes
	RotMatrix3D axisToSurface;		//Rotates on the texture sphere towards the texture coordinates
	double psi;						//parameter to perform the velocity abberation

	//dependent values
	Vec3D carthesicPosition;		//observer position in carthesic coordinates
	bool singularityFlag;			//If the system is singular
	bool r0Reached;					//If close to the true singularity of r=0

	
	double lastR, nextR;			//variables for the central falling verlet solver

	//Parts of displayToMovement
	RotMatrix3D yXRotator, displayToStandard,
		standardToAxis, orbitPlaneTilt, tiltedAxisToMovement1;

	//Parts of movementToAxis
	RotMatrix3D movementToTiltedAxis2, orbitPlaneUntilt;

	//Correction on Sphere for geodesic transformation
	RotMatrix3D yInverse;

	//Utilities
	GeodesicSolver solver;		//solves the light ray traces
	TransUtilities matUtil;		//Matrix manipulations
	Orbit orbit;				//Represents the orbit the observer is on
	//Benchmark* geodesicBench,* matricesBench;


	///
	/// @brief Checks if the system is singular and sets the flags
	///
	void singularityChecks()
	{
		r0Reached = schwarzR && position[0] / schwarzR < SINGULARITY_BOUND;
		singularityFlag = schwarzR && (r0Reached ||
			fabs(position[0] / schwarzR - 1) < SINGULARITY_BOUND);
		if (position[0] <= schwarzR)
			rFalling = true;
	}

	///
	/// @brief h(r) is a commonly needed quantity
	///
	double h()
	{
		return 1 - schwarzR / position[0];
	}

	///
	/// @brief calculates the velocity in the "non moving" mode
	///
	void schwarzschildSpectator()
	{
		//outside only velocity in t direction
		if (position[0] > schwarzR)
		{
			velocity[0] = 1 / sqrt(h());
			velocity[1] = 0;
		}
		//inside only velocity in -r direction (the future)
		else
		{
			velocity[0] = 0;
			velocity[1] = -1 * sqrt(-h());
		}
		velocity[2] = 0;
	}

	///
	/// @brief calculates the velocity if the observer is falling without angular velocity
	/// If the distance is higher than the start of the fall, it defaults to no movement
	///
	void centralFallingSpectator()
	{
		if (position[0] == schwarzR)
			return;
		if (E * E < h())
			return schwarzschildSpectator();
		velocity[0] = E / h();
		velocity[1] = -sqrt(E * E - h()) * (rFalling ? 1 : -1);
		velocity[2] = 0;
	}

	///
	/// @brief Adjusts the position according to user inputs or the simulation depending on mode
	/// @param desiredDirection First two coordinates determine movement in the horizontal plane wrt. the camera
	///							Last coordinate is movement along the absolute vertical axis
	///
	void doStep(const Vec3D & desiredDirection)
	{
		switch (mode)
		{
		case FREE_FALLING:		//Moving into the desired direction
		case SCHWARZSCHILD:
			matUtil.toCarthesic(position, carthesicPosition);
			carthesicPosition[2] += desiredDirection[2] * MOVEMENT_STEP;
			carthesicPosition[0] += (cos(camera[0]) * desiredDirection[0]
				- sin(camera[0]) * desiredDirection[1])	* MOVEMENT_STEP;
			carthesicPosition[1] += (sin(camera[0]) * desiredDirection[0]
				+ cos(camera[0]) * desiredDirection[1])	* MOVEMENT_STEP;
			matUtil.toPolar(carthesicPosition, position);
			break;
		case FIXED_FALLING:		//Following the central geodesic for one time step
			if (!r0Reached)
				doCentralVerletStep();
			break;
		case FIXED_ORBIT:		//Making one time step on the orbit
			if(!r0Reached)
				orbit.doStep(timeStep);
			break;
		default:
			;
		}
		singularityChecks();
	}

	///
	/// @brief Performs one time step for falling spectator without angular velocity with Verlet integration
	///
	void doCentralVerletStep()
	{
		nextR = 2 * position[0] - lastR - timeStep * timeStep * schwarzR /
			(2 * position[0] * position[0]);
		lastR = position[0];
		position[0] = nextR;
		tau += timeStep;
 	}

	///
	/// @brief updates velocity according to new position
	///
	void updateSpectator()
	{
		switch (mode)
		{
		case SCHWARZSCHILD:
			schwarzschildSpectator();
			break;
		case FIXED_FALLING:
		case FREE_FALLING:
			centralFallingSpectator();
			break;
		case FIXED_ORBIT:
			orbit.readOrbitSpectator(velocity);
			break;
		default:
			;
		}
	}
public:
	//TODO , , , modeChangers, getters,

	///
	/// @brief Constructor, initializes important components
	/// @param rasterResolution number of light rays
	/// @param step step size in the light ray simulation
	/// @param inPosition The starting position for the observer
	/// @param inSchwarzR The Schwarzschild radius
	/// @param inSurfaceR Radius of the texture-sphere
	/// @param phi, theta initial camera orientation
	/// @param inTimeStep simulation speed
	/// @param inBounce If the observer should bounce off the texture-sphere
	///
	Spectator(unsigned int rasterResolution, double step, Vec3D inPosition, double inSchwarzR, double inSurfaceR, double phi = M_PI,
		double theta = 0, double inTimeStep = 1./60, bool inBounce = false) :
		geodesics(rasterResolution), position(), velocity(),
		orbit(&position), solver(step)
	{
		position[0] = 10;
		surfaceBounce = false;
		mode = SCHWARZSCHILD;
		tau = 0;
		camera[0] = M_PI;
		camera[1] = 0;

		Vec3D first = { 0,-1,0 };
		Vec3D second = { 1,0,0 };
		Vec3D third = { 0,0,1 };
		yXRotator = RotMatrix3D(first, second, third);
		yInverse = RotMatrix3D(second, first, third);

		setPositionRadius(inPosition, inSchwarzR);
		setSurfaceRadius(inSurfaceR);
		setCamera(phi, theta);
		setPace(inTimeStep);
		setBounce(inBounce);
	}

	//Destructor has nothing to do
	~Spectator() {}

	///
	/// @brief sets a position and the Schwarzschild radius
	///
	void setPositionRadius(const Vec3D inPosition, const double inSchwarzR)
	{
		position[0] = fabs(inPosition[0]);
		position[1] = inPosition[1];
		position[2] = inPosition[2];
		schwarzR = inSchwarzR;
		singularityChecks();
	}


	///
	/// @brief Sets the radius of the texture-sphere
	///
	void setSurfaceRadius(const double inSurfaceR) { surfaceR = inSurfaceR; }

	///
	/// @brief Sets the camera orientation
	///
	void setCamera(const double phi, const double theta)
	{
		camera[0] = phi;
		camera[1] = theta;
	}

	///
	/// @brief Sets the speed of the simulation
	///
	void setPace(const double inTimeStep) { timeStep = inTimeStep; }

 	///
	/// @brief Sets if a fixed falling spectator should bounce off the texture-sphere
	///
	void setBounce(const bool inBounce) { surfaceBounce = inBounce; }

	///
	/// @brief Returns if the simulation is at a singular point
	///
	bool isSingular() const { return singularityFlag; }  

	///
	/// @brief Starts a new orbit from the current position
	/// @param inL The angular momentum for the orbit
	///
	void startOrbit(const double inL)
	{
		Vec3D direction = { -sin(position[1]),cos(position[1]),0 };
		bool success = orbit.initNewOrbit(direction, inL, schwarzR);
		if(success)
		{
			mode = FIXED_ORBIT;
			cout << "The orbit is " << orbit.isStable() << endl;
		}
		else
			cout << "Cannot initialize orbit within the black hole." << endl;
	}

	///
	/// @brief enters the FREE_FALLING mode, see SpectatorMode for details
	/// @param startOfFall The distance from the black hole, from where the fall started.
	///					   By default the current position.
	///					   Needs to be outside the event horizon
	///
	void startFreeFall(double startOfFall = 0)
	{
		if (startOfFall == 0)
			startOfFall = position[0];
		if (startOfFall <= schwarzR * (1 + SINGULARITY_BOUND))
			return;
		mode = FREE_FALLING;
		E = sqrt(1 - schwarzR / startOfFall);
		rFalling = true;
	}

	///
	/// @brief Starts a simulated fall from the current position.
	/// 		Current position needs to be outside the event horizon
	///
	void startFixedFall()
	{
		if (position[0] <= schwarzR * (1 + SINGULARITY_BOUND))
			return;
		mode = FIXED_FALLING;
		E = sqrt(h());
		lastR = position[0];
		position[0] = lastR - timeStep * timeStep * schwarzR /	//Verlet scheme initializing
			(4 * lastR * lastR);
		tau += timeStep;
		singularityChecks();
	}

	///
	/// @brief Switches to the mode without velocity in space
	///
	void schwarzschildMode()
	{
		mode = SCHWARZSCHILD;
	}

	///
	/// @brief Fast forwards the position simulation without rendering
	/// @param numberOfSteps The amount of timesteps to skip
	///
	void advanceInTime(const unsigned int numberOfSteps)
	{
		if (mode == FREE_FALLING || mode == SCHWARZSCHILD)
			return;
		Vec3D dummy = { 0,0,0 };
		for (unsigned int i = 0; i < numberOfSteps; i++)
			doStep(dummy);
	}

	///
	/// @brief Performs all calculations to advance one step in time and prepares all transformation parameters/matrices
	/// @param desiredDirection Input to move the camera, see doStep()
	///
	void prepare(const Vec3D & desiredDirection)
	{
		//updating the position
		doStep(desiredDirection);
		
		//Stop optical calculations if the position is singular, only update the camera
		if (singularityFlag)
		{
			matUtil.yieldTransMatrix(camera, displayToStandard);
			displayToMovement.setToIdentity();
			displayToMovement.multiplyOnto(tiltedAxisToMovement1);
			displayToMovement.multiplyOnto(orbitPlaneTilt);
			displayToMovement.multiplyOnto(standardToAxis);
			displayToMovement.multiplyOnto(displayToStandard);
			displayToMovement.multiplyOnto(yXRotator);
			return;
		}
			
		//updates velocity for further calculations
		updateSpectator();
		
		//Solving the geodesics for the Schwarzschild spectator
		solver.solveGeodesic(schwarzR, position[0], surfaceR, geodesics);

		//Calculating transformation matrices
		matUtil.yieldTransMatrix(camera, displayToStandard);

		Vec2D axis = { position[1], position[2] };
		matUtil.yieldTransMatrix(axis, axisToSurface);

		Vec2D invAxis = { axis[0] + M_PI, -axis[1] };
		matUtil.yieldTransMatrix(invAxis, standardToAxis);
		standardToAxis.transpose();

		double r = position[0];
		//Angle between central vector and direction of movement in two different frames of reference
		double planeAngle1, planeAngle2;
		
		//Calculate velocity abberation factor relative to default/Schwarzschild observer
		psi = r > schwarzR ? velocity[0] * velocity[0] * h()
			: -velocity[1] * velocity[1] / h();
		if ((psi - 1) < SINGULARITY_BOUND)
			psi = 1;
		if (mode == FIXED_ORBIT)
		{
			planeAngle1 = acos(-velocity[0] * velocity[1] * (r > schwarzR ? 1 : -1) /
				sqrt((1 + r*r*velocity[2] * velocity[2]) * (psi * (psi - 1))));
			if (r > schwarzR)
			{
				
				planeAngle2 = acos(-velocity[1] / sqrt((psi -1) * h()));
			}
			else
			{
				planeAngle2 = acos(-velocity[0] * sqrt(-h() / (psi - 1)));
			}
			matUtil.yieldTiltMatrix(orbit.tiltCorrectionAngle(), orbitPlaneTilt);
			matUtil.yieldPlaneRotMatrix(planeAngle1, tiltedAxisToMovement1);
			matUtil.yieldPlaneRotMatrix(planeAngle2, movementToTiltedAxis2);
		}
		else
		{
			orbitPlaneTilt.setToIdentity();
			tiltedAxisToMovement1.setToIdentity();
			movementToTiltedAxis2.setToIdentity();
		}
		orbitPlaneUntilt = orbitPlaneTilt;
		//Invert rotations by transposing
		orbitPlaneUntilt.transpose();
		movementToTiltedAxis2.transpose();

		//Composing displayToMovement
		displayToMovement.setToIdentity();
		displayToMovement.multiplyOnto(tiltedAxisToMovement1);
		displayToMovement.multiplyOnto(orbitPlaneTilt);
		displayToMovement.multiplyOnto(standardToAxis);
		displayToMovement.multiplyOnto(displayToStandard);
		displayToMovement.multiplyOnto(yXRotator);

		//Composing movementToAxis
		movementToAxis.setToIdentity();
		movementToAxis.multiplyOnto(orbitPlaneUntilt);
		movementToAxis.multiplyOnto(movementToTiltedAxis2);

		//Adjusting for inverted y-coordinate
		axisToSurface.multiplyOnto(yInverse);
	}

	///
	/// @brief Turns the camera by specified directions
	/// @note Angles in radiant
	///
	void controlCamera(const double horizontalAngle, const double verticalAngle)
	{
		camera[0] += horizontalAngle;
		camera[1] += verticalAngle;
		//Limit vertical camera angle
		camera[1] = fmin(fmax(camera[1], -VERTICAL_RESTRICTION), VERTICAL_RESTRICTION);
		//Truncate large angles
		if (fabs(camera[0]) > 15)
			camera[0] = fmod(camera[0], 2*M_PI);
	}

	///
	/// @brief Outputs all the raw parameters to be used in the transformation pipeline (on the GPU)
	///
	void outputResults(float* firstMatrix, float* secondMatrix,
		float* thirdMatrix, double & psiOut, float* lightRayRaster) const 
	{
		displayToMovement.toFloatArray(firstMatrix);
		movementToAxis.toFloatArray(secondMatrix);
		axisToSurface.toFloatArray(thirdMatrix);
		psiOut = psi;
		geodesics.toFloatArray(lightRayRaster);
	}
};