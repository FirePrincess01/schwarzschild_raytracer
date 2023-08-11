///
/// @file	Orbit.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

#include <math.h>
#include "RotMatrix3D.h"
#include "TransUtilities.h"
#include <string>

///
/// @brief Represents the position on an orbit around the black hole and simulates movement on it.
/// @note Initialize an orbit before using the other functions.
///
class Orbit
{
	//Black hole radius
	double schwarzR;
	//Reference to polar position in the Spectator class
	Vec3D* positionRef;
	//Wether the orbit has been initialized
	bool ready;
	
	TransUtilities util;

	//Constant properties for each orbit
	//Where the orbit intersects the horizontal plane from below
	double firstPhi;
	//Angle between orbit and horizontal plane
	double tiltAngle;
	//Transformation of the orbit plane into the horizontal plane
	RotMatrix3D planeTilt;
	//Energy and angular momentum of the orbit
	double E, L;

	//Radius of the position
	double r;
	//Inverted Radius and its derivative
	double u, uBar;
	//Position angle inside the orbit plane, starting from the position of firstPhi
	double orbitAngle;
	
	//not implemented
	bool bounce;
public:
	///
	/// @brief Initializes an empty orbit
	/// @param position a reference to the current position, used for reading and writing
	///
	Orbit(Vec3D* position): positionRef(position)	
	{
		bounce = false;
		ready = false;
	}

	//Not responsible for the reference
	~Orbit() {}

	///
	/// @brief Setup for a new orbit from the current position
	/// @param desiredDirection The direction where the orbit will go in carthesic coordinates
	///							Will be projected onto plane normal to the radial direction
	/// @param inL The angular momentum of the orbit
	/// @param inSchwarzR The current radius (mass) of the black hole
	/// @return Wether the initialization was successful
	///
	bool initNewOrbit(const Vec3D & desiredDirection, const double inL, const double inSchwarzR)
	{
		schwarzR = inSchwarzR;
		r = (*positionRef)[0];
		//Not able to initialize inside
		if (r < schwarzR)
			return false;
		u = 1 / r;
		uBar = 0;
		L = inL;
		E = sqrt((1 - schwarzR / r) * (1 + L * L / (r * r)));

		Vec3D internalCarthPosition;
		util.toCarthesic(*positionRef, internalCarthPosition);
		Vec3D planeNormal;
		Vec3D z = { 0,0,1 };
		util.crossProduct(internalCarthPosition, desiredDirection, planeNormal);
		tiltAngle = util.angle(planeNormal, z);
		//horizontal orbit plane is simpler
		if (tiltAngle < 1e-10 || M_PI - tiltAngle < 1e-10)
		{
			tiltAngle = 0;
			firstPhi = 0;
			orbitAngle = (*positionRef)[1];
			planeTilt.setToIdentity();
		}
		else
		{
			//Where the orbit plane and the horizontal plane intersect
			Vec3D intersection;
			util.crossProduct(z, planeNormal, intersection);

			// Overcomplicated old version
			// if (planeNormal[2] < 1e-10)
			// {
			// 	intersection[1] = planeNormal[0];
			// 	intersection[0] = -planeNormal[1];
			// 	intersection[2] = 0;
			// }
			// else
			// {
			// 	Vec3D planeNormalProjection;
			// 	planeNormalProjection[0] = planeNormal[0];
			// 	planeNormalProjection[1] = planeNormal[1];
			// 	planeNormalProjection[2] = 0;
			// 	if (planeNormal[2] > 0)
			// 		util.crossProduct(planeNormal, planeNormalProjection, intersection);
			// 	else
			// 		util.crossProduct(planeNormalProjection, planeNormal, intersection);
			// }


			orbitAngle = util.angle(intersection, internalCarthPosition);
			//The orbit first goes up then down
			if (internalCarthPosition[2] < 0)
				orbitAngle = 2 * M_PI - orbitAngle;
			firstPhi = atan2(intersection[1], intersection[0]);
			util.yieldPlaneRotMatrix(-tiltAngle, planeTilt);
		}
		ready = true;
		return true;
	}

	///
	/// @brief Moves the position along the orbit for one timeStep
	///	@cond Orbit needs to be initialized
	///
	void doStep(const double timeStep)
	{
		if (!ready)
			throw -1;
		
		//Time stepping variables
		double deltaPhi;
		double aPhi, bPhi, cPhi, aU, bU, cU, aUBar, bUBar, cUBar,
			nextU, nextUBar;

		//Dirty Runge Kutta, tried simultanious updates in phi and u, but this didnt work
		/*aPhi = timeStep / 2 * L * u * u;
		aU = u + aPhi * uBar;
		aUBar = uBar + aPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * u * u) - u);

		bPhi = timeStep / 2 * L * aU * aU;
		bU = u + bPhi * aUBar;
		bUBar = uBar + bPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * aU * aU) - aU);

		cPhi = timeStep * L * bU * bU;
		cU = u + cPhi * bUBar;
		cUBar = uBar + cPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * bU * bU) - bU);

		deltaPhi = timeStep * L / 2 * (u * u / 6 + aU * aU / 3 + bU * bU / 3 + cU * cU / 6);
		nextU = u + deltaPhi * (uBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
		nextUBar = uBar + deltaPhi * (schwarzR / (2 * L*L) + 
			3 * schwarzR / 2 * (u * u / 6 + aU * aU / 3 + bU * bU / 3 + cU * cU / 6)
			-(u + 2 * aU + 2 * bU + cU) / 6);*/

		//Approximate the angle step according to the time step
		//These are iterated updates, because phi and u are interdependent
		//Errors in deltaPhi only influence the simulation speed, whereas errors in u can cause orbit decay.
		deltaPhi = timeStep * L * u * u / 2;
		nextU = u + deltaPhi * uBar;
		deltaPhi = timeStep * L / 4 * (u * u + nextU * nextU);
		nextU = u + deltaPhi * uBar;
		deltaPhi = timeStep * L / 4 * (u * u + nextU * nextU);
		nextU = u + deltaPhi * uBar;
		deltaPhi = timeStep * L / 4 * (u * u + nextU * nextU);

		//Break down into smaller steps if the step size is too large
		unsigned int stepFragments = 1 + (unsigned int)(deltaPhi * 1000);
		if (stepFragments > 1000000)	//Simulation fails at the singularity
			stepFragments = 1;
		if (stepFragments != 1)
			for (unsigned int i = 0; i < stepFragments; i++)
				doStep(timeStep / stepFragments);
		else
		{
			//Runge Kutta 4 scheme
			aU = u + deltaPhi / 2 * uBar;
			aUBar = uBar + deltaPhi / 2 * (schwarzR * (1 / (2 * L *L) + 3 / 2 * u * u) - u);
			bU = u + deltaPhi / 2 * aUBar;
			bUBar = uBar + deltaPhi / 2 * (schwarzR * (1 / (2 * L *L) + 3 / 2 * aU * aU) - aU);
			cU = u + deltaPhi * bUBar;
			cUBar = uBar + deltaPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * bU * bU) - bU);
			nextU = u + deltaPhi * (uBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
			nextUBar = uBar + deltaPhi * (schwarzR / (2 * L*L) +
				3 * schwarzR / 2 * (u * u / 6 + aU * aU / 3 + bU * bU / 3 + cU * cU / 6)
				- (u + 2 * aU + 2 * bU + cU) / 6);

			u = nextU;
			uBar = nextUBar;
			r = 1 / u;
			orbitAngle += deltaPhi;
			if (isnan(r))
				r = 0;


			//Calculating and saving total position
			Vec2D polarPos;
			Vec3D dummy1, dummy2;
			polarPos[1] = 0;
			polarPos[0] = orbitAngle;
			util.polarTransform(polarPos, dummy1, dummy2, planeTilt);
			(*positionRef)[0] = r;
			(*positionRef)[1] = polarPos[0] + firstPhi;
			(*positionRef)[2] = polarPos[1];
		}
	}

	///
	/// @brief saves the represented orbiting spectator to spec
	///	@cond Orbit needs to be initialized
	///
	void readOrbitSpectator(Vec3D & spec) const 
	{
		if (!ready)
			throw -1;
		spec[0] = E / (1 - schwarzR / r);
		spec[1] = (uBar > 0 ? -1 : 1) *
			sqrt(E * E - (1 - schwarzR / r) * (L  *L / (r * r) + 1));
		spec[2] = L / (r * r);
	}

	///
	/// @brief Calculates the angle between the orbit plane and the zy-plane outlined in TransUtilities::yieldTransMatrix
	///	@cond Orbit needs to be initialized
	///
	double tiltCorrectionAngle() const 
	{
		if (!ready)
			throw -1;
		return tiltAngle * cos((*positionRef)[1] - firstPhi);
	}

	///
	/// @brief Calculates wether the Orbit is stable, instable (falls into Black hole), or on an escape trajectory.
	///
	string isStable() const 
	{
		if (L*L < 3 * schwarzR * schwarzR)
			return "instable";
		double r1 = L * L / schwarzR * (1 - sqrt(1 - 3 * schwarzR * schwarzR / (L * L)));
		if (r < r1)
			return "instable";
		if (E > 1)
			return "escaping";
		if (E * E - (1 - schwarzR / r1) * (L * L / (r1 * r1) + 1) < 0)
			return "stable";
		return "instable";
	}
};