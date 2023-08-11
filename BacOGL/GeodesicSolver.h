///
/// @file	GeodesicSolver.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

#include "RasterFunction.h"
#include <math.h>
//#include <omp.h>

///
/// @brief This class simulates an 180° fan of light rays and calculates the angle travelled by each ray until the texture sphere is hit.
///
class GeodesicSolver
{
	static constexpr double SINGULARITY_BOUND = 1e-10;
	unsigned int maxIterations;
	double standardStep;

	///
	/// @brief filters some cases for one light ray and prepares for solving
	/// @param E 		Energy of the light ray
	/// @param L 		Angular momentum of the light ray
	/// @param rStart 	Starting distance
	/// @param rFalling Wether the light ray goes inward or outward
	/// @param schwarzR Radius of the black hole
	/// @param surfaceR Radius of the texture sphere
	/// @return The angle travelled to hit the texture sphere (or NO_VALUE)
	///
	double PrehandlerRKTheta(const double E, const double L, const double rStart,
			const bool rFalling, const double schwarzR, const double surfaceR) const 
	{
		double b = E ? L / E : 1e20;
		bool outside = rStart > schwarzR;
		bool surfaceOutside = surfaceR > schwarzR;
		bool innerView = rStart < surfaceR;

		//If L = 0 no ODE to solve
		if (L < SINGULARITY_BOUND)
		{
			if (innerView)
				if (outside)
					return rFalling ? (schwarzR ? RasterFunction180::NO_VALUE : M_PI) : 0;
				else
					return surfaceOutside ?
					(E > 0 ? 0 : RasterFunction180::NO_VALUE): /*The weirdest case*/0;
			else
				if (surfaceOutside)
					return rFalling ? 0 : RasterFunction180::NO_VALUE;
				else
					return RasterFunction180::NO_VALUE;
		}
		//Energy requirement to leave the 3R/2 barrier
		bool barrier3R_2 = schwarzR &&
			1 / (b * b) < 4 / (27 * schwarzR * schwarzR);
		//Wether r and surfaceR are on different sides of the 3R/2 barrier
		bool differentSides3R_2 = ((rStart < 3 * schwarzR / 2) ^ (surfaceR < 3 * schwarzR / 2))
			&& fabs(rStart - 3 * schwarzR / 2) > SINGULARITY_BOUND;
		
		//Some cases where the geodesis won't hit the surface
		if ((!innerView && !surfaceOutside) ||
			(!outside && surfaceOutside && E < 0) ||
			(barrier3R_2 && differentSides3R_2) ||
			(rStart < 3 * schwarzR / 2 && innerView && rFalling) ||
			(rStart > 3 * schwarzR / 2 && !innerView && !rFalling))
		{
			return RasterFunction180::NO_VALUE;
		}

		double u = 1 / rStart;
		double uBar = (rFalling ? 1 : -1) *
			sqrt(1 / (b  * b) - (1 - schwarzR / rStart) / rStart / rStart);
		//if (fabs(uBar) < SINGULARITY_BOUND)
		//	return RasterFunction180::NO_VALUE;
		return RungeKuttaSolver(u, uBar, schwarzR, surfaceR);
	}

	///
	/// @brief Solves the orbit equation u'' + u = 3R/2 u^2 and calculates the intersection with the texture sphere
	/// @param u 		initial inverted radius
	/// @param uBar 	initial inverted radius derivative
	/// @param R 		radius of black hole
	/// @param surfaceR radius of texture sphere
	/// @return Travelled angle until texture sphere is hit
	///
	double RungeKuttaSolver(const double u, const double uBar, const double R, const double surfaceR) const 
	{

		double bound = 0.9 * fmin(u, 1 / fmax(surfaceR, 2 * R));
		double step = standardStep;
		unsigned int finishRefinement = 3;
		//if (fabs(uBar) > 1)
		//	step /= fabs(uBar);
		double stepHalf = step / 2;
		double R3_2 = 3 * R / 2;
		double angle = 0;

		double currentU = u;
		double currentUBar = uBar;
		double nextU, nextUBar, aU, aUBar, bU, bUBar, cU, cUBar;
		double surfaceU = 1 / surfaceR;
		unsigned int iteration = 0;

		//While the ray didnt fall into the black hole or fly away
		while ((!(R && currentU > 1 / R && currentUBar > 0)) && 
			iteration < maxIterations && currentU > 0)
		{
			aU = currentU + stepHalf * currentUBar;
			aUBar = currentUBar + stepHalf * (-currentU + R3_2 * currentU*currentU);
			bU = currentU + stepHalf * aUBar;
			bUBar = currentUBar + stepHalf * (-aU + R3_2 * aU*aU);
			cU = currentU + step * bUBar;
			cUBar = currentUBar + step * (-bU + R3_2 * bU*bU);

			nextU = currentU + step * (currentUBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
			nextUBar = currentUBar + step * ((-currentU + R3_2 * currentU*currentU) / 6 +
				(-aU + R3_2 * aU*aU) / 3 + (-bU + R3_2 * bU*bU) / 3 + (-cU + R3_2 * cU*cU) / 6);

			//check if the ray has passed through the surface, then do some newton to find the precise cut.
			//The Newton method works with the function of one RK4 step from the previous position
			if ((nextU > surfaceU) ^ (u > surfaceU))
			{
				double value, derivitive, newtonStep;
				//Start at side with larger slope
				if (fabs(currentUBar) > fabs(nextUBar))
				{
					newtonStep = 0;
					value = currentU;
					derivitive = currentUBar;
				}
				else
				{
					newtonStep = step;
					value = nextU;
					derivitive = nextUBar;
				}

				for (int i = 0; i < finishRefinement; i++)
				{
					newtonStep -= (value - surfaceU) / derivitive;

					aU = currentU + newtonStep * currentUBar / 2;
					aUBar = currentUBar + newtonStep * (-currentU + R3_2 * currentU*currentU) / 2;
					bU = currentU + newtonStep * aUBar / 2;
					bUBar = currentUBar + newtonStep * (-aU + R3_2 * aU*aU) / 2;
					cU = currentU + newtonStep * bUBar;
					cUBar = currentUBar + newtonStep * (-bU + R3_2 * bU*bU);

					value = currentU + newtonStep * (currentUBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
					derivitive = currentUBar + newtonStep * ((-currentU + R3_2 * currentU*currentU) / 6 +
						(-aU + R3_2 * aU*aU) / 3 + (-bU + R3_2 * bU*bU) / 3 + (-cU + R3_2 * cU*cU) / 6);
				}
				return angle + newtonStep;
			}

			//check if the ray leaves the relevant domain
			if (nextU < bound)
				return RasterFunction180::NO_VALUE;
			currentU = nextU;
			currentUBar = nextUBar;
			iteration++;
			angle += step;
		}
		return RasterFunction180::NO_VALUE;
	}


public:
	///
	/// @brief Initializes the class with a default step size
	///
	GeodesicSolver(const double step = M_PI / 100) :
		maxIterations((int)(60/step)), standardStep(step)
	{}

	///
	/// @brief Calculates the endpoints on the texture sphere of the light ray fan defined by raster
	/// @param schwarzR 	Radius of the black hole
	/// @param r 			Distance to the black hole
	/// @param surfaceR 	Radius of the texture sphere, where the light rays may hit
	/// @param raster 		The raster object in which the results are saved
	/// @note If a light ray falls into the black hole or escapes, then a NO_VALUE is saved
	///
	void solveGeodesic(const double schwarzR, const double r, 
		const double surfaceR, RasterFunction180 & raster) const 
	{
		unsigned int nodes = raster.getNrNodes();
		
		//designed to run parallel, but it was too fast for parallelization
//#pragma omp parallel for
		for(unsigned int i = 0; i < nodes; i++)
		{
			double theta, L, E;
			bool rFalling;
			theta = raster.PositionOfNode(i);
			L = r* cos(theta);
			//inside the black hole things get weird
			if (r < schwarzR)
			{
				rFalling = false;
				E = sin(-theta) * sqrt(schwarzR / r - 1);
			}
			else
			{
				//checks if they are comming from lower r
				rFalling = sin(theta) > 0;
				E = sqrt(1 - schwarzR / r);
			}
			double result = PrehandlerRKTheta(E, L, r, rFalling, schwarzR, surfaceR);
			//NO_VALUE must be preserved
			raster.writeValue(result == RasterFunction180::NO_VALUE ?
				RasterFunction180::NO_VALUE : M_PI_2 - result, i);
		}
	}
};