///
/// @file	TransUtilities.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

#define _USE_MATH_DEFINES
#include "RotMatrix3D.h"

///
/// @brief A collection of 3D Vector functions needed for the application, including polar transformations
/// @note Some of the functions are no longer in use as they are now performed on the GPU.
///			The whole program uses exclusively radians.
///		  	Any used polar coordinates are either (r, phi, theta) or (phi, theta) with theta in [-pi/2, pi/2] being the elevation
///
class TransUtilities
{
public:
	///
	/// @brief Transforms spherical coordinates (phi, theta) with radius 1 to carthesic coordinates
	///
	void toCarthesic(const Vec2D & pVec, Vec3D & cartVec) const 
	{
		cartVec[0] = cos(pVec[0])*cos(pVec[1]);
		cartVec[1] = sin(pVec[0])*cos(pVec[1]);
		cartVec[2] = sin(pVec[1]);
	}

	///
	/// @brief Transforms spherical coordinates (r,phi, theta) to carthesic coordinates
	///
	void toCarthesic(const Vec3D & pVec, Vec3D & cartVec) const 
	{
		cartVec[0] = pVec[0] * cos(pVec[1])*cos(pVec[2]);
		cartVec[1] = pVec[0] * sin(pVec[1])*cos(pVec[2]);
		cartVec[2] = pVec[0] * sin(pVec[2]);
	}

	///
	/// @brief Transforms a carthesic Vector to spherical coordinates (r discarded)
	///
	void toPolar(const Vec3D & cartVec, Vec2D & pVec) const 
	{
		pVec[0] = atan2(cartVec[1], cartVec[0]);
		pVec[1] = asin(cartVec[2]);
	}

	///
	/// @brief Transforms a carthesic Vector to spherical coordinates
	///
	void toPolar(const Vec3D & cartVec, Vec3D & pVec) const 
	{
		pVec[0] = sqrt(cartVec[0] * cartVec[0] + cartVec[1] * cartVec[1] 
			+ cartVec[2] * cartVec[2]);
		pVec[1] = atan2(cartVec[1], cartVec[0]);
		pVec[2] = asin(cartVec[2] / pVec[0]);
	}

	///
	/// @brief Transforms inOutPolar with the rotation defined by rotator
	/// @param dummy1, dummy2 are there to avoid repeatedly allocating memory inside the function, this was believed to improve performance at the time
	///
	void polarTransform(Vec2D & inOutPolar, Vec3D & dummy1, Vec3D & dummy2, const RotMatrix3D & rotator) const 
	{
		toCarthesic(inOutPolar, dummy1);
		rotator.transform(dummy1, dummy2);
		toPolar(dummy2, inOutPolar);
	}


	///
	/// @brief Yields a rotation matrix, which transforms the z coordinate to 'axis' and the x coordinate 90° down from that.
	/// @param axis the direction z gets transformed into
	/// @param target Where the resulting matrix gets saved
	///
	void yieldTransMatrix(const Vec2D & axis, RotMatrix3D & target) const 
	{
		Vec3D z;
		toCarthesic(axis, z);

		//turning z 90 degree down yields x
		Vec2D xPolar;
		//toPolar(z, zToXPolar);
		xPolar[1] = axis[1] - M_PI_2;
		xPolar[0] = axis[0];
		Vec3D x;
		toCarthesic(xPolar, x);

		//y is the cross product ZxX
		Vec3D y;
		crossProduct(z, x, y);

		target = RotMatrix3D(x, y, z);
		target.clearArtifacts();
	}


	///
	/// @brief Yields a rotation matrix in the x,y plane by angle 'phi'
	/// @note Used to match the orbit plane with the y,z plane
	///
	void yieldTiltMatrix(const double phi, RotMatrix3D & target) const 
	{
		Vec3D x, y, z;
		y[1] = x[0] = cos(phi);
		y[0] = sin(phi);
		x[1] = -sin(phi);
		z[2] = 1;
		x[2] = y[2] = z[0] = z[1] = 0;

		target = RotMatrix3D(x, y, z);
		target.transpose();
		target.clearArtifacts();
	}

	///
	/// @brief Yields a rotation matrix in the y,z plane by angle 'phi' to align z with the velocity vector
	/// @note The angle between center direction and velocity vector depends on the observer
	///
	void yieldPlaneRotMatrix(const double phi, RotMatrix3D & target) const 
	{
		Vec3D x, y, z;
		y[1] = z[2] = cos(phi);
		z[1] = sin(phi);
		y[2] = -sin(phi);
		x[0] = 1;
		x[2] = y[0] = z[0] = x[1] = 0;
		target = RotMatrix3D(x, y, z);
		target.clearArtifacts();
	}

	///
	/// @brief calculates the cross product of a and b, which is saved in result
	///
	void crossProduct(const Vec3D & a, const Vec3D & b, Vec3D & result) const 
	{
		result[0] = a[1] * b[2] - a[2] * b[1];
		result[1] = a[2] * b[0] - a[0] * b[2];
		result[2] = a[0] * b[1] - a[1] * b[0];
	}

	///
	/// @brief Calculates the angle between a and b
	///
	double angle(const Vec3D & a, const Vec3D & b) const 
	{
		double scalar = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		if (fabs(scalar) > 0)
			scalar /= sqrt((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) *
				(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]));
		return acos(scalar);
	}
};