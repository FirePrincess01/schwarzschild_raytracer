///
/// @file	RotMatrix3D.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

#include <iostream>
#include <math.h>

typedef double Vec3D[3];
typedef double Vec2D[2];
typedef double Matrix33[3][3];

using namespace std;

///
/// @brief Represents a rotation matrix, very basic implementation
///
class RotMatrix3D
{
	Matrix33 mat;
public:
	~RotMatrix3D() {}

	///
	/// @brief Default constructur creates an identity matrix
	///
	RotMatrix3D(): mat() {setToIdentity();}

	///
	/// @brief Constructs a matrix that contains the inputs as columns
	///
	RotMatrix3D(const Vec3D & column1, const Vec3D & column2, const Vec3D & column3):mat()
	{
		for(int i = 0; i < 3; i++)
		{
			mat[i][0] = column1[i];
			mat[i][1] = column2[i];
			mat[i][2] = column3[i];
		}
	}

	//Copy constructor
	RotMatrix3D(const RotMatrix3D & copy):mat()
	{
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				mat[i][j] = copy.mat[i][j];
	}

	//Assignment
	void operator=(const RotMatrix3D & copy)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				mat[i][j] = copy.mat[i][j];
	}

	///
	/// @brief Saves ThisMatrix*input to target
	///
	void transform(const Vec3D & input, Vec3D & target) const 
	{
		target[0] = mat[0][0]*input[0] + mat[0][1]*input[1] + mat[0][2]*input[2];
		target[1] = mat[1][0]*input[0] + mat[1][1]*input[1] + mat[1][2]*input[2];
		target[2] = mat[2][0]*input[0] + mat[2][1]*input[1] + mat[2][2]*input[2];
	}

	///
	/// @brief Saves ThisMatrix^T*input to target
	/// @note For rotation matrices this is the inverse Transformation
	///
	void invTransform(const Vec3D & input, Vec3D & target) const 
	{
		target[0] = mat[0][0]*input[0] + mat[1][0]*input[1] + mat[2][0]*input[2];
		target[1] = mat[0][1]*input[0] + mat[1][1]*input[1] + mat[2][1]*input[2];
		target[2] = mat[0][2]*input[0] + mat[1][2]*input[1] + mat[2][2]*input[2];
	}

	///
	/// @brief Permanently transposes the Matrix
	///
	void transpose()
	{
		double dummy;
		dummy = mat[0][1];
		mat[0][1] = mat[1][0];
		mat[1][0] = dummy;

		dummy = mat[2][1];
		mat[2][1] = mat[1][2];
		mat[1][2] = dummy;

		dummy = mat[0][2];
		mat[0][2] = mat[2][0];
		mat[2][0] = dummy;
	}

	///
	/// @brief Returns the matrix ThisMatrix*right
	///
	RotMatrix3D multiply(const RotMatrix3D & right) const 
	{
		RotMatrix3D result = RotMatrix3D();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.mat[i][j] = mat[i][0] * right.mat[0][j] + mat[i][1] * right.mat[1][j]
				+ mat[i][2] * right.mat[2][j];
		return result;
	}

	///
	/// @brief ThisMatrix = ThisMatrix*right
	///
	void multiplyOnto(const RotMatrix3D & right)
	{
		*this = this->multiply(right);
	}

	///
	/// @brief clears close to zero entries
	///
	void clearArtifacts()
	{
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				if(fabs((mat[i][j])) < 1e-10)
					mat[i][j] = 0;
	}

	///
	/// @brief Sets the object to be the identity matrix
	///
	void setToIdentity()
	{
		mat[0][0] = mat[1][1] = mat[2][2] = 1;
		mat[1][0] = mat[2][0] = mat[0][1] = mat[0][2] = mat[1][2] = mat[2][1] = 0;
	}

	///
	/// @brief Prints the matrix
	///
	void print(ostream& os) const 
	{
		for(int i = 0; i < 3; i++)
			os << mat[i][0] << ' ' << mat[i][1] << ' ' << mat[i][2] << endl;
	}

	///
	/// @brief puts the entries into a size 9 float array to be used by OpenGL
	///			the entries are saved row by row
	///
	void toFloatArray(float* outArray) const 
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				outArray[i * 3 + j] = mat[i][j];
	}
};
