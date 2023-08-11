///
/// @file	RasterFunction.h
/// @author Cecilia
/// @date 	3.2016
///
/// @copyright MIT Public Licence
///
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

///
/// @brief This class is a regular grid over [-pi/2, pi/2] with function values at the nodes and interpolation functions
/// In this application it's used to approximate the angle travelled by light rays depending on the angle towards the black hole
/// It also represents rays entering the black hole with a high negative value for computational efficiency, which is physically motivated.
/// nodes are indexed from 0 to nrNodes
///

class RasterFunction180
{
	unsigned int nrNodes;
	double* values;

	RasterFunction180(const RasterFunction180 & a) = delete;
public:
	static constexpr double NO_VALUE = -100000.;

	///
	/// @brief Creates an empty Raster Function with a specified number of nodes
	/// @param inNrNodes must be greater than 2
	///
	RasterFunction180(const unsigned int inNrNodes) : nrNodes(inNrNodes)
	{
		if (nrNodes < 2)
			throw;
		values = new double[nrNodes];
		resetValues();
	}

	~RasterFunction180()
	{
		delete [] values;
	}

	///
	/// @brief Resets Raster Function and contained information with a specified number of nodes
	/// @param inNrNodes must be greater than 2
	///
	void resetNrNodes(const unsigned int inNrNodes)
	{
		if (nrNodes < 2)
			throw;
		nrNodes = inNrNodes;
		delete [] values;
		values = new double[inNrNodes];
		resetValues();
	}

	///
	/// @brief sets all entries to NO_VALUE
	///
	void resetValues()
	{
		for (unsigned int i = 0; i < nrNodes; i++)
			values[i] = NO_VALUE;
	}

	///
	/// @brief returns the number of Nodes
	///
	unsigned int getNrNodes() const 
	{
		return nrNodes;
	}

	///
	/// @brief returns the position of the node nr in the interval [-pi/2, pi/2]
	///
	double PositionOfNode(const unsigned int nr) const 
	{
		return M_PI_2 - nr * M_PI / (nrNodes - 1);
	}

	///
	/// @brief writes a value to a specified node
	///
	void writeValue(const double value, const unsigned int nr)
	{
		if (nr >= nrNodes)
			throw;
		values[nr] = value;
	}

	///
	/// @brief returns the value in the node with the number nr
	///
	double valueAt(const unsigned int nr) const 
	{
		if(nr < nrNodes)
			return values[nr];
		return NO_VALUE;
	}

	///
	/// @brief gives the interpolated value for x in the range of [-pi/2, pi/2]
	///
	double interpolatedValue(const double x) const 
	{
		if (x < -M_PI_2 || x > M_PI_2)
			return NO_VALUE;

		//linear interpolation between the neightboring nodes
		double normedX = (M_PI_2 - x) / M_PI * (nrNodes - 1);
		double leftNode = values[(unsigned int)floor(normedX)];
		double rightNode = values[(unsigned int)ceil(normedX)];
		double weight = normedX - floor(normedX);
		return (leftNode == NO_VALUE || rightNode == NO_VALUE) ?
			NO_VALUE : leftNode * (1 - weight) + rightNode * weight;
	}

	///
	/// @brief prints the data in a readable format (depending on number of nodes)
	///
	void Print(ostream& os) const 
	{
		for (unsigned int i = 0; i < nrNodes; i++)
			os << '{' << PositionOfNode(i) << ',' << valueAt(i) << '}' << ',' << endl;
		os << endl;
	}

	///
	/// @brief Converts the contained values into a plain float array
	/// @note This function is used to transfer the values to the GPU with OpenGL. 
	/// However i couldnt fix the problem with OpenGL not working with 1D 1-channel Textures.
	/// The workaround is using 1D Vec2 Textures, thus the duplicates entries
	/// @param The array for the values to be saved in
	///
	void toFloatArray(float* in) const 
	{
		for (unsigned int i = 0; i < nrNodes; i++){
			in[2 * i + 1] = in[2 * i] = values[i];//OGL Hack!!
		}
	}
};


