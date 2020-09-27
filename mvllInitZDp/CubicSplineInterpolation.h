// CubicSplineInterpolation.h

/*
Cubic spline interpolation class.

- Editor: Yahui Liu.
- Data:   2015-08-16
- Email:  yahui.cvrs@gmail.com
- Address: Computer Vision and Remote Sensing(CVRS), Lab.
*/

#ifndef CUBIC_SPLINE_INTERPOLATION_H
#pragma once
#define CUBIC_SPLINE_INTERPOLATION_H

#include <iostream>
#include <vector>
#include <math.h>

#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

/* Cubic spline interpolation coefficients */
class CubicSplineCoeffs
{
public:
	CubicSplineCoeffs(const int &count)
	{
		a = std::vector<double>(count);
		b = std::vector<double>(count);
		c = std::vector<double>(count);
		d = std::vector<double>(count);
	}
	~CubicSplineCoeffs()
	{
		std::vector<double>().swap(a);
		std::vector<double>().swap(b);
		std::vector<double>().swap(c);
		std::vector<double>().swap(d);
	}

public:
	std::vector<double> a, b, c, d;
};

enum CubicSplineMode
{
	CUBIC_NATURAL,    // Natural
	CUBIC_CLAMPED,    // TODO: Clamped 
	CUBIC_NOT_A_KNOT  // TODO: Not a knot 
};

enum SplineFilterMode
{
	CUBIC_WITHOUT_FILTER, // without filter
	CUBIC_MEDIAN_FILTER  // median filter
};

/* Cubic spline interpolation */
class CubicSplineInterpolation
{
public:
	CubicSplineInterpolation() {}
	~CubicSplineInterpolation() {}

public:

	/*
	Calculate cubic spline coefficients.
	- node list x (input_x);
	- node list y (input_y);
	- output coefficients (cubicCoeffs);
	- ends mode (splineMode).
	*/
	void calCubicSplineCoeffs(std::vector<double> &input_x,
		std::vector<double> &input_y, CubicSplineCoeffs *&cubicCoeffs,
		CubicSplineMode splineMode = CUBIC_NATURAL,
		SplineFilterMode filterMode = CUBIC_MEDIAN_FILTER);

	/*
	Cubic spline interpolation for a list.
	- input coefficients (cubicCoeffs);
	- input node list x (input_x);
	- output node list x (output_x);
	- output node list y (output_y);
	- interpolation step (interStep).
	*/
	void cubicSplineInterpolation(CubicSplineCoeffs *&cubicCoeffs,
		std::vector<double> &input_x, std::vector<double> &output_x,
		std::vector<double> &output_y, const double interStep = 0.5);

	/*
	Cubic spline interpolation for a value.
	- input coefficients (cubicCoeffs);
	- input a value(x);
	- output interpolation value(y);
	*/
	void cubicSplineInterpolation2(CubicSplineCoeffs *&cubicCoeffs,
		std::vector<double> &input_x, double &x, double &y);

	/*
	calculate  tridiagonal matrices with Thomas Algorithm(TDMA) :

	example:
	| b1 c1 0  0  0  0  |  |x1 |   |d1 |
	| a2 b2 c2 0  0  0  |  |x2 |   |d2 |
	| 0  a3 b3 c3 0  0  |  |x3 | = |d3 |
	| ...         ...   |  |...|   |...|
	| 0  0  0  0  an bn |  |xn |   |dn |

	Ci = ci/bi , i=1; ci / (bi - Ci-1 * ai) , i = 2, 3, ... n-1;
	Di = di/bi , i=1; ( di - Di-1 * ai )/(bi - Ci-1 * ai) , i = 2, 3, ..., n-1

	xi = Di - Ci*xi+1 , i = n-1, n-2, 1;
	*/
	bool caltridiagonalMatrices(cv::Mat_<double> &input_a,
		cv::Mat_<double> &input_b, cv::Mat_<double> &input_c,
		cv::Mat_<double> &input_d, cv::Mat_<double> &output_x);

	/* Calculate the curve index interpolation belongs to */
	int calInterpolationIndex(double &pt, std::vector<double> &input_x);

	/* median filtering */
	void cubicMedianFilter(std::vector<double> &input, const int filterSize = 5);

	double cubicSort(std::vector<double> &input);
	// double cubicNearestValue( std::vector );
};

#endif // CUBIC_SPLINE_INTERPOLATION_H