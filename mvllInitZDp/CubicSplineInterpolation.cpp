// CubicSplineInterpolation.cpp

#include "CubicSplineInterpolation.h"

void CubicSplineInterpolation::calCubicSplineCoeffs(
	std::vector<double> &input_x,
	std::vector<double> &input_y,
	CubicSplineCoeffs *&cubicCoeffs,
	CubicSplineMode splineMode /* = CUBIC_NATURAL */,
	SplineFilterMode filterMode /*= CUBIC_MEDIAN_FILTER*/)
{
	int sizeOfx = input_x.size();
	int sizeOfy = input_y.size();

	if (sizeOfx != sizeOfy)
	{
		std::cout << "Data input error!" << std::endl <<
			"Location: CubicSplineInterpolation.cpp" <<
			" -> calCubicSplineCoeffs()" << std::endl;

		return;
	}

	/*
	hi*mi + 2*(hi + hi+1)*mi+1 + hi+1*mi+2
	=  6{ (yi+2 - yi+1)/hi+1 - (yi+1 - yi)/hi }

	so, ignore the both ends:
	| -     -     -        0           ...             0     |  |m0 |
	| h0 2(h0+h1) h1       0           ...             0     |  |m1 |
	| 0     h1    2(h1+h2) h2 0        ...                   |  |m2 |
	|         ...                      ...             0     |  |...|
	| 0       ...           0 h(n-2) 2(h(n-2)+h(n-1)) h(n-1) |  |   |
	| 0       ...                      ...             -     |  |mn |

	*/

	std::vector<double> copy_y = input_y;

	if (filterMode == CUBIC_MEDIAN_FILTER)
	{
		cubicMedianFilter(copy_y, 5);
	}

	const int count = sizeOfx;
	const int count1 = sizeOfx - 1;
	const int count2 = sizeOfx - 2;
	const int count3 = sizeOfx - 3;

	cubicCoeffs = new CubicSplineCoeffs(count1);

	std::vector<double> step_h(count1, 0.0);

	// for m matrix
	cv::Mat_<double> m_a(1, count2, 0.0);
	cv::Mat_<double> m_b(1, count2, 0.0);
	cv::Mat_<double> m_c(1, count2, 0.0);
	cv::Mat_<double> m_d(1, count2, 0.0);
	cv::Mat_<double> m_part(1, count2, 0.0);

	cv::Mat_<double> m_all(1, count, 0.0);

	// initial step hi
	for (int idx = 0; idx < count1; idx++)
	{
		step_h[idx] = input_x[idx + 1] - input_x[idx];
	}
	// initial coefficients
	for (int idx = 0; idx < count3; idx++)
	{
		m_a(idx) = step_h[idx];
		m_b(idx) = 2 * (step_h[idx] + step_h[idx + 1]);
		m_c(idx) = step_h[idx + 1];
	}
	// initial d
	for (int idx = 0; idx < count3; idx++)
	{
		m_d(idx) = 6 * (
			(copy_y[idx + 2] - copy_y[idx + 1]) / step_h[idx + 1] -
			(copy_y[idx + 1] - copy_y[idx]) / step_h[idx]);
	}

	//cv::Mat_<double> matOfm( count2,  )
	bool isSucceed = caltridiagonalMatrices(m_a, m_b, m_c, m_d, m_part);
	if (!isSucceed)
	{
		std::cout << "Calculate tridiagonal matrices failed!" << std::endl <<
			"Location: CubicSplineInterpolation.cpp -> " <<
			"caltridiagonalMatrices()" << std::endl;

		return;
	}

	if (splineMode == CUBIC_NATURAL)
	{
		m_all(0) = 0.0;
		m_all(count1) = 0.0;

		for (int i = 1; i<count1; i++)
		{
			m_all(i) = m_part(i - 1);
		}

		for (int i = 0; i<count1; i++)
		{
			cubicCoeffs->a[i] = copy_y[i];
			cubicCoeffs->b[i] = (copy_y[i + 1] - copy_y[i]) / step_h[i] -
				step_h[i] * (2 * m_all(i) + m_all(i + 1)) / 6;
			cubicCoeffs->c[i] = m_all(i) / 2.0;
			cubicCoeffs->d[i] = (m_all(i + 1) - m_all(i)) / (6.0 * step_h[i]);
		}
	}
	else
	{
		std::cout << "Not define the interpolation mode!" << std::endl;
	}
}

void CubicSplineInterpolation::cubicSplineInterpolation(
	CubicSplineCoeffs *&cubicCoeffs,
	std::vector<double> &input_x,
	std::vector<double> &output_x,
	std::vector<double> &output_y,
	const double interStep)
{
	const int count = input_x.size();

	double low = input_x[0];
	double high = input_x[count - 1];

	double interBegin = low;
	for (; interBegin < high; interBegin += interStep)
	{
		int index = calInterpolationIndex(interBegin, input_x);
		if (index >= 0)
		{
			double dertx = interBegin - input_x[index];
			double y = cubicCoeffs->a[index] + cubicCoeffs->b[index] * dertx +
				cubicCoeffs->c[index] * dertx * dertx +
				cubicCoeffs->d[index] * dertx * dertx * dertx;
			output_x.push_back(interBegin);
			output_y.push_back(y);
		}
	}
}

void CubicSplineInterpolation::cubicSplineInterpolation2(
	CubicSplineCoeffs *&cubicCoeffs,
	std::vector<double> &input_x, double &x, double &y)
{
	const int count = input_x.size();

	double low = input_x[0];
	double high = input_x[count - 1];

	if (x<low || x>high)
	{
		std::cout << "The interpolation value is out of range!" << std::endl;
	}
	else
	{
		int index = calInterpolationIndex(x, input_x);
		if (index > 0)
		{
			double dertx = x - input_x[index];
			y = cubicCoeffs->a[index] + cubicCoeffs->b[index] * dertx +
				cubicCoeffs->c[index] * dertx * dertx +
				cubicCoeffs->d[index] * dertx * dertx * dertx;
		}
		else
		{
			std::cout << "Can't find the interpolation range!" << std::endl;
		}
	}
}

bool CubicSplineInterpolation::caltridiagonalMatrices(
	cv::Mat_<double> &input_a,
	cv::Mat_<double> &input_b,
	cv::Mat_<double> &input_c,
	cv::Mat_<double> &input_d,
	cv::Mat_<double> &output_x)
{
	int rows = input_a.rows;
	int cols = input_a.cols;

	if ((rows == 1 && cols > rows) ||
		(cols == 1 && rows > cols))
	{
		const int count = (rows > cols ? rows : cols) - 1;

		output_x = cv::Mat_<double>::zeros(rows, cols);

		cv::Mat_<double> cCopy, dCopy;
		input_c.copyTo(cCopy);
		input_d.copyTo(dCopy);

		if (input_b(0) != 0)
		{
			cCopy(0) /= input_b(0);
			dCopy(0) /= input_b(0);
		}
		else
		{
			return false;
		}

		for (int i = 1; i < count; i++)
		{
			double temp = input_b(i) - input_a(i) * cCopy(i - 1);
			if (temp == 0.0)
			{
				return false;
			}

			cCopy(i) /= temp;
			dCopy(i) = (dCopy(i) - dCopy(i - 1)*input_a(i)) / temp;
		}

		output_x(count) = dCopy(count);
		for (int i = count - 2; i > 0; i--)
		{
			output_x(i) = dCopy(i) - cCopy(i)*output_x(i + 1);
		}
		return true;
	}
	else
	{
		return false;
	}
}

int CubicSplineInterpolation::calInterpolationIndex(
	double &pt, std::vector<double> &input_x)
{
	const int count = input_x.size() - 1;
	int index = -1;
	for (int i = 0; i<count; i++)
	{
		if (pt > input_x[i] && pt <= input_x[i + 1])
		{
			index = i;
			return index;
		}
	}
	return index;
}

void CubicSplineInterpolation::cubicMedianFilter(
	std::vector<double> &input, const int filterSize /* = 5 */)
{
	const int count = input.size();
	for (int i = filterSize / 2; i<count - filterSize / 2; i++)
	{
		std::vector<double> temp(filterSize, 0.0);
		for (int j = 0; j<filterSize; j++)
		{
			temp[j] = input[i + j - filterSize / 2];
		}

		input[i] = cubicSort(temp);

		std::vector<double>().swap(temp);
	}

	for (int i = 0; i<filterSize / 2; i++)
	{
		std::vector<double> temp(filterSize, 0.0);
		for (int j = 0; j<filterSize; j++)
		{
			temp[j] = input[j];
		}

		input[i] = cubicSort(temp);
		std::vector<double>().swap(temp);
	}

	for (int i = count - filterSize / 2; i<count; i++)
	{
		std::vector<double> temp(filterSize, 0.0);
		for (int j = 0; j<filterSize; j++)
		{
			temp[j] = input[j];
		}

		input[i] = cubicSort(temp);
		std::vector<double>().swap(temp);
	}
}

double CubicSplineInterpolation::cubicSort(std::vector<double> &input)
{
	int iCount = input.size();
	for (int j = 0; j<iCount - 1; j++)
	{
		for (int k = iCount - 1; k>j; k--)
		{
			if (input[k - 1] > input[k])
			{
				double tp = input[k];
				input[k] = input[k - 1];
				input[k - 1] = tp;
			}
		}
	}
	return input[iCount / 2];
}