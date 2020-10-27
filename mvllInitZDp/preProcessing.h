#pragma once
/* ²Î¿¼£º https://www.cnblogs.com/konglongdanfo/p/9215091.html */


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
using namespace cv;

constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();


class preProcessing
{
public:

	bool histMatch_Value(Mat matSrc, Mat matDst, Mat& matRet);
	int histogram_Matching(Mat Src, Mat matDst, Mat& matResult);

	void deleteNodataValue(const Mat& src, Mat &dst);
};

