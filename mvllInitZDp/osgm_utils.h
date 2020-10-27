/**
* @copyright Copyright(C), 2020, PMRS Lab, IRSA, CAS
* @file osgm.h
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief osgm工程中定义需要的函数等
* @version 1.0
* @par 修改历史:
* <作者>  <时间>  <版本编号>  <描述>\n
*/
#include"osgm_types.h"
#include "../include/RFMBaseFunction.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
using namespace cv;
using namespace std;
constexpr auto Z_resolution = 1;

// -----------------------------------------------------------------
// 相关系数，最小二乘匹配
void vectorsort(std::vector < cv::Point3f>& Temp_sort)
{
	for (int i = 0; i < Temp_sort.size() - 1; i++) {
		float tem = 0;
		float temx = 0;
		float temy = 0;
		// 内层for循环控制相邻的两个元素进行比较
		for (int j = i + 1; j < Temp_sort.size(); j++) {
			if (Temp_sort.at(i).z < Temp_sort.at(j).z) {
				tem = Temp_sort.at(j).z;
				Temp_sort.at(j).z = Temp_sort.at(i).z;
				Temp_sort.at(i).z = tem;

				temx = Temp_sort.at(j).x;
				Temp_sort.at(j).x = Temp_sort.at(i).x;
				Temp_sort.at(i).x = temx;

				temy = Temp_sort.at(j).y;
				Temp_sort.at(j).y = Temp_sort.at(i).y;
				Temp_sort.at(i).y = temy;
			}
		}
	}
}

//相关系数图像匹配
float Get_coefficient(cv::Mat matchLeftWindow, cv::Mat imageRight, int x, int y)
{
	//根据左搜索窗口确定右搜索窗口的大小
	cv::Mat Rmatchwindow;
	Rmatchwindow.create(matchLeftWindow.rows, matchLeftWindow.cols, CV_32FC1);
	float aveRImg = 0;
	for (int m = 0; m < matchLeftWindow.rows; m++)
	{
		for (int n = 0; n < matchLeftWindow.cols; n++)
		{
			aveRImg += imageRight.at<uchar>(x + m, y + n);
			Rmatchwindow.at<float>(m, n) = imageRight.at<uchar>(x + m, y + n);
		}
	}
	aveRImg = aveRImg / (matchLeftWindow.rows * matchLeftWindow.cols);
	for (int m = 0; m < matchLeftWindow.rows; m++)
	{
		for (int n = 0; n < matchLeftWindow.cols; n++)
		{
			Rmatchwindow.at<float>(m, n) -= aveRImg;
		}
	}
	//开始计算相关系数
	float cofficent1 = 0;
	float cofficent2 = 0;
	float cofficent3 = 0;
	for (int m = 0; m < matchLeftWindow.rows; m++)
	{
		for (int n = 0; n < matchLeftWindow.cols; n++)
		{
			cofficent1 += matchLeftWindow.at<float>(m, n) * Rmatchwindow.at<float>(m, n);
			cofficent2 += Rmatchwindow.at<float>(m, n) * Rmatchwindow.at<float>(m, n);
			cofficent3 += matchLeftWindow.at<float>(m, n) * matchLeftWindow.at<float>(m, n);
		}
	}
	double cofficent = cofficent1 / sqrt(cofficent2 * cofficent3);
	return cofficent;
}

int kyhMatchImgbyLST(const cv::Mat& imageLeft, const cv::Mat& imageRight, std::vector<cv::Point3f> featurePointLeft)
{

	int matchsize = 9;//相关系数的正方形窗口的边长
	int half_matchsize = matchsize / 2;//边长的一半

	std::vector<cv::Point3f> featurePointRight;//右片匹配到的数据

	float lowst_door = 0.7; //相关系数法匹配的阈值
	int dist_width = 30;//左相片与右相片的相对距离，在这里通过手动观察

	//进行f数据的预处理 删除不符合规范的数据
	for (size_t i = 0; i < featurePointLeft.size(); i++)
	{
		//这里的 5 = half_matchsize + 1
		if ((featurePointLeft.at(i).y + dist_width < imageLeft.cols) || (imageLeft.cols - featurePointLeft.at(i).y < 5))
		{
			featurePointLeft.erase(featurePointLeft.begin() + i);
			i--;
			continue;
		}
		if ((featurePointLeft.at(i).x < 5) || (imageLeft.rows - featurePointLeft.at(i).x < 5))
		{
			featurePointLeft.erase(featurePointLeft.begin() + i);
			i--;
			continue;
		}

	}
	//创建左窗口的小窗口
	cv::Mat matchLeftWindow;
	matchLeftWindow.create(matchsize, matchsize, CV_32FC1);
	for (size_t i = 0; i < featurePointLeft.size(); i++)
	{
		float aveLImg = 0;
		for (int m = 0; m < matchsize; m++)
		{
			for (int n = 0; n < matchsize; n++)
			{
				aveLImg += imageLeft.at<uchar>(featurePointLeft.at(i).x - half_matchsize + m, featurePointLeft.at(i).y - half_matchsize + n);
				matchLeftWindow.at<float>(m, n) = imageLeft.at<uchar>(featurePointLeft.at(i).x - half_matchsize + m, featurePointLeft.at(i).y - half_matchsize + n);
			}
		}
		aveLImg = aveLImg / (matchsize * matchsize);//求取左窗口平均值
		//均除某个值
		for (int m = 0; m < matchsize; m++)
		{
			for (int n = 0; n < matchsize; n++)
			{
				matchLeftWindow.at<float>(m, n) = matchLeftWindow.at<float>(m, n) - aveLImg;
			}
		}
		//***************************对右窗口进行计算
		//首先预估右窗口的位置
		int halflengthsize = 10; //搜索区的半径
		std::vector < cv::Point3f> tempfeatureRightPoint;
		//去除跑到窗口外的点
		for (int ii = -halflengthsize; ii <= halflengthsize; ii++)
		{
			for (int jj = -halflengthsize; jj <= halflengthsize; jj++)
			{
				//为了省事…… 把边缘超限的都给整没了
				if ((featurePointLeft.at(i).x < (halflengthsize + 5)) || (imageRight.rows - featurePointLeft.at(i).x) < (halflengthsize + 5)
					|| (featurePointLeft.at(i).y + dist_width - imageLeft.cols) < (halflengthsize + 5))
				{
					cv::Point3f temphalflengthsize;
					temphalflengthsize.x = 0;
					temphalflengthsize.y = 0;
					temphalflengthsize.z = 0;
					tempfeatureRightPoint.push_back(temphalflengthsize);
				}
				else
				{
					cv::Point3f temphalflengthsize;
					int x = featurePointLeft.at(i).x + ii - half_matchsize;
					int y = featurePointLeft.at(i).y + dist_width - imageLeft.cols + jj - half_matchsize;
					float  coffee = Get_coefficient(matchLeftWindow, imageRight, x, y);
					temphalflengthsize.x = featurePointLeft.at(i).x + ii;
					temphalflengthsize.y = featurePointLeft.at(i).y + dist_width - imageLeft.cols + jj;
					temphalflengthsize.z = coffee;
					tempfeatureRightPoint.push_back(temphalflengthsize);
				}
			}
		}
		vectorsort(tempfeatureRightPoint);

		if (tempfeatureRightPoint.at(0).z > lowst_door && tempfeatureRightPoint.at(0).z < 1)
		{
			cv::Point3f tempr;
			tempr.x = tempfeatureRightPoint.at(0).x;
			tempr.y = tempfeatureRightPoint.at(0).y;
			tempr.z = tempfeatureRightPoint.at(0).z;
			featurePointRight.push_back(tempr);
		}
		else
		{
			featurePointLeft.erase(featurePointLeft.begin() + i);
			i--;
			continue;
		}
	}
	//得到左右两片的同名点初始值

	/*正式开始最小二乘匹配*/
	std::vector<cv::Point3f> featureRightPointLST;//存储最小二乘匹配到的点
	//求几何畸变的初始值
	cv::Mat formerP = cv::Mat::eye(2 * featurePointLeft.size(), 2 * featurePointLeft.size(), CV_32F)/*权矩阵*/,
		formerL = cv::Mat::zeros(2 * featurePointLeft.size(), 1, CV_32F)/*常数项*/,
		formerA = cv::Mat::zeros(2 * featurePointLeft.size(), 6, CV_32F)/*系数矩阵*/;
	for (int i = 0; i < featurePointLeft.size(); i++)
	{
		float x1 = featurePointLeft.at(i).x;
		float y1 = featurePointLeft.at(i).y;
		float x2 = featurePointRight.at(i).x;
		float y2 = featurePointRight.at(i).y;
		float coef = featurePointRight.at(i).z;//初始同名点的相关系数作为权重
		formerP.at<float>(2 * i, 2 * i) = coef;
		formerP.at<float>(2 * i + 1, 2 * i + 1) = coef;
		formerL.at<float>(2 * i, 0) = x2;
		formerL.at<float>(2 * i + 1, 0) = y2;
		formerA.at<float>(2 * i, 0) = 1; formerA.at<float>(2 * i, 1) = x1; formerA.at<float>(2 * i, 2) = y1;
		formerA.at<float>(2 * i + 1, 3) = 1; formerA.at<float>(2 * i + 1, 4) = x1; formerA.at<float>(2 * i + 1, 5) = y1;
	}
	cv::Mat Nbb = formerA.t() * formerP * formerA, U = formerA.t() * formerP * formerL;
	cv::Mat formerR = Nbb.inv() * U;
	//开始进行最小二乘匹配
	for (int i = 0; i < featurePointLeft.size(); i++)
	{
		//坐标的迭代初始值
		float x1 = featurePointLeft.at(i).x;
		float y1 = featurePointLeft.at(i).y;
		float x2 = featurePointRight.at(i).x;
		float y2 = featurePointRight.at(i).y;
		//几何畸变参数迭代初始值
		float a0 = formerR.at<float>(0, 0); float a1 = formerR.at<float>(1, 0); float a2 = formerR.at<float>(2, 0);
		float b0 = formerR.at<float>(3, 0); float b1 = formerR.at<float>(4, 0); float b2 = formerR.at<float>(5, 0);
		//辐射畸变迭代初始值
		float h0 = 0, h1 = 1;

		//当后一次相关系数小于前一次，迭代停止
		float beforeCorrelationCoe = 0/*前一个相关系数*/, CorrelationCoe = 0;
		float xs = 0, ys = 0;

		while (beforeCorrelationCoe <= CorrelationCoe)
		{
			beforeCorrelationCoe = CorrelationCoe;
			cv::Mat C = cv::Mat::zeros(matchsize * matchsize, 8, CV_32F);//系数矩阵，matchsize为左片目标窗口大小
			cv::Mat L = cv::Mat::zeros(matchsize * matchsize, 1, CV_32F);//常数项
			cv::Mat P = cv::Mat::eye(matchsize * matchsize, matchsize * matchsize, CV_32F);//权矩阵
			float sumgxSquare = 0, sumgySquare = 0, sumXgxSquare = 0, sumYgySquare = 0;
			int dimension = 0;//用于矩阵赋值
			float sumLImg = 0, sumLImgSquare = 0, sumRImg = 0, sumRImgSquare = 0, sumLR = 0;

			for (int m = x1 - half_matchsize; m <= x1 + half_matchsize; m++)
			{
				for (int n = y1 - half_matchsize; n <= y1 + half_matchsize; n++)
				{
					float x2 = a0 + a1 * m + a2 * n;
					float y2 = b0 + b1 * m + b2 * n;
					int I = std::floor(x2); int J = std::floor(y2);//不大于自变量的最大整数
					if (I <= 1 || I >= imageRight.rows - 1 || J <= 1 || J >= imageRight.cols - 1)
					{
						I = 2; J = 2; P.at<float>((m - (y1 - 5) - 1) * (2 * 4 + 1) + n - (x1 - 5), (m - (y1 - 5) - 1) * (2 * 4 + 1) + n - (x1 - 5)) = 0;
					}


					//双线性内插重采样
					float linerGray = (J + 1 - y2) * ((I + 1 - x2) * imageRight.at<uchar>(I, J) + (x2 - I) * imageRight.at<uchar>(I + 1, J))
						+ (y2 - J) * ((I + 1 - x2) * imageRight.at<uchar>(I, J + 1) + (x2 - I) * imageRight.at<uchar>(I + 1, J + 1));
					//辐射校正
					float radioGray = h0 + h1 * linerGray;//得到相应灰度

					sumRImg += radioGray;
					sumRImgSquare += radioGray * radioGray;
					//确定系数矩阵
					float gy = 0.5 * (imageRight.at<uchar>(I, J + 1) - imageRight.at<uchar>(I, J - 1));
					float gx = 0.5 * (imageRight.at<uchar>(I + 1, J) - imageRight.at<uchar>(I - 1, J));
					C.at<float>(dimension, 0) = 1; C.at<float>(dimension, 1) = linerGray;
					C.at<float>(dimension, 2) = gx; C.at<float>(dimension, 3) = x2 * gx;
					C.at<float>(dimension, 4) = y2 * gx; C.at<float>(dimension, 5) = gy;
					C.at<float>(dimension, 6) = x2 * gy; C.at<float>(dimension, 7) = y2 * gy;
					//常数项赋值
					L.at<float>(dimension, 0) = imageLeft.at<uchar>(m, n) - radioGray;
					dimension = dimension + 1;
					//左窗口加权平均
					float gyLeft = 0.5 * (imageLeft.at<uchar>(m, n + 1) - imageLeft.at<uchar>(m, n - 1));
					float gxLeft = 0.5 * (imageLeft.at<uchar>(m + 1, n) - imageLeft.at<uchar>(m - 1, n));
					sumgxSquare += gxLeft * gxLeft;
					sumgySquare += gyLeft * gyLeft;
					sumXgxSquare += m * gxLeft * gxLeft;
					sumYgySquare += n * gyLeft * gyLeft;
					//左片灰度相加用于求相关系数
					sumLImg += imageLeft.at<uchar>(m, n);
					sumLImgSquare += imageLeft.at<uchar>(m, n) * imageLeft.at<uchar>(m, n);
					sumLR += radioGray * imageLeft.at<uchar>(m, n);
				}
			}
			//计算相关系数
			float coefficent1 = sumLR - sumLImg * sumRImg / (matchsize * matchsize);
			float coefficent2 = sumLImgSquare - sumLImg * sumLImg / (matchsize * matchsize);
			float coefficent3 = sumRImgSquare - sumRImg * sumRImg / (matchsize * matchsize);
			CorrelationCoe = coefficent1 / sqrt(coefficent2 * coefficent3);
			//计算辐射畸变和几何变形的参数
			cv::Mat Nb = C.t() * P * C, Ub = C.t() * P * L;
			cv::Mat parameter = Nb.inv() * Ub;
			float dh0 = parameter.at<float>(0, 0); float dh1 = parameter.at<float>(1, 0);
			float da0 = parameter.at<float>(2, 0); float da1 = parameter.at<float>(3, 0); float da2 = parameter.at<float>(4, 0);
			float db0 = parameter.at<float>(5, 0); float db1 = parameter.at<float>(6, 0); float db2 = parameter.at<float>(7, 0);

			a0 = a0 + da0 + a0 * da1 + b0 * da2;
			a1 = a1 + a1 * da1 + b1 * da2;
			a2 = a2 + a2 * da1 + b2 * da2;
			b0 = b0 + db0 + a0 * db1 + b0 * db2;
			b1 = b1 + a1 * db1 + b1 * db2;
			b2 = b2 + a2 * db1 + b2 * db2;
			h0 = h0 + dh0 + h0 * dh1;
			h1 = h1 + h1 * dh1;

			float xt = sumXgxSquare / sumgxSquare;
			float yt = sumYgySquare / sumgySquare;
			xs = a0 + a1 * xt + a2 * yt;
			ys = b0 + b1 * xt + b2 * yt;
		}
		cv::Point3f tempPoint;
		tempPoint.x = xs;
		tempPoint.y = ys;
		tempPoint.z = CorrelationCoe;
		featureRightPointLST.push_back(tempPoint);
	}
	return 0;
}






/**
* @fn initializeFirstScanPaths
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief 初始化扫描路径
* @param[in] vec3dPts  三维匹配点
* @param[in] strPath DEM路径
* @param[in] dbDemRect DEM生成范围
* @param[out] paths 路径
* @version 1.0
* @retval TRUE 成功
* @retval FALSE 失败
* @par 修改历史：
* <作者>    <时间>   <版本编号>    <修改原因>\n
*/
void initializeFirstScanPaths(std::vector<path> &paths, unsigned short pathCount){
	/* 这里用了 4 个或者 8 个 */
	for (unsigned short i = 0; i < pathCount; ++i)
	{
		paths.push_back(path());
	}

	if (paths.size() >= 1)
	{
		paths[0].rowDiff = 0;
		paths[0].colDiff = -1;
		paths[0].index = 1;
	}

	if (paths.size() >= 2)
	{
		paths[1].rowDiff = -1;
		paths[1].colDiff = 0;
		paths[1].index = 2;
	}

	if (paths.size() >= 4) {
		paths[2].rowDiff = -1;
		paths[2].colDiff = 1;
		paths[2].index = 4;

		paths[3].rowDiff = -1;
		paths[3].colDiff = -1;
		paths[3].index = 7;
	}

	if (paths.size() >= 8) {
		paths[4].rowDiff = -2;
		paths[4].colDiff = 1;
		paths[4].index = 8;

		paths[5].rowDiff = -2;
		paths[5].colDiff = -1;
		paths[5].index = 9;

		paths[6].rowDiff = -1;
		paths[6].colDiff = -2;
		paths[6].index = 13;

		paths[7].rowDiff = -1;
		paths[7].colDiff = 2;
		paths[7].index = 15;
	}
}


// TODO
/**
* @fn initializeFirstScanPaths
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief 初始化扫描路径
* @param[in] vec3dPts  三维匹配点
* @param[in] strPath DEM路径
* @param[in] dbDemRect DEM生成范围
* @param[out] paths 路径
* @version 1.0
* @retval TRUE 成功
* @retval FALSE 失败
* @par 修改历史：
* <作者>    <时间>   <版本编号>    <修改原因>\n
*/
void initializeSecondScanPaths(std::vector<path> &paths, unsigned short pathCount){
	for (unsigned short i = 0; i < pathCount; ++i)
	{
		paths.push_back(path());
	}

	if (paths.size() >= 1) {
		paths[0].rowDiff = 0;
		paths[0].colDiff = 1;
		paths[0].index = 0;
	}

	if (paths.size() >= 2) {
		paths[1].rowDiff = 1;
		paths[1].colDiff = 0;
		paths[1].index = 3;
	}

	if (paths.size() >= 4) {
		paths[2].rowDiff = 1;
		paths[2].colDiff = 1;
		paths[2].index = 5;

		paths[3].rowDiff = 1;
		paths[3].colDiff = -1;
		paths[3].index = 6;
	}

	if (paths.size() >= 8) {
		paths[4].rowDiff = 2;
		paths[4].colDiff = 1;
		paths[4].index = 10;

		paths[5].rowDiff = 2;
		paths[5].colDiff = -1;
		paths[5].index = 11;

		paths[6].rowDiff = 1;
		paths[6].colDiff = -2;
		paths[6].index = 12;

		paths[7].rowDiff = 1;
		paths[7].colDiff = 2;
		paths[7].index = 14;
	}
}

// TODO
/**
* @fn initializeFirstScanPaths
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief 初始化扫描路径
* @param[in] vec3dPts  三维匹配点
* @param[in] strPath DEM路径
* @param[in] dbDemRect DEM生成范围
* @param[out] paths 路径
* @version 1.0
* @retval TRUE 成功
* @retval FALSE 失败
* @par 修改历史：
* <作者>    <时间>   <版本编号>    <修改原因>\n
*/uchar get_origin_value(cv::Mat& input_img, float sample, float line)
{
	int i = line;
	int j = sample;

	//注意处理边界问题，容易越界,若越界则暂时将灰度值设为0
	if (i + 1 >= input_img.rows || j + 1 >= input_img.cols)
	{
		//uchar* p = input_img.ptr<uchar>(i);
		return 0; //改于 0121.19:04
		//return p[j];
	}
	if (i <= 0 || j <= 0)
	{
		return 0;
	}

	uchar* p = input_img.ptr<uchar>(i);
	return p[j];
}


// TODO
/**
* @fn initializeFirstScanPaths
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief 初始化扫描路径
* @param[in] vec3dPts  三维匹配点
* @param[in] strPath DEM路径
* @param[in] dbDemRect DEM生成范围
* @param[out] paths 路径
* @version 1.0
* @retval TRUE 成功
* @retval FALSE 失败
* @par 修改历史：
* <作者>    <时间>   <版本编号>    <修改原因>\n
*/
float printProgress(unsigned int current, unsigned int max, int lastProgressPrinted){
	int progress = floor(100 * current / (float)max);
	if (progress >= lastProgressPrinted + 5) {
		lastProgressPrinted = lastProgressPrinted + 5;
		std::cout << lastProgressPrinted << "%" << std::endl;
	}
	return lastProgressPrinted;
}

float aggregateCost(int row, int col, int ThisZ, path &p, int rows, int cols, float  **TmpThisZMax, float  **TmpThisZMin, float ***C, float ***A, int sufferZ, int largePenalty, int smallPenalty){

	float aggregatedCost = 0;
	aggregatedCost += C[row][col][ThisZ]; /* 像素匹配的 cost 值 */
	/* 处理超出图像边缘外的情况 */
	if (row + p.rowDiff < 0 || row + p.rowDiff >= rows || col + p.colDiff < 0 || col + p.colDiff >= cols)
	{
		// border
		A[row][col][ThisZ] += aggregatedCost;
		return A[row][col][ThisZ];
	}

	/* 没有超出图像边缘外 */
	double minPrev, minPrevOther, prev, prevPlus, prevMinus;

	/* 初始值为最大值  */
	prev = minPrev = minPrevOther = prevPlus = prevMinus = 100000;//取一个最大值

	/*
	minPrev: 对应路径的视差代价最小值
	*/

	/* 在视差之间进行循环，理解其中的意思 */
	int PreheightRange = ceil((TmpThisZMax[row + p.rowDiff][col + p.colDiff] - TmpThisZMin[row + p.rowDiff][col + p.colDiff]) / Z_resolution + 1);
	for (int prevZ = 0; prevZ < PreheightRange; ++prevZ) //heightRange为上一个像素的搜索范围
	{
		unsigned short tmp = A[row + p.rowDiff][col + p.colDiff][prevZ];
		//找到这个路径下，前一个像素取不同disparity值时最小的A，这就是最后减去的那一项
		if (minPrev > tmp)
		{
			minPrev = tmp;
		}
		//前一个像素disparity取值为d是，其最小的A，这是公式中的第一项
		if (prevZ == ThisZ)    /* 视差与当前像素的视差相等 */
		{
			prev = tmp;
		}
		else if (prevZ == ThisZ + sufferZ)  /* 视差与当前像素的视差差一 */   //这是公式中的第三项，最后加的惩罚系数P1
		{
			prevPlus = tmp;
		}
		else if (prevZ == ThisZ - sufferZ) //这是公式中的第二项，最后加的惩罚系数P1
		{
			prevMinus = tmp;
		}
		else
		{
			/* 视差与当前像素视差大于 2 的情况 */
			//这是公式中的第四项，除了前一个像素的disparity取值为d的情况，其他情况下的最小的A，可是为什么加了两边P2  ？？？？？
			if (minPrevOther > tmp + largePenalty)
			{
				minPrevOther = tmp + largePenalty;
			}
		}
	} // for (int disp = 0; disp < heightRange; ++disp) 

	/* 计算最小值 */
	aggregatedCost += min(min((int)prevPlus + smallPenalty, (int)prevMinus + smallPenalty), min((int)prev, (int)minPrevOther + largePenalty));
	aggregatedCost -= minPrev;

	A[row][col][ThisZ] += aggregatedCost;
	return A[row][col][ThisZ];
}

// TODO
/**
* @fn initializeFirstScanPaths
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief 初始化扫描路径
* @param[in] vec3dPts  三维匹配点
* @param[in] strPath DEM路径
* @param[in] dbDemRect DEM生成范围
* @param[out] paths 路径
* @version 1.0
* @retval TRUE 成功
* @retval FALSE 失败
* @par 修改历史：
* <作者>    <时间>   <版本编号>    <修改原因>\n
*/
void aggregateCosts(int rows, int cols, float  **TmpThisZMax, float  **TmpThisZMin, float  ***C, float  ****A, float  ***S, float sufferZ, int largePenalty, int smallPenalty){
	int NumOfPaths = 8;
	std::vector<path> firstScanPaths;
	std::vector<path> secondScanPaths;
	/* 初始化扫描路径           */
	/* #define TheseAggreArgs.NumOfPaths 4 */
	initializeFirstScanPaths(firstScanPaths, NumOfPaths);
	initializeSecondScanPaths(secondScanPaths, NumOfPaths);

	/*
	9       8
	13  7   2   4    15
	1       0
	12  6   3   5    14
	11      10

	first : 1,2,4,7, 8, 9,13,15
	second:	0,3,5,6,10,11,12,14
	*/

	int lastProgressPrinted = 0;
	std::cout << "First scan..." << std::endl;
	/* 第一次扫描 */
	for (int row = 0; row < rows; ++row)
	{
		for (int col = 0; col < cols; ++col)
		{
			for (unsigned int path = 0; path < firstScanPaths.size(); ++path)
			{
				int heightRange = ceil((TmpThisZMax[row][col] - TmpThisZMin[row][col]) / Z_resolution + 1);

				for (int z = 0; z < heightRange; ++z)
				{
					S[row][col][z] += aggregateCost(row, col, z, firstScanPaths[path], rows, cols, TmpThisZMax, TmpThisZMin, C, A[path], sufferZ, largePenalty, smallPenalty);
				}
			}
		}
		lastProgressPrinted = printProgress(row, rows - 1, lastProgressPrinted);
	}

	lastProgressPrinted = 0;
	std::cout << "Second scan..." << std::endl;
	/* 第二次扫描，顺序与第一次扫描不一样 */
	for (int row = rows - 1; row >= 0; --row)
	{
		for (int col = cols - 1; col >= 0; --col)
		{
			for (unsigned int path = 0; path < secondScanPaths.size(); ++path)
			{
				int heightRange = ceil((TmpThisZMax[row][col] - TmpThisZMin[row][col]) / Z_resolution + 1);

				for (int z = 0; z < heightRange; ++z)
				{
					S[row][col][z] += aggregateCost(row, col, z, secondScanPaths[path], rows, cols, TmpThisZMax, TmpThisZMin, C, A[path], sufferZ, largePenalty, smallPenalty);
				}
			}
		}
		//打印进度条
		lastProgressPrinted = printProgress(rows - 1 - row, rows - 1, lastProgressPrinted);
	}
}



double groundCalCorr(int LengthOfWin, P3D ground, double roughRes,
	double UL_lon, double UL_lat, Mat img1, Mat img2, Mat RoughDem, vector<RPCMODEL>  rpcs, int baseindex, int searchIndex){
	double thinRes = roughRes / 3;
	//定义两个小窗口Mat
	cv::Mat win1(LengthOfWin, LengthOfWin, CV_8UC1); win1.setTo(0);
	cv::Mat win2(LengthOfWin, LengthOfWin, CV_8UC1); win2.setTo(0);

	//给win1和win2赋值
	for (int i = 0; i < LengthOfWin; i++)
	{
		uchar* p_win1 = win1.ptr<uchar>(i);
		uchar* p_win2 = win2.ptr<uchar>(i); //指针用于赋值

		for (int j = 0; j <= LengthOfWin; j++)
		{
			double tmpSampleBase, tmpLineBase, tmpSampleSearch, tmpLineSearch;
			P3D tmpGroundP3D; int tmpX_ind, tmpY_ind;
			tmpGroundP3D.lat = thinRes*i + ground.lat - thinRes*(LengthOfWin - 1) / 2;
			tmpGroundP3D.lon = thinRes*j + ground.lon - thinRes*(LengthOfWin - 1) / 2;
			tmpX_ind = (tmpGroundP3D.lon - UL_lon) / roughRes;
			tmpY_ind = (UL_lat - tmpGroundP3D.lat) / roughRes;
			/*float* p = RoughDem.ptr<float>(tmpY_ind);*/

			tmpGroundP3D.z = ground.z; //和ground的Z值一致
			//tmpGroundP3D.z = p[tmpX_ind]; //roughDem行列处的高程

			if (tmpX_ind < 0 || tmpX_ind > RoughDem.cols
				|| tmpY_ind < 0 || tmpY_ind> RoughDem.rows || abs(tmpGroundP3D.z)>100000)
			{
				return 0;
			}

			//1. 确定地面点的三维坐标
			API_LATLONGHEIFHT2LineSample(rpcs[baseindex], tmpGroundP3D.lat, tmpGroundP3D.lon, tmpGroundP3D.z, tmpSampleBase, tmpLineBase);
			API_LATLONGHEIFHT2LineSample(rpcs[searchIndex], tmpGroundP3D.lat, tmpGroundP3D.lon, tmpGroundP3D.z, tmpSampleSearch, tmpLineSearch);

			//2. 投影至像方，得到像方点坐标，进而得到像方点的灰度值，赋给小mat
			p_win1[j] = get_origin_value(img1, tmpSampleBase, tmpLineBase);
			p_win2[j] = get_origin_value(img2, tmpSampleSearch, tmpLineSearch);
			//测试
			//cout << (int)p_win1[j] << " ";
		}
		//cout << endl;
	}

	//计算win1和win2的相关系数并返回
	double corr2 = 0;
	double Amean2 = 0;
	double Bmean2 = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			Amean2 = Amean2 + dataA[n];
			Bmean2 = Bmean2 + dataB[n];
		}
	}
	Amean2 = Amean2 / (win1.rows * win1.cols);
	Bmean2 = Bmean2 / (win2.rows * win2.cols);
	double Cov = 0;
	double Astd = 0;
	double Bstd = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			//协方差
			Cov = Cov + (dataA[n] - Amean2) * (dataB[n] - Bmean2);
			//A的方差
			Astd = Astd + (dataA[n] - Amean2) * (dataA[n] - Amean2);
			//B的方差
			Bstd = Bstd + (dataB[n] - Bmean2) * (dataB[n] - Bmean2);
		}
	}
	corr2 = Cov / (sqrt(Astd * Bstd));
	return corr2;
	//https://blog.csdn.net/u013162930/article/details/50887019
};

double CalCorr(int LengthOfWin, Mat img1, Mat img2, ImgPoint P1, ImgPoint P2, float subpixelLength){
	//若出边界，则暂时舍弃该点，之后再考虑减小窗口等
	if (P1.sample - (LengthOfWin - 1) / 2 < 0 || P1.sample + (LengthOfWin - 1) / 2 > img1.cols
		|| P1.line - (LengthOfWin - 1) / 2 < 0 || P1.line + (LengthOfWin - 1) / 2 > img1.rows)
	{
		return 0;
	}

	//定义两个小窗口Mat
	cv::Mat win1(LengthOfWin, LengthOfWin, CV_8UC1); win1.setTo(0);
	cv::Mat win2(LengthOfWin, LengthOfWin, CV_8UC1); win2.setTo(0);

	//给win1和win2赋值
	for (int i = 0; i <LengthOfWin; i++)
	{
		uchar* p_win1 = win1.ptr<uchar>(i);
		uchar* p_win2 = win2.ptr<uchar>(i); //指针用于赋值

		for (int j = 0; j <= LengthOfWin; j++)
		{
			p_win1[j] = get_origin_value(img1, subpixelLength*i + P1.sample - subpixelLength*(LengthOfWin - 1) / 2, subpixelLength*j + P1.line - subpixelLength*(LengthOfWin - 1) / 2);
			p_win2[j] = get_origin_value(img2, subpixelLength*i + P2.sample - subpixelLength*(LengthOfWin - 1) / 2, subpixelLength*j + P2.line - subpixelLength*(LengthOfWin - 1) / 2);
			//测试
			//cout << (int)p_win1[j] << " ";
		}
		//cout << endl;
	}

	/*对比测试是否子像素
	cout << "-------------------------------" << endl;
	for (int i = 0; i <LengthOfWin; i++)
	{
	uchar* p_win1 = win1.ptr<uchar>(i);
	uchar* p_win2 = win2.ptr<uchar>(i); //指针用于赋值

	for (int j = 0; j <= LengthOfWin; j++)
	{
	p_win1[j] = get_origin_value(img1, i + P1.sample - (LengthOfWin - 1) / 2, j + P1.line - (LengthOfWin - 1) / 2);
	p_win2[j] = get_origin_value(img2, i + P2.sample - (LengthOfWin - 1) / 2, j + P2.line - (LengthOfWin - 1) / 2);
	//测试
	cout << (int)p_win1[j] << " ";
	}
	cout << endl;
	}

	cout << "-------------------------------" << endl;
	*/

	//计算win1和win2的相关系数并返回
	double corr2 = 0;
	double Amean2 = 0;
	double Bmean2 = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			Amean2 = Amean2 + dataA[n];
			Bmean2 = Bmean2 + dataB[n];
		}
	}
	Amean2 = Amean2 / (win1.rows * win1.cols);
	Bmean2 = Bmean2 / (win2.rows * win2.cols);
	double Cov = 0;
	double Astd = 0;
	double Bstd = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			//协方差
			Cov = Cov + (dataA[n] - Amean2) * (dataB[n] - Bmean2);
			//A的方差
			Astd = Astd + (dataA[n] - Amean2) * (dataA[n] - Amean2);
			//B的方差
			Bstd = Bstd + (dataB[n] - Bmean2) * (dataB[n] - Bmean2);
		}
	}
	corr2 = Cov / (sqrt(Astd * Bstd));
	return corr2;
	//https://blog.csdn.net/u013162930/article/details/50887019
};

void WriteArguPath(ofstream &writeArguPath, int sufferZ, int largePenalty, int smallPenalty, int N_times_res, int corrWin, int NumOfImgs){
	if (writeArguPath){
		writeArguPath << "sufferZ: " << sufferZ << endl <<
			"largePenalty: " << largePenalty << endl <<
			"smallPenalty: " << smallPenalty << endl <<
			"细化倍数 N_times_res: " << N_times_res << endl <<
			"相关系数窗口：" << corrWin << endl <<
			NumOfImgs << "张影像" << endl <<
			" z搜索间距: " << Z_resolution << "米" <<
			endl << "备注： 四片影像细化倍数为3倍，roughDem为imread(F:/A_工作笔记/开题/论文/已有工作/zhinvRoughDem.tif, -1);" << endl;
	}
}
