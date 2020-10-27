/**
* @copyright Copyright(C), 2020, PMRS Lab, IRSA, CAS
* @file osgm.h
* @date 2020.09.24
* @author Ҷ�ּ� ljye_bj@163.com
* @brief osgm�����ж�����Ҫ�ĺ�����
* @version 1.0
* @par �޸���ʷ:
* <����>  <ʱ��>  <�汾���>  <����>\n
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
// ���ϵ������С����ƥ��
void vectorsort(std::vector < cv::Point3f>& Temp_sort)
{
	for (int i = 0; i < Temp_sort.size() - 1; i++) {
		float tem = 0;
		float temx = 0;
		float temy = 0;
		// �ڲ�forѭ���������ڵ�����Ԫ�ؽ��бȽ�
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

//���ϵ��ͼ��ƥ��
float Get_coefficient(cv::Mat matchLeftWindow, cv::Mat imageRight, int x, int y)
{
	//��������������ȷ�����������ڵĴ�С
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
	//��ʼ�������ϵ��
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

	int matchsize = 9;//���ϵ���������δ��ڵı߳�
	int half_matchsize = matchsize / 2;//�߳���һ��

	std::vector<cv::Point3f> featurePointRight;//��Ƭƥ�䵽������

	float lowst_door = 0.7; //���ϵ����ƥ�����ֵ
	int dist_width = 30;//����Ƭ������Ƭ����Ծ��룬������ͨ���ֶ��۲�

	//����f���ݵ�Ԥ���� ɾ�������Ϲ淶������
	for (size_t i = 0; i < featurePointLeft.size(); i++)
	{
		//����� 5 = half_matchsize + 1
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
	//�����󴰿ڵ�С����
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
		aveLImg = aveLImg / (matchsize * matchsize);//��ȡ�󴰿�ƽ��ֵ
		//����ĳ��ֵ
		for (int m = 0; m < matchsize; m++)
		{
			for (int n = 0; n < matchsize; n++)
			{
				matchLeftWindow.at<float>(m, n) = matchLeftWindow.at<float>(m, n) - aveLImg;
			}
		}
		//***************************���Ҵ��ڽ��м���
		//����Ԥ���Ҵ��ڵ�λ��
		int halflengthsize = 10; //�������İ뾶
		std::vector < cv::Point3f> tempfeatureRightPoint;
		//ȥ���ܵ�������ĵ�
		for (int ii = -halflengthsize; ii <= halflengthsize; ii++)
		{
			for (int jj = -halflengthsize; jj <= halflengthsize; jj++)
			{
				//Ϊ��ʡ�¡��� �ѱ�Ե���޵Ķ�����û��
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
	//�õ�������Ƭ��ͬ�����ʼֵ

	/*��ʽ��ʼ��С����ƥ��*/
	std::vector<cv::Point3f> featureRightPointLST;//�洢��С����ƥ�䵽�ĵ�
	//�󼸺λ���ĳ�ʼֵ
	cv::Mat formerP = cv::Mat::eye(2 * featurePointLeft.size(), 2 * featurePointLeft.size(), CV_32F)/*Ȩ����*/,
		formerL = cv::Mat::zeros(2 * featurePointLeft.size(), 1, CV_32F)/*������*/,
		formerA = cv::Mat::zeros(2 * featurePointLeft.size(), 6, CV_32F)/*ϵ������*/;
	for (int i = 0; i < featurePointLeft.size(); i++)
	{
		float x1 = featurePointLeft.at(i).x;
		float y1 = featurePointLeft.at(i).y;
		float x2 = featurePointRight.at(i).x;
		float y2 = featurePointRight.at(i).y;
		float coef = featurePointRight.at(i).z;//��ʼͬ��������ϵ����ΪȨ��
		formerP.at<float>(2 * i, 2 * i) = coef;
		formerP.at<float>(2 * i + 1, 2 * i + 1) = coef;
		formerL.at<float>(2 * i, 0) = x2;
		formerL.at<float>(2 * i + 1, 0) = y2;
		formerA.at<float>(2 * i, 0) = 1; formerA.at<float>(2 * i, 1) = x1; formerA.at<float>(2 * i, 2) = y1;
		formerA.at<float>(2 * i + 1, 3) = 1; formerA.at<float>(2 * i + 1, 4) = x1; formerA.at<float>(2 * i + 1, 5) = y1;
	}
	cv::Mat Nbb = formerA.t() * formerP * formerA, U = formerA.t() * formerP * formerL;
	cv::Mat formerR = Nbb.inv() * U;
	//��ʼ������С����ƥ��
	for (int i = 0; i < featurePointLeft.size(); i++)
	{
		//����ĵ�����ʼֵ
		float x1 = featurePointLeft.at(i).x;
		float y1 = featurePointLeft.at(i).y;
		float x2 = featurePointRight.at(i).x;
		float y2 = featurePointRight.at(i).y;
		//���λ������������ʼֵ
		float a0 = formerR.at<float>(0, 0); float a1 = formerR.at<float>(1, 0); float a2 = formerR.at<float>(2, 0);
		float b0 = formerR.at<float>(3, 0); float b1 = formerR.at<float>(4, 0); float b2 = formerR.at<float>(5, 0);
		//������������ʼֵ
		float h0 = 0, h1 = 1;

		//����һ�����ϵ��С��ǰһ�Σ�����ֹͣ
		float beforeCorrelationCoe = 0/*ǰһ�����ϵ��*/, CorrelationCoe = 0;
		float xs = 0, ys = 0;

		while (beforeCorrelationCoe <= CorrelationCoe)
		{
			beforeCorrelationCoe = CorrelationCoe;
			cv::Mat C = cv::Mat::zeros(matchsize * matchsize, 8, CV_32F);//ϵ������matchsizeΪ��ƬĿ�괰�ڴ�С
			cv::Mat L = cv::Mat::zeros(matchsize * matchsize, 1, CV_32F);//������
			cv::Mat P = cv::Mat::eye(matchsize * matchsize, matchsize * matchsize, CV_32F);//Ȩ����
			float sumgxSquare = 0, sumgySquare = 0, sumXgxSquare = 0, sumYgySquare = 0;
			int dimension = 0;//���ھ���ֵ
			float sumLImg = 0, sumLImgSquare = 0, sumRImg = 0, sumRImgSquare = 0, sumLR = 0;

			for (int m = x1 - half_matchsize; m <= x1 + half_matchsize; m++)
			{
				for (int n = y1 - half_matchsize; n <= y1 + half_matchsize; n++)
				{
					float x2 = a0 + a1 * m + a2 * n;
					float y2 = b0 + b1 * m + b2 * n;
					int I = std::floor(x2); int J = std::floor(y2);//�������Ա������������
					if (I <= 1 || I >= imageRight.rows - 1 || J <= 1 || J >= imageRight.cols - 1)
					{
						I = 2; J = 2; P.at<float>((m - (y1 - 5) - 1) * (2 * 4 + 1) + n - (x1 - 5), (m - (y1 - 5) - 1) * (2 * 4 + 1) + n - (x1 - 5)) = 0;
					}


					//˫�����ڲ��ز���
					float linerGray = (J + 1 - y2) * ((I + 1 - x2) * imageRight.at<uchar>(I, J) + (x2 - I) * imageRight.at<uchar>(I + 1, J))
						+ (y2 - J) * ((I + 1 - x2) * imageRight.at<uchar>(I, J + 1) + (x2 - I) * imageRight.at<uchar>(I + 1, J + 1));
					//����У��
					float radioGray = h0 + h1 * linerGray;//�õ���Ӧ�Ҷ�

					sumRImg += radioGray;
					sumRImgSquare += radioGray * radioGray;
					//ȷ��ϵ������
					float gy = 0.5 * (imageRight.at<uchar>(I, J + 1) - imageRight.at<uchar>(I, J - 1));
					float gx = 0.5 * (imageRight.at<uchar>(I + 1, J) - imageRight.at<uchar>(I - 1, J));
					C.at<float>(dimension, 0) = 1; C.at<float>(dimension, 1) = linerGray;
					C.at<float>(dimension, 2) = gx; C.at<float>(dimension, 3) = x2 * gx;
					C.at<float>(dimension, 4) = y2 * gx; C.at<float>(dimension, 5) = gy;
					C.at<float>(dimension, 6) = x2 * gy; C.at<float>(dimension, 7) = y2 * gy;
					//�����ֵ
					L.at<float>(dimension, 0) = imageLeft.at<uchar>(m, n) - radioGray;
					dimension = dimension + 1;
					//�󴰿ڼ�Ȩƽ��
					float gyLeft = 0.5 * (imageLeft.at<uchar>(m, n + 1) - imageLeft.at<uchar>(m, n - 1));
					float gxLeft = 0.5 * (imageLeft.at<uchar>(m + 1, n) - imageLeft.at<uchar>(m - 1, n));
					sumgxSquare += gxLeft * gxLeft;
					sumgySquare += gyLeft * gyLeft;
					sumXgxSquare += m * gxLeft * gxLeft;
					sumYgySquare += n * gyLeft * gyLeft;
					//��Ƭ�Ҷ�������������ϵ��
					sumLImg += imageLeft.at<uchar>(m, n);
					sumLImgSquare += imageLeft.at<uchar>(m, n) * imageLeft.at<uchar>(m, n);
					sumLR += radioGray * imageLeft.at<uchar>(m, n);
				}
			}
			//�������ϵ��
			float coefficent1 = sumLR - sumLImg * sumRImg / (matchsize * matchsize);
			float coefficent2 = sumLImgSquare - sumLImg * sumLImg / (matchsize * matchsize);
			float coefficent3 = sumRImgSquare - sumRImg * sumRImg / (matchsize * matchsize);
			CorrelationCoe = coefficent1 / sqrt(coefficent2 * coefficent3);
			//����������ͼ��α��εĲ���
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
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ʼ��ɨ��·��
* @param[in] vec3dPts  ��άƥ���
* @param[in] strPath DEM·��
* @param[in] dbDemRect DEM���ɷ�Χ
* @param[out] paths ·��
* @version 1.0
* @retval TRUE �ɹ�
* @retval FALSE ʧ��
* @par �޸���ʷ��
* <����>    <ʱ��>   <�汾���>    <�޸�ԭ��>\n
*/
void initializeFirstScanPaths(std::vector<path> &paths, unsigned short pathCount){
	/* �������� 4 ������ 8 �� */
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
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ʼ��ɨ��·��
* @param[in] vec3dPts  ��άƥ���
* @param[in] strPath DEM·��
* @param[in] dbDemRect DEM���ɷ�Χ
* @param[out] paths ·��
* @version 1.0
* @retval TRUE �ɹ�
* @retval FALSE ʧ��
* @par �޸���ʷ��
* <����>    <ʱ��>   <�汾���>    <�޸�ԭ��>\n
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
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ʼ��ɨ��·��
* @param[in] vec3dPts  ��άƥ���
* @param[in] strPath DEM·��
* @param[in] dbDemRect DEM���ɷ�Χ
* @param[out] paths ·��
* @version 1.0
* @retval TRUE �ɹ�
* @retval FALSE ʧ��
* @par �޸���ʷ��
* <����>    <ʱ��>   <�汾���>    <�޸�ԭ��>\n
*/uchar get_origin_value(cv::Mat& input_img, float sample, float line)
{
	int i = line;
	int j = sample;

	//ע�⴦��߽����⣬����Խ��,��Խ������ʱ���Ҷ�ֵ��Ϊ0
	if (i + 1 >= input_img.rows || j + 1 >= input_img.cols)
	{
		//uchar* p = input_img.ptr<uchar>(i);
		return 0; //���� 0121.19:04
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
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ʼ��ɨ��·��
* @param[in] vec3dPts  ��άƥ���
* @param[in] strPath DEM·��
* @param[in] dbDemRect DEM���ɷ�Χ
* @param[out] paths ·��
* @version 1.0
* @retval TRUE �ɹ�
* @retval FALSE ʧ��
* @par �޸���ʷ��
* <����>    <ʱ��>   <�汾���>    <�޸�ԭ��>\n
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
	aggregatedCost += C[row][col][ThisZ]; /* ����ƥ��� cost ֵ */
	/* ������ͼ���Ե������ */
	if (row + p.rowDiff < 0 || row + p.rowDiff >= rows || col + p.colDiff < 0 || col + p.colDiff >= cols)
	{
		// border
		A[row][col][ThisZ] += aggregatedCost;
		return A[row][col][ThisZ];
	}

	/* û�г���ͼ���Ե�� */
	double minPrev, minPrevOther, prev, prevPlus, prevMinus;

	/* ��ʼֵΪ���ֵ  */
	prev = minPrev = minPrevOther = prevPlus = prevMinus = 100000;//ȡһ�����ֵ

	/*
	minPrev: ��Ӧ·�����Ӳ������Сֵ
	*/

	/* ���Ӳ�֮�����ѭ����������е���˼ */
	int PreheightRange = ceil((TmpThisZMax[row + p.rowDiff][col + p.colDiff] - TmpThisZMin[row + p.rowDiff][col + p.colDiff]) / Z_resolution + 1);
	for (int prevZ = 0; prevZ < PreheightRange; ++prevZ) //heightRangeΪ��һ�����ص�������Χ
	{
		unsigned short tmp = A[row + p.rowDiff][col + p.colDiff][prevZ];
		//�ҵ����·���£�ǰһ������ȡ��ͬdisparityֵʱ��С��A�����������ȥ����һ��
		if (minPrev > tmp)
		{
			minPrev = tmp;
		}
		//ǰһ������disparityȡֵΪd�ǣ�����С��A�����ǹ�ʽ�еĵ�һ��
		if (prevZ == ThisZ)    /* �Ӳ��뵱ǰ���ص��Ӳ���� */
		{
			prev = tmp;
		}
		else if (prevZ == ThisZ + sufferZ)  /* �Ӳ��뵱ǰ���ص��Ӳ��һ */   //���ǹ�ʽ�еĵ�������ӵĳͷ�ϵ��P1
		{
			prevPlus = tmp;
		}
		else if (prevZ == ThisZ - sufferZ) //���ǹ�ʽ�еĵڶ�����ӵĳͷ�ϵ��P1
		{
			prevMinus = tmp;
		}
		else
		{
			/* �Ӳ��뵱ǰ�����Ӳ���� 2 ����� */
			//���ǹ�ʽ�еĵ��������ǰһ�����ص�disparityȡֵΪd���������������µ���С��A������Ϊʲô��������P2  ����������
			if (minPrevOther > tmp + largePenalty)
			{
				minPrevOther = tmp + largePenalty;
			}
		}
	} // for (int disp = 0; disp < heightRange; ++disp) 

	/* ������Сֵ */
	aggregatedCost += min(min((int)prevPlus + smallPenalty, (int)prevMinus + smallPenalty), min((int)prev, (int)minPrevOther + largePenalty));
	aggregatedCost -= minPrev;

	A[row][col][ThisZ] += aggregatedCost;
	return A[row][col][ThisZ];
}

// TODO
/**
* @fn initializeFirstScanPaths
* @date 2020.09.24
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ʼ��ɨ��·��
* @param[in] vec3dPts  ��άƥ���
* @param[in] strPath DEM·��
* @param[in] dbDemRect DEM���ɷ�Χ
* @param[out] paths ·��
* @version 1.0
* @retval TRUE �ɹ�
* @retval FALSE ʧ��
* @par �޸���ʷ��
* <����>    <ʱ��>   <�汾���>    <�޸�ԭ��>\n
*/
void aggregateCosts(int rows, int cols, float  **TmpThisZMax, float  **TmpThisZMin, float  ***C, float  ****A, float  ***S, float sufferZ, int largePenalty, int smallPenalty){
	int NumOfPaths = 8;
	std::vector<path> firstScanPaths;
	std::vector<path> secondScanPaths;
	/* ��ʼ��ɨ��·��           */
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
	/* ��һ��ɨ�� */
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
	/* �ڶ���ɨ�裬˳�����һ��ɨ�費һ�� */
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
		//��ӡ������
		lastProgressPrinted = printProgress(rows - 1 - row, rows - 1, lastProgressPrinted);
	}
}



double groundCalCorr(int LengthOfWin, P3D ground, double roughRes,
	double UL_lon, double UL_lat, Mat img1, Mat img2, Mat RoughDem, vector<RPCMODEL>  rpcs, int baseindex, int searchIndex){
	double thinRes = roughRes / 3;
	//��������С����Mat
	cv::Mat win1(LengthOfWin, LengthOfWin, CV_8UC1); win1.setTo(0);
	cv::Mat win2(LengthOfWin, LengthOfWin, CV_8UC1); win2.setTo(0);

	//��win1��win2��ֵ
	for (int i = 0; i < LengthOfWin; i++)
	{
		uchar* p_win1 = win1.ptr<uchar>(i);
		uchar* p_win2 = win2.ptr<uchar>(i); //ָ�����ڸ�ֵ

		for (int j = 0; j <= LengthOfWin; j++)
		{
			double tmpSampleBase, tmpLineBase, tmpSampleSearch, tmpLineSearch;
			P3D tmpGroundP3D; int tmpX_ind, tmpY_ind;
			tmpGroundP3D.lat = thinRes*i + ground.lat - thinRes*(LengthOfWin - 1) / 2;
			tmpGroundP3D.lon = thinRes*j + ground.lon - thinRes*(LengthOfWin - 1) / 2;
			tmpX_ind = (tmpGroundP3D.lon - UL_lon) / roughRes;
			tmpY_ind = (UL_lat - tmpGroundP3D.lat) / roughRes;
			/*float* p = RoughDem.ptr<float>(tmpY_ind);*/

			tmpGroundP3D.z = ground.z; //��ground��Zֵһ��
			//tmpGroundP3D.z = p[tmpX_ind]; //roughDem���д��ĸ߳�

			if (tmpX_ind < 0 || tmpX_ind > RoughDem.cols
				|| tmpY_ind < 0 || tmpY_ind> RoughDem.rows || abs(tmpGroundP3D.z)>100000)
			{
				return 0;
			}

			//1. ȷ����������ά����
			API_LATLONGHEIFHT2LineSample(rpcs[baseindex], tmpGroundP3D.lat, tmpGroundP3D.lon, tmpGroundP3D.z, tmpSampleBase, tmpLineBase);
			API_LATLONGHEIFHT2LineSample(rpcs[searchIndex], tmpGroundP3D.lat, tmpGroundP3D.lon, tmpGroundP3D.z, tmpSampleSearch, tmpLineSearch);

			//2. ͶӰ���񷽣��õ��񷽵����꣬�����õ��񷽵�ĻҶ�ֵ������Сmat
			p_win1[j] = get_origin_value(img1, tmpSampleBase, tmpLineBase);
			p_win2[j] = get_origin_value(img2, tmpSampleSearch, tmpLineSearch);
			//����
			//cout << (int)p_win1[j] << " ";
		}
		//cout << endl;
	}

	//����win1��win2�����ϵ��������
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
			//Э����
			Cov = Cov + (dataA[n] - Amean2) * (dataB[n] - Bmean2);
			//A�ķ���
			Astd = Astd + (dataA[n] - Amean2) * (dataA[n] - Amean2);
			//B�ķ���
			Bstd = Bstd + (dataB[n] - Bmean2) * (dataB[n] - Bmean2);
		}
	}
	corr2 = Cov / (sqrt(Astd * Bstd));
	return corr2;
	//https://blog.csdn.net/u013162930/article/details/50887019
};

double CalCorr(int LengthOfWin, Mat img1, Mat img2, ImgPoint P1, ImgPoint P2, float subpixelLength){
	//�����߽磬����ʱ�����õ㣬֮���ٿ��Ǽ�С���ڵ�
	if (P1.sample - (LengthOfWin - 1) / 2 < 0 || P1.sample + (LengthOfWin - 1) / 2 > img1.cols
		|| P1.line - (LengthOfWin - 1) / 2 < 0 || P1.line + (LengthOfWin - 1) / 2 > img1.rows)
	{
		return 0;
	}

	//��������С����Mat
	cv::Mat win1(LengthOfWin, LengthOfWin, CV_8UC1); win1.setTo(0);
	cv::Mat win2(LengthOfWin, LengthOfWin, CV_8UC1); win2.setTo(0);

	//��win1��win2��ֵ
	for (int i = 0; i <LengthOfWin; i++)
	{
		uchar* p_win1 = win1.ptr<uchar>(i);
		uchar* p_win2 = win2.ptr<uchar>(i); //ָ�����ڸ�ֵ

		for (int j = 0; j <= LengthOfWin; j++)
		{
			p_win1[j] = get_origin_value(img1, subpixelLength*i + P1.sample - subpixelLength*(LengthOfWin - 1) / 2, subpixelLength*j + P1.line - subpixelLength*(LengthOfWin - 1) / 2);
			p_win2[j] = get_origin_value(img2, subpixelLength*i + P2.sample - subpixelLength*(LengthOfWin - 1) / 2, subpixelLength*j + P2.line - subpixelLength*(LengthOfWin - 1) / 2);
			//����
			//cout << (int)p_win1[j] << " ";
		}
		//cout << endl;
	}

	/*�ԱȲ����Ƿ�������
	cout << "-------------------------------" << endl;
	for (int i = 0; i <LengthOfWin; i++)
	{
	uchar* p_win1 = win1.ptr<uchar>(i);
	uchar* p_win2 = win2.ptr<uchar>(i); //ָ�����ڸ�ֵ

	for (int j = 0; j <= LengthOfWin; j++)
	{
	p_win1[j] = get_origin_value(img1, i + P1.sample - (LengthOfWin - 1) / 2, j + P1.line - (LengthOfWin - 1) / 2);
	p_win2[j] = get_origin_value(img2, i + P2.sample - (LengthOfWin - 1) / 2, j + P2.line - (LengthOfWin - 1) / 2);
	//����
	cout << (int)p_win1[j] << " ";
	}
	cout << endl;
	}

	cout << "-------------------------------" << endl;
	*/

	//����win1��win2�����ϵ��������
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
			//Э����
			Cov = Cov + (dataA[n] - Amean2) * (dataB[n] - Bmean2);
			//A�ķ���
			Astd = Astd + (dataA[n] - Amean2) * (dataA[n] - Amean2);
			//B�ķ���
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
			"ϸ������ N_times_res: " << N_times_res << endl <<
			"���ϵ�����ڣ�" << corrWin << endl <<
			NumOfImgs << "��Ӱ��" << endl <<
			" z�������: " << Z_resolution << "��" <<
			endl << "��ע�� ��ƬӰ��ϸ������Ϊ3����roughDemΪimread(F:/A_�����ʼ�/����/����/���й���/zhinvRoughDem.tif, -1);" << endl;
	}
}
