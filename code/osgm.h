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
#include <vector>
using namespace std;

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




