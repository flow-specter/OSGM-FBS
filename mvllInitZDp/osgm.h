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
#include <vector>
using namespace std;

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




