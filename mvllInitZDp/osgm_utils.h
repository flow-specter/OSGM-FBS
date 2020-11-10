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
#include <algorithm>

#ifndef SAFE_DELETE
#define SAFE_DELETE(P) {if(P) delete[](P);(P)=nullptr;}
#endif

using namespace cv;
using namespace std;
constexpr auto Z_resolution = 0.5;
constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

namespace osgm_util
{
	//············ 影像预处理工作集（去除nodata，分离通道等）
	bool preConvert2OneChannel(const Mat &src_bgr, Mat &dst_gray);

	/*
	\brief: 读取路径文件夹中的所有影像，并放入vector中
	\para[in]: ImgPath 
	\para[out] imgs 输出的影像
	*/
	bool preReadImgs(string ImgPath, vector<Mat> &imgs);


	/*
	\brief: 将多视影像做直方图匹配
	\para[in] src 输入的多视影像
	\para[in] baseIdx 直方图匹配的目标影像的序标
	\para[out] dst_uint8 直方图匹配后的多视影像,类型为uint8*
	*/
	bool prehistMatch(vector<Mat>& src, int baseIdx, vector<uint8*>& dst_uint8);

	void deleteNodataValue(const Mat& src, Mat& dst);

	bool histMatch_Value(Mat matSrc, Mat matDst, Mat& matRet);

	int histogram_Matching(Mat Src, Mat matDst, Mat& matResult);


	/*
	brief: getP3DSearchRange 得到初始粗DEM每个格网的最高高程以及最低高程，以设定搜索范围。
	para[in]: roughDem 已经经过resize的sldem
	para[in]: dilated_up_dem 膨胀后的结果，即每个三维点高程的搜索最高高程
	para[in]: erode_down_dem 腐蚀后的结果，即每个三维点高程的搜索最低高程
	para[in]: adjustHeis, 最高高程与最低高程之差
	*/
	void getP3DSearchRange(Mat roughDem, Mat& dilated_up_dem, Mat& erode_down_dem, Mat& adjustHeis, sint8 structuringElementSize = 5);


	//············ census工具集
	// census变换

	/**
	 * \brief census变换
	 * \param source	输入，影像数据
	 * \param census	输出，census值数组
	 * \param width		输入，影像宽
	 * \param height	输入，影像高
	 */
	void census_transform_5x5(const uint8* source, uint32* census, const sint32& width, const sint32& height);
	void census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height);
	// Hamming距离
	uint8 Hamming32(const uint32& x, const uint32& y);
	uint8 Hamming64(const uint64& x, const uint64& y);

	// ············ 物方投影至像方工具集
	/*
	\brief: 将目标区域行列号转换为经纬度坐标
	*/
	void rowCol2LatLon(int row, int col, float UL_lon, float UL_lat, float reso, float& lat, float& lon);


	/*
	\brief: 判断像点(sample,line)所在的计算窗口是否在影像内
	\para[in]: sample 像方的sample坐标，或者说x坐标
	\para[in]: line 像方的line坐标，或者说y坐标
	\para[in]: corrWin 相关系数窗口大小
	\para[in]: img_rows 影像行数
	\para[in]: img_cols 影像列数
	\para[out]: 返回的bool值，是否在影像内
	*/
	bool ifInImg(double sample, double line, int corrWin, int img_rows, int img_cols);

	/*
	\brief: chooseBaseImg 确定物方平面点投影至影像集时所确定的基准影像
	\para[in]: Imgs 多视影像
	\para[in]: rpcs 多视影像对应的有理函数模型
	\para[in]: img_rows 影像行数
	\para[in]: img_cols 影像列数
	\para[in]: lat 纬度
	\para[in]: lon 经度，和上面的纬度组成确定了该平面点位置
	\para[in]: localMin 该平面位置的最低搜索高程
	\para[in]: localMax 该平面位置的最高搜索高程
	\para[in]: corrWin 当设定的相关系数计算窗口为规则方形窗口时，corrWin为设定的窗口边长
	\para[out]: BaseIdx 选取的该平面位置的基准影像
	*/
	void chooseBaseImg(vector<uint8*> Imgs, vector<RPCMODEL> rpcs, int img_rows, int img_cols, float lat, float lon, float localMin, float localMax, int corrWin, int& BaseIdx);


	// ... 图像后处理工具集
	/**
	 * \brief 中值滤波
	 * \param in				输入，源数据
	 * \param out				输出，目标数据
	 * \param width				输入，宽度
	 * \param height			输入，高度
	 * \param wnd_size			输入，窗口宽度
	 */
	void MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height, const sint32 wnd_size);


	/**
	 * \brief 剔除小连通区
	 * \param disparity_map		输入，视差图
	 * \param width				输入，宽度
	 * \param height			输入，高度
	 * \param diff_insame		输入，同一连通区内的局部像素差异
	 * \param min_speckle_aera	输入，最小连通区面积
	 * \param invalid_val		输入，无效值
	 */
	void RemoveSpeckles(float32* disparity_map, const sint32& width, const sint32& height, const sint32& diff_insame, const uint32& min_speckle_aera, const float32& invalid_val);

	// ... 图像预处理数据集

}


