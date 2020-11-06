#pragma once
#include "osgm_utils.h"

using namespace cv;
using namespace std;

/*
brief: OSGM多视匹配类
*/
class OSGMMatchMVS
{
public:
	OSGMMatchMVS();
	~OSGMMatchMVS();

	/** \brief Census窗口尺寸类型 */
	enum CensusSize {
		Census5x5,
		Census9x7 = 0
	};

	/** \brief osgm参数结构体 */
	struct OSGM_Option {
		uint8	num_paths;		// 聚合路径数
		uint8	dilate_erode_win;	// 膨胀腐蚀的窗口大小   
		CensusSize census_size;		// census窗口尺寸

		// P1,P2 
		// P2 = P2_int / (Ip-Iq)
		sint32  p1;				// 惩罚项参数P1
		sint32  p2_init;			// 惩罚项参数P2

		// 目标范围位置
		uint16	dst_rows;		// 目标影像行数 
		uint16	dst_cols;		// 目标影像列数
		float64 reso;			
		float64 UL_lat;
		float64 UL_lon;

		OSGM_Option() : num_paths(8), dilate_erode_win(3), census_size(Census9x7), p1(10), p2_init(150), dst_rows(0), dst_cols(0),reso(0.0019531249), UL_lat(0), UL_lon(0){}
	};

public:
	/**
	* \brief 类的初始化,确定需要分配的内存大小，参数的预设置等
	* para[in] img_rows 多视影像的行数
	* para[in] img_cols 多视影像的列数
	* para[in] num_imgs 多视影像的张数
	* para[in] roughdem 未resize的roughDEM
	* para[in] option   输入，算法参数
	* \param 
	*/
	bool Initialize(const sint32& img_rows, const sint32& img_cols, const uint8 num_imgs, const Mat& roughDem, const OSGM_Option& option, sint32& cost_size);

	/*
	 * \brief 执行匹配
	 * \param[in] imgs 多视影像
	 * \param[in] rpcs 多视影像对应的几何模型
	 * \param[out] res		输出DEM的指针（需要预先开辟内存）
	 */
	bool Match(const vector<uint8*> imgs, const vector<RPCMODEL> rpcs, float32* res);

private: // 私有函数

	/** \brief Census变换 */
	void CensusTransform() const;

	/** \brief 代价计算	 */
	void ComputeCost() const;

	/** \brief 代价聚合	 */
	void CostAggregation() const;

	/** \brief 视差计算	 */
	void ComputeDisparity() const;

	/** \brief 内存释放	 */
	void Release();


private:
	/** \brief OSGM参数	 */
	OSGM_Option option_;

	/** \brief 目标DEM列数	 */
	sint32 dst_cols_;

	/** \brief 目标影像行数	 */
	sint32 dst_rows_;

	/** \brief 目标区域左上角经度	 */
	float64 UL_lon_;

	/** \brief 目标区域左上角纬度	 */
	float64 UL_lat_;

	/** \brief 目标区域分辨率 GSD	 */
	float32 reso_;

	/** \brief 每一个数组元素对应着该位置之前的高程范围数值，便于计算cost的位置	 */
	float32* accumulate_cost_size_;



	/** \brief 多视影像数据	 */
	vector<uint8*> imgs_;

	/** \brief 多视影像列数	 */
	sint32 img_cols_;

	/** \brief 多视影像行数	 */
	sint32 img_rows_;

	/** \brief 多视影像数据对应的rpc	 */
	vector<RPCMODEL> rpcs_;

	/** \brief 多视影像数据对应的census值	 */
	vector<uint32*> census_imgs_;

	/** \brief 初始匹配代价	 */
	uint8* cost_init_; 

	/** \brief 聚合匹配代价	 */
	uint16* cost_aggr_;

	// ↘ ↓ ↙   5  3  7
	// →    ←	 1    2
	// ↗ ↑ ↖   8  4  6
	/** \brief 聚合匹配代价-方向1	*/
	uint8* cost_aggr_1_;
	/** \brief 聚合匹配代价-方向2	*/
	uint8* cost_aggr_2_;
	/** \brief 聚合匹配代价-方向3	*/
	uint8* cost_aggr_3_;
	/** \brief 聚合匹配代价-方向4	*/
	uint8* cost_aggr_4_;
	/** \brief 聚合匹配代价-方向5	*/
	uint8* cost_aggr_5_;
	/** \brief 聚合匹配代价-方向6	*/
	uint8* cost_aggr_6_;
	/** \brief 聚合匹配代价-方向7	*/
	uint8* cost_aggr_7_;
	/** \brief 聚合匹配代价-方向8	*/
	uint8* cost_aggr_8_;

	/** \brief 初始DEM，initialDem	 */
	float* initial_Dem_;

	/** \brief DEM搜索最大高程，dilated_up_dem_	 */
	float* dilated_up_Dem_;

	/** \brief DEM搜索最低高程，dilated_up_dem_	 */
	float* erode_down_Dem_;

	/** \brief 目标DEM图，res	 */
	float* res_;

	/** \brief 是否初始化标志	 */
	bool is_initialized_;
};

