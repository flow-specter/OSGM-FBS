#include "OSGMMatchMVS.h"

#include <chrono>
using namespace std::chrono;

OSGMMatchMVS::OSGMMatchMVS()
{
}

OSGMMatchMVS::~OSGMMatchMVS()
{
	Release();
	is_initialized_ = false;
}

bool OSGMMatchMVS::Initialize(const sint32& img_rows, const sint32& img_cols, const uint8 num_imgs, const Mat &roughDem, const OSGM_Option& option,sint32 &cost_size)
{
	// ... 参数赋值
	option_ = option;
	img_rows_ = img_rows;
	img_cols_ = img_cols;
	dst_rows_ = option_.dst_rows;
	dst_cols_ = option_.dst_cols;
	reso_ = option_.reso;
	UL_lat_ = option_.UL_lat;
	UL_lon_ = option_.UL_lon;

	// ... 开辟内存空间

	// census值

	const sint32 img_size = img_rows * img_cols;

	for (int i = 0; i < num_imgs; ++i) {
		uint32* census_tmp = new uint32[img_size]();
		census_imgs_.push_back(census_tmp);
	}

	// 匹配代价（初始/聚合）

	// 计算高程范围
	Mat resize_sldem_ori = Mat::zeros(option_.dst_rows, option_.dst_cols, CV_32F);;
	resize(roughDem, resize_sldem_ori, resize_sldem_ori.size(), 0, 0, INTER_AREA);

	Mat dilated_up_dem = resize_sldem_ori.clone();
	Mat erode_down_dem = resize_sldem_ori.clone();
	Mat adjustHeis = Mat::zeros(resize_sldem_ori.size(), CV_32F);

	osgm_util::getP3DSearchRange(resize_sldem_ori, dilated_up_dem, erode_down_dem, adjustHeis);

	// 根据adjustHeis计算代价空间size

	Mat adjustSize = adjustHeis/ Z_resolution + 1;
	sint32 size = cv::sum(adjustSize)[0] ; // 三维转为一维数组的size
	cout << size << endl;

	//sint32 size = cv::sum(adjustHeis)[0] / Z_resolution + 1; // 三维转为一维数组的size
	//if (size < 0) return false;
	//cost_size = size; 
	//cout << size << endl;

	// 为accumulat_cost_size分配空间，并赋值

	accumulate_cost_size_ = new float32[size]();
	float pre_adjustSize = 0;
	for (int i = 0; i < dst_rows_; ++i) {
		float* p_adjustSize = adjustSize.ptr<float>(i);
		for (int j = 0; j < dst_cols_; ++j) {
			accumulate_cost_size_[i * option_.dst_cols + j] = p_adjustSize[j] + pre_adjustSize;
			pre_adjustSize = accumulate_cost_size_[i * option_.dst_cols + j];
		}
	}

	//cout << static_cast<float>(adjustHeis.data[101]) << " " << static_cast<float>(adjustHeis.data[102]) << " " << static_cast<float>(adjustHeis.data[103] )<< endl;
	//cout << adjustSize.data[101] << " " << adjustSize.data[102] << " " << adjustSize.data[103] << endl;
	//cout << accumulate_cost_size_[101] << " "<< accumulate_cost_size_[102] << " "<<accumulate_cost_size_[103]<<endl;

	// 为局部高程最大值以及最低值赋值
	dilated_up_Dem_ = new float32[dst_rows_ * dst_cols_]();
	erode_down_Dem_ = new float32[dst_rows_ * dst_cols_]();

	// 为osgm类中的局部高程范围指针赋值
	for (int i = 0; i < dst_rows_; ++i) {
		float* p_dilated = dilated_up_dem.ptr<float>(i);
		float* p_eroded = erode_down_dem.ptr<float>(i);
		for (int j = 0; j < dst_cols_; ++j) {
			dilated_up_Dem_[i * dst_cols_ + j] = p_dilated[j];
			erode_down_Dem_[i * dst_cols_ + j] = p_eroded[j];
		}
	}

	//cout << dilated_up_Dem_[0] << endl;
	//cout << erode_down_Dem_[60 * dst_cols_ + 65] << endl;

	// 匹配代价（初始/聚合）
	cost_init_ = new uint8[size]();
	for (int i = 0; i < size; ++i) {
		cost_init_[i] = UINT8_MAX / 2;
	}

	cost_aggr_ = new uint16[size]();
	cost_aggr_1_ = new uint8[size]();
	cost_aggr_2_ = new uint8[size]();
	cost_aggr_3_ = new uint8[size]();
	cost_aggr_4_ = new uint8[size]();
	cost_aggr_5_ = new uint8[size]();
	cost_aggr_6_ = new uint8[size]();
	cost_aggr_7_ = new uint8[size]();
	cost_aggr_8_ = new uint8[size]();

	// 目标DEM图，res
	res_ = new float32[dst_rows_ * dst_cols_]();

	is_initialized_ = census_imgs_[0] && cost_init_ && cost_aggr_ && res_;

	return is_initialized_;
}

bool OSGMMatchMVS::Match(const vector<uint8*> imgs, const vector<RPCMODEL> rpcs, float32* res)
{
	if (!is_initialized_) {
		return false;
	}

	imgs_ = imgs;
	rpcs_ = rpcs;

	for (int i = 0; i < imgs.size(); ++i) {
		if (imgs[i] == nullptr) {
			return false;
		}
	}

	// census变换
	CensusTransform();

	// 代价计算
	ComputeCost();

	// 代价聚合
	CostAggregation();

	// 视差计算
	ComputeDisparity();

	// 输出最终的dem图
	memcpy(res, res_, dst_rows_ * dst_cols_ * sizeof(float32));

	return true;
}

void OSGMMatchMVS::CensusTransform() const
{
	int num_imgs = census_imgs_.size();

	// 多视影像census变换
	if (option_.census_size == Census5x5) {

		for (int i = 0; i < num_imgs; ++i) {
			osgm_util::census_transform_5x5(imgs_[i], static_cast<uint32*>(census_imgs_[i]), img_cols_, img_rows_);
		}
	}
	else {
		for (int i = 0; i < num_imgs; ++i) {
			osgm_util::census_transform_9x7(imgs_[i], reinterpret_cast<uint64*>(census_imgs_[i]), img_cols_, img_rows_);
		}
	}

}

void OSGMMatchMVS::ComputeCost() const 
{

	cout << dst_rows_ << endl;
	cout << dst_cols_ << endl;
	// 计算代价（基于Hamming距离）
	for (sint32 i = 0; i < dst_rows_; i++) {
		for (sint32 j = 0; j < dst_cols_; j++) {

			// 确定base影像
			int tmp_baseidx = INT_MAX;
			float tmp_lat, tmp_lon;
			osgm_util::rowCol2LatLon(i, j, UL_lon_, UL_lat_, reso_, tmp_lat, tmp_lon);
			//cout << reso_ << endl;


			float localMin = erode_down_Dem_[i * dst_cols_ + j];
			float localMax = dilated_up_Dem_[i * dst_cols_ + j];

			// 局部高程范围无效，则不计算该平面位置
			if (localMin == 0 || localMax==0) {
				continue;
			}

			if (option_.census_size == Census5x5) {
				osgm_util::chooseBaseImg(imgs_, rpcs_, img_rows_, img_cols_, tmp_lat, tmp_lon, localMin, localMax, 5, tmp_baseidx);
			}
			else {
				osgm_util::chooseBaseImg(imgs_, rpcs_, img_rows_, img_cols_, tmp_lat, tmp_lon, localMin, localMax, 9, tmp_baseidx);
			}

			if (tmp_baseidx == INT_MAX) {
				continue;
			}

			uint32* census_base = census_imgs_[tmp_baseidx];


			// 确定每个高程上的多视影像上的像方坐标
			for (sint32 hei = localMin; hei <= localMax; hei += Z_resolution) {

				// 计算该三维点对应的cost_idx		
				int cost_idx = accumulate_cost_size_[i * dst_cols_ + j] - (localMax - hei) / Z_resolution;
				auto& cost = cost_init_[cost_idx];

				// 确定baseIdx的投影像点位置BaseSample,BaseLine
				double tmp_baseSample, tmp_baseLine;
				API_LATLONGHEIFHT2LineSample(rpcs_[tmp_baseidx], tmp_lat, tmp_lon, hei, tmp_baseSample, tmp_baseLine);
				int census_base_val_idx = ceil(tmp_baseLine) * img_cols_ + ceil(tmp_baseSample);
				uint32 census_base_val = census_base[census_base_val_idx];

				int NumOfCalPairs = 0; // 计算了几对窗口

				for (int i = 0; i < imgs_.size(); i++) {
					double tmp_searchSample, tmp_searchLine;

					if (i == tmp_baseidx) continue;
					API_LATLONGHEIFHT2LineSample(rpcs_[i], tmp_lat, tmp_lon, hei, tmp_searchSample, tmp_searchLine);
					
					bool validSearch = osgm_util::ifInImg(tmp_searchSample, tmp_searchLine, 9,img_rows_,img_cols_); // 投影至搜索影像上的坐标是否在影像有效范围内

					if (validSearch) {
						int tmp_census_search_val_idx = ceil(tmp_searchLine) * img_cols_ + ceil(tmp_searchSample);
						uint32 census_search_val = census_imgs_[i][tmp_census_search_val_idx];
						if (NumOfCalPairs == 0) {
							cost = osgm_util::Hamming32(census_base_val, census_search_val);
						}
						else {
							cost += osgm_util::Hamming32(census_base_val, census_search_val);
						}
						NumOfCalPairs++;
					}
				}
				if (NumOfCalPairs == 0) { //仅在base影像上有点
					cost = UINT8_MAX / 2;
					continue;
				}
				cost = cost / NumOfCalPairs;
				//cout << (int)cost << endl;
			}
		}
	}
}

void OSGMMatchMVS::CostAggregation() const
{
}

//void OSGMMatchMVS::CostAggregation() const
//{
//	// 路径聚合
//   // 1、左->右/右->左
//   // 2、上->下/下->上
//   // 3、左上->右下/右下->左上
//   // 4、右上->左上/左下->右上
//   //
//   // ↘ ↓ ↙   5  3  7
//   // →    ←	 1    2
//   // ↗ ↑ ↖   8  4  6
//   //
//
//	// 计算三维cost数组的size
//	const auto& min_disparity = option_.min_disparity;
//	const auto& max_disparity = option_.max_disparity;
//	assert(max_disparity > min_disparity);
//
//	const sint32 size = width_ * height_ * (max_disparity - min_disparity);
//	if (size <= 0) {
//		return;
//	}
//
//	// 惩罚参数赋值
//	const auto& P1 = option_.p1;
//	const auto& P2_Int = option_.p2_init;
//
//	if (option_.num_paths == 4 || option_.num_paths == 8) {
//		// 左右聚合
//		osgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_1_, true);
//		osgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_2_, false);
//		// 上下聚合
//		osgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_3_, true);
//		osgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_4_, false);
//	}
//
//	if (option_.num_paths == 8) {
//		// 对角线1聚合
//		osgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_5_, true);
//		osgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_6_, false);
//		// 对角线2聚合
//		osgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_7_, true);
//		osgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_8_, false);
//	}
//
//	// 把4/8个方向加起来
//	for (sint32 i = 0; i < size; i++) {
//		if (option_.num_paths == 4 || option_.num_paths == 8) {
//			cost_aggr_[i] = cost_aggr_1_[i] + cost_aggr_2_[i] + cost_aggr_3_[i] + cost_aggr_4_[i];
//		}
//		if (option_.num_paths == 8) {
//			cost_aggr_[i] += cost_aggr_5_[i] + cost_aggr_6_[i] + cost_aggr_7_[i] + cost_aggr_8_[i];
//		}
//	}
//}

void OSGMMatchMVS::ComputeDisparity() const
{
	//cout << accumulate_cost_size_[101] << endl; // 相差3.5，实际上应为8，算上最低高程以及最高高程的话
	//cout << accumulate_cost_size_[102] << endl; // 应为15
	//cout << accumulate_cost_size_[103] << endl; // 


	auto cost_ptr = cost_init_;

	// 逐像素计算最佳高程
	for (sint32 i = 0; i < dst_rows_; ++i) {
		for (sint32 j = 0; j < dst_cols_; ++j) {
			uint16 min_cost = UINT16_MAX;
			uint16 max_cost = 0;
			float32 best_hei = 0;
			sint32 local_min = erode_down_Dem_[i * dst_cols_ + j];
			sint32 local_max = dilated_up_Dem_[i * dst_cols_ + j];

			// 遍历自适应高程范围内的所有代价值，输出最小代价值以及对应的高程。
			for (float32 hei = local_min; hei <=local_max; hei += Z_resolution) {
				const sint32 hei_idx = (hei - local_min) / Z_resolution;

				int cost_idx = accumulate_cost_size_[i * dst_cols_ + j] - (local_max - hei) / Z_resolution;
				const auto& cost = cost_ptr[cost_idx];



				//cout << cost << endl;
				//cout << static_cast<uint16>(cost) << endl;

				if (min_cost > cost) {
					min_cost = cost;
					best_hei = hei;
				}

				max_cost = max(max_cost, static_cast<uint16>(cost));
			}

			//if (best_hei != 0) cout << best_hei << endl;
			//res_[i * dst_cols_ + j] = static_cast<float>(best_hei);

			//res_[i * dst_cols_ + j] = static_cast<float>(best_hei);

			 //最小代价值对应的高程值即为平面位置的最优高程
			if (max_cost == min_cost) { // 无效
				res_[i * dst_cols_ + j] = Invalid_Float;
			}
			else {
				// 如果所有高程下的代价值都一样，则该像素无效
				//cout << min_cost << " " << max_cost << " " << best_hei << endl;
				res_[i * dst_cols_ + j] = static_cast<float>(best_hei);
			}
		}
	}

	// 测试最佳高程是否正确
	Mat testRes = Mat::zeros(dst_rows_,dst_cols_,CV_32F);
	for (int i = 0; i < dst_rows_; ++i) {
		float* p_testRes = testRes.ptr<float>(i);
		for (int j = 0; j < dst_cols_; ++j) {
			if (res_[i * dst_cols_ + j] == 0)
			{
				p_testRes[j] = Invalid_Float;
			}
			else 
			{
				p_testRes[j] = res_[i * dst_cols_ + j];
			}
		}
	}

 }

void OSGMMatchMVS::Release()
{
	// 释放内存
	int num_imgs = census_imgs_.size();
	for (int i = 0; i < num_imgs; ++i) {
		SAFE_DELETE(census_imgs_[i]);
	}

	SAFE_DELETE(erode_down_Dem_);
	SAFE_DELETE(dilated_up_Dem_);
	SAFE_DELETE(cost_init_);
	SAFE_DELETE(accumulate_cost_size_);
	SAFE_DELETE(cost_aggr_);
	SAFE_DELETE(cost_aggr_1_);
	SAFE_DELETE(cost_aggr_2_);
	SAFE_DELETE(cost_aggr_3_);
	SAFE_DELETE(cost_aggr_4_);
	SAFE_DELETE(cost_aggr_5_);
	SAFE_DELETE(cost_aggr_6_);
	SAFE_DELETE(cost_aggr_7_);
	SAFE_DELETE(cost_aggr_8_);
	SAFE_DELETE(res_);
}




