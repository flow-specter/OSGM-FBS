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

	// ... 为初始DEM（已上采样）分配空间，并赋值
	
	// 预处理sldem（上采样，去除无效值）
	Mat resize_sldem_ori = Mat::zeros(option_.dst_rows, option_.dst_cols, CV_32F);;
	resize(roughDem, resize_sldem_ori, resize_sldem_ori.size(), 0, 0, INTER_AREA); //将初始dem上采样至目标大小
	Mat resize_sldem = resize_sldem_ori.clone();
	osgm_util::deleteNodataValue(resize_sldem_ori, resize_sldem);

	// 分配空间，并赋值 【todo： 待检查】
	initial_Dem_ = new float32[dst_rows_ * dst_cols_]();
	
	for (int i = 0; i < dst_rows_; ++i) 
	{
		float* pResizeSLDEM = resize_sldem.ptr<float>(i);
		for (int j = 0; j < dst_cols_; ++j) 
		{
			initial_Dem_[i * dst_cols_ + j] = pResizeSLDEM[j];
		}
	}

	// 为局部高程最低值、最高值以及每个物方格网的一维序标分配空间，并赋值。
	Mat dilated_up_dem = resize_sldem_ori.clone();
	Mat erode_down_dem = resize_sldem_ori.clone();
	Mat adjustHeis = Mat::zeros(resize_sldem_ori.size(), CV_32F);

	osgm_util::getP3DSearchRange(resize_sldem_ori, dilated_up_dem, erode_down_dem, adjustHeis);

	// 根据adjustHeis计算代价空间size

	Mat adjustSize = adjustHeis/ Z_resolution + 1;
	size_ = cv::sum(adjustSize)[0] ; // 三维转为一维数组的size
	//cout << size << endl; 
	 
	// 为accumulat_cost_size_分配空间，并赋值
	accumulate_cost_size_ = new uint32[dst_cols_ * dst_rows_]();
	float pre_adjustSize = 0;
	for (int i = 0; i < dst_rows_; ++i) 
	{
		const float* p_adjustSize = adjustSize.ptr<float>(i);
		for (int j = 0; j < dst_cols_; ++j) 
		{
			accumulate_cost_size_[i * dst_cols_ + j] = p_adjustSize[j] + pre_adjustSize;
			pre_adjustSize = accumulate_cost_size_[i * option_.dst_cols + j];
		}
	}

	// 为per_cost_size_分配空间，并赋值 【未检查】
	per_cost_size_ = new uint32[dst_cols_ * dst_rows_]();
	for (int i = 0; i < dst_rows_; ++i)
	{
		const float* p_adjustSize = adjustSize.ptr<float>(i);
		for (int j = 0; j < dst_cols_; ++j)
		{
			per_cost_size_[i * dst_cols_ + j] = p_adjustSize[j];
		}
	}

	// 为局部高程最大值以及最低值分配空间，并赋值
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

	// 匹配代价（初始/聚合）
	cost_init_ = new uint8[size_]();
	for (int i = 0; i < size_; ++i) {
		cost_init_[i] = UINT8_MAX / 2;
	}

	cost_aggr_ = new uint16[size_]();
	cost_aggr_1_ = new uint8[size_]();
	cost_aggr_2_ = new uint8[size_]();
	cost_aggr_3_ = new uint8[size_]();
	cost_aggr_4_ = new uint8[size_]();
	cost_aggr_5_ = new uint8[size_]();
	cost_aggr_6_ = new uint8[size_]();
	cost_aggr_7_ = new uint8[size_]();
	cost_aggr_8_ = new uint8[size_]();

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

void OSGMMatchMVS::CostAggregation() 
{
	//// 测试左右路径
	//CostAggregateLeftRight(cost_aggr_1_, true);
	//CostAggregateLeftRight(cost_aggr_2_, false);

	//// 相加得到cost_aggr_
	//for (sint32 i = 0; i < size_; i++) {
	//	cost_aggr_[i] = cost_aggr_1_[i] + cost_aggr_2_[i] ;
	//}

	// 测试上->下路径 + 下->上路径

	CostAggregateLeftRight(cost_aggr_1_, true);
	CostAggregateLeftRight(cost_aggr_2_, false);
	CostAggregateUpDown(cost_aggr_3_);
	CostAggregateDownUp(cost_aggr_4_);
	for (sint32 i = 0; i < size_; i++) 
	{
		cost_aggr_[i] = cost_aggr_1_[i] + cost_aggr_2_[i] + cost_aggr_3_[i] + cost_aggr_4_[i];
	}

}

void OSGMMatchMVS::ComputeDisparity() const
{
	//cout << accumulate_cost_size_[101] << endl; // 相差3.5，实际上应为8，算上最低高程以及最高高程的话
	//cout << accumulate_cost_size_[102] << endl; // 应为15
	//cout << accumulate_cost_size_[103] << endl; // 

	//auto cost_ptr = cost_init_; // 测试初始代价
	auto cost_ptr = cost_aggr_; // 测试单路径聚合代价

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

void OSGMMatchMVS::CostAggregateLeftRight(uint8* cost_aggr, bool is_forward)
{
	cout << is_forward << endl;

	// 取惩罚参数P1,P2
	const float32& P1 = option_.p1;
	const float32& P2_Init = option_.p2_init;

	// 正向(左->右) ：is_forward = true ; direction = 1
	// 反向(右->左) ：is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// 聚合
	for (sint32 i = 0u; i < dst_rows_; i++) {

		//if (is_forward == false)
		//{
		//	cout << "反向第" <<i<<"行" <<endl;

		//}

		// 路径头为每一行的首(尾,dir=-1)列像素，首先取得三维格网(i,j,local_min)位置在一维数组中的下标，再取得其地址。
		int cost_leftmost_idx = accumulate_cost_size_[i * dst_cols_] - (dilated_up_Dem_[i * dst_cols_] - erode_down_Dem_[i*dst_cols_]) / Z_resolution;
		int cost_rightmost_idx = accumulate_cost_size_[i * dst_cols_ + dst_cols_ - 1] - (dilated_up_Dem_[i * dst_cols_ + dst_cols_ - 1] - erode_down_Dem_[i * dst_cols_ + dst_cols_ - 1]) / Z_resolution;;

		auto cost_init_row = (is_forward) ? (cost_init_ + cost_leftmost_idx) : (cost_init_ + cost_rightmost_idx);
		auto cost_aggr_row = (is_forward) ? (cost_aggr + cost_leftmost_idx) : (cost_aggr + cost_rightmost_idx);
		auto img_row = (is_forward) ? (initial_Dem_ + i * dst_cols_) : (initial_Dem_ + i * dst_cols_ + dst_cols_ - 1);

		// 路径上当前的初始高程值和上一个平面位置的初始高程值
		float32 gray = *img_row;
		float32 gray_last = *img_row;

		// 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
		// 左->右 与 右->左方向不同，则代价数组的大小也不同。
		// 或者直接拿该行最大的之类的？ 比如100之类的？
		std::vector<uint8> cost_last_path(100, UINT8_MAX);
		
		// 初始化：第一个像素的聚合代价值等于初始代价值
		::memcpy(cost_aggr_row, cost_init_row, per_cost_size_[i*dst_cols_] * sizeof(uint8));
		::memcpy(&cost_last_path[1], cost_aggr_row, per_cost_size_[i * dst_cols_] * sizeof(uint8));

		if (is_forward == true)
		{
			cost_init_row += per_cost_size_[i * dst_cols_]; // cost_init_[i][1][0]位置
			cost_aggr_row += per_cost_size_[i * dst_cols_];
		}
		else
		{
			int ctmp_idx = accumulate_cost_size_[i * dst_cols_ + dst_cols_ - 2] - (dilated_up_Dem_[i * dst_cols_ + dst_cols_ - 2] - erode_down_Dem_[i * dst_cols_ + dst_cols_ - 2]) / Z_resolution;
			cost_init_row = cost_init_ + ctmp_idx;
			cost_aggr_row = cost_aggr + ctmp_idx;
		}

		img_row += direction;

		// 路径上上个像素的最小代价值
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// 自方向上第2个像素开始按顺序聚合
		for (sint32 j = 0; j < dst_cols_ - 1; j++) //这里的j仅仅代表着个数，而非下标，且不考虑边界的情况
		{ 
			gray = *img_row;
			uint8 min_cost = UINT8_MAX;


			int thisHeightSize;
			if (is_forward == true)
			{
				thisHeightSize = per_cost_size_[i * dst_cols_ + j + 1];
			}
			else 
			{
				thisHeightSize = per_cost_size_[i * dst_cols_ + dst_cols_ -j -2];
			}

			for (sint32 d = 0; d < thisHeightSize; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
			    // 不处理边界值，从1开始存储
				//if (is_forward == false)
				//{
				//	cout << thisHeightSize << endl;
				//}
				
				const uint8  cost = cost_init_row[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;

				// 判断初始DEM给的信息是否有效，若无效，则直接加P2_init即可
				float32 deltaInitialHei;
				if (gray > -10000 && gray < 10000 && gray_last>-10000 && gray_last < 10000)
				{
					deltaInitialHei = abs(gray - gray_last);
				}
				else
				{
					deltaInitialHei = 0; 
				}

				const uint16  l4 = mincost_last_path + std::max(P1, P2_Init / (deltaInitialHei + 1));
				
				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_row[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// 重置上个像素的最小代价值和代价数组
			mincost_last_path = min_cost;

			for (int i = 0; i < cost_last_path.size(); ++i) 
			{
				cost_last_path[i] = UINT8_MAX;
			}

			::memcpy(&cost_last_path[1], cost_aggr_row, thisHeightSize * sizeof(uint8));

			// ...更新下一个像素的地址
			if (is_forward == true)
			{
				cost_init_row += per_cost_size_[i * dst_cols_ + j + 1];
				cost_aggr_row += per_cost_size_[i * dst_cols_ + j + 1];
			}
			else
			{
				// [i][dst_cols_ - j - 2][0]位置的地址
				float32 local_max = dilated_up_Dem_[i * dst_cols_ + dst_cols_ - j - 2];
				float32 local_min = erode_down_Dem_[i * dst_cols_ + dst_cols_ - j - 2];
				int tmp_index = accumulate_cost_size_[i * dst_cols_] - (local_max - local_min) / Z_resolution;
				cost_init_row = cost_init_ + tmp_index;
				cost_aggr_row = cost_init_ + tmp_index;
			}

			img_row += direction;

			// 像素值重新赋值
			gray_last = gray;
		}
	}
}

/* \brief: 从上往下聚合 */
void OSGMMatchMVS::CostAggregateUpDown(uint8* cost_aggr)
{
	// P1,P2
	const float32& P1 = option_.p1;
	const float32& P2_Init = option_.p2_init;

	// 正向(上->下) ：is_forward = true ; direction = 1
	const sint32 direction = 1;

	// 聚合
	for (sint32 j = 0; j < dst_cols_; j++) {
		// 路径头为每一列的首(尾,dir=-1)行像素
		// 即cost_init的第[0][j][0]元素所在的地址
		int cost_upmost_idx = accumulate_cost_size_[j] - (dilated_up_Dem_[j] - erode_down_Dem_[j]) / Z_resolution;
		auto cost_init_col = cost_init_ + cost_upmost_idx ;
		auto cost_aggr_col = cost_aggr + cost_upmost_idx; 
		auto img_col = initial_Dem_ + j;

		// 路径上当前灰度值和上一个灰度值
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
		std::vector<uint8> cost_last_path(100 + 2, UINT8_MAX);

		// 初始化：第一个像素的聚合代价值等于初始代价值
		::memcpy(cost_aggr_col, cost_init_col, per_cost_size_[j] * sizeof(uint8));
		::memcpy(&cost_last_path[1], cost_aggr_col, per_cost_size_[j] * sizeof(uint8));

		//cost_init_[1][j][0]位置
		int cost_secondUpMost_idx = accumulate_cost_size_[dst_cols_ + j] - (dilated_up_Dem_[dst_cols_ + j] - erode_down_Dem_[dst_cols_ + j]) / Z_resolution;
		cost_init_col = cost_init_ + cost_secondUpMost_idx;
		cost_aggr_col = cost_aggr + cost_secondUpMost_idx;
		img_col += dst_cols_;

		// 路径上上个像素的最小代价值
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// 自方向上第2个像素开始按顺序聚合
		for (sint32 i = 1; i < dst_rows_ ; i++)  
		{
			
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;

			// 每个平面位置的坐标为： [i][j]列，对于从上到下聚合而言，上一个平面位置的坐标为[i-1][j]。
			for (sint32 d = 0; d < per_cost_size_[i*dst_cols_ + j]; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))

				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;

				// 判断初始DEM给的信息是否有效，若无效，则直接加P2_init即可
				float32 deltaInitialHei;
				if (gray > -10000 && gray < 10000 && gray_last>-10000 && gray_last < 10000)
				{
					deltaInitialHei = abs(gray - gray_last);
				}
				else
				{
					deltaInitialHei = 0;
				}

				const uint16  l4 = mincost_last_path + std::max(P1, P2_Init / (deltaInitialHei + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// 重置上个像素的最小代价值和代价数组
			mincost_last_path = min_cost;
			::memcpy(&cost_last_path[1], cost_aggr_col, per_cost_size_[i* dst_cols_ + j] * sizeof(uint8));

			// 下一个像素,下一个平面坐标是[i+1][j]
			// TODO 检查用per_cost_size是否正确
			/*{
				int nextRowIndex1 = accumulate_cost_size_[(i + 1) * dst_cols_ + j] - (dilated_up_Dem_[(i + 1) * dst_cols_ + j] - erode_down_Dem_[(i + 1) * dst_cols_ + j])/Z_resolution;
				cout << nextRowIndex1 << endl;
				int nextRowIndex2 = accumulate_cost_size_[(i + 1) * dst_cols_ + j] - per_cost_size_[(i + 1) * dst_cols_ + j] + 1;
				cout << nextRowIndex2 << endl;
			}*/

			int nextRowIndex = accumulate_cost_size_[(i + 1) * dst_cols_ + j] - per_cost_size_[(i + 1) * dst_cols_ + j] + 1;
			cost_init_col = cost_init_ + nextRowIndex;
			cost_aggr_col = cost_aggr + nextRowIndex;
			img_col += direction * dst_cols_;

			// 像素值重新赋值
			gray_last = gray;
		}
	}

}

void OSGMMatchMVS::CostAggregateDownUp(uint8* cost_aggr)
{

	// P1,P2
	const float32& P1 = option_.p1;
	const float32& P2_Init = option_.p2_init;

	// 聚合,逐列地从下至上聚合
	for (sint32 j = 0; j < dst_cols_; j++) {
		// 从下向上是每一列的最后一行像素
		// 即cost_init的第[dst_rows -1][j][0]元素所在的地址
		int cost_downmost_idx = accumulate_cost_size_[(dst_rows_ - 1) * dst_cols_ + j] - per_cost_size_[(dst_rows_ - 1)* dst_cols_ + j] + 1;
		auto cost_init_col = cost_init_ + cost_downmost_idx;
		auto cost_aggr_col = cost_aggr + cost_downmost_idx;
		auto img_col = initial_Dem_ + (dst_rows_ - 1) * dst_cols_ + j;

		// 路径上当前灰度值和上一个灰度值
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
		std::vector<uint8> cost_last_path(100 + 2, UINT8_MAX);

		// 初始化：第一个像素的聚合代价值等于初始代价值
		::memcpy(cost_aggr_col, cost_init_col, per_cost_size_[(dst_rows_ - 1) * dst_cols_ + j] * sizeof(uint8));
		::memcpy(&cost_last_path[1], cost_aggr_col, per_cost_size_[(dst_rows_ - 1) * dst_cols_ + j] * sizeof(uint8));

		//cost_init_[dst_rows_ - 2][j][0]位置
		int cost_secondDownMost_idx = accumulate_cost_size_[(dst_rows_ - 2) * dst_cols_ + j] - per_cost_size_[(dst_rows_ - 2) * dst_cols_ + j] + 1;
		cost_init_col = cost_init_ + cost_secondDownMost_idx;
		cost_aggr_col = cost_aggr + cost_secondDownMost_idx;
		img_col -= dst_cols_;

		// 路径上上个像素的最小代价值
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// 自下而上方向上第2个像素开始按顺序聚合
		for (sint32 i = dst_rows_-2; i > 0 ; i--)
		{

			gray = *img_col;
			uint8 min_cost = UINT8_MAX;

			// 每个平面位置的坐标为： [i][j]列，对于从下到上聚合而言，上一个平面位置的坐标为[i+1][j]。
			for (sint32 d = 0; d < per_cost_size_[i * dst_cols_ + j]; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))

				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;

				// 判断初始DEM给的信息是否有效，若无效，则直接加P2_init即可
				float32 deltaInitialHei;
				if (gray > -10000 && gray < 10000 && gray_last>-10000 && gray_last < 10000)
				{
					deltaInitialHei = abs(gray - gray_last);
				}
				else
				{
					deltaInitialHei = 0;
				}

				const uint16  l4 = mincost_last_path + std::max(P1, P2_Init / (deltaInitialHei + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// 重置上个像素的最小代价值和代价数组
			mincost_last_path = min_cost;
			::memcpy(&cost_last_path[1], cost_aggr_col, per_cost_size_[i * dst_cols_ + j] * sizeof(uint8));


			// TODO 检查用per_cost_size是否正确
			/*{
				int nextRowIndex1 = accumulate_cost_size_[(i + 1) * dst_cols_ + j] - (dilated_up_Dem_[(i + 1) * dst_cols_ + j] - erode_down_Dem_[(i + 1) * dst_cols_ + j])/Z_resolution;
				cout << nextRowIndex1 << endl;
				int nextRowIndex2 = accumulate_cost_size_[(i + 1) * dst_cols_ + j] - per_cost_size_[(i + 1) * dst_cols_ + j] + 1;
				cout << nextRowIndex2 << endl;
			}*/

			// 下一个像素,下一个平面坐标是[i-1][j]
			int nextRowIndex = accumulate_cost_size_[(i - 1) * dst_cols_ + j] - per_cost_size_[(i - 1) * dst_cols_ + j] + 1;
			cost_init_col = cost_init_ + nextRowIndex;
			cost_aggr_col = cost_aggr + nextRowIndex;
			img_col -=   dst_cols_;

			// 像素值重新赋值
			gray_last = gray;
		}
	}

}

//void OSGMMatchMVS::CostAggregateDownUp(uint8* cost_aggr)
//{
//
//	assert(width > 0 && height > 0 && max_disparity > min_disparity);
//
//	// 视差范围
//	const sint32 disp_range = max_disparity - min_disparity;
//
//	// P1,P2
//	const auto& P1 = p1;
//	const auto& P2_Init = p2_init;
//
//	// 正向(上->下) ：is_forward = true ; direction = 1
//	// 反向(下->上) ：is_forward = false; direction = -1;
//	const sint32 direction = is_forward ? 1 : -1;
//
//	// 聚合
//	for (sint32 j = 0; j < width; j++) {
//		// 路径头为每一列的首(尾,dir=-1)行像素
//		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
//		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
//		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);
//
//		// 路径上当前灰度值和上一个灰度值
//		uint8 gray = *img_col;
//		uint8 gray_last = *img_col;
//
//		// 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
//		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);
//
//		// 初始化：第一个像素的聚合代价值等于初始代价值
//		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
//		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));
//		cost_init_col += direction * width * disp_range;
//		cost_aggr_col += direction * width * disp_range;
//		img_col += direction * width;
//
//		// 路径上上个像素的最小代价值
//		uint8 mincost_last_path = UINT8_MAX;
//		for (auto cost : cost_last_path) {
//			mincost_last_path = std::min(mincost_last_path, cost);
//		}
//
//		// 自方向上第2个像素开始按顺序聚合
//		for (sint32 i = 0; i < height - 1; i++) {
//			gray = *img_col;
//			uint8 min_cost = UINT8_MAX;
//			for (sint32 d = 0; d < disp_range; d++) {
//				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
//
//				const uint8  cost = cost_init_col[d];
//				const uint16 l1 = cost_last_path[d + 1];
//				const uint16 l2 = cost_last_path[d] + P1;
//				const uint16 l3 = cost_last_path[d + 2] + P1;
//				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));
//
//				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);
//
//				cost_aggr_col[d] = cost_s;
//				min_cost = std::min(min_cost, cost_s);
//			}
//
//			// 重置上个像素的最小代价值和代价数组
//			mincost_last_path = min_cost;
//			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));
//
//			// 下一个像素
//			cost_init_col += direction * width * disp_range;
//			cost_aggr_col += direction * width * disp_range;
//			img_col += direction * width;
//
//			// 像素值重新赋值
//			gray_last = gray;
//		}
//	}
//}





