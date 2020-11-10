#include "osgm_utils.h"
#include <cassert>
#include <queue>

bool osgm_util::preConvert2OneChannel(const Mat& src_bgr, Mat& dst_gray)
{
	if (src_bgr.channels() != 3) {
		cout << "src影像请输入三通道影像"<<endl;
		return false;
	}
	std::vector<Mat> src_bgrs;
	split(src_bgr, (vector<Mat>&)src_bgrs);
	dst_gray = src_bgrs[0].clone();
	return true;
}

bool osgm_util::preReadImgs(string ImgPath, vector<Mat>& imgs)
{
	return false;
}

bool osgm_util::prehistMatch(vector<Mat>& src, int baseIdx, vector<uint8*>& dst_uint8)
{
	int num_imgs = src.size();
	Mat baseImg = src[baseIdx].clone();
	vector<Mat> dst(num_imgs);
	for (int i = 0; i < num_imgs; ++i) {
		dst[i] = Mat::zeros(src[i].size(), CV_8U);
	}

	for (int i = 0; i < num_imgs; ++i) 
	{
		// 非目标影像不需要匹配
		if (i != baseIdx) 
		{
			Mat src_img = src[i].clone();;
			// 直方图匹配，并剔除无效值
			Mat tmp;
			osgm_util::histogram_Matching(src_img, baseImg, tmp);
			osgm_util::deleteNodataValue(tmp, dst[i]);
			dst_uint8[i] = dst[i].data;
		}
		else {

			dst_uint8[i] = baseImg.data;
		}
	}
	
	return true;
}

void osgm_util::census_transform_5x5(const uint8* source, uint32* census, const sint32& width,
	const sint32& height)
{
	if (source == nullptr || census == nullptr || width <= 5 || height <= 5) {
		return;
	}

	// 逐像素计算census值
	for (sint32 i = 2; i < height - 2; i++) {
		for (sint32 j = 2; j < width - 2; j++) {

			// 中心像素值
			const uint8 gray_center = source[i * width + j];

			// 遍历大小为5x5的窗口内邻域像素，逐一比较像素值与中心像素值的的大小，计算census值
			uint32 census_val = 0u;
			for (sint32 r = -2; r <= 2; r++) {
				for (sint32 c = -2; c <= 2; c++) {
					census_val <<= 1;
					const uint8 gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}

			// 中心像素的census值
			census[i * width + j] = census_val;
		}
	}
}

void osgm_util::census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height)
{
	if (source == nullptr || census == nullptr || width <= 9 || height <= 7) {
		return;
	}

	// 逐像素计算census值
	for (sint32 i = 4; i < height - 4; i++) {
		for (sint32 j = 3; j < width - 3; j++) {

			// 中心像素值
			const uint8 gray_center = source[i * width + j];

			// 遍历大小为5x5的窗口内邻域像素，逐一比较像素值与中心像素值的的大小，计算census值
			uint64 census_val = 0u;
			for (sint32 r = -4; r <= 4; r++) {
				for (sint32 c = -3; c <= 3; c++) {
					census_val <<= 1;
					const uint8 gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}
			// 中心像素的census值
			//cout << i * width + j << endl;
			census[i * width + j] = census_val;
		}
	}

	// 测试census变换是否正确
	Mat testRes = Mat::zeros(height, width, CV_32F);
	for (int i = 0; i < height; ++i) 
	{
		float* p_testRes = testRes.ptr<float>(i);
		for (int j = 0; j < width; ++j) 
		{
			p_testRes[j] = census[i * width + j];
		}
	}


}

uint8 osgm_util::Hamming32(const uint32& x, const uint32& y)
{
	uint32 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

uint8 osgm_util::Hamming64(const uint64& x, const uint64& y)
{
	uint64 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

void osgm_util::rowCol2LatLon(int row, int col, float UL_lon, float UL_lat, float reso, float& lat, float& lon)
{
		lat = UL_lat + row * reso;
		lon = UL_lon + col * reso;	
}

/*
\brief: 判断像点(sample,line)所在的计算窗口是否在影像内
\para[in]: sample 像方的sample坐标，或者说x坐标
\para[in]: line 像方的line坐标，或者说y坐标
\para[in]: corrWin 相关系数窗口大小
\para[in]: img_rows 影像行数
\para[in]: img_cols 影像列数
\para[out]: 返回的bool值，是否在影像内
*/
bool osgm_util::ifInImg(double sample, double line, int corrWin, int img_rows, int img_cols) {

	if ((sample - ceil(corrWin / 2)) > 0 && (sample + ceil(corrWin / 2)) < img_cols && (line - ceil(corrWin / 2)) > 0 && (line + ceil(corrWin / 2)) < img_rows) {
		return true;
	}
	else {
		return false;
	}
}

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
void osgm_util::chooseBaseImg(vector<uint8*> Imgs, vector<RPCMODEL> rpcs, int img_rows, int img_cols, float lat, float lon, float localMin, float localMax, int corrWin, int& BaseIdx) {
	int imgNums = Imgs.size();
	
	for (int i = 0; i < imgNums; ++i) {
		// 最低点以及最高点投影至影像，若均在影像范围内，则break，返回i
		double tmp_Line_min, tmp_Sample_min;
		double tmp_Line_max, tmp_Sample_max;

		API_LATLONGHEIFHT2LineSample(rpcs[i], lat, lon, localMin, tmp_Sample_min, tmp_Line_min);
		API_LATLONGHEIFHT2LineSample(rpcs[i], lat, lon, localMax, tmp_Sample_max, tmp_Line_max);

		// 判断三维最低点以及最高点对应像点是否在影像内
		bool localMinInImg = ifInImg(tmp_Sample_min, tmp_Line_min, corrWin, img_rows, img_cols);
		bool localMaxInImg = ifInImg(tmp_Sample_max, tmp_Line_max, corrWin, img_rows, img_cols);

		if (localMinInImg && localMaxInImg) {
			BaseIdx = i;
			return;
		}
	}
}

void osgm_util::deleteNodataValue(const Mat& src, Mat& dst)
{
	// 仅支持单通道 float型mat数据
	CV_Assert(src.channels()==1 );

	int rows = src.rows;
	int cols = src.cols;

	if (src.type() == CV_32F) {
		for (int i = 0; i < rows; ++i) {
			const float* ptr = src.ptr<float>(i);
			float* ptr_dst = dst.ptr<float>(i);
			for (int j = 0; j < cols; ++j) {
				if (ptr[j] >= -10000 && ptr[j] <= 10000 && ptr[j] != 0)  ptr_dst[j] = ptr[j];
				else  ptr_dst[j] = Invalid_Float;
			}
		}
	}
	else if (src.type() == CV_8U) {
		for (int i = 0; i < rows; ++i) {
			const uint8* ptr = src.ptr<uint8>(i);
			uint8* ptr_dst = dst.ptr<uint8>(i);
			for (int j = 0; j < cols; ++j) {
				if (ptr[j] >= -10000 && ptr[j] <= 10000 && ptr[j] != 0)  ptr_dst[j] = ptr[j];
				else  ptr_dst[j] = Invalid_Float;
			}
		}
	}
	else { //其他类型，默认为uchar
		for (int i = 0; i < rows; ++i) {
			const uchar* ptr = src.ptr<uchar>(i);
			uchar* ptr_dst = dst.ptr<uchar>(i);
			for (int j = 0; j < cols; ++j) {
				if (ptr[j] >= -10000 && ptr[j] <= 10000 && ptr[j] != 0)  ptr_dst[j] = ptr[j];
				else  ptr_dst[j] = 0;
			}
		}
	}
	
}

bool osgm_util::histMatch_Value(Mat matSrc, Mat matDst, Mat& matRet)
{
	if (matSrc.empty() || matDst.empty() || 1 != matSrc.channels() || 1 != matDst.channels())
		return false;
	int nHeight = matDst.rows;
	int nWidth = matDst.cols;
	int nDstPixNum = nHeight * nWidth;
	int nSrcPixNum = 0;

	int arraySrcNum[256] = { 0 };                // 源图像各灰度统计个数
	int arrayDstNum[256] = { 0 };                // 目标图像个灰度统计个数
	double arraySrcProbability[256] = { 0.0 };   // 源图像各个灰度概率
	double arrayDstProbability[256] = { 0.0 };   // 目标图像各个灰度概率
	// 统计源图像
	for (int j = 0; j < nHeight; j++)
	{
		for (int i = 0; i < nWidth; i++)
		{
			arrayDstNum[matDst.at<uchar>(j, i)]++;
		}
	}
	// 统计目标图像
	nHeight = matSrc.rows;
	nWidth = matSrc.cols;
	nSrcPixNum = nHeight * nWidth;
	for (int j = 0; j < nHeight; j++)
	{
		for (int i = 0; i < nWidth; i++)
		{
			arraySrcNum[matSrc.at<uchar>(j, i)]++;
		}
	}
	// 计算概率
	for (int i = 0; i < 256; i++)
	{
		arraySrcProbability[i] = (double)(1.0 * arraySrcNum[i] / nSrcPixNum);
		arrayDstProbability[i] = (double)(1.0 * arrayDstNum[i] / nDstPixNum);
	}
	// 构建直方图均衡映射
	int L = 256;
	int arraySrcMap[256] = { 0 };
	int arrayDstMap[256] = { 0 };
	for (int i = 0; i < L; i++)
	{
		double dSrcTemp = 0.0;
		double dDstTemp = 0.0;
		for (int j = 0; j <= i; j++)
		{
			dSrcTemp += arraySrcProbability[j];
			dDstTemp += arrayDstProbability[j];
		}
		arraySrcMap[i] = (int)((L - 1) * dSrcTemp + 0.5);// 减去1，然后四舍五入
		arrayDstMap[i] = (int)((L - 1) * dDstTemp + 0.5);// 减去1，然后四舍五入
	}
	// 构建直方图匹配灰度映射
	int grayMatchMap[256] = { 0 };
	for (int i = 0; i < L; i++) // i表示源图像灰度值
	{
		int nValue = 0;    // 记录映射后的灰度值
		int nValue_1 = 0;  // 记录如果没有找到相应的灰度值时，最接近的灰度值
		int k = 0;
		int nTemp = arraySrcMap[i];
		for (int j = 0; j < L; j++) // j表示目标图像灰度值
		{
			// 因为在离散情况下，之风图均衡化函数已经不是严格单调的了，
			// 所以反函数可能出现一对多的情况，所以这里做个平均。
			if (nTemp == arrayDstMap[j])
			{
				nValue += j;
				k++;
			}
			if (nTemp < arrayDstMap[j])
			{
				nValue_1 = j;
				break;
			}
		}
		if (k == 0)// 离散情况下，反函数可能有些值找不到相对应的，这里去最接近的一个值
		{
			nValue = nValue_1;
			k = 1;
		}
		grayMatchMap[i] = nValue / k;
	}
	// 构建新图像
	matRet = Mat::zeros(nHeight, nWidth, CV_8UC1);
	for (int j = 0; j < nHeight; j++)
	{
		for (int i = 0; i < nWidth; i++)
		{
			matRet.at<uchar>(j, i) = grayMatchMap[matSrc.at<uchar>(j, i)];
		}
	}
	return true;
}

int osgm_util::histogram_Matching(Mat matSrc, Mat matDst, Mat& matResult)
{
	// 分离三通道的影像
	Mat srcBGR[3];
	Mat dstBGR[3];
	Mat retBGR[3];
	split(matSrc, srcBGR);
	split(matDst, dstBGR);

	// 直方图匹配单通道
	histMatch_Value(srcBGR[0], dstBGR[0], retBGR[0]);

	// 返回单通道影像
	matResult = retBGR[0].clone();

	return 0;
}

void osgm_util::getP3DSearchRange(Mat roughDem, Mat& dilated_up_dem, Mat& erode_down_dem, Mat& adjustHeis, sint8 structuringElementSize)
{
	Mat element_dilated = getStructuringElement(MORPH_RECT, Size(structuringElementSize, structuringElementSize));
	dilate(roughDem, dilated_up_dem, element_dilated);

	// 查看膨胀高程最大值
	//double max, min;
	//cv::Point min_loc, max_loc;
	//cv::minMaxLoc(dilated_up_dem, &min, &max, &min_loc, &max_loc);

	Mat element_erode = getStructuringElement(MORPH_RECT, Size(structuringElementSize, structuringElementSize));
	erode(roughDem, erode_down_dem, element_erode);

	// 查看腐蚀高程最小值
	//double maxErode, minErode;
	//cv::Point min_erode_loc, max_erode_loc;
	//cv::minMaxLoc(erode_down_dem, &minErode, &maxErode, &min_erode_loc, &max_erode_loc);

	// 去除最高高程以及最低高程中的无效值
	int dst_rows = dilated_up_dem.rows;
	int dst_cols = dilated_up_dem.cols;

	for (int i = 0; i < dst_rows; ++i) {
		float* p_dilated_up = dilated_up_dem.ptr<float>(i);
		float* p_erodeed_down = erode_down_dem.ptr<float>(i);
		for (int j = 0; j < dst_cols; ++j) {
			// 任一不满足，则置0
			if (!(p_dilated_up[j] > -10000 && p_dilated_up[j] < 10000) || !(p_erodeed_down[j] > -10000 && p_erodeed_down[j] < 10000)) {
				p_dilated_up[j] = 0;
				p_erodeed_down[j] = 0;
			}
		}
	}

	adjustHeis = dilated_up_dem - erode_down_dem;
}
