#include "osgm_utils.h"
#include <cassert>
#include <queue>

bool osgm_util::preConvert2OneChannel(const Mat& src_bgr, Mat& dst_gray)
{
	if (src_bgr.channels() != 3) {
		cout << "srcӰ����������ͨ��Ӱ��"<<endl;
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
		// ��Ŀ��Ӱ����Ҫƥ��
		if (i != baseIdx) 
		{
			Mat src_img = src[i].clone();;
			// ֱ��ͼƥ�䣬���޳���Чֵ
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

	// �����ؼ���censusֵ
	for (sint32 i = 2; i < height - 2; i++) {
		for (sint32 j = 2; j < width - 2; j++) {

			// ��������ֵ
			const uint8 gray_center = source[i * width + j];

			// ������СΪ5x5�Ĵ������������أ���һ�Ƚ�����ֵ����������ֵ�ĵĴ�С������censusֵ
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

			// �������ص�censusֵ
			census[i * width + j] = census_val;
		}
	}
}

void osgm_util::census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height)
{
	if (source == nullptr || census == nullptr || width <= 9 || height <= 7) {
		return;
	}

	// �����ؼ���censusֵ
	for (sint32 i = 4; i < height - 4; i++) {
		for (sint32 j = 3; j < width - 3; j++) {

			// ��������ֵ
			const uint8 gray_center = source[i * width + j];

			// ������СΪ5x5�Ĵ������������أ���һ�Ƚ�����ֵ����������ֵ�ĵĴ�С������censusֵ
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
			// �������ص�censusֵ
			//cout << i * width + j << endl;
			census[i * width + j] = census_val;
		}
	}

	// ����census�任�Ƿ���ȷ
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
\brief: �ж����(sample,line)���ڵļ��㴰���Ƿ���Ӱ����
\para[in]: sample �񷽵�sample���꣬����˵x����
\para[in]: line �񷽵�line���꣬����˵y����
\para[in]: corrWin ���ϵ�����ڴ�С
\para[in]: img_rows Ӱ������
\para[in]: img_cols Ӱ������
\para[out]: ���ص�boolֵ���Ƿ���Ӱ����
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
\brief: chooseBaseImg ȷ���﷽ƽ���ͶӰ��Ӱ��ʱ��ȷ���Ļ�׼Ӱ��
\para[in]: Imgs ����Ӱ��
\para[in]: rpcs ����Ӱ���Ӧ��������ģ��
\para[in]: img_rows Ӱ������
\para[in]: img_cols Ӱ������
\para[in]: lat γ��
\para[in]: lon ���ȣ��������γ�����ȷ���˸�ƽ���λ��
\para[in]: localMin ��ƽ��λ�õ���������߳�
\para[in]: localMax ��ƽ��λ�õ���������߳�
\para[in]: corrWin ���趨�����ϵ�����㴰��Ϊ�����δ���ʱ��corrWinΪ�趨�Ĵ��ڱ߳�
\para[out]: BaseIdx ѡȡ�ĸ�ƽ��λ�õĻ�׼Ӱ��
*/
void osgm_util::chooseBaseImg(vector<uint8*> Imgs, vector<RPCMODEL> rpcs, int img_rows, int img_cols, float lat, float lon, float localMin, float localMax, int corrWin, int& BaseIdx) {
	int imgNums = Imgs.size();
	
	for (int i = 0; i < imgNums; ++i) {
		// ��͵��Լ���ߵ�ͶӰ��Ӱ��������Ӱ��Χ�ڣ���break������i
		double tmp_Line_min, tmp_Sample_min;
		double tmp_Line_max, tmp_Sample_max;

		API_LATLONGHEIFHT2LineSample(rpcs[i], lat, lon, localMin, tmp_Sample_min, tmp_Line_min);
		API_LATLONGHEIFHT2LineSample(rpcs[i], lat, lon, localMax, tmp_Sample_max, tmp_Line_max);

		// �ж���ά��͵��Լ���ߵ��Ӧ����Ƿ���Ӱ����
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
	// ��֧�ֵ�ͨ�� float��mat����
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
	else { //�������ͣ�Ĭ��Ϊuchar
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

	int arraySrcNum[256] = { 0 };                // Դͼ����Ҷ�ͳ�Ƹ���
	int arrayDstNum[256] = { 0 };                // Ŀ��ͼ����Ҷ�ͳ�Ƹ���
	double arraySrcProbability[256] = { 0.0 };   // Դͼ������Ҷȸ���
	double arrayDstProbability[256] = { 0.0 };   // Ŀ��ͼ������Ҷȸ���
	// ͳ��Դͼ��
	for (int j = 0; j < nHeight; j++)
	{
		for (int i = 0; i < nWidth; i++)
		{
			arrayDstNum[matDst.at<uchar>(j, i)]++;
		}
	}
	// ͳ��Ŀ��ͼ��
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
	// �������
	for (int i = 0; i < 256; i++)
	{
		arraySrcProbability[i] = (double)(1.0 * arraySrcNum[i] / nSrcPixNum);
		arrayDstProbability[i] = (double)(1.0 * arrayDstNum[i] / nDstPixNum);
	}
	// ����ֱ��ͼ����ӳ��
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
		arraySrcMap[i] = (int)((L - 1) * dSrcTemp + 0.5);// ��ȥ1��Ȼ����������
		arrayDstMap[i] = (int)((L - 1) * dDstTemp + 0.5);// ��ȥ1��Ȼ����������
	}
	// ����ֱ��ͼƥ��Ҷ�ӳ��
	int grayMatchMap[256] = { 0 };
	for (int i = 0; i < L; i++) // i��ʾԴͼ��Ҷ�ֵ
	{
		int nValue = 0;    // ��¼ӳ���ĻҶ�ֵ
		int nValue_1 = 0;  // ��¼���û���ҵ���Ӧ�ĻҶ�ֵʱ����ӽ��ĻҶ�ֵ
		int k = 0;
		int nTemp = arraySrcMap[i];
		for (int j = 0; j < L; j++) // j��ʾĿ��ͼ��Ҷ�ֵ
		{
			// ��Ϊ����ɢ����£�֮��ͼ���⻯�����Ѿ������ϸ񵥵����ˣ�
			// ���Է��������ܳ���һ�Զ�������������������ƽ����
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
		if (k == 0)// ��ɢ����£�������������Щֵ�Ҳ������Ӧ�ģ�����ȥ��ӽ���һ��ֵ
		{
			nValue = nValue_1;
			k = 1;
		}
		grayMatchMap[i] = nValue / k;
	}
	// ������ͼ��
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
	// ������ͨ����Ӱ��
	Mat srcBGR[3];
	Mat dstBGR[3];
	Mat retBGR[3];
	split(matSrc, srcBGR);
	split(matDst, dstBGR);

	// ֱ��ͼƥ�䵥ͨ��
	histMatch_Value(srcBGR[0], dstBGR[0], retBGR[0]);

	// ���ص�ͨ��Ӱ��
	matResult = retBGR[0].clone();

	return 0;
}

void osgm_util::getP3DSearchRange(Mat roughDem, Mat& dilated_up_dem, Mat& erode_down_dem, Mat& adjustHeis, sint8 structuringElementSize)
{
	Mat element_dilated = getStructuringElement(MORPH_RECT, Size(structuringElementSize, structuringElementSize));
	dilate(roughDem, dilated_up_dem, element_dilated);

	// �鿴���͸߳����ֵ
	//double max, min;
	//cv::Point min_loc, max_loc;
	//cv::minMaxLoc(dilated_up_dem, &min, &max, &min_loc, &max_loc);

	Mat element_erode = getStructuringElement(MORPH_RECT, Size(structuringElementSize, structuringElementSize));
	erode(roughDem, erode_down_dem, element_erode);

	// �鿴��ʴ�߳���Сֵ
	//double maxErode, minErode;
	//cv::Point min_erode_loc, max_erode_loc;
	//cv::minMaxLoc(erode_down_dem, &minErode, &maxErode, &min_erode_loc, &max_erode_loc);

	// ȥ����߸߳��Լ���͸߳��е���Чֵ
	int dst_rows = dilated_up_dem.rows;
	int dst_cols = dilated_up_dem.cols;

	for (int i = 0; i < dst_rows; ++i) {
		float* p_dilated_up = dilated_up_dem.ptr<float>(i);
		float* p_erodeed_down = erode_down_dem.ptr<float>(i);
		for (int j = 0; j < dst_cols; ++j) {
			// ��һ�����㣬����0
			if (!(p_dilated_up[j] > -10000 && p_dilated_up[j] < 10000) || !(p_erodeed_down[j] > -10000 && p_erodeed_down[j] < 10000)) {
				p_dilated_up[j] = 0;
				p_erodeed_down[j] = 0;
			}
		}
	}

	adjustHeis = dilated_up_dem - erode_down_dem;
}
