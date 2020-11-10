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
	//������������������������ Ӱ��Ԥ����������ȥ��nodata������ͨ���ȣ�
	bool preConvert2OneChannel(const Mat &src_bgr, Mat &dst_gray);

	/*
	\brief: ��ȡ·���ļ����е�����Ӱ�񣬲�����vector��
	\para[in]: ImgPath 
	\para[out] imgs �����Ӱ��
	*/
	bool preReadImgs(string ImgPath, vector<Mat> &imgs);


	/*
	\brief: ������Ӱ����ֱ��ͼƥ��
	\para[in] src ����Ķ���Ӱ��
	\para[in] baseIdx ֱ��ͼƥ���Ŀ��Ӱ������
	\para[out] dst_uint8 ֱ��ͼƥ���Ķ���Ӱ��,����Ϊuint8*
	*/
	bool prehistMatch(vector<Mat>& src, int baseIdx, vector<uint8*>& dst_uint8);

	void deleteNodataValue(const Mat& src, Mat& dst);

	bool histMatch_Value(Mat matSrc, Mat matDst, Mat& matRet);

	int histogram_Matching(Mat Src, Mat matDst, Mat& matResult);


	/*
	brief: getP3DSearchRange �õ���ʼ��DEMÿ����������߸߳��Լ���͸̣߳����趨������Χ��
	para[in]: roughDem �Ѿ�����resize��sldem
	para[in]: dilated_up_dem ���ͺ�Ľ������ÿ����ά��̵߳�������߸߳�
	para[in]: erode_down_dem ��ʴ��Ľ������ÿ����ά��̵߳�������͸߳�
	para[in]: adjustHeis, ��߸߳�����͸߳�֮��
	*/
	void getP3DSearchRange(Mat roughDem, Mat& dilated_up_dem, Mat& erode_down_dem, Mat& adjustHeis, sint8 structuringElementSize = 5);


	//������������������������ census���߼�
	// census�任

	/**
	 * \brief census�任
	 * \param source	���룬Ӱ������
	 * \param census	�����censusֵ����
	 * \param width		���룬Ӱ���
	 * \param height	���룬Ӱ���
	 */
	void census_transform_5x5(const uint8* source, uint32* census, const sint32& width, const sint32& height);
	void census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height);
	// Hamming����
	uint8 Hamming32(const uint32& x, const uint32& y);
	uint8 Hamming64(const uint64& x, const uint64& y);

	// ������������������������ �﷽ͶӰ���񷽹��߼�
	/*
	\brief: ��Ŀ���������к�ת��Ϊ��γ������
	*/
	void rowCol2LatLon(int row, int col, float UL_lon, float UL_lat, float reso, float& lat, float& lon);


	/*
	\brief: �ж����(sample,line)���ڵļ��㴰���Ƿ���Ӱ����
	\para[in]: sample �񷽵�sample���꣬����˵x����
	\para[in]: line �񷽵�line���꣬����˵y����
	\para[in]: corrWin ���ϵ�����ڴ�С
	\para[in]: img_rows Ӱ������
	\para[in]: img_cols Ӱ������
	\para[out]: ���ص�boolֵ���Ƿ���Ӱ����
	*/
	bool ifInImg(double sample, double line, int corrWin, int img_rows, int img_cols);

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
	void chooseBaseImg(vector<uint8*> Imgs, vector<RPCMODEL> rpcs, int img_rows, int img_cols, float lat, float lon, float localMin, float localMax, int corrWin, int& BaseIdx);


	// ... ͼ������߼�
	/**
	 * \brief ��ֵ�˲�
	 * \param in				���룬Դ����
	 * \param out				�����Ŀ������
	 * \param width				���룬���
	 * \param height			���룬�߶�
	 * \param wnd_size			���룬���ڿ��
	 */
	void MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height, const sint32 wnd_size);


	/**
	 * \brief �޳�С��ͨ��
	 * \param disparity_map		���룬�Ӳ�ͼ
	 * \param width				���룬���
	 * \param height			���룬�߶�
	 * \param diff_insame		���룬ͬһ��ͨ���ڵľֲ����ز���
	 * \param min_speckle_aera	���룬��С��ͨ�����
	 * \param invalid_val		���룬��Чֵ
	 */
	void RemoveSpeckles(float32* disparity_map, const sint32& width, const sint32& height, const sint32& diff_insame, const uint32& min_speckle_aera, const float32& invalid_val);

	// ... ͼ��Ԥ�������ݼ�

}


