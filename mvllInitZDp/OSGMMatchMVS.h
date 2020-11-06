#pragma once
#include "osgm_utils.h"

using namespace cv;
using namespace std;

/*
brief: OSGM����ƥ����
*/
class OSGMMatchMVS
{
public:
	OSGMMatchMVS();
	~OSGMMatchMVS();

	/** \brief Census���ڳߴ����� */
	enum CensusSize {
		Census5x5,
		Census9x7 = 0
	};

	/** \brief osgm�����ṹ�� */
	struct OSGM_Option {
		uint8	num_paths;		// �ۺ�·����
		uint8	dilate_erode_win;	// ���͸�ʴ�Ĵ��ڴ�С   
		CensusSize census_size;		// census���ڳߴ�

		// P1,P2 
		// P2 = P2_int / (Ip-Iq)
		sint32  p1;				// �ͷ������P1
		sint32  p2_init;			// �ͷ������P2

		// Ŀ�귶Χλ��
		uint16	dst_rows;		// Ŀ��Ӱ������ 
		uint16	dst_cols;		// Ŀ��Ӱ������
		float64 reso;			
		float64 UL_lat;
		float64 UL_lon;

		OSGM_Option() : num_paths(8), dilate_erode_win(3), census_size(Census9x7), p1(10), p2_init(150), dst_rows(0), dst_cols(0),reso(0.0019531249), UL_lat(0), UL_lon(0){}
	};

public:
	/**
	* \brief ��ĳ�ʼ��,ȷ����Ҫ������ڴ��С��������Ԥ���õ�
	* para[in] img_rows ����Ӱ�������
	* para[in] img_cols ����Ӱ�������
	* para[in] num_imgs ����Ӱ�������
	* para[in] roughdem δresize��roughDEM
	* para[in] option   ���룬�㷨����
	* \param 
	*/
	bool Initialize(const sint32& img_rows, const sint32& img_cols, const uint8 num_imgs, const Mat& roughDem, const OSGM_Option& option, sint32& cost_size);

	/*
	 * \brief ִ��ƥ��
	 * \param[in] imgs ����Ӱ��
	 * \param[in] rpcs ����Ӱ���Ӧ�ļ���ģ��
	 * \param[out] res		���DEM��ָ�루��ҪԤ�ȿ����ڴ棩
	 */
	bool Match(const vector<uint8*> imgs, const vector<RPCMODEL> rpcs, float32* res);

private: // ˽�к���

	/** \brief Census�任 */
	void CensusTransform() const;

	/** \brief ���ۼ���	 */
	void ComputeCost() const;

	/** \brief ���۾ۺ�	 */
	void CostAggregation() const;

	/** \brief �Ӳ����	 */
	void ComputeDisparity() const;

	/** \brief �ڴ��ͷ�	 */
	void Release();


private:
	/** \brief OSGM����	 */
	OSGM_Option option_;

	/** \brief Ŀ��DEM����	 */
	sint32 dst_cols_;

	/** \brief Ŀ��Ӱ������	 */
	sint32 dst_rows_;

	/** \brief Ŀ���������ϽǾ���	 */
	float64 UL_lon_;

	/** \brief Ŀ���������Ͻ�γ��	 */
	float64 UL_lat_;

	/** \brief Ŀ������ֱ��� GSD	 */
	float32 reso_;

	/** \brief ÿһ������Ԫ�ض�Ӧ�Ÿ�λ��֮ǰ�ĸ̷߳�Χ��ֵ�����ڼ���cost��λ��	 */
	float32* accumulate_cost_size_;



	/** \brief ����Ӱ������	 */
	vector<uint8*> imgs_;

	/** \brief ����Ӱ������	 */
	sint32 img_cols_;

	/** \brief ����Ӱ������	 */
	sint32 img_rows_;

	/** \brief ����Ӱ�����ݶ�Ӧ��rpc	 */
	vector<RPCMODEL> rpcs_;

	/** \brief ����Ӱ�����ݶ�Ӧ��censusֵ	 */
	vector<uint32*> census_imgs_;

	/** \brief ��ʼƥ�����	 */
	uint8* cost_init_; 

	/** \brief �ۺ�ƥ�����	 */
	uint16* cost_aggr_;

	// �K �� �L   5  3  7
	// ��    ��	 1    2
	// �J �� �I   8  4  6
	/** \brief �ۺ�ƥ�����-����1	*/
	uint8* cost_aggr_1_;
	/** \brief �ۺ�ƥ�����-����2	*/
	uint8* cost_aggr_2_;
	/** \brief �ۺ�ƥ�����-����3	*/
	uint8* cost_aggr_3_;
	/** \brief �ۺ�ƥ�����-����4	*/
	uint8* cost_aggr_4_;
	/** \brief �ۺ�ƥ�����-����5	*/
	uint8* cost_aggr_5_;
	/** \brief �ۺ�ƥ�����-����6	*/
	uint8* cost_aggr_6_;
	/** \brief �ۺ�ƥ�����-����7	*/
	uint8* cost_aggr_7_;
	/** \brief �ۺ�ƥ�����-����8	*/
	uint8* cost_aggr_8_;

	/** \brief ��ʼDEM��initialDem	 */
	float* initial_Dem_;

	/** \brief DEM�������̣߳�dilated_up_dem_	 */
	float* dilated_up_Dem_;

	/** \brief DEM������͸̣߳�dilated_up_dem_	 */
	float* erode_down_Dem_;

	/** \brief Ŀ��DEMͼ��res	 */
	float* res_;

	/** \brief �Ƿ��ʼ����־	 */
	bool is_initialized_;
};

