#include "OSGMMatchMVS.h"
#include <chrono>

using namespace std::chrono;


int main() {

	auto start = std::chrono::steady_clock::now();

	// ... ����Ӱ��Ԥ����Ӱ��
	// ��ȡ�ļ����е�Ӱ��
	string strPath = "E:\\mvllInitZDp\\data\\ap11\\";
	vector<uint8*> imgs;
	Mat Img0_ori = imread(strPath + "m1126972080le.tif", cv::IMREAD_GRAYSCALE); // Ŀ��ͼ��
	Mat Img1_ori = imread(strPath + "m1114021499re.tif",cv::IMREAD_GRAYSCALE); // Դͼ��
	Mat Img2_ori = imread(strPath + "m1126986303re.tif", cv::IMREAD_GRAYSCALE); // Դͼ��
	Mat Img3_ori = imread(strPath + "m1114007294re.tif", cv::IMREAD_GRAYSCALE); // Դͼ��
	Mat Img4_ori = imread(strPath + "m1114014396re.tif", cv::IMREAD_GRAYSCALE); // Դͼ��
	Mat Img5_ori = imread(strPath + "m1126986303re.tif", cv::IMREAD_GRAYSCALE); // Դͼ��
	Mat Img6_ori = imread(strPath + "m1129340193re.tif", cv::IMREAD_GRAYSCALE); // Դͼ��

	Mat Img0 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);
	Mat Img1 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);
	Mat Img2 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);
	Mat Img3 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);
	Mat Img4 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);
	Mat Img5 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);
	Mat Img6 = Mat::zeros(Img0_ori.rows, Img0_ori.cols, CV_8UC1);

	osgm_util::deleteNodataValue(Img1_ori, Img1);
	osgm_util::deleteNodataValue(Img0_ori, Img0);
	osgm_util::deleteNodataValue(Img2_ori, Img2);
	osgm_util::deleteNodataValue(Img3_ori, Img3);
	osgm_util::deleteNodataValue(Img4_ori, Img4);
	osgm_util::deleteNodataValue(Img5_ori, Img5);
	osgm_util::deleteNodataValue(Img6_ori, Img6);

	imgs.push_back(Img0.data);
	imgs.push_back(Img1.data);
	imgs.push_back(Img2.data);
	imgs.push_back(Img3.data);
	imgs.push_back(Img4.data);
	imgs.push_back(Img5.data);
	imgs.push_back(Img6.data);

	// ����Ԥ�����ļ����е�Ӱ��
	// osgm_util::prehistMatch(imgs_mat, 0, imgs);
	// cout << imgs.size();
	// baseImg�� ��������������
	
	// ��ȡrpc
	RPCMODEL rpc1, rpc2, rpc3, rpc4, rpc5, rpc6, rpc7;
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le_rpc.txt", rpc1);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re_rpc.txt", rpc2);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc3);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114007294re_rpc.txt", rpc4);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114014396re_rpc.txt", rpc5);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc6);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1129340193re_rpc.txt", rpc7);

	vector<RPCMODEL> rpcs;
	rpcs.push_back(rpc1);
	rpcs.push_back(rpc2);
	rpcs.push_back(rpc3);
	rpcs.push_back(rpc4);
	rpcs.push_back(rpc5);
	rpcs.push_back(rpc6);
	rpcs.push_back(rpc7);

	Mat roughDem_ori = imread("G:/A_�����ʼ�/computers_and_geosciences/roughSLDEM/sldemNacAp1.tif", -1);
	Mat roughDem = roughDem_ori.clone();
	osgm_util::deleteNodataValue(roughDem_ori, roughDem);
	int dst_cols = roughDem.cols * 3;
	int dst_rows = roughDem.rows * 3;
	Mat resize_sldem_ori = Mat::zeros(dst_rows, dst_cols, CV_32F);;
	resize(roughDem, resize_sldem_ori, resize_sldem_ori.size(), 0, 0, INTER_AREA); 
	Mat resize_sldem = resize_sldem_ori.clone();
	osgm_util::deleteNodataValue(resize_sldem_ori, resize_sldem);

	double UL_lon = 23.3722757552958, UL_lat = 1.23648399230611;
	double rough_lat_res = 0.0019531249, rough_lon_res = 0.0019531249;

	// ������ osgm ƥ��
	const uint32 img_cols = static_cast<uint32>(Img0.cols);
	const uint32 img_rows = static_cast<uint32>(Img0.rows);

	// �趨����
	OSGMMatchMVS::OSGM_Option osgm_option;
	osgm_option.num_paths = 8;
	osgm_option.dilate_erode_win = 3;
	osgm_option.dst_cols = dst_cols;
	osgm_option.dst_rows = dst_rows;
	osgm_option.p1 = 10;
	osgm_option.p2_init = 150;
	osgm_option.reso = 0.0019531249 / 3;
	osgm_option.UL_lat = 1.23648399230611;
	osgm_option.UL_lon = 23.3722757552958;
	/*osgm_option.census_size = Census9x7;*/

	OSGMMatchMVS osgm;

	// ��ʼ��
	sint32 cost_size = 0;
	if (!osgm.Initialize(img_rows, img_cols, imgs.size(), resize_sldem, osgm_option, cost_size)) {
		std::cout << "OSGM��ʼ��ʧ�ܣ�" << std::endl;
		return -2;
	}

	// ƥ��
	auto res = new float32[cost_size]();

 	if (!osgm.Match(imgs,rpcs,res) ){
		std::cout << "SGMƥ��ʧ�ܣ�" << std::endl;
		return -2;
	}
	
	auto end = std::chrono::steady_clock::now();
	auto tt = duration_cast<std::chrono::milliseconds>(end - start);
	printf("��ʼ���ۼ��㼰ƥ�� Done! Timing : %lf s\n\n", tt.count() / 1000.0);

	// ��ʾresͼ
	cv::Mat res_mat = cv::Mat(dst_rows, dst_cols, CV_8UC1);
	for (uint32 i = 0; i < dst_rows; i++) {
		for (uint32 j = 0; j < dst_cols; j++) {
			const float32 disp = res[i * dst_cols + j];
			if (disp == Invalid_Float) {
				res_mat.data[i * dst_cols + j] = 0;
			}
			else {
				res_mat.data[i * dst_cols + j] = 2 * static_cast<uchar>(disp);
			}
		}
	}

	return 0;
}








//#include<numeric>
//#include <algorithm> //vector��Ա����ͷ�ļ�
//#include<windows.h>    //ͷ�ļ�  
//#include <iomanip>
//#include <iostream>
//
////#include "preProcessing.h"
//#include "osgm_types.h"
//#include "osgm_utils.h"
//#include "CubicSplineInterpolation.h"
//
//
//
//#define PATHS_PER_SCAN 8
//#define DEBUG falses
//#define TEST_TWO_IMAGE true
//
//
////constexpr auto Z_resolution = 1;
//using namespace cv;
//using namespace std;
//
//
//
//
//
//
//
///*
//\brief: chooseBaseImg ȷ���﷽ƽ���ͶӰ��Ӱ��ʱ��ȷ���Ļ�׼Ӱ��
//\para[in]: Imgs ����Ӱ��
//\para[in]: rpcs ����Ӱ���Ӧ��������ģ��
//\para[in]: lat γ��
//\para[in]: lon ���ȣ��������γ�����ȷ���˸�ƽ���λ��
//\para[in]: localMin ��ƽ��λ�õ���������߳�
//\para[in]: localMax ��ƽ��λ�õ���������߳�
//\para[in]: corrWin ���趨�����ϵ�����㴰��Ϊ�����δ���ʱ��corrWinΪ�趨�Ĵ��ڱ߳�
//\para[out]: BaseIdx ѡȡ�ĸ�ƽ��λ�õĻ�׼Ӱ��
//*/
//void chooseBaseImg(vector<Mat>& Imgs, vector<RPCMODEL>& rpcs, float lat, float lon, float localMin, float localMax, int corrWin, int& BaseIdx);
//
///*
//\brief: calSNCC �������ά���Ӧ��SNCCֵ
//\para[in] lat γ��
//\para[in] lon ����
//\para[in] hei �߳�
//\para[in] imgs ����Ӱ��
//\para[in] rpcs ����Ӱ���Ӧ��������ģ��
//\para[in] baseImgIdx ��׼Ӱ�����
//\para[in] corrWin �������ϵ���Ĵ��ڴ�С
//\para[out] SNCC ���صĸ���ά���Ӧ��SNCCֵ
//*/
//void calSNCC(double lat, double lon, double hei, vector<Mat>& imgs, vector<RPCMODEL> rpcs, int baseImgIdx, int corrWin, float& SNCC);
//
//
////void testLamMatch() {
////	cv::Mat img_left = cv::imread("E:/SemiGlobalMatching/Data/cone/im2.png", cv::IMREAD_GRAYSCALE);
////	cv::Mat img_right = cv::imread("E:/SemiGlobalMatching/Data/cone/im6.png", cv::IMREAD_GRAYSCALE);
////	vector<cv::Point3f> leftPoints;
////	Point3f tmp1; tmp1.x = 100; tmp1.y = 60;
////	Point3f tmp2; tmp2.x = 60; tmp2.y = 100;
////	leftPoints.push_back(tmp1);
////	leftPoints.push_back(tmp2);
////
////	kyhMatchImgbyLST(img_left, img_right, leftPoints);
////}
//
//void allocateCAS(float*** &C, float*** &S, float**** &A, const Mat& dilated_up_dem, const Mat& erode_down_dem);
//
//void rowCol2LatLon(int row, int col, float &lat, float &lon, float UL_lon, float UL_lat, float reso) {
//	lat = UL_lat + row * reso;
//	lon = UL_lon + col * reso;
//}
//
///*
//\brief: ���ݴ��ۿռ䣬����WTA�������õ��ֽ��
//\para[in]: cost �����жϵĴ��ۿռ�
//\para[in]: Z_reso �̲߳���,��Ӧcost
//\para[in]: localMin Ŀ���ؽ��������͸߳�
//\para[out]: res ����WTA�����õ���Ŀ�������ؽ��������DEM
//*/
//void WTA(float*** cost, float Z_reso, Mat& localMin, Mat& res);
//
//
//
//int main(){
//	
//
//
//	//std::string strPath = "G:\\A_�����ʼ�\\computers_and_geosciences\\test\\ap11\\";
//	//Mat matSrc0 = imread(strPath + "m1126986303re.tif"); // Դͼ��
//	//Mat matSrc = imread(strPath + "m1114021499re.tif"); // Դͼ��
//	//Mat matDst = imread(strPath + "m1126972080le.tif"); // Ŀ��ͼ��
//	//Mat result,result1;
//	//myPreProcessing.histogram_Matching(matSrc, matDst,result);
//	//myPreProcessing.histogram_Matching(matSrc0, matDst, result1);
//
//	//testLamMatch(); // TODO�� ������С���˷���
//	clock_t startTime, endTime;
//	startTime = clock();
//	preProcessing myPreProcessing;
//	//------------------------------------------------------------------------------
//	//-----��������
//	float sufferZ = 1;
//	int corrWin = 45;
//	int largePenalty = 16;
//	int smallPenalty = 2;
//	int N_times_res = 3; //ϸ������ _ 
//	float uniquessRatio = 1.01;
//
//	string dirName = "G:\\A_daily\\0719\\test4";  //����test�ļ���
//	string path = dirName + "\\points3d.txt"; //3D��·��������
//	string arguPath = dirName + "\\parameter.txt"; //����·��
//	string costPath = dirName + "\\cost.txt"; //����·��
//	string initialDemPath = dirName + "\\initialDem.txt";
//	ofstream writeArguPath(arguPath);
//	ofstream Cost(costPath);
//	//------------------------------------------------------------------------------------1. sldemȡϸ���������͸�ʴ����Ӧ�̷߳�Χ
//	//sldem Nac
//	Mat roughDem_ori = imread("G:/A_�����ʼ�/computers_and_geosciences/roughSLDEM/sldemNacAp1.tif", -1);
//	Mat roughDem = roughDem_ori.clone();
//	myPreProcessing.deleteNodataValue(roughDem_ori, roughDem);
//
//
//	double UL_lon = 23.3722757552958, UL_lat = 1.23648399230611;
//	double rough_lat_res = 0.0019531249, rough_lon_res = 0.0019531249;
//
//	// ... Ԥ����sldem����sldem�������ͺ͸�ʴ������ӦP3D��������Χ�����õ�ÿ�����������������ߵ��Լ���͵�
//	int rough_rows = roughDem.rows, rough_cols = roughDem.cols;
//	int dst_rows = rough_rows*N_times_res;
//	int dst_cols = rough_cols*N_times_res;
//	Mat resize_sldem_ori = Mat::zeros(dst_rows, dst_cols, CV_32F);;
//	resize(roughDem, resize_sldem_ori, resize_sldem_ori.size(), 0, 0, INTER_AREA); 
//
//
//	Mat dilated_up_dem_ori = resize_sldem_ori.clone();
//	Mat erode_down_dem_ori = resize_sldem_ori.clone();
//
//	osgm_util::getP3DSearchRange(resize_sldem_ori, dilated_up_dem_ori, erode_down_dem_ori);
//	Mat resize_sldem = resize_sldem_ori.clone();
//	Mat dilated_up_dem = dilated_up_dem_ori.clone();
//	Mat erode_down_dem = erode_down_dem_ori.clone();
//	myPreProcessing.deleteNodataValue(resize_sldem_ori, resize_sldem);
//	myPreProcessing.deleteNodataValue(dilated_up_dem_ori, dilated_up_dem);
//	myPreProcessing.deleteNodataValue(erode_down_dem_ori, erode_down_dem);
//
//	//... ����Ӱ���Լ�rpc
//	vector<Mat> imgs;
//	vector<RPCMODEL> rpcs;
//	RPCMODEL rpc1, rpc2, rpc3;
//
//	//sldem Nac�Ͷ�λ����һ�µ���Ƭ����Ƭ
//	Mat img1 = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le.tif", 0);
//	Mat img2_origin = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re.tif",0);
//	Mat img3__origin = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126986303re.tif",0); // Դͼ��
//
//	Mat img2, img3;
//	myPreProcessing.histMatch_Value(img2_origin, img1, img2);
//	myPreProcessing.histMatch_Value(img3__origin, img1, img3);
//
//	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le_rpc.txt", rpc1);
//	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re_rpc.txt", rpc2);
//	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc3);
//
//	imgs.push_back(img1); imgs.push_back(img2); imgs.push_back(img3);
//	rpcs.push_back(rpc1); rpcs.push_back(rpc2); rpcs.push_back(rpc3);
//
//	int NumOfImgs = imgs.size();
//
//
//
//
//
//
//
//
//		system("pause");
//
//		return 0;
//	}
//
//
//void allocateCAS(float*** &C, float*** &S, float**** &A, const Mat& dilated_up_dem, const Mat& erode_down_dem) {
//	int dst_rows = dilated_up_dem.rows;
//	int dst_cols = dilated_up_dem.cols;
//	
//	for (int row = 0; row < dst_rows; ++row)
//	{
//	
//		C[row] = new float* [dst_cols];
//		S[row] = new float* [dst_cols];
//
//		const float* ptr_localMax = dilated_up_dem.ptr<float>(row);
//		const float* ptr_localMin = erode_down_dem.ptr<float>(row);
//
//		for (int col = 0; col < dst_cols; ++col)
//		{
//
//			if (ptr_localMax[col] == Invalid_Float || ptr_localMin[col] == Invalid_Float) {
//				C[row][col] = nullptr;
//				S[row][col] = nullptr;
//				continue;
//			}
//
//			int adjustThisZ = (ptr_localMax[col] - ptr_localMin[col]) / Z_resolution + 1;
//			//cout << ptr_localMax[col] << " "<< ptr_localMin[col] << " "<< adjustThisZ << endl;
//
//			C[row][col] = new float[adjustThisZ]();
//			S[row][col] = new float[adjustThisZ](); // initialize to 0
//
//			//if (row == 10 && col == 10) { cout << adjustThisZ; }
//
//			for (int hei_ind = 0; hei_ind < adjustThisZ; hei_ind++) {
//				C[row][col][hei_ind] = Invalid_Float;
//				S[row][col][hei_ind] = Invalid_Float;
//				//cout << row << " " << col << " " << hei_ind << endl;
//			}
//		}
//	}
//
//	// #define PATHS_PER_SCAN 4
//	A = new float*** [PATHS_PER_SCAN];
//	for (int path = 0; path < PATHS_PER_SCAN; ++path)
//	{
//		A[path] = new float** [dst_rows];
//		for (int row = 0; row < dst_rows; ++row)
//		{
//			A[path][row] = new float* [dst_cols];
//			for (int col = 0; col < dst_cols; ++col)
//			{
//
//				const float* ptr_localMax = dilated_up_dem.ptr<float>(row);
//				const float* ptr_localMin = erode_down_dem.ptr<float>(row);
//
//				if (ptr_localMax[col] == Invalid_Float || ptr_localMin[col] == Invalid_Float) {
//					A[path][row][col] = nullptr;
//					continue;
//				}
//
//				int adjustThisZ = (ptr_localMax[col] - ptr_localMin[col]) / Z_resolution + 1;
//				//cout << ptr_localMax[col] << " "<< ptr_localMin[col] << " "<< adjustThisZ << endl;
//
//				A[path][row][col] = new float[adjustThisZ]();
//
//				for (int hei_ind = 0; hei_ind < adjustThisZ; hei_ind++) {
//	
//					A[path][row][col][hei_ind] = Invalid_Float;
//				}
//			}
//		}
//	}
//
//	cout << C[14][14][3] << endl;
//
//
//};
//
//
//
//
//
//
///*
//\brief: calSNCC �������ά���Ӧ��SNCCֵ
//\para[in] lat γ��
//\para[in] lon ����
//\para[in] hei �߳�
//\para[in] imgs ����Ӱ��
//\para[in] rpcs ����Ӱ���Ӧ��������ģ��
//\para[in] baseImgIdx ��׼Ӱ�����
//\para[in] corrWin �������ϵ���Ĵ��ڴ�С
//\para[out] SNCC ���صĸ���ά���Ӧ��SNCCֵ
//*/
////void calSNCC(double lat, double lon, double hei, vector<Mat>& imgs, vector<RPCMODEL> rpcs, int baseImgIdx, int corrWin, float& SNCC) {
////	int NumOfImgs = imgs.size();
////
////
////	// ȷ��baseIdx��ͶӰ���λ��BaseSample,BaseLine
////	double BaseSample, BaseLine;
////	ImgPoint Pbase;
////	API_LATLONGHEIFHT2LineSample(rpcs[baseImgIdx], lat, lon, hei, BaseSample, BaseLine);
////	Pbase.line = BaseLine;
////	Pbase.sample = BaseSample;
////
////	int NumOfCalPairs = 0; // �����˼��Դ���
////	for (int i = 0; i < NumOfImgs; i++) {
////		double tmpSearchSample, tmpSearchLine;
////		ImgPoint tmp_Psearch;
////
////		if (i == baseImgIdx) continue;
////		API_LATLONGHEIFHT2LineSample(rpcs[i], lat, lon, hei, tmpSearchSample, tmpSearchLine);
////		tmp_Psearch.sample = tmpSearchSample;
////		tmp_Psearch.line = tmpSearchLine;
////
////
////		bool validSearch = osgm_util::ifInImg(tmpSearchSample, tmpSearchLine, corrWin, imgs[i]); // ͶӰ������Ӱ���ϵ������Ƿ���Ӱ����Ч��Χ��
////
////		if (validSearch) {
////			double tempCorr = CalCorr(corrWin, imgs[baseImgIdx], imgs[i], Pbase, tmp_Psearch, 0.3);
////			SNCC += tempCorr;
////			NumOfCalPairs++;
////		}
////	}
////
////	SNCC = SNCC / NumOfCalPairs;
////}
//
//
///*
//\brief: ���ݴ��ۿռ䣬����WTA�������õ��ֽ��
//\para[in]: cost �����жϵĴ��ۿռ�
//\para[in]: Z_reso �̲߳���,��Ӧcost
//\para[in]: localMin Ŀ���ؽ��������͸߳�
//\para[out]: res ����WTA�����õ���Ŀ�������ؽ��������DEM
//*/
//void WTA(float*** cost, float Z_reso, Mat &localMin, Mat &res) {
//
//	int rows = res.rows;
//	int cols = res.cols;
//
//	for (int i = 0; i < rows; ++i) {
//		float* p_localMin = localMin.ptr<float>(i);
//		float* p_res = res.ptr<float>(i);
//		for (int j = 0; j < cols; ++j) {
//			// ����cost[i][j]ָ��ָ�����������������߳�����
//			int t = _msize(cost[i][j]);
//			int capacity = t / sizeof(float);
//
//			int tmp_minCost = *min_element(cost[i][j], cost[i][j] + capacity);
//
//			if(tmp_minCost == Invalid_Float){ // ��ƽ���û���ҵ���
//				p_res[j] = Invalid_Float;
//				continue;
//			}
//
//			int tmp_minCost_Position = min_element(cost[i][j], cost[i][j] + capacity) - cost[i][j];
//			
//			float WTA_hei = p_localMin[j] + tmp_minCost_Position * Z_reso;
//			p_res[j] = WTA_hei;
//		}
//	}
//}
//
//

