#include "OSGMMatchMVS.h"
#include <chrono>

using namespace std::chrono;


int main() {

	auto start = std::chrono::steady_clock::now();

	// ... 输入影像，预处理影像，
	// 读取文件夹中的影像
	string strPath = "E:\\mvllInitZDp\\data\\ap11\\";
	vector<uint8*> imgs;
	Mat Img0_ori = imread(strPath + "m1126972080le.tif", cv::IMREAD_GRAYSCALE); // 目标图像
	Mat Img1_ori = imread(strPath + "m1114021499re.tif",cv::IMREAD_GRAYSCALE); // 源图像
	Mat Img2_ori = imread(strPath + "m1126986303re.tif", cv::IMREAD_GRAYSCALE); // 源图像
	Mat Img3_ori = imread(strPath + "m1114007294re.tif", cv::IMREAD_GRAYSCALE); // 源图像
	Mat Img4_ori = imread(strPath + "m1114014396re.tif", cv::IMREAD_GRAYSCALE); // 源图像
	Mat Img5_ori = imread(strPath + "m1126986303re.tif", cv::IMREAD_GRAYSCALE); // 源图像
	Mat Img6_ori = imread(strPath + "m1129340193re.tif", cv::IMREAD_GRAYSCALE); // 源图像

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

	// 批量预处理文件夹中的影像
	// osgm_util::prehistMatch(imgs_mat, 0, imgs);
	// cout << imgs.size();
	// baseImg， 用来给定行列数
	
	// 读取rpc
	RPCMODEL rpc1, rpc2, rpc3, rpc4, rpc5, rpc6, rpc7;
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126972080le_rpc.txt", rpc1);
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1114021499re_rpc.txt", rpc2);
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc3);
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1114007294re_rpc.txt", rpc4);
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1114014396re_rpc.txt", rpc5);
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc6);
	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1129340193re_rpc.txt", rpc7);

	vector<RPCMODEL> rpcs;
	rpcs.push_back(rpc1);
	rpcs.push_back(rpc2);
	rpcs.push_back(rpc3);
	rpcs.push_back(rpc4);
	rpcs.push_back(rpc5);
	rpcs.push_back(rpc6);
	rpcs.push_back(rpc7);

	Mat roughDem_ori = imread("G:/A_工作笔记/computers_and_geosciences/roughSLDEM/sldemNacAp1.tif", -1);
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

	// ··· osgm 匹配
	const uint32 img_cols = static_cast<uint32>(Img0.cols);
	const uint32 img_rows = static_cast<uint32>(Img0.rows);

	// 设定参数
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

	// 初始化
	sint32 cost_size = 0;
	if (!osgm.Initialize(img_rows, img_cols, imgs.size(), resize_sldem, osgm_option, cost_size)) {
		std::cout << "OSGM初始化失败！" << std::endl;
		return -2;
	}

	// 匹配
	auto res = new float32[cost_size]();

 	if (!osgm.Match(imgs,rpcs,res) ){
		std::cout << "SGM匹配失败！" << std::endl;
		return -2;
	}
	
	auto end = std::chrono::steady_clock::now();
	auto tt = duration_cast<std::chrono::milliseconds>(end - start);
	printf("初始代价计算及匹配 Done! Timing : %lf s\n\n", tt.count() / 1000.0);

	// 显示res图
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
//#include <algorithm> //vector成员函数头文件
//#include<windows.h>    //头文件  
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
//\brief: chooseBaseImg 确定物方平面点投影至影像集时所确定的基准影像
//\para[in]: Imgs 多视影像
//\para[in]: rpcs 多视影像对应的有理函数模型
//\para[in]: lat 纬度
//\para[in]: lon 经度，和上面的纬度组成确定了该平面点位置
//\para[in]: localMin 该平面位置的最低搜索高程
//\para[in]: localMax 该平面位置的最高搜索高程
//\para[in]: corrWin 当设定的相关系数计算窗口为规则方形窗口时，corrWin为设定的窗口边长
//\para[out]: BaseIdx 选取的该平面位置的基准影像
//*/
//void chooseBaseImg(vector<Mat>& Imgs, vector<RPCMODEL>& rpcs, float lat, float lon, float localMin, float localMax, int corrWin, int& BaseIdx);
//
///*
//\brief: calSNCC 计算该三维点对应的SNCC值
//\para[in] lat 纬度
//\para[in] lon 经度
//\para[in] hei 高程
//\para[in] imgs 多视影像
//\para[in] rpcs 多视影像对应的有理函数模型
//\para[in] baseImgIdx 基准影像序标
//\para[in] corrWin 计算相关系数的窗口大小
//\para[out] SNCC 返回的该三维点对应的SNCC值
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
//\brief: 根据代价空间，利用WTA方法，得到粗结果
//\para[in]: cost 用来判断的代价空间
//\para[in]: Z_reso 高程步长,对应cost
//\para[in]: localMin 目标重建区域的最低高程
//\para[out]: res 根据WTA方法得到的目标区域重建结果，即DEM
//*/
//void WTA(float*** cost, float Z_reso, Mat& localMin, Mat& res);
//
//
//
//int main(){
//	
//
//
//	//std::string strPath = "G:\\A_工作笔记\\computers_and_geosciences\\test\\ap11\\";
//	//Mat matSrc0 = imread(strPath + "m1126986303re.tif"); // 源图像
//	//Mat matSrc = imread(strPath + "m1114021499re.tif"); // 源图像
//	//Mat matDst = imread(strPath + "m1126972080le.tif"); // 目标图像
//	//Mat result,result1;
//	//myPreProcessing.histogram_Matching(matSrc, matDst,result);
//	//myPreProcessing.histogram_Matching(matSrc0, matDst, result1);
//
//	//testLamMatch(); // TODO： 测试最小二乘方法
//	clock_t startTime, endTime;
//	startTime = clock();
//	preProcessing myPreProcessing;
//	//------------------------------------------------------------------------------
//	//-----参数定义
//	float sufferZ = 1;
//	int corrWin = 45;
//	int largePenalty = 16;
//	int smallPenalty = 2;
//	int N_times_res = 3; //细化倍数 _ 
//	float uniquessRatio = 1.01;
//
//	string dirName = "G:\\A_daily\\0719\\test4";  //创建test文件夹
//	string path = dirName + "\\points3d.txt"; //3D点路径及名称
//	string arguPath = dirName + "\\parameter.txt"; //参数路径
//	string costPath = dirName + "\\cost.txt"; //参数路径
//	string initialDemPath = dirName + "\\initialDem.txt";
//	ofstream writeArguPath(arguPath);
//	ofstream Cost(costPath);
//	//------------------------------------------------------------------------------------1. sldem取细格网并膨胀腐蚀自适应高程范围
//	//sldem Nac
//	Mat roughDem_ori = imread("G:/A_工作笔记/computers_and_geosciences/roughSLDEM/sldemNacAp1.tif", -1);
//	Mat roughDem = roughDem_ori.clone();
//	myPreProcessing.deleteNodataValue(roughDem_ori, roughDem);
//
//
//	double UL_lon = 23.3722757552958, UL_lat = 1.23648399230611;
//	double rough_lat_res = 0.0019531249, rough_lon_res = 0.0019531249;
//
//	// ... 预处理sldem，对sldem进行膨胀和腐蚀，自适应P3D的搜索范围，即得到每个待求格网的搜索最高点以及最低点
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
//	//... 输入影像以及rpc
//	vector<Mat> imgs;
//	vector<RPCMODEL> rpcs;
//	RPCMODEL rpc1, rpc2, rpc3;
//
//	//sldem Nac和定位保持一致的两片与三片
//	Mat img1 = imread("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126972080le.tif", 0);
//	Mat img2_origin = imread("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1114021499re.tif",0);
//	Mat img3__origin = imread("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126986303re.tif",0); // 源图像
//
//	Mat img2, img3;
//	myPreProcessing.histMatch_Value(img2_origin, img1, img2);
//	myPreProcessing.histMatch_Value(img3__origin, img1, img3);
//
//	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126972080le_rpc.txt", rpc1);
//	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1114021499re_rpc.txt", rpc2);
//	API_GetRPCMODEL("G:/A_工作笔记/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc3);
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
//\brief: calSNCC 计算该三维点对应的SNCC值
//\para[in] lat 纬度
//\para[in] lon 经度
//\para[in] hei 高程
//\para[in] imgs 多视影像
//\para[in] rpcs 多视影像对应的有理函数模型
//\para[in] baseImgIdx 基准影像序标
//\para[in] corrWin 计算相关系数的窗口大小
//\para[out] SNCC 返回的该三维点对应的SNCC值
//*/
////void calSNCC(double lat, double lon, double hei, vector<Mat>& imgs, vector<RPCMODEL> rpcs, int baseImgIdx, int corrWin, float& SNCC) {
////	int NumOfImgs = imgs.size();
////
////
////	// 确定baseIdx的投影像点位置BaseSample,BaseLine
////	double BaseSample, BaseLine;
////	ImgPoint Pbase;
////	API_LATLONGHEIFHT2LineSample(rpcs[baseImgIdx], lat, lon, hei, BaseSample, BaseLine);
////	Pbase.line = BaseLine;
////	Pbase.sample = BaseSample;
////
////	int NumOfCalPairs = 0; // 计算了几对窗口
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
////		bool validSearch = osgm_util::ifInImg(tmpSearchSample, tmpSearchLine, corrWin, imgs[i]); // 投影至搜索影像上的坐标是否在影像有效范围内
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
//\brief: 根据代价空间，利用WTA方法，得到粗结果
//\para[in]: cost 用来判断的代价空间
//\para[in]: Z_reso 高程步长,对应cost
//\para[in]: localMin 目标重建区域的最低高程
//\para[out]: res 根据WTA方法得到的目标区域重建结果，即DEM
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
//			// 计算cost[i][j]指针指向的数组的容量，即高程容量
//			int t = _msize(cost[i][j]);
//			int capacity = t / sizeof(float);
//
//			int tmp_minCost = *min_element(cost[i][j], cost[i][j] + capacity);
//
//			if(tmp_minCost == Invalid_Float){ // 该平面点没有找到解
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

