#include<numeric>
#include <algorithm> //vector��Ա����ͷ�ļ�
#include<windows.h>    //ͷ�ļ�  
#include <iomanip>
#include <iostream>

#include "preProcessing.h"
#include "osgm_types.h"
#include "osgm_utils.h"
#include "CubicSplineInterpolation.h"
#include "../include/mlRVML.h"
#include "../include/mlTypes.h"


#define PATHS_PER_SCAN 8
#define DEBUG falses
#define TEST_TWO_IMAGE true


//constexpr auto Z_resolution = 1;
using namespace cv;
using namespace std;

/*
brief: getP3DSearchRange �õ���ʼ��DEMÿ����������߸߳��Լ���͸̣߳����趨������Χ��
para[in]: 
para[in]:
para[in]:

*/
void getP3DSearchRange(Mat roughDem, Mat &dilated_up_dem, Mat &erode_down_dem,sint8 structuringElementSize=3) {
	
	Mat element_dilated = getStructuringElement(MORPH_RECT, Size(structuringElementSize, structuringElementSize));
	dilate(roughDem, dilated_up_dem, element_dilated);

	////�鿴���͸߳����ֵ
	//double max, min;
	//cv::Point min_loc, max_loc;
	//cv::minMaxLoc(dilated_up_dem, &min, &max, &min_loc, &max_loc);

	Mat element_erode = getStructuringElement(MORPH_RECT, Size(structuringElementSize, structuringElementSize));
	erode(roughDem, erode_down_dem, element_erode);

	//�鿴��ʴ�߳���Сֵ
	//double maxErode, minErode;
	//cv::Point min_erode_loc, max_erode_loc;
	//cv::minMaxLoc(erode_down_dem, &minErode, &maxErode, &min_erode_loc, &max_erode_loc);

}

void testLamMatch() {
	cv::Mat img_left = cv::imread("E:/SemiGlobalMatching/Data/cone/im2.png", cv::IMREAD_GRAYSCALE);
	cv::Mat img_right = cv::imread("E:/SemiGlobalMatching/Data/cone/im6.png", cv::IMREAD_GRAYSCALE);
	vector<cv::Point3f> leftPoints;
	Point3f tmp1; tmp1.x = 100; tmp1.y = 60;
	Point3f tmp2; tmp2.x = 60; tmp2.y = 100;
	leftPoints.push_back(tmp1);
	leftPoints.push_back(tmp2);

	kyhMatchImgbyLST(img_left, img_right, leftPoints);
}

void allocateCAS(float*** C, float*** S, float**** A, const Mat& dilated_up_dem, const Mat& erode_down_dem);

void rowCol2LatLon(int row, int col, float &lat, float &lon, float UL_lon, float UL_lat, float reso) {
	lat = UL_lat + row * reso;
	lon = UL_lon + col * reso;
}

void calSNCC(float lat, float lon, float hei, const Mat& baseImg, const vector<Mat>& searchImgs, const vector<RPCMODEL> rpcs, float SNCC_res);


int main(){
	


	//std::string strPath = "G:\\A_�����ʼ�\\computers_and_geosciences\\test\\ap11\\";
	//Mat matSrc0 = imread(strPath + "m1126986303re.tif"); // Դͼ��
	//Mat matSrc = imread(strPath + "m1114021499re.tif"); // Դͼ��
	//Mat matDst = imread(strPath + "m1126972080le.tif"); // Ŀ��ͼ��
	//Mat result,result1;
	//myPreProcessing.histogram_Matching(matSrc, matDst,result);
	//myPreProcessing.histogram_Matching(matSrc0, matDst, result1);

	//testLamMatch(); // TODO�� ������С���˷���
	clock_t startTime, endTime;
	startTime = clock();
	preProcessing myPreProcessing;
	//------------------------------------------------------------------------------
	//-----��������
	float sufferZ = 1;
	int corrWin = 45;
	int largePenalty = 16;
	int smallPenalty = 2;
	int N_times_res = 3; //ϸ������ _ 
	float uniquessRatio = 1.01;

	string dirName = "G:\\A_daily\\0719\\test4";  //����test�ļ���
	string path = dirName + "\\points3d.txt"; //3D��·��������
	string arguPath = dirName + "\\parameter.txt"; //����·��
	string costPath = dirName + "\\cost.txt"; //����·��
	string initialDemPath = dirName + "\\initialDem.txt";
	ofstream writeArguPath(arguPath);
	ofstream Cost(costPath);
	//------------------------------------------------------------------------------------1. sldemȡϸ���������͸�ʴ����Ӧ�̷߳�Χ
	//sldem Nac
	Mat roughDem_ori = imread("G:/A_�����ʼ�/computers_and_geosciences/roughSLDEM/sldemNacAp1.tif", -1);
	Mat roughDem = roughDem_ori.clone();
	myPreProcessing.deleteNodataValue(roughDem_ori, roughDem);


	double UL_lon = 23.3722757552958, UL_lat = 1.23648399230611;
	double rough_lat_res = 0.0019531249, rough_lon_res = 0.0019531249;

	// ... Ԥ����sldem����sldem�������ͺ͸�ʴ������ӦP3D��������Χ�����õ�ÿ�����������������ߵ��Լ���͵�
	int rough_rows = roughDem.rows, rough_cols = roughDem.cols;
	int dst_rows = rough_rows*N_times_res;
	int dst_cols = rough_cols*N_times_res;
	Mat resize_sldem_ori = Mat::zeros(dst_rows, dst_cols, CV_32F);;
	resize(roughDem, resize_sldem_ori, resize_sldem_ori.size(), 0, 0, INTER_AREA);

	Mat dilated_up_dem_ori, erode_down_dem_ori;
	getP3DSearchRange(resize_sldem_ori, dilated_up_dem_ori, erode_down_dem_ori);
	Mat resize_sldem = resize_sldem_ori.clone();
	Mat dilated_up_dem = dilated_up_dem_ori.clone();
	Mat erode_down_dem = erode_down_dem_ori.clone();
	myPreProcessing.deleteNodataValue(resize_sldem_ori, resize_sldem);
	myPreProcessing.deleteNodataValue(dilated_up_dem_ori, dilated_up_dem);
	myPreProcessing.deleteNodataValue(erode_down_dem_ori, erode_down_dem);

	//... ����Ӱ���Լ�rpc
	vector<Mat> imgs;
	vector<RPCMODEL> rpcs;
	RPCMODEL rpc1, rpc2, rpc3;

	//sldem Nac�Ͷ�λ����һ�µ���Ƭ����Ƭ
	Mat img1 = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le.tif", 0);
	Mat img2_origin = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re.tif",0);
	Mat img3__origin = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126986303re.tif",0); // Դͼ��

	Mat img2, img3;
	myPreProcessing.histMatch_Value(img2_origin, img1, img2);
	myPreProcessing.histMatch_Value(img3__origin, img1, img3);

	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le_rpc.txt", rpc1);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re_rpc.txt", rpc2);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126986303re_rpc.txt", rpc3);

	imgs.push_back(img1); imgs.push_back(img2); imgs.push_back(img3);
	rpcs.push_back(rpc1); rpcs.push_back(rpc2); rpcs.push_back(rpc3);

	int NumOfImgs = imgs.size();

	//... 3. ���仺����
	float*** C = NULL;  // pixel cost array W x H x D
	float*** S = NULL;;  // aggregated cost array W x H x D
	float**** A = NULL;; // single path cost array 2 x W x H x D

	allocateCAS(C, S, A, dilated_up_dem, erode_down_dem);

	//... ��ʼ���ۼ��㣬 C��ֵ
	float reso = rough_lat_res / N_times_res;
	for (int row = 0; row < dst_rows; row++) {
		
		const float* ptr_localMax = dilated_up_dem.ptr<float>(row);
		const float* ptr_localMin = erode_down_dem.ptr<float>(row);

		for (int col = 0; col < dst_cols; col++) {
			float tmp_lat, tmp_lon;
			rowCol2LatLon(row, col, tmp_lat, tmp_lon, UL_lon, UL_lat, reso);
			//cout << tmp_lat << " " << tmp_lon << endl;
			for (int height = ptr_localMin[col]; height < ptr_localMax[col]; height += Z_resolution) {
				
				// ����ƽ����� SNCC
				int hei_ind = (height - ptr_localMin[col]) / Z_resolution;
				double SumCorrOfThisHei = 0;
				double AveCorrOfThisHei = 0;



			}
		}
	}

	
	
	
	//double X, Y, Z;
	//double Sample, Line, BaseSample, BaseLine;
	//ImgPoint Pbase;
	//vector<ImgPoint> P(NumOfImgs);
	//double TempSample, TempLine;
	//int lastProgressPrinted = 0;
	//int highCorrNum = 0;
	//int trashcount = 0;

	////cout << P3Ds.size() << endl;

	for (int i = 0; i < P3Ds.size(); i++){
		P3D tmpP3D;
		X = P3Ds[i].lon;
		tmpP3D.lon = X;
		//if (X < min_x){ min_x = X; }
		int X_ind = (X - UL_lon) / rough_lat_res * N_times_res;
		if (X_ind >= dst_cols){ trashcount += 1; continue; }
		Y = P3Ds[i].lat;
		tmpP3D.lat = Y;

		//if (Y < min_y){ min_y = Y; }
		int Y_ind = (UL_lat - Y) / rough_lat_res * N_times_res;
		if (Y_ind >= dst_rows){ trashcount += 1; continue; }
		float Z_init = P3Ds[i].downHei;
		float Z_max = P3Ds[i].upHei;
		if (Z_max - Z_init == 0){ continue; }
		UINT candidateZNum = (Z_max - Z_init) / Z_resolution + 1;
		vector<double> AllZValueOfThisXY(candidateZNum);
		vector<double> AllCorrOfThisXY(candidateZNum);


		for (float Z = Z_init; Z <= Z_max; Z += Z_resolution){ //�Ƿ�����̳߳�ʼ��Χ��ʵ��
			tmpP3D.z = Z;
			//for (float Z = ThisCubioInfo.Z_min; Z <= ThisCubioInfo.Z_max; Z += Z_resolution){
			int Z_ind = (Z - Z_init) / Z_resolution;
			double SumCorrOfThisZ = 0;
			double AveCorrOfThisZ = 0;
			//double maxCorr = -1; //��ʼ��Ϊ���ϵ����Сֵ
			double avgCorr = 0; //��ʼ��Ϊ0,�ڶ���ѡ��ѡ��ƽ�����ϵ��
			//1. �Զ��Ӱ����б��������м���Ӱ�����к�ѡͬ������
			int TempInNum = 0;
			for (int i = 0; i<NumOfImgs; i++){
				API_LATLONGHEIFHT2LineSample(rpcs[i], Y, X, Z, TempSample, TempLine);
				if (TempSample - ceil(corrWin / 2)>0 && TempSample + ceil(corrWin / 2)< img1.cols && TempLine - ceil(corrWin / 2)>0 && TempLine + ceil(corrWin / 2) < img1.rows){
					//��¼�ڵ�Ӱ��+1���Լ���¼��Ӧ�ڵ���Ϣ
					TempInNum += 1;
					P[i].line = TempLine;
					P[i].sample = TempSample;
				}
				else{
					//���򣬽�����Ϊ-1
					P[i].line = -1;
					P[i].sample = -1;
				}
			}
			//2. ���������ص����ϣ���ѡ����׼Ӱ��������ϵ�������������ø̵߳�
			int baseIndex;
			if (TempInNum >= 2){
				//2.1 ��������˳��ѡ��baseӰ��
				for (int i = 0; i<NumOfImgs; i++){
					if (P[i].sample - ceil(corrWin / 2)>0 && P[i].sample + ceil(corrWin / 2)< img1.cols && P[i].line - ceil(corrWin / 2)>0 && P[i].line + ceil(corrWin / 2) < img1.rows){
						Pbase.sample = P[i].sample;
						Pbase.line = P[i].line;
						baseIndex = i;
						break;
					}
				}
				//2.2 ѡ��base�󣬼�����������Ӱ��֮������ϵ��֮�ͣ�����ƽ�������ֵ��

				for (int i = baseIndex + 1; i < NumOfImgs; i++){
					if (P[i].sample - ceil(corrWin / 2)>0 && P[i].sample + ceil(corrWin / 2)< img1.cols && P[i].line - ceil(corrWin / 2)>0 && P[i].line + ceil(corrWin / 2) < img1.rows){
						//double tempCorr = groundCalCorr(corrWin, tmpP3D, rough_lat_res, UL_lon, UL_lat, imgs[baseIndex], imgs[i], roughDem, rpcs, baseIndex, i);

						double tempCorr = CalCorr(corrWin, imgs[baseIndex], imgs[i], Pbase, P[i], 0.3);
						//double tempCorr = (tempCorr1 + tempCorr2) / 2;
						avgCorr += tempCorr;
						//if (maxCorr <= tempCorr){
						//	maxCorr = tempCorr;
						//	//}
						//}
					}
				}

				avgCorr = avgCorr / (NumOfImgs - 1);
				AllZValueOfThisXY[Z_ind] = Z;
				AllCorrOfThisXY[Z_ind] = avgCorr; //����ѡ��avgCorr����maxCorr������������ѡ��AllCorrOfThisXY[Z_ind] = maxCorr; 
				if (avgCorr >= 0.7){ 
					highCorrNum++;  				
					cout << candidateZNum << endl;
					cout << avgCorr << endl;
				}

				C[Y_ind][X_ind][Z_ind] = 1 - avgCorr; //����������ѡ��C[Y_ind][X_ind][Z_ind] = 1 - maxCorr;
				//AllCorrOfThisXY[Z_ind] = maxCorr; //����ѡ��avgCorr����maxCorr������������ѡ��AllCorrOfThisXY[Z_ind] = maxCorr; 
				//C[Y_ind][X_ind][Z_ind] = 1 - maxCorr; //����������ѡ��C[Y_ind][X_ind][Z_ind] = 1 - maxCorr;
				//if (maxCorr >= 0.7){ highCorrNum++; }
			}
		}
	}

	//	cout << highCorrNum << endl;

	//	//------------------------------------------------------------------------------------4. ����Ӧ�̷߳�Χ�ڴ��۾ۼ�
	//	aggregateCosts(dst_rows, dst_cols, TmpThisZ_max, TmpThisZ_min, C, A, S, sufferZ, largePenalty, smallPenalty);

	//	//------------------------------------------------------------------------------------5. ����������ֵ��ȡ0.1�׸߳̾��ȣ�д����ά��txt

	//	// ͨ���ۼ��Ĵ����������õ���ѵĸ̣߳���д��txt�ļ�
	//	float bestZ = 0, minCost;
	//	ofstream Points3D(path);
	//	vector<float> Delta;
	//	vector<P3D> OSGMP3Ds; //�����洢�����������ά����
	//	P3D tmpOSGMP3D;
	//	for (int i = 0; i < P3Ds.size(); i++){
	//		minCost = FLT_MAX;
	//		X = P3Ds[i].lon;
	//		int X_ind = (X - UL_lon) / rough_lat_res * N_times_res;
	//		if (X_ind >= dst_cols){ continue; };
	//		Y = P3Ds[i].lat;
	//		int Y_ind = (UL_lat - Y) / rough_lat_res * N_times_res;
	//		if (Y_ind >= dst_rows){ continue; };
	//		float Z_init = P3Ds[i].downHei;
	//		float Z_max = P3Ds[i].upHei;
	//		if (Z_max - Z_init == 0){ continue; }

	//		int ThisZHeightRange = ((Z_max - Z_init) / Z_resolution + 1);
	//		std::vector<double> input_x(ThisZHeightRange), input_y(ThisZHeightRange);
	//		for (float Z = Z_init; Z <= Z_max; Z += Z_resolution){
	//			int Z_ind = (Z - Z_init) / Z_resolution;
	//			//���߳��Լ����ۣ��ֱ���Ϊx��y vector���뺯�������������������������С��y����Ӧ��x�����߳�ֵ��
	//			//�ڶ��ַ�ʽ���������������0.1�׼���ľֲ���Сֵ
	//			input_x[Z_ind] = Z;
	//			input_y[Z_ind] = S[Y_ind][X_ind][Z_ind];
	//		}

	//		CubicSplineCoeffs *cubicCoeffs;
	//		CubicSplineInterpolation cubicSpline;
	//		cubicSpline.calCubicSplineCoeffs(input_x, input_y, cubicCoeffs, CUBIC_NATURAL, CUBIC_WITHOUT_FILTER);
	//		std::vector<double> output_x, output_y;
	//		cubicSpline.cubicSplineInterpolation(cubicCoeffs, input_x, output_x, output_y, 0.1);

	//		// --------------------------�ҵ���С�ʹ�С��costֵ���������ֵ����֮Ϊ�����Ա�ֵ��

	//		//�ҵ�output_y����Сֵ��λ�ã���λ�õ�outputx��Ϊ0.1�׾��ȵĸ߳�ֵ����ΪbestZ��
	//		vector<double>::iterator smallest = min_element(begin(output_y), end(output_y));
	//		int position = distance(begin(output_y), smallest);
	//		float smallestScost, secondSmallScost, signifiRatio; // ������СScostֵ�Լ���СCostֵ���Լ����ֵsignifiRatio
	//		smallestScost = output_y[position]; //��ʱ������Сֵ

	//		//ɾ����outputy�е���Сֵ����������Сֵ��Ϊ��Сֵ
	//		output_y.erase(smallest);
	//		vector<double>::iterator secondSmallest = min_element(begin(output_y), end(output_y));
	//		int secondPosition = distance(begin(output_y), secondSmallest);
	//		secondSmallScost = output_y[secondPosition];
	//		signifiRatio = secondSmallScost / smallestScost;
	//		cout << signifiRatio << endl;
	//		if (signifiRatio >= uniquessRatio) {
	//			tmpOSGMP3D.lat = P3Ds[i].lat;
	//			tmpOSGMP3D.lon = P3Ds[i].lon;
	//			bestZ = output_x[position];
	//			tmpOSGMP3D.z = bestZ;
	//			OSGMP3Ds.push_back(tmpOSGMP3D);
	//			int tmpDelta = bestZ - P3Ds[i].z;
	//			Delta.push_back(tmpDelta);
	//		}
	//	}

	//	//... ��SLDEM��ֵ��ϣ����������������Ϊ�ֲ�
	//	//��1������Delta�������
	//	double sum = std::accumulate(std::begin(Delta), std::end(Delta), 0.0);
	//	double mean = sum / Delta.size(); //��ֵ
	//	double accum = 0.0;
	//	std::for_each(std::begin(Delta), std::end(Delta), [&](const double d) {accum += (d - mean)*(d - mean); });
	//	double stdev = sqrt(accum / (Delta.size() - 1)); //�߲��
	//	//��2��������ά�㣬��С�ڵ������������������ Points3D��txt�С�
	//	float tolerance = 3 * stdev;

	//	for (int i = 0; i < OSGMP3Ds.size(); i++){
	//		if (abs(Delta[i]) > tolerance){ continue; }
	//		if (Points3D){
	//			Points3D << fixed << setprecision(17) << OSGMP3Ds[i].lon << " " << fixed << setprecision(17) << \
	//				OSGMP3Ds[i].lat << " " << fixed << setprecision(17) << OSGMP3Ds[i].z << endl;
	//		}
	//	}

	//	//------------------------------------------------------------------------------------6. �������ʱ��	
	//	endTime = clock();
	//	cout << "Totle Time : " << (double)(endTime - startTime) / (CLOCKS_PER_SEC * 60) << "min";
	//	//if (writeArguPath){
	//	//	writeArguPath << endl << "��ʱ" << (double)(endTime - startTime) / (CLOCKS_PER_SEC * 60) << "min" << endl;
	//	//	writeArguPath << "��" << P3Ds.size() << "���㡣 " <<
	//	//		"���ϵ������0.6�ĵ�����" << highCorrNum << "����" << endl;
	//	//}

		system("pause");

		return 0;
	}


void allocateCAS(float*** C, float*** S, float**** A, const Mat& dilated_up_dem, const Mat& erode_down_dem) {
	int dst_rows = dilated_up_dem.rows;
	int dst_cols = dilated_up_dem.cols;
	C = new float** [dst_rows];
	S = new float** [dst_rows];

	for (int row = 0; row < dst_rows; ++row)
	{
		C[row] = new float* [dst_cols];
		S[row] = new float* [dst_cols];

		const float* ptr_localMax = dilated_up_dem.ptr<float>(row);
		const float* ptr_localMin = erode_down_dem.ptr<float>(row);

		for (int col = 0; col < dst_cols; ++col)
		{

			if (ptr_localMax[col] == Invalid_Float || ptr_localMin[col] == Invalid_Float) {
				C[row][col] = nullptr;
				S[row][col] = nullptr;
				continue;
			}

			int adjustThisZ = (ptr_localMax[col] - ptr_localMin[col]) / Z_resolution + 1;
			//cout << ptr_localMax[col] << " "<< ptr_localMin[col] << " "<< adjustThisZ << endl;

			C[row][col] = new float[adjustThisZ]();
			S[row][col] = new float[adjustThisZ](); // initialize to 0

			//if (row == 10 && col == 10) { cout << adjustThisZ; }

			for (int hei_ind = 0; hei_ind < adjustThisZ; hei_ind++) {
				C[row][col][hei_ind] = Invalid_Float;
				S[row][col][hei_ind] = Invalid_Float;
				//cout << row << " " << col << " " << hei_ind << endl;
			}
		}
	}

	// #define PATHS_PER_SCAN 4
	A = new float*** [PATHS_PER_SCAN];
	for (int path = 0; path < PATHS_PER_SCAN; ++path)
	{
		A[path] = new float** [dst_rows];
		for (int row = 0; row < dst_rows; ++row)
		{
			A[path][row] = new float* [dst_cols];
			for (int col = 0; col < dst_cols; ++col)
			{

				const float* ptr_localMax = dilated_up_dem.ptr<float>(row);
				const float* ptr_localMin = erode_down_dem.ptr<float>(row);

				if (ptr_localMax[col] == Invalid_Float || ptr_localMin[col] == Invalid_Float) {
					A[path][row][col] = nullptr;
					continue;
				}

				int adjustThisZ = (ptr_localMax[col] - ptr_localMin[col]) / Z_resolution + 1;
				//cout << ptr_localMax[col] << " "<< ptr_localMin[col] << " "<< adjustThisZ << endl;

				A[path][row][col] = new float[adjustThisZ]();

				for (int hei_ind = 0; hei_ind < adjustThisZ; hei_ind++) {
					A[path][row][col][hei_ind] = Invalid_Float;
				}
			}
		}
	}
};



// TODO
void calSNCC(float lat, float lon, float hei, const vector<Mat>& Imgs, const vector<RPCMODEL> rpcs, int corrWin,  float& SNCC_res) {
	
	int NumOfImgs = Imgs.size();
	int TempInNum = 0;
	double TempSample, TempLine;
	for (int i = 0; i < NumOfImgs; i++) {
		API_LATLONGHEIFHT2LineSample(rpcs[i], lat, lon, hei, TempSample, TempLine);
		if ((TempSample - ceil(corrWin / 2)) > 0 && (TempSample + ceil(corrWin / 2)) < Imgs[i].cols && (TempLine - ceil(corrWin / 2)) > 0 && 
			(TempLine + ceil(corrWin / 2)) < Imgs[i].rows) {
			//��¼�ڵ�Ӱ��+1���Լ���¼��Ӧ�ڵ���Ϣ
			TempInNum += 1;
			P[i].line = TempLine;
			P[i].sample = TempSample;
		}
		else {
			//���򣬽�����Ϊ-1
			P[i].line = -1;
			P[i].sample = -1;
		}
	}


}
