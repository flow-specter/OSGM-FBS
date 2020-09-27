#include<numeric>
#include <algorithm> //vector��Ա����ͷ�ļ�
#include<windows.h>    //ͷ�ļ�  
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "osgm_types.h"
#include "osgm.h"
#include "G:/MVLL/MVLL/testYLJ/include/RFMBaseFunction.h"
#include "CubicSplineInterpolation.h"
#define PATHS_PER_SCAN 8
#define DEBUG false
#define TEST_TWO_IMAGE true
using namespace cv;










double groundCalCorr(int LengthOfWin, P3D ground,double roughRes,
	double UL_lon, double UL_lat,Mat img1, Mat img2, Mat RoughDem, vector<RPCMODEL>  rpcs, int baseindex, int searchIndex){
	/*
	����ʾ����


	*/

	double thinRes = roughRes / 3;
	//��������С����Mat
	cv::Mat win1(LengthOfWin, LengthOfWin, CV_8UC1); win1.setTo(0);
	cv::Mat win2(LengthOfWin, LengthOfWin, CV_8UC1); win2.setTo(0);

	//��win1��win2��ֵ
	for (int i = 0; i < LengthOfWin; i++)
	{
		uchar* p_win1 = win1.ptr<uchar>(i);
		uchar* p_win2 = win2.ptr<uchar>(i); //ָ�����ڸ�ֵ

		for (int j = 0; j <= LengthOfWin; j++)
		{
			double tmpSampleBase, tmpLineBase, tmpSampleSearch, tmpLineSearch;
			P3D tmpGroundP3D; int tmpX_ind, tmpY_ind;
			tmpGroundP3D.lat = thinRes*i + ground.lat - thinRes*(LengthOfWin - 1) / 2;
			tmpGroundP3D.lon = thinRes*j + ground.lon - thinRes*(LengthOfWin - 1) / 2;
			tmpX_ind = (tmpGroundP3D.lon - UL_lon) / roughRes;
			tmpY_ind = (UL_lat - tmpGroundP3D.lat) / roughRes;
			/*float* p = RoughDem.ptr<float>(tmpY_ind);*/

			tmpGroundP3D.z = ground.z; //��ground��Zֵһ��
			//tmpGroundP3D.z = p[tmpX_ind]; //roughDem���д��ĸ߳�

			if (tmpX_ind < 0 || tmpX_ind > RoughDem.cols
				|| tmpY_ind < 0 || tmpY_ind> RoughDem.rows || abs(tmpGroundP3D.z)>100000)
			{
				return 0;
			}

			//1. ȷ����������ά����
			API_LATLONGHEIFHT2LineSample(rpcs[baseindex], tmpGroundP3D.lat, tmpGroundP3D.lon, tmpGroundP3D.z, tmpSampleBase, tmpLineBase);
			API_LATLONGHEIFHT2LineSample(rpcs[searchIndex], tmpGroundP3D.lat, tmpGroundP3D.lon, tmpGroundP3D.z, tmpSampleSearch, tmpLineSearch);

			//2. ͶӰ���񷽣��õ��񷽵����꣬�����õ��񷽵�ĻҶ�ֵ������Сmat
			p_win1[j] = get_origin_value(img1, tmpSampleBase, tmpLineBase);
			p_win2[j] = get_origin_value(img2, tmpSampleSearch, tmpLineSearch);
			//����
			//cout << (int)p_win1[j] << " ";
		}
		//cout << endl;
	}

	//����win1��win2�����ϵ��������
	double corr2 = 0;
	double Amean2 = 0;
	double Bmean2 = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			Amean2 = Amean2 + dataA[n];
			Bmean2 = Bmean2 + dataB[n];
		}
	}
	Amean2 = Amean2 / (win1.rows * win1.cols);
	Bmean2 = Bmean2 / (win2.rows * win2.cols);
	double Cov = 0;
	double Astd = 0;
	double Bstd = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			//Э����
			Cov = Cov + (dataA[n] - Amean2) * (dataB[n] - Bmean2);
			//A�ķ���
			Astd = Astd + (dataA[n] - Amean2) * (dataA[n] - Amean2);
			//B�ķ���
			Bstd = Bstd + (dataB[n] - Bmean2) * (dataB[n] - Bmean2);
		}
	}
	corr2 = Cov / (sqrt(Astd * Bstd));
	return corr2;
	//https://blog.csdn.net/u013162930/article/details/50887019
};

double CalCorr(int LengthOfWin, Mat img1, Mat img2, ImgPoint P1, ImgPoint P2, float subpixelLength){
	//�����߽磬����ʱ�����õ㣬֮���ٿ��Ǽ�С���ڵ�
	if (P1.sample - (LengthOfWin - 1) / 2 < 0 || P1.sample + (LengthOfWin - 1) / 2 > img1.cols
		|| P1.line - (LengthOfWin - 1) / 2 < 0 || P1.line + (LengthOfWin - 1) / 2 > img1.rows)
	{
		return 0;
	}

	//��������С����Mat
	cv::Mat win1(LengthOfWin, LengthOfWin, CV_8UC1); win1.setTo(0);
	cv::Mat win2(LengthOfWin, LengthOfWin, CV_8UC1); win2.setTo(0);

	//��win1��win2��ֵ
	for (int i = 0; i <LengthOfWin; i++)
	{
		uchar* p_win1 = win1.ptr<uchar>(i);
		uchar* p_win2 = win2.ptr<uchar>(i); //ָ�����ڸ�ֵ

		for (int j = 0; j <= LengthOfWin; j++)
		{
			p_win1[j] = get_origin_value(img1, subpixelLength*i + P1.sample - subpixelLength*(LengthOfWin - 1) / 2, subpixelLength*j + P1.line - subpixelLength*(LengthOfWin - 1) / 2);
			p_win2[j] = get_origin_value(img2, subpixelLength*i + P2.sample - subpixelLength*(LengthOfWin - 1) / 2, subpixelLength*j + P2.line - subpixelLength*(LengthOfWin - 1) / 2);
			//����
			//cout << (int)p_win1[j] << " ";
		}
		//cout << endl;
	}

	/*�ԱȲ����Ƿ�������
	cout << "-------------------------------" << endl;
	for (int i = 0; i <LengthOfWin; i++)
	{
	uchar* p_win1 = win1.ptr<uchar>(i);
	uchar* p_win2 = win2.ptr<uchar>(i); //ָ�����ڸ�ֵ

	for (int j = 0; j <= LengthOfWin; j++)
	{
	p_win1[j] = get_origin_value(img1, i + P1.sample - (LengthOfWin - 1) / 2, j + P1.line - (LengthOfWin - 1) / 2);
	p_win2[j] = get_origin_value(img2, i + P2.sample - (LengthOfWin - 1) / 2, j + P2.line - (LengthOfWin - 1) / 2);
	//����
	cout << (int)p_win1[j] << " ";
	}
	cout << endl;
	}

	cout << "-------------------------------" << endl;
	*/

	//����win1��win2�����ϵ��������
	double corr2 = 0;
	double Amean2 = 0;
	double Bmean2 = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			Amean2 = Amean2 + dataA[n];
			Bmean2 = Bmean2 + dataB[n];
		}
	}
	Amean2 = Amean2 / (win1.rows * win1.cols);
	Bmean2 = Bmean2 / (win2.rows * win2.cols);
	double Cov = 0;
	double Astd = 0;
	double Bstd = 0;
	for (int m = 0; m < win1.rows; m++) {
		uchar* dataA = win1.ptr<uchar>(m);
		uchar* dataB = win2.ptr<uchar>(m);
		for (int n = 0; n < win1.cols; n++) {
			//Э����
			Cov = Cov + (dataA[n] - Amean2) * (dataB[n] - Bmean2);
			//A�ķ���
			Astd = Astd + (dataA[n] - Amean2) * (dataA[n] - Amean2);
			//B�ķ���
			Bstd = Bstd + (dataB[n] - Bmean2) * (dataB[n] - Bmean2);
		}
	}
	corr2 = Cov / (sqrt(Astd * Bstd));
	return corr2;
	//https://blog.csdn.net/u013162930/article/details/50887019
};

void WriteArguPath(ofstream &writeArguPath, int sufferZ, int largePenalty, int smallPenalty, int N_times_res, int corrWin, int NumOfImgs){
	if (writeArguPath){
		writeArguPath << "sufferZ: " << sufferZ << endl <<
			"largePenalty: " << largePenalty << endl <<
			"smallPenalty: " << smallPenalty << endl <<
			"ϸ������ N_times_res: " << N_times_res << endl <<
			"���ϵ�����ڣ�" << corrWin << endl <<
			NumOfImgs << "��Ӱ��" << endl <<
			" z�������: " << Z_resolution << "��" <<
			endl << "��ע�� ��ƬӰ��ϸ������Ϊ3����roughDemΪimread(F:/A_�����ʼ�/����/����/���й���/zhinvRoughDem.tif, -1);" << endl;
	}
}

int main(){
	clock_t startTime, endTime;
	startTime = clock();

	//------------------------------------------------------------------------------------0. �����������¼
	float sufferZ = Z_resolution;
	int corrWin = 45;
	int largePenalty = 16;
	int smallPenalty = 2;
	int N_times_res = 3; //ϸ������ _ 
	float uniquessRatio = 1.01;

	string dirName = "G:\\A_daily\\0719\\test4";  //����test�ļ���
	//bool flag = CreateDirectory(dirName.c_str(), NULL);
	//if (flag == false)
	//{
	//	cout << "�����ļ���ʧ��" << endl;
	//	system("pause");
	//	return 0;
	//}

	string path = dirName + "\\points3d.txt"; //3D��·��������
	string arguPath = dirName + "\\parameter.txt"; //����·��
	string costPath = dirName + "\\cost.txt"; //����·��
	string initialDemPath = dirName + "\\initialDem.txt";
	ofstream writeArguPath(arguPath);
	ofstream Cost(costPath);

	//------------------------------------------------------------------------------------1. sldemȡϸ���������͸�ʴ����Ӧ�̷߳�Χ

	//sldem Nac
	Mat roughDem = imread("G:/A_�����ʼ�/computers_and_geosciences/roughSLDEM/sldemNacAp1.tif", -1);
	double UL_lon = 23.3722757552958, UL_lat = 1.23648399230611;
	double rough_lat_res = 0.0019531249, rough_lon_res = 0.0019531249;

	// Ԥ������sldem�������ͺ͸�ʴ������ӦP3D��������Χ�����õ�ÿ�����������������ߵ��Լ���͵�
	Mat dilated_up_dem;
	Mat element_dilated = getStructuringElement(MORPH_RECT, Size(3, 3));
	dilate(roughDem, dilated_up_dem, element_dilated);

	//�鿴���͸߳����ֵ

	double max, min;
	cv::Point min_loc, max_loc;
	cv::minMaxLoc(dilated_up_dem, &min, &max, &min_loc, &max_loc);

	Mat erode_down_dem;
	Mat element_erode = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(roughDem, erode_down_dem, element_erode);

	//�鿴��ʴ�߳���Сֵ

	double maxErode, minErode;
	cv::Point min_erode_loc, max_erode_loc;
	cv::minMaxLoc(erode_down_dem, &minErode, &maxErode, &min_erode_loc, &max_erode_loc);

	int rough_rows = roughDem.rows, rough_cols = roughDem.cols;
	int detail_rows = rough_rows*N_times_res;
	int detail_cols = rough_cols*N_times_res;

	vector<P3D> P3Ds; P3D tmpP3D;
	double tmp_lat, tmp_lon; float tmp_z;
	float tmp_upHei, tmp_downHei;
	ofstream writeinitialDemPath(initialDemPath);
	int lat_ind, lon_ind;
	for (double lat = UL_lat; lat > UL_lat - rough_rows*rough_lat_res; lat -= rough_lat_res){
		lat_ind = (UL_lat - lat) / rough_lat_res;
		for (double lon = UL_lon; lon < UL_lon + rough_cols*rough_lon_res; lon += rough_lon_res)	{
			lon_ind = (lon - UL_lon) / rough_lat_res;
			float* p = roughDem.ptr<float>(lat_ind);
			tmp_z = p[lon_ind]; //roughDem���д��ĸ߳�
			float* p_upHei = dilated_up_dem.ptr<float>(lat_ind);
			float* p_downHei = erode_down_dem.ptr<float>(lat_ind);

			tmp_upHei = p_upHei[lon_ind] + Z_resolution;
			tmp_downHei = p_downHei[lon_ind] - Z_resolution;

			if (abs(tmp_z) >= 100000){ tmp_z = 0; };
			//if (abs(tmp_upHei) >= 100000 || abs(tmp_downHei) >= 100000){ tmp_upHei = roughZmean + Z_resolution; tmp_downHei = roughZmean - Z_resolution; };
			if (abs(tmp_upHei) >= 100000 || abs(tmp_downHei) >= 100000){ tmp_upHei = 0; tmp_downHei = 0; };

			//һ������ϸ��Ϊ9���������γɵ�һ�������
			for (int i = 0; i <= (N_times_res - 1); i++){
				for (int j = 0; j <= (N_times_res - 1); j++){
					tmp_lat = lat - i*rough_lat_res / N_times_res;
					tmp_lon = lon + j*rough_lat_res / N_times_res;
					tmpP3D.lat = tmp_lat;
					tmpP3D.lon = tmp_lon;
					tmpP3D.z = tmp_z; //��Ϊ��ϸ������ĳ�ֵ��Ӧ�ڸ�zֵ����һ����Χ�ڽ��б�������temp_zvalue-40*deltaz:temp_zvalue+40*deltaz
					tmpP3D.downHei = tmp_downHei;
					tmpP3D.upHei = tmp_upHei;
					P3Ds.push_back(tmpP3D);
				}
			}
		}
	}
	//------------------------------------------------------------------------------------2. ����Ӱ���Լ�rpc���������ڴ棬���ڴ洢C��S��A�Լ�����Ӧ�ĸ̷߳�Χ

	//����Ӱ���Լ�rpc
	vector<Mat> imgs;
	vector<RPCMODEL> rpcs;
	RPCMODEL rpc1, rpc2, rpc3, rpc4, rpc5, rpc6, rpc7;
	
	//sldem Nac�Ͷ�λ����һ�µ���Ƭ����Ƭ
	Mat img1 = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le.tif", 0);
	Mat img2 = imread("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re.tif", 0);

	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1126972080le_rpc.txt", rpc1);
	API_GetRPCMODEL("G:/A_�����ʼ�/computers_and_geosciences/test/ap11/m1114021499re_rpc.txt", rpc2);

	imgs.push_back(img1); imgs.push_back(img2);
	rpcs.push_back(rpc1); rpcs.push_back(rpc2);

	int NumOfImgs = imgs.size();
	WriteArguPath(writeArguPath, sufferZ, largePenalty, smallPenalty, N_times_res, corrWin, NumOfImgs);

	//3. ���仺����
	float ***C;  // pixel cost array W x H x D
	float ***S;  // aggregated cost array W x H x D
	float **TmpThisZ_max;   //�洢ÿһ��ƽ�������������Z_max��Z_min,��Ӧ��p3d���Ӧ
	float **TmpThisZ_min;   //�洢ÿһ��ƽ�������������Z_max��Z_min
	float  ****A; // single path cost array 2 x W x H x D

	C = new float **[detail_rows];
	S = new float **[detail_rows];
	TmpThisZ_max = new float *[detail_rows];
	TmpThisZ_min = new float *[detail_rows];

	//int row = detail_rows+1 ;
	//int k = row / N_times_res-1 ;
	//float* p_dilated = dilated_up_dem.ptr<float>(k);
	for (int row = 0; row < detail_rows; ++row)
	{
		C[row] = new float *[detail_cols]();
		S[row] = new float *[detail_cols]();
		TmpThisZ_max[row] = new float[detail_cols]();
		TmpThisZ_min[row] = new float[detail_cols]();

		for (int col = 0; col < detail_cols; ++col)
		{
			float* p_dilated; float* p_erode;
			p_dilated = dilated_up_dem.ptr<float>(row / N_times_res);
			p_erode = erode_down_dem.ptr<float>(row / N_times_res);
			float ThisZ_max, ThisZ_min;
			ThisZ_max = p_dilated[(col / N_times_res)] + Z_resolution;
			ThisZ_min = p_erode[(col / N_times_res)] - Z_resolution;

			//if (abs(ThisZ_max) >= 100000 || (abs(ThisZ_max) >= 100000)){ ThisZ_max = roughZmean + Z_resolution; ThisZ_min = roughZmean - Z_resolution; }
			if ((abs(ThisZ_max) >= 100000) || ((abs(ThisZ_min) >= 100000))){ ThisZ_max = 0; ThisZ_min = 0; }

			TmpThisZ_min[row][col] = ThisZ_min;
			TmpThisZ_max[row][col] = ThisZ_max;

			int adjustThisZ = (ThisZ_max - ThisZ_min) / Z_resolution + 1;
			C[row][col] = new float[adjustThisZ]();
			S[row][col] = new float[adjustThisZ](); // initialize to 0
		}
	}

	// #define PATHS_PER_SCAN 4
	A = new float  ***[PATHS_PER_SCAN];
	for (int path = 0; path < PATHS_PER_SCAN; ++path)
	{
		A[path] = new float **[detail_rows];
		for (int row = 0; row < detail_rows; ++row)
		{
			A[path][row] = new float *[detail_cols];
			for (int col = 0; col < detail_cols; ++col)
			{
				float* p_dilated; float* p_erode;

				p_dilated = dilated_up_dem.ptr<float>(row / N_times_res);
				p_erode = erode_down_dem.ptr<float>(row / N_times_res);

				float ThisZ_max, ThisZ_min;

				ThisZ_max = p_dilated[(col / N_times_res)] + Z_resolution;
				ThisZ_min = p_erode[(col / N_times_res)] - Z_resolution;

				//if (abs(ThisZ_max) >= 100000 || (abs(ThisZ_max) <= 100000)){ ThisZ_max = roughZmean + Z_resolution; ThisZ_min = roughZmean - Z_resolution; }
				if (abs(ThisZ_max) >= 100000 || (abs(ThisZ_min) >= 100000)){ ThisZ_max = 0; ThisZ_min = 0; }
				int adjustThisZ = (ThisZ_max - ThisZ_min) / Z_resolution + 1;

				A[path][row][col] = new float[adjustThisZ];
				for (unsigned int d = 0; d < adjustThisZ; ++d)
				{
					A[path][row][col][d] = 0;
				}
			}
		}
	}


	//------------------------------------------------------------------------------------3. calculate the cost of every voxel�������������ϵ������

	double X, Y, Z;
	double Sample, Line, BaseSample, BaseLine;
	ImgPoint Pbase;
	vector<ImgPoint> P(NumOfImgs);
	double TempSample, TempLine;
	int lastProgressPrinted = 0;
	int highCorrNum = 0;
	int trashcount = 0;
	for (int i = 0; i < P3Ds.size(); i++){
		P3D tmpP3D;
		X = P3Ds[i].lon;
		tmpP3D.lon = X;
		//if (X < min_x){ min_x = X; }
		int X_ind = (X - UL_lon) / rough_lat_res * N_times_res;
		if (X_ind >= detail_cols){ trashcount += 1; continue; }
		Y = P3Ds[i].lat;
		tmpP3D.lat = Y;

		//if (Y < min_y){ min_y = Y; }
		int Y_ind = (UL_lat - Y) / rough_lat_res * N_times_res;
		if (Y_ind >= detail_rows){ trashcount += 1; continue; }
		float Z_init = P3Ds[i].downHei;
		float Z_max = P3Ds[i].upHei;
		if (Z_max - Z_init == 0){ continue; }
		vector<double> AllZValueOfThisXY((Z_max - Z_init) / Z_resolution + 1);
		vector<double> AllCorrOfThisXY((Z_max - Z_init) / Z_resolution + 1);
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
				if (avgCorr >= 0.7){ highCorrNum++; }

				C[Y_ind][X_ind][Z_ind] = 1 - avgCorr; //����������ѡ��C[Y_ind][X_ind][Z_ind] = 1 - maxCorr;
				//AllCorrOfThisXY[Z_ind] = maxCorr; //����ѡ��avgCorr����maxCorr������������ѡ��AllCorrOfThisXY[Z_ind] = maxCorr; 
				//C[Y_ind][X_ind][Z_ind] = 1 - maxCorr; //����������ѡ��C[Y_ind][X_ind][Z_ind] = 1 - maxCorr;
				//if (maxCorr >= 0.7){ highCorrNum++; }

				if (Cost){
					Cost << fixed << setprecision(17) << X << " " << X_ind << " " << fixed << setprecision(17) << Y << " " << Y_ind << " " << Z << " " << Z_ind << " "
						<< fixed << setprecision(17) << C[Y_ind][X_ind][Z_ind] << endl;
				}
			}
		}
	}

		cout << highCorrNum << endl;

		//------------------------------------------------------------------------------------4. ����Ӧ�̷߳�Χ�ڴ��۾ۼ�
		aggregateCosts(detail_rows, detail_cols, TmpThisZ_max, TmpThisZ_min, C, A, S, sufferZ, largePenalty, smallPenalty);

		//------------------------------------------------------------------------------------5. ����������ֵ��ȡ0.1�׸߳̾��ȣ�д����ά��txt

		// ͨ���ۼ��Ĵ����������õ���ѵĸ̣߳���д��txt�ļ�
		float bestZ = 0, minCost;
		ofstream Points3D(path);
		vector<float> Delta;
		vector<P3D> OSGMP3Ds; //�����洢�����������ά����
		P3D tmpOSGMP3D;
		for (int i = 0; i < P3Ds.size(); i++){
			minCost = FLT_MAX;
			X = P3Ds[i].lon;
			int X_ind = (X - UL_lon) / rough_lat_res * N_times_res;
			if (X_ind >= detail_cols){ continue; };
			Y = P3Ds[i].lat;
			int Y_ind = (UL_lat - Y) / rough_lat_res * N_times_res;
			if (Y_ind >= detail_rows){ continue; };
			float Z_init = P3Ds[i].downHei;
			float Z_max = P3Ds[i].upHei;
			if (Z_max - Z_init == 0){ continue; }

			int ThisZHeightRange = ((Z_max - Z_init) / Z_resolution + 1);
			std::vector<double> input_x(ThisZHeightRange), input_y(ThisZHeightRange);
			for (float Z = Z_init; Z <= Z_max; Z += Z_resolution){
				int Z_ind = (Z - Z_init) / Z_resolution;
				//���߳��Լ����ۣ��ֱ���Ϊx��y vector���뺯�������������������������С��y����Ӧ��x�����߳�ֵ��
				//�ڶ��ַ�ʽ���������������0.1�׼���ľֲ���Сֵ
				input_x[Z_ind] = Z;
				input_y[Z_ind] = S[Y_ind][X_ind][Z_ind];
			}

			CubicSplineCoeffs *cubicCoeffs;
			CubicSplineInterpolation cubicSpline;
			cubicSpline.calCubicSplineCoeffs(input_x, input_y, cubicCoeffs, CUBIC_NATURAL, CUBIC_WITHOUT_FILTER);
			std::vector<double> output_x, output_y;
			cubicSpline.cubicSplineInterpolation(cubicCoeffs, input_x, output_x, output_y, 0.1);

			// --------------------------�ҵ���С�ʹ�С��costֵ���������ֵ����֮Ϊ�����Ա�ֵ��

			//�ҵ�output_y����Сֵ��λ�ã���λ�õ�outputx��Ϊ0.1�׾��ȵĸ߳�ֵ����ΪbestZ��
			vector<double>::iterator smallest = min_element(begin(output_y), end(output_y));
			int position = distance(begin(output_y), smallest);
			float smallestScost, secondSmallScost, signifiRatio; // ������СScostֵ�Լ���СCostֵ���Լ����ֵsignifiRatio
			smallestScost = output_y[position]; //��ʱ������Сֵ

			//ɾ����outputy�е���Сֵ����������Сֵ��Ϊ��Сֵ
			output_y.erase(smallest);
			vector<double>::iterator secondSmallest = min_element(begin(output_y), end(output_y));
			int secondPosition = distance(begin(output_y), secondSmallest);
			secondSmallScost = output_y[secondPosition];
			signifiRatio = secondSmallScost / smallestScost;
			cout << signifiRatio << endl;
			if (signifiRatio >= uniquessRatio) {
				tmpOSGMP3D.lat = P3Ds[i].lat;
				tmpOSGMP3D.lon = P3Ds[i].lon;
				bestZ = output_x[position];
				tmpOSGMP3D.z = bestZ;
				OSGMP3Ds.push_back(tmpOSGMP3D);
				int tmpDelta = bestZ - P3Ds[i].z;
				Delta.push_back(tmpDelta);
			}
		}

		//��SLDEM��ֵ��ϣ����������������Ϊ�ֲ�
		//��1������Delta�������
		double sum = std::accumulate(std::begin(Delta), std::end(Delta), 0.0);
		double mean = sum / Delta.size(); //��ֵ
		double accum = 0.0;
		std::for_each(std::begin(Delta), std::end(Delta), [&](const double d) {accum += (d - mean)*(d - mean); });
		double stdev = sqrt(accum / (Delta.size() - 1)); //�߲��
		//��2��������ά�㣬��С�ڵ������������������ Points3D��txt�С�
		float tolerance = 3 * stdev;

		for (int i = 0; i < OSGMP3Ds.size(); i++){
			if (abs(Delta[i]) > tolerance){ continue; }
			if (Points3D){
				Points3D << fixed << setprecision(17) << OSGMP3Ds[i].lon << " " << fixed << setprecision(17) << \
					OSGMP3Ds[i].lat << " " << fixed << setprecision(17) << OSGMP3Ds[i].z << endl;
			}
		}

		//------------------------------------------------------------------------------------6. �������ʱ��	
		endTime = clock();
		cout << "Totle Time : " << (double)(endTime - startTime) / (CLOCKS_PER_SEC * 60) << "min";
		if (writeArguPath){
			writeArguPath << endl << "��ʱ" << (double)(endTime - startTime) / (CLOCKS_PER_SEC * 60) << "min" << endl;
			writeArguPath << "��" << P3Ds.size() << "���㡣 " <<
				"���ϵ������0.6�ĵ�����" << highCorrNum << "����" << endl;
		}

		system("pause");

		return 0;
	}
