#ifndef _RFM_STRUCT_TYPEDEF_
#define _RFM_STRUCT_TYPEDEF_

#include "HIPStructDef.h"


#include <fstream>
#include <string>
#include <vector>


using namespace std;

typedef struct  
{
	double h;//�ò�ĸ߳�
	double x0,y0;//���Ͻǵ�����
	double dx,dy;//x�����y������
	int nx,ny;//�����Ŀ
	double *ddx,*ddy;//�ڼ���ǰ��lat��lon���꣬�������x,y����в�
	double *lon,*lat;//add by fei ������ͼ�񱣴澭γ��
}RPCGCPSTRUCT;

typedef struct{
	double LNUM[20],LDEN[20],SNUM[20],SDEN[20];
	double LAT_OFF,LAT_SCALE,LONG_OFF,LONG_SCALE,HEIGHT_OFF,HEIGHT_SCALE;
	double LINE_OFF,LINE_SCALE,SAMP_OFF,SAMP_SCALE;
	double px[3],py[3];
	double invpx[3],invpy[3];
	int nz;
	RPCGCPSTRUCT *rpcgcp;

	double ptx[3],pty[3];
	double invptx[3],invpty[3];//add by liubin for Time Refinement

}RPCMODEL;


#endif