#ifndef _RFM_STRUCT_TYPEDEF_
#define _RFM_STRUCT_TYPEDEF_

#include "HIPStructDef.h"


#include <fstream>
#include <string>
#include <vector>


using namespace std;

typedef struct  
{
	double h;//该层的高程
	double x0,y0;//左上角点坐标
	double dx,dy;//x方向和y方向间隔
	int nx,ny;//间隔数目
	double *ddx,*ddy;//在计算前是lat，lon坐标，计算后是x,y方向残差
	double *lon,*lat;//add by fei 用于主图像保存经纬度
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