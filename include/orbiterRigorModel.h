#ifndef _HIP_ORBITER_PL_RIGOR_MODEL_H_LLL
#define _HIP_ORBITER_PL_RIGOR_MODEL_H_LLL

#include "HIPgeoModelInterface.h"

#include "../../include/HIPtypeDef.h"



#ifdef ORBITERRIGORMOEL_EXPORTS
#define BASE_API			HIP_EXPORT
#define BASE_CLASS          HIP_EXPORT
#else
#define BASE_API		HIP_IMPORT
#define BASE_CLASS		HIP_IMPORT
#endif


class PLRigorModel;

 class BASE_CLASS OrbiterRigorModel : public HIPgeoModel
{

public: 
	virtual bool transxyh2XYZ(double x,double y,double h,double &X,double &Y,double &Z);
	virtual bool transXYZ2xy(double X,double Y,double Z,double &x,double &y);

	//从影像坐标+高程解算到大地坐标
	virtual bool transxyh2latlon(double x,double y,double h,double &lat,double &lon); 
	//从大地坐标+高程解算到影像坐标
	virtual bool translatlonh2xy(double lat,double lon,double h,double &x,double &y);

	virtual bool getModelRange(double &x,double &y);
public:
	//获取光线在星固坐标系下的指向，用于严密模型空间前方交会
	bool getligtVectorIAU(double line, double sample,double *ligtVector,double *spoition);

	bool outputTimeTable(char *timetbFile);

	bool getImageRange(double &wid,double &hei);
public:
	bool outputEO(char *outPutFile);

public:
	bool setModelPara(OrbiterSensorType sensorType,char *cubImg,char *fursh);

private:
	PLRigorModel *m_plRigorModel;
	bool is_Set;

public:
	OrbiterRigorModel();
	~OrbiterRigorModel();

private:
	void Compute_avAnddx(double *a,int num,double &av,double &dx);

private:

	double LAT_OFF,LONG_OFF,LINE_OFF,SAMP_OFF,LAT_SCALE,LONG_SCALE,LINE_SCALE,SAMP_SCALE;
	double m_pixelsizeLBH,m_pixelsize2LBH;
	double m_psLBH[3],m_plLBH[3];

private:
	
	bool PredictbasedAffineLBH(double lat,double lon,double H,double &x,double &y);
	double distanceOnePixelLBH(double x,double y,double H,double dx,double dy);
	bool PreciseBasedAffineLBH(double &x,double &y,double lat,double lon,double H,double dx,double dy);
	void ComputerGlobalAffineLBH();
/*
private:

	double m_pixelsizeXYZ,m_pixelsize2XYZ;
	double m_psXYZ[3],m_plXYZ[3];
	double X_OFF,Y_OFF,X_SCALE,Y_SCALE,l_OFF,l_SCALE,s_OFF,s_SCALE;//LINE_SCALE,SAMP_SCALE,LINE_OFF,SAMP_OFF

private:
	
	bool PredictbasedAffineXYZ(double lat,double lon,double H,double &x,double &y);
	double distanceOnePixelXYZ(double x,double y,double H,double dx,double dy);
	bool PreciseBasedAffineXYZ(double &x,double &y,double lat,double lon,double H,double dx,double dy);
	void ComputerGlobalAffineXYZ();
*/
};

 /*

 BASE_API bool outputforssj(char *infilename,char *outputfilename);
 BASE_API bool outEO(PLRigorModel *m1,char *eoFile);
 
 */
 BASE_API bool rigourModelSpaceIntersection(OrbiterRigorModel *m1,double x1,double y1,OrbiterRigorModel *m2,double x2,double y2,double &lat,double &lon,double &H);

 
 BASE_API bool ResampleImage(OrbiterRigorModel *m1,char *img1,OrbiterRigorModel *resampleM2,char *resapleImge,char *dem,double heicon);

#endif