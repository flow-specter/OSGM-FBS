#ifndef _HIP_GEO_MODEL_H_GOBLE_
#define _HIP_GEO_MODEL_H_GOBLE_

class HIPgeoModel
{
public: 
	virtual bool transxyh2XYZ(double x,double y,double h,double &X,double &Y,double &Z) = 0;
	virtual bool transXYZ2xy(double X,double Y,double Z,double &x,double &y) = 0;

	//从影像坐标+高程解算到大地坐标
	virtual bool transxyh2latlon(double x,double y,double h,double &lat,double &lon)  = 0; 
	//从大地坐标+高程解算到影像坐标
	virtual bool translatlonh2xy(double lat,double lon,double h,double &x,double &y) = 0;

	//模型范围
	virtual bool getModelRange(double &x,double &y) = 0;
};

#endif