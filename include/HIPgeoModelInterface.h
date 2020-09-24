#ifndef _HIP_GEO_MODEL_H_GOBLE_
#define _HIP_GEO_MODEL_H_GOBLE_

class HIPgeoModel
{
public: 
	virtual bool transxyh2XYZ(double x,double y,double h,double &X,double &Y,double &Z) = 0;
	virtual bool transXYZ2xy(double X,double Y,double Z,double &x,double &y) = 0;

	//��Ӱ������+�߳̽��㵽�������
	virtual bool transxyh2latlon(double x,double y,double h,double &lat,double &lon)  = 0; 
	//�Ӵ������+�߳̽��㵽Ӱ������
	virtual bool translatlonh2xy(double lat,double lon,double h,double &x,double &y) = 0;

	//ģ�ͷ�Χ
	virtual bool getModelRange(double &x,double &y) = 0;
};

#endif