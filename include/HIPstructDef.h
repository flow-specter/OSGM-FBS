#ifndef _STRUCT_TYPEDEF_H
#define _STRUCT_TYPEDEF_H

#include "HIPtypeDef.h"


#include <fstream>
#include <string>
#include <vector>


using namespace std;

//轨道积分
typedef struct{

	double majorAxis;	/* semi-major axis (m)		*/
	double minorAxis;	/* semi-minor axis (m)		*/
	double e;		/* numeric eccentricity 	（轨道第一偏心率，sqrt（a*a-b*b）/a）*/
	double e2;		/* numeric eccentricity squared */

	double xOff;		/* offset relative to WGS_84 (m)*/
	double yOff;		/* offset relative to WGS_84 (m)*/
	double zOff;		/* offset relative to WGS_84 (m)*/

} DATUM;

//DATUM WGS84[] = 
//{6378137.0, 6356752.314, 0.08181919, 0.006694379, 0.0, 0.0, 0.0};
//   DATUM IAUMOON[] = 
//	{1737400.0, 1737400.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   DATUM IAUMARS[] = 
//	{1737400.0, 1737400.0, 0.0, 0.0, 0.0, 0.0, 0.0};

typedef struct 
{
	double Xs[21];
	double Ys[21];
	double Zs[21];
	double Xvs[21];
	double Yvs[21];
	double Zvs[21];
	double roll[21];
	double pitch[21];
	double yaw[21];
	double x[21];
} OrbitAttitudeRegressionModel;

typedef struct 
{
	double latitude;
	double longitude;
	double altitude;
	double X;
	double Y;
	double Z;
	double Xv;
	double Yv;
	double Zv;
	double UT;
	int year,month,day,hour,minute;
	double second;
} POINT_Ep;

//typedef struct 
//{
//	double Satellite_Altitude;
//	double NaDir_Lat;
//	double NaDir_Lon;
//	CArray <POINT_Ep,POINT_Ep > Ep_Array;
//} Ephemeris_Para;

typedef struct 
{
	double UT;
	double YAW;
	double PITCH;
	double ROLL;
	double vYAW;
	double vPITCH;
	double vROLL;
	double q1,q2,q3,q0;
	double vq1,vq2,vq3;
	double frameID;
	double t;
	int year,month,day,hour,minute;
	double second;
} Attitude;

typedef struct {
	double ExposureTime;
	double EphemerisTime;
	int LineStart;
}LineScanTime;


typedef struct 
{
	double x,y,z;

} POINT3D;

typedef struct DOUBLEPOINT
{
	double x;
	double y;

	bool operator==(DOUBLEPOINT& dPoint) const
	{
		if(x != dPoint.x || y != dPoint.y)
			return false;
		return true;

	}
	bool operator!=(DOUBLEPOINT dPoint) const
	{
		return !operator==(dPoint);		
	}

}POINT2D;
#define EMPOINT POINT2D 

#endif