#ifndef _RFM_MAKER_INTERFACE_ORBITER_PL_RIGOR_MODEL_H_LLL
#define _RFM_MAKER_INTERFACE_ORBITER_PL_RIGOR_MODEL_H_LLL

#include "HIPtypeDef.h"
#include "HIPgeoModelInterface.h"
#include "HIPRFMstructDef.h"

#ifdef RFM_MAKER_EXPORTS
#define BASE_API			HIP_EXPORT
#define BASE_CLASS          HIP_EXPORT
#else
#define BASE_API		HIP_IMPORT
#define BASE_CLASS		HIP_IMPORT
#endif

//class TimeSearchTable;

BASE_API bool RFMmaker(HIPgeoModel *geomodel,double maxHei,double minHei,int dx,int dy,int nz,char *outputfilename);
BASE_API bool T_RFMmaker(char *timeTableFile,HIPgeoModel *geomodel,double maxHei,double minHei,int dx,int dy,int nz,char *outputfilename);
BASE_API bool XYZ_RFMmaker(HIPgeoModel *geomodel,double maxHei,double minHei,int dx,int dy,int nz,char *outputfilename);
BASE_API bool XYZ_T_RFMmaker(char *timeTableFile,HIPgeoModel *geomodel,double maxHei,double minHei,int dx,int dy,int nz,char *outputfilename);
/*
{
	CRFMmaker RFMmaker;
	if (geomodel == NULL)
		return false;

	
	RFMmaker.InitTimeRPCInput(geomodel,dx,dy,dz,outputfilename);
	RFMmaker.GetRPCfile(outputfilename);

	return true;
}
*/
#endif