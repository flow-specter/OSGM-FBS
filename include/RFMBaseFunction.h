#ifndef _RFM_BASEFUNCTION_INTERFACE_ORBITER_PL_RIGOR_MODEL_H_LLL
#define _RFM_BASEFUNCTION_INTERFACE_ORBITER_PL_RIGOR_MODEL_H_LLL

#include "HIPtypeDef.h"
#include "HIPgeoModelInterface.h"
#include "HIPRFMstructDef.h"

#ifdef RFM_BASEFUNCTION_EXPORTS
#define BASE_API			HIP_EXPORT
#define BASE_CLASS          HIP_EXPORT
#else
#define BASE_API		HIP_IMPORT
#define BASE_CLASS		HIP_IMPORT
#endif

//class TimeSearchTable;
BASE_API bool API_GetRPCMODEL(char *RpcFile, RPCMODEL &rpcmodel);

BASE_API bool API_LineSample2LATLONGHEIFHT(double line, double sample, double height, RPCMODEL rpcmodel, double &latitude, double &longitude);
BASE_API bool API_LATLONGHEIFHT2LineSample(RPCMODEL rpcmodel, double latitude, double longitude, double height, double &x, double &y);

#endif