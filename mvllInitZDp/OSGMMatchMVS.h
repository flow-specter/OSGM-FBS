#pragma once
#include "osgm_types.h"

/*
brief: OSGM多视匹配类
*/
class OSGMMatchMVS
{
public:
	OSGMMatchMVS();
	~OSGMMatchMVS();

public:
	/**
	* \brief 类的初始化，完成一些内存的预分配、参数的预设置等
	* \param width		输入，基准影像影像宽
	* \param height		输入，基准影像影像高
	* \param option		输入，算法参数
	* \param 
	*/
	bool Initialize(const sint32& width, const sint32& height, const OSGMOption& option);

private:
	/** \brief OSGM参数	 */
	OSGMOption option_;


};

