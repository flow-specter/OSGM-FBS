#pragma once
#include "osgm_types.h"

/*
brief: OSGM����ƥ����
*/
class OSGMMatchMVS
{
public:
	OSGMMatchMVS();
	~OSGMMatchMVS();

public:
	/**
	* \brief ��ĳ�ʼ�������һЩ�ڴ��Ԥ���䡢������Ԥ���õ�
	* \param width		���룬��׼Ӱ��Ӱ���
	* \param height		���룬��׼Ӱ��Ӱ���
	* \param option		���룬�㷨����
	* \param 
	*/
	bool Initialize(const sint32& width, const sint32& height, const OSGMOption& option);

private:
	/** \brief OSGM����	 */
	OSGMOption option_;


};

