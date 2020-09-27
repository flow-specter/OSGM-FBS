/**
* @copyright Copyright(C), 2020, PMRS Lab, IRSA, CAS
* @file mlTypes.h
* @date 2020.09.24
* @author Ҷ�ּ� ljye_bj@163.com
* @brief osgm�ж�����Ҫ�Ľṹ������
* @version 1.0
* @par �޸���ʷ:
* <����>  <ʱ��>  <�汾���>  <����>\n
*/

#ifndef _OSGM_TYPES_H
#define _OSGM_TYPES_H

#define Z_resolution 1
#include <iostream>



/**
* @struct path
* @date 2020.09
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ·����ʶ
* @version 1.0
*/
struct path {
	short rowDiff;
	short colDiff;
	short index;
};

/**
* @struct P3D
* @date 2020.09
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��������������Χ����ά��
* @version 1.0
*/
struct P3D{
	double lat;
	double lon;
	float z;
	float upHei;
	float downHei;
};

/**
* @struct ImgPoint
* @date 2020.09
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ά��
* @version 1.0
*/
struct ImgPoint{
	double line;
	double sample;
};

#endif