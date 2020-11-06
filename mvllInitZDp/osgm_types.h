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


#include <iostream>
#include <cstdint>


/** \brief �������ͱ��� */
typedef int8_t			sint8;		// �з���8λ����
typedef uint8_t			uint8;		// �޷���8λ����
typedef int16_t			sint16;		// �з���16λ����
typedef uint16_t		uint16;		// �޷���16λ����
typedef int32_t			sint32;		// �з���32λ����
typedef uint32_t		uint32;		// �޷���32λ����
typedef int64_t			sint64;		// �з���64λ����
typedef uint64_t		uint64;		// �޷���64λ����
typedef float			float32;	// �����ȸ���
typedef double			float64;	// ˫���ȸ���


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

/**
* @struct P3D ��ά��
* @date 2020.10
* @author Ҷ�ּ� ljye_bj@163.com
* @brief ��ά��
* @version 1.0
*/
struct P3D {
	double lat;
	double lon;
	float hei;
};




#endif