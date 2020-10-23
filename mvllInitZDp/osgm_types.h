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

/** \brief PMS�����ṹ�� */
struct OSGMOption {
	float32 sufferZ; // �����ܵĸ̱߳仯���綨p1��p2




	sint32	patch_size;			// patch�ߴ磬�ֲ�����Ϊ patch_size*patch_size
	sint32  min_disparity;		// ��С�Ӳ�
	sint32	max_disparity;		// ����Ӳ�

	float32	gamma;				// gamma Ȩֵ����
	float32	alpha;				// alpha ���ƶ�ƽ������
	float32	tau_col;			// tau for color	���ƶȼ�����ɫ�ռ�ľ��Բ���½ض���ֵ
	float32	tau_grad;			// tau for gradient ���ƶȼ����ݶȿռ�ľ��Բ��½ض���ֵ

	sint32	num_iters;			// ������������

	bool	is_check_lr;		// �Ƿ�������һ����
	float32	lrcheck_thres;		// ����һ����Լ����ֵ

	bool	is_fill_holes;		// �Ƿ�����Ӳ�ն�

	bool	is_fource_fpw;		// �Ƿ�ǿ��ΪFrontal-Parallel Window
	bool	is_integer_disp;	// �Ƿ�Ϊ�������Ӳ�

	OSGMOption() : patch_size(35), min_disparity(0), max_disparity(64), gamma(10.0f), alpha(0.9f), tau_col(10.0f),
		tau_grad(2.0f), num_iters(3),
		is_check_lr(false),
		lrcheck_thres(0),
		is_fill_holes(false), is_fource_fpw(false), is_integer_disp(false) { }
};


#endif