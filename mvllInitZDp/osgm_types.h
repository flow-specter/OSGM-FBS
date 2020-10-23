/**
* @copyright Copyright(C), 2020, PMRS Lab, IRSA, CAS
* @file mlTypes.h
* @date 2020.09.24
* @author 叶乐佳 ljye_bj@163.com
* @brief osgm中定义需要的结构体类型
* @version 1.0
* @par 修改历史:
* <作者>  <时间>  <版本编号>  <描述>\n
*/

#ifndef _OSGM_TYPES_H
#define _OSGM_TYPES_H

#define Z_resolution 1
#include <iostream>
#include <cstdint>


/** \brief 基础类型别名 */
typedef int8_t			sint8;		// 有符号8位整数
typedef uint8_t			uint8;		// 无符号8位整数
typedef int16_t			sint16;		// 有符号16位整数
typedef uint16_t		uint16;		// 无符号16位整数
typedef int32_t			sint32;		// 有符号32位整数
typedef uint32_t		uint32;		// 无符号32位整数
typedef int64_t			sint64;		// 有符号64位整数
typedef uint64_t		uint64;		// 无符号64位整数
typedef float			float32;	// 单精度浮点
typedef double			float64;	// 双精度浮点


/**
* @struct path
* @date 2020.09
* @author 叶乐佳 ljye_bj@163.com
* @brief 路径标识
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
* @author 叶乐佳 ljye_bj@163.com
* @brief 带有上下搜索范围的三维点
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
* @author 叶乐佳 ljye_bj@163.com
* @brief 二维点
* @version 1.0
*/
struct ImgPoint{
	double line;
	double sample;
};

/** \brief PMS参数结构体 */
struct OSGMOption {
	float32 sufferZ; // 可忍受的高程变化，界定p1与p2




	sint32	patch_size;			// patch尺寸，局部窗口为 patch_size*patch_size
	sint32  min_disparity;		// 最小视差
	sint32	max_disparity;		// 最大视差

	float32	gamma;				// gamma 权值因子
	float32	alpha;				// alpha 相似度平衡因子
	float32	tau_col;			// tau for color	相似度计算颜色空间的绝对差的下截断阈值
	float32	tau_grad;			// tau for gradient 相似度计算梯度空间的绝对差下截断阈值

	sint32	num_iters;			// 传播迭代次数

	bool	is_check_lr;		// 是否检查左右一致性
	float32	lrcheck_thres;		// 左右一致性约束阈值

	bool	is_fill_holes;		// 是否填充视差空洞

	bool	is_fource_fpw;		// 是否强制为Frontal-Parallel Window
	bool	is_integer_disp;	// 是否为整像素视差

	OSGMOption() : patch_size(35), min_disparity(0), max_disparity(64), gamma(10.0f), alpha(0.9f), tau_col(10.0f),
		tau_grad(2.0f), num_iters(3),
		is_check_lr(false),
		lrcheck_thres(0),
		is_fill_holes(false), is_fource_fpw(false), is_integer_disp(false) { }
};


#endif