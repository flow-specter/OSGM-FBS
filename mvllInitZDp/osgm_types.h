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

/**
* @struct P3D 三维点
* @date 2020.10
* @author 叶乐佳 ljye_bj@163.com
* @brief 二维点
* @version 1.0
*/
struct P3D {
	double lat;
	double lon;
	float hei;
};




#endif