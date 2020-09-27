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

#endif