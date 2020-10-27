#include "preProcessing.h"

bool preProcessing::histMatch_Value(Mat matSrc, Mat matDst, Mat& matRet)
{
    if (matSrc.empty() || matDst.empty() || 1 != matSrc.channels() || 1 != matDst.channels())
        return false;
    int nHeight = matDst.rows;
    int nWidth = matDst.cols;
    int nDstPixNum = nHeight * nWidth;
    int nSrcPixNum = 0;

    int arraySrcNum[256] = { 0 };                // 源图像各灰度统计个数
    int arrayDstNum[256] = { 0 };                // 目标图像个灰度统计个数
    double arraySrcProbability[256] = { 0.0 };   // 源图像各个灰度概率
    double arrayDstProbability[256] = { 0.0 };   // 目标图像各个灰度概率
    // 统计源图像
    for (int j = 0; j < nHeight; j++)
    {
        for (int i = 0; i < nWidth; i++)
        {
            arrayDstNum[matDst.at<uchar>(j, i)]++;
        }
    }
    // 统计目标图像
    nHeight = matSrc.rows;
    nWidth = matSrc.cols;
    nSrcPixNum = nHeight * nWidth;
    for (int j = 0; j < nHeight; j++)
    {
        for (int i = 0; i < nWidth; i++)
        {
            arraySrcNum[matSrc.at<uchar>(j, i)]++;
        }
    }
    // 计算概率
    for (int i = 0; i < 256; i++)
    {
        arraySrcProbability[i] = (double)(1.0 * arraySrcNum[i] / nSrcPixNum);
        arrayDstProbability[i] = (double)(1.0 * arrayDstNum[i] / nDstPixNum);
    }
    // 构建直方图均衡映射
    int L = 256;
    int arraySrcMap[256] = { 0 };
    int arrayDstMap[256] = { 0 };
    for (int i = 0; i < L; i++)
    {
        double dSrcTemp = 0.0;
        double dDstTemp = 0.0;
        for (int j = 0; j <= i; j++)
        {
            dSrcTemp += arraySrcProbability[j];
            dDstTemp += arrayDstProbability[j];
        }
        arraySrcMap[i] = (int)((L - 1) * dSrcTemp + 0.5);// 减去1，然后四舍五入
        arrayDstMap[i] = (int)((L - 1) * dDstTemp + 0.5);// 减去1，然后四舍五入
    }
    // 构建直方图匹配灰度映射
    int grayMatchMap[256] = { 0 };
    for (int i = 0; i < L; i++) // i表示源图像灰度值
    {
        int nValue = 0;    // 记录映射后的灰度值
        int nValue_1 = 0;  // 记录如果没有找到相应的灰度值时，最接近的灰度值
        int k = 0;
        int nTemp = arraySrcMap[i];
        for (int j = 0; j < L; j++) // j表示目标图像灰度值
        {
            // 因为在离散情况下，之风图均衡化函数已经不是严格单调的了，
            // 所以反函数可能出现一对多的情况，所以这里做个平均。
            if (nTemp == arrayDstMap[j])
            {
                nValue += j;
                k++;
            }
            if (nTemp < arrayDstMap[j])
            {
                nValue_1 = j;
                break;
            }
        }
        if (k == 0)// 离散情况下，反函数可能有些值找不到相对应的，这里去最接近的一个值
        {
            nValue = nValue_1;
            k = 1;
        }
        grayMatchMap[i] = nValue / k;
    }
    // 构建新图像
    matRet = Mat::zeros(nHeight, nWidth, CV_8UC1);
    for (int j = 0; j < nHeight; j++)
    {
        for (int i = 0; i < nWidth; i++)
        {
            matRet.at<uchar>(j, i) = grayMatchMap[matSrc.at<uchar>(j, i)];
        }
    }
    return true;
}


int preProcessing::histogram_Matching(Mat matSrc, Mat matDst, Mat& matResult)
{


    Mat srcBGR[3];
    Mat dstBGR[3];
    Mat retBGR[3];
    split(matSrc, srcBGR);
    split(matDst, dstBGR);

    histMatch_Value(srcBGR[0], dstBGR[0], retBGR[0]);
    histMatch_Value(srcBGR[1], dstBGR[1], retBGR[1]);
    histMatch_Value(srcBGR[2], dstBGR[2], retBGR[2]);

    merge(retBGR, 3, matResult);
    //imshow("src", matSrc);
    //imshow("dst", matDst);
    //imshow("Ret", matResult);
    //imwrite(strPath + "hist_match_value.jpg", matResult);
    //cvWaitKey();
    return 0;
}

void preProcessing::deleteNodataValue(const Mat &src, Mat& dst) {
    int rows = src.rows;
    int cols = src.cols;

    for (int i = 0; i < rows; ++i) {
        const float* ptr = src.ptr<float>(i);
        float* ptr_dst = dst.ptr<float>(i);
        for (int j = 0; j < cols; ++j) {
            if (ptr[j] >= -10000 && ptr[j] <= 10000 && ptr[j] != 0)  ptr_dst[j] = ptr[j]; 
            else  ptr_dst[j] = Invalid_Float;
        }
    }
};
