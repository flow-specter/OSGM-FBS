#include "preProcessing.h"

bool preProcessing::histMatch_Value(Mat matSrc, Mat matDst, Mat& matRet)
{
    if (matSrc.empty() || matDst.empty() || 1 != matSrc.channels() || 1 != matDst.channels())
        return false;
    int nHeight = matDst.rows;
    int nWidth = matDst.cols;
    int nDstPixNum = nHeight * nWidth;
    int nSrcPixNum = 0;

    int arraySrcNum[256] = { 0 };                // Դͼ����Ҷ�ͳ�Ƹ���
    int arrayDstNum[256] = { 0 };                // Ŀ��ͼ����Ҷ�ͳ�Ƹ���
    double arraySrcProbability[256] = { 0.0 };   // Դͼ������Ҷȸ���
    double arrayDstProbability[256] = { 0.0 };   // Ŀ��ͼ������Ҷȸ���
    // ͳ��Դͼ��
    for (int j = 0; j < nHeight; j++)
    {
        for (int i = 0; i < nWidth; i++)
        {
            arrayDstNum[matDst.at<uchar>(j, i)]++;
        }
    }
    // ͳ��Ŀ��ͼ��
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
    // �������
    for (int i = 0; i < 256; i++)
    {
        arraySrcProbability[i] = (double)(1.0 * arraySrcNum[i] / nSrcPixNum);
        arrayDstProbability[i] = (double)(1.0 * arrayDstNum[i] / nDstPixNum);
    }
    // ����ֱ��ͼ����ӳ��
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
        arraySrcMap[i] = (int)((L - 1) * dSrcTemp + 0.5);// ��ȥ1��Ȼ����������
        arrayDstMap[i] = (int)((L - 1) * dDstTemp + 0.5);// ��ȥ1��Ȼ����������
    }
    // ����ֱ��ͼƥ��Ҷ�ӳ��
    int grayMatchMap[256] = { 0 };
    for (int i = 0; i < L; i++) // i��ʾԴͼ��Ҷ�ֵ
    {
        int nValue = 0;    // ��¼ӳ���ĻҶ�ֵ
        int nValue_1 = 0;  // ��¼���û���ҵ���Ӧ�ĻҶ�ֵʱ����ӽ��ĻҶ�ֵ
        int k = 0;
        int nTemp = arraySrcMap[i];
        for (int j = 0; j < L; j++) // j��ʾĿ��ͼ��Ҷ�ֵ
        {
            // ��Ϊ����ɢ����£�֮��ͼ���⻯�����Ѿ������ϸ񵥵����ˣ�
            // ���Է��������ܳ���һ�Զ�������������������ƽ����
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
        if (k == 0)// ��ɢ����£�������������Щֵ�Ҳ������Ӧ�ģ�����ȥ��ӽ���һ��ֵ
        {
            nValue = nValue_1;
            k = 1;
        }
        grayMatchMap[i] = nValue / k;
    }
    // ������ͼ��
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
