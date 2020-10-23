#pragma once

/*
 * @Author: Han
 * @Date: 2017-12-16 15:19:00
 * һЩ�򵥵�Ӱ������
 */
#pragma once
#include "common.h"

namespace h2o {
    // ��Ӱ��ת��Ϊ 8bit Ӱ�񣬻��Զ����ŵ� 1 - 255, ���� 0 ����Чֵ�� ��ض�ǰ�� clip ����
    cv::Mat convert_to_8bit(const cv::Mat& mat, const cv::Mat& mask = cv::Mat(), double clip = 0.0);

    // ���㵥����Ӱ��������Сֵ
    Vector2d image_minmax(const cv::Mat& mat, const cv::Mat& mask = cv::Mat());

    // ����ನ�ε������Сֵ������Ĳ��Σ�
    std::tuple<Vector4f, Vector4f> image_minmax_channel(const cv::Mat& mat, const cv::Mat& mask = cv::Mat());

    // ����ֱ��ͼ���ض���λ�����������Сֵ
    std::tuple<Vector4f, Vector4f> image_minmax_channel_clip(const cv::Mat& mat, double clip,
        const cv::Mat& mask = cv::Mat());
    std::tuple<float, float> image_minmax_clip(const cv::Mat& mat, double clip, const cv::Mat& mask = cv::Mat());
    // ��ֵ���������ݣ����� split ����Ϊ���ô������ڴ棩
    void image_copy_channel(const cv::Mat& src, cv::Mat& dst, int channel, const cv::Mat& mask = cv::Mat());

    // ��ȡһ��������
    cv::Mat image_extract_channel(const cv::Mat& src, int channel);

    // ��ƽ��������
    cv::Mat smooth_resize(const cv::Mat& mat, const Vector2i& dsize);

    template <typename T>
    T image_interpolate(const cv::Mat& mat, const Vector2f& pt, int border_type = cv::BORDER_REPLICATE);
    /**
     * \brief ��Ӱ�������С���ֵ���ŵ� 1 - 255������0��������nodata
     * \param image ԭʼӰ����Ҫ�ǵ�����
     * \param mask ��Чֵ
     * \param percent ���ű���
     */
    cv::Mat image_percent_scale_8u(const cv::Mat& image, cv::Mat mask = cv::Mat(), double percent = 2.0);

    /**
     * \brief opencv �� remap �����ڴ���Ӱ���С���� short ����ʱ����� bug
     * \param src
     * \param mapx ������ CV_32F ��ʽ
     * \param mapy ������ CV_32F ��ʽ
     */
    cv::Mat remap_ipp(const cv::Mat& src, const cv::Mat& mapx, const cv::Mat& mapy, int interpolation = cv::INTER_LINEAR);

    // dest = A * source
    Matrix23d compute_affine(const std::vector<Vector2d>& source, const std::vector<Vector2d>& dest);

    Matrix23d revers_affine(const Matrix23d& M);

    // dest = H * source
    Matrix3d compute_homography(const std::vector<Vector2d>& source, const std::vector<Vector2d>& dest);

} // namespace h2o

///////////////////////////////////////////////////////////////////////////////////////////
///
/// implementations
namespace h2o {
    template <typename T> T image_interpolate(const cv::Mat& mat, const Vector2f& pt, int border_type) {
        int x = (int)pt.x();
        int y = (int)pt.y();

        int x0 = cv::borderInterpolate(x, mat.cols, border_type);
        int x1 = cv::borderInterpolate(x + 1, mat.cols, border_type);
        int y0 = cv::borderInterpolate(y, mat.rows, border_type);
        int y1 = cv::borderInterpolate(y + 1, mat.rows, border_type);
        float a = pt.x() - (float)x;
        float c = pt.y() - (float)y;

        const T& p00 = mat.ptr<T>(y0)[x0];
        const T& p01 = mat.ptr<T>(y1)[x0];
        const T& p10 = mat.ptr<T>(y0)[x1];
        const T& p11 = mat.ptr<T>(y1)[x1];

        T p = (p00 * (1.0f - a) + p01 * a) * (1.0f - c) + (p10 * (1.0f - a) + p11 * a) * c;
        return p;
    }
} // namespace h2o
