//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//
//using namespace cv;
//
//int main()
//{
//
//	//载入图像
//	Mat image = imread("1.jpg");
//
//	//创建窗口
//	namedWindow("双边滤波原图");
//	namedWindow("双边滤波效果图");
//
//	imshow("双边滤波原图", image);
//
//	//进行滤波
//	Mat out;
//	bilateralFilter(image, out, 50, 50 * 2, 50 / 2);
//	imshow("双边滤波效果图", out);
//
//	waitKey(0);
//
//	return 0;
//}
