//#include "cv.h"  
//#include <cxcore.h>  
//#include <highgui.h>  
//  
//  
//int main()  
//{  
//        //��ͼ���ļ��������ڴ� ����ͼ�����ݽṹ����Ҫ���ڿ� ����һ��ָ�����ݽṹIplImage���ڴ�飺  
//        IplImage *img = cvLoadImage("lena(5).bmp");  
//        //��Ҫ�����ͼƬ�������project���棬�����ҵģ�&user name\Documents\Visual Studio 2010\Projects\opencvhello\opencvhello�ļ�������  
//        //�����������ڣ����ɴ�С����������HighGUI���ṩ���ڶ����������Ϊ0���򴰿ڴ�С������ͼ��Ĵ�С���ı䡣  
//        cvNamedWindow("Image-in",CV_WINDOW_AUTOSIZE);  
//        cvNamedWindow("GaussSmoothing",CV_WINDOW_AUTOSIZE);  
//        //����ʾԭjpgͼ  
//        cvShowImage("Image-in",img);  
//       //����ռ�洢������ͼ��  
//        IplImage *out=cvCreateImage(  
//            cvGetSize(img),//��ǰͼ���С  
//            IPL_DEPTH_8U,//��ͨ��ÿ�����ص������  
//            3//ͨ������  
//            );  
////���и�˹�����������ָ��imgָ����ڴ棬�����������ݽ���outָ��ָ����ڴ棬��ÿ��������Χ3x3��������и�˹ƽ��������ʵ�������ͼ���������ͬ�ģ�  
//        cvSmooth(img,out,CV_GAUSSIAN,3,3);  
////��ʾ������ͼ��  
//        cvShowImage("GaussSmoothing",out);  
//
//		IplImage* sobel=cvCreateImage(cvGetSize(out),out->depth,1);
//		cvNamedWindow("sobel", CV_WINDOW_AUTOSIZE); 
//		/// ת��Ϊ�Ҷ�ͼ
//		cvCvtColor( out, sobel, CV_RGB2GRAY );
//		cvSobel(sobel,sobel,1,0,3);
//		cvShowImage("sobel",sobel);  
//
//      //�������  
//        cvReleaseImage(&out);  
//        cvReleaseImage(&img);  
//		  cvReleaseImage(&sobel);
//  
//        //cvWaitKey�Ĳ����������ֵ��������ȴ���ֵ�����룬Ȼ��������У�����Ǹ�ֵ����0���ͻ�ȴ��û���������������Ȼ���������  
//        cvWaitKey();  
//  
//       //���ٴ��ڣ����ɺ�ϰ��  
//        cvDestroyWindow("Image-in");  
//        cvDestroyWindow("GaussSmoothing");  
//		  cvDestroyWindow("sobel"); 
//  
//        return 0;  
//  
//} 


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

/** @���� main */
int main( int argc, char argv[] )
{
  Mat src, src_gray, dst;
  int kernel_size = 3;
  int scale = 3;
  int delta = 0;
  int ddepth = CV_16S;
  char* window_name = "Laplace Demo";

  int c;

  /// װ��ͼ��
  src = imread( "test(5).bmp" );

  /// ʹ�ø�˹�˲���������
  GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

  /// ת��Ϊ�Ҷ�ͼ
  cvtColor( src, src_gray, CV_RGB2GRAY );

  /// ������ʾ����
  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// ʹ��Laplace����
  Mat abs_dst;

  Laplacian( src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( dst, abs_dst );

  /// ��ʾ���
  imshow( window_name, abs_dst );

  waitKey(0);

  return 0;
  }