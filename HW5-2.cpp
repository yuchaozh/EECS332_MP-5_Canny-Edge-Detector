//// createGuassFilter.cpp : 定义控制台应用程序的入口点。 
////
//
////#include "stdafx.h"
//#include <cv.h>
//#include <highgui.h>
//#include <cxcore.h>
//
////#ifdef DEBUG
////#pragma comment(lib,"cxcore200d.lib")
////#pragma comment(lib,"cv200d.lib")
////#pragma comment(lib,"highgui200d.lib")
////#else
////#pragma comment(lib,"cxcore200.lib")
////#pragma comment(lib,"cv200.lib")
////#pragma comment(lib,"highgui200.lib")
////#endif
//
//void creatGuassFilter(float *GuassFilter, int nsize,float sigma=0);
//void calGradientAndAmplitue(IplImage * src , IplImage * gradientX,IplImage * gradientY ,IplImage * amplitue );
//void noMaxSuppress(IplImage * gradientX, IplImage *gradientY, IplImage *amplitue,IplImage * noMaxSupress );
//void EstimateThreshold(IplImage * noMaxSuppressImg , IplImage * amplitue , double &lowthreahold ,double &highthreahold);
//void hysteresis(IplImage * amplitue , IplImage * noMaxSuppressImg , float highthreshold , float lowthreshold);
//void trackEdge(int x , int y , int lowthreshold, IplImage * amplitue , IplImage * noMaxSuppressImg);
//void canny(IplImage * src, IplImage * dst ,float sigma , int size , float highthreshold, float lowthreshold);
//
////int _tmain(int argc, _TCHAR* argv[])
//int main()
//{
//
//    //IplImage * im = cvLoadImage("gun1(5).bmp",CV_LOAD_IMAGE_GRAYSCALE);
//	IplImage * im = cvLoadImage("lena(5).bmp",CV_LOAD_IMAGE_GRAYSCALE);
//	//IplImage * im = cvLoadImage("joy1(5).bmp",CV_LOAD_IMAGE_GRAYSCALE);
//	//IplImage * im = cvLoadImage("test(5).bmp",CV_LOAD_IMAGE_GRAYSCALE);
//    IplImage * im2 = cvCreateImage(cvSize(im->width,im->height),8,1);
//    cvResize(im,im2);
//    IplImage * dst = cvCreateImage(cvGetSize(im2),8,1);
//
//    canny(im2,dst,3,3,0,0);
//
//    cvWaitKey(0);
//
//    return 0;
//}
//
//
//
///*
//    函数名 : creatGuassFilter
//    功能：生成高斯滤波模板
//    参数：mat_GuassFilter --- out 用于存储生成的高斯滤波模板
//              nsize ----- in 滤波窗口尺寸
//              sigma ----- in 高斯半径  
//*/
//void createGuassFilter(float *GuassFilter, int nsize,float sigma)
//{
//        float distX,distY; //到窗口中心的距离
//        int ncenter = nsize/2; //窗口中心
//        float sigmaA = 1.0 / (sqrt(2 * CV_PI) * sigma) ;
//        float sum =0.0; 
//
//        //如果sigma = 0 ，自动计算sigma值
//        if (0 == sigma)
//        {
//            sigma = (nsize /2 -1) *0.3 + 0.8;
//        }
//
//        for (int i = 0 ; i < nsize ; ++i)
//        {
//            for (int j = 0 ; j < nsize ; ++j)
//            {
//                distX = i - ncenter;
//                distY = j - ncenter;
//                *(GuassFilter + i * nsize + j ) = sigmaA * exp( -1.0 * ( distX * distX + distY * distY) / (2 * sigma * sigma)) ; 
//                sum +=*(GuassFilter + i * nsize + j ) ;
//            }
//        }
//
//        //归一化
//        for (int i = 0 ; i <nsize ; ++i)
//        {
//            for (int j=0; j <nsize; ++j)
//            {
//                *(GuassFilter + i * nsize + j ) /=sum;
//            }
//        }
//
//}
//
//
///*
//    函数：calGradientAndAmplitue 
//    功能：计算图像梯度方向 和幅度
//    参数：src -- in 原图
//              gradientX Y -- out x，y方向梯度值 像素类型 int
//              amplitue -- out 幅度值 像素类型double
//*/
//void calGradientAndAmplitue(IplImage * src , IplImage * gradientX,IplImage * gradientY ,IplImage * amplitue )
//{
//    ////计算 x ，y 方向梯度
//    //float filterx[3][3] = {
//    //    -1 , 0 , 1 ,
//    //    -2 , 0 , 2,
//    //    -1 , 0 , 1
//    //};
//
//    //float filtery[3][3] = {
//    //    1 , 2 , 1,
//    //    0 , 0 , 0,
//    //    -1 ,-2, -1
//    //};
//
//	    //计算 x ，y 方向梯度
//    float filterx[3][3] = {
//        -1 , 0 , 1 ,
//        -1 , 0 , 1,
//        -1 , 0 , 1
//    };
//
//    float filtery[3][3] = {
//        1 , 1 , 1,
//        0 , 0 , 0,
//        -1 ,-1, -1
//    };
//
//		  //  //计算 x ，y 方向梯度
//    //float filterx[2][2] = {
//    //    1 , 0 , 
//    //    0 , -1 , 
//    //};
//
//    //float filtery[2][2] = {
//    //    0 , -1 ,
//    //    1 , 0 , 
//    //};
//
//    CvMat mat_filterx ; 
//    CvMat mat_filtery;
//    cvInitMatHeader(&mat_filterx , 2, 2,CV_32FC1 ,filterx);
//    cvInitMatHeader(&mat_filtery , 2 ,2,CV_32FC1, filtery);
//    cvFilter2D(src,gradientX,&mat_filterx);
//    cvFilter2D(src,gradientY,&mat_filtery);
//
//    //计算幅度值
//    int nwidth = src->width ; 
//    int nheight = src->height;
//    float Gx, Gy; //x y 方向梯度
//    float * pGx , *pGy ,*pAmp;
//    for (int i = 0 ; i < nheight ; ++ i)
//    {
//        pGx = (float *)(gradientX->imageData + i* gradientX->widthStep );
//        pGy = (float *)(gradientY->imageData + i* gradientX->widthStep );
//        pAmp = (float *)(amplitue->imageData + i * amplitue->widthStep);
//        for (int j = 0 ;j < nwidth ; ++j)
//        {
//            Gx = *(pGx + j);
//            Gy = *(pGy + j);
//            *(pAmp + j) = sqrt(1.0 * Gx * Gx + 1.0 * Gy * Gy);
//        }
//    }
//
//}
//
///*
//    函数：noMaxSuppress
//    功能：非极大值抑制
//*/
//void noMaxSuppress(IplImage * gradientX, IplImage *gradientY, IplImage *amplitue,IplImage * noMaxSupress )
//{
//    int nwidth = noMaxSupress->width ; 
//    int nheight = noMaxSupress->height;
//    //非极大值抑制
//
//    float Gx ,Gy ; // x,y梯度值
//    float temp1,temp2,temp3,temp4; //用于计算插值得到梯度方向邻域幅度值的中间变量
//    float amp , amp1 ,amp2; // 该点幅度值 该点梯度方向上的幅度值
//    float angle = 0.0; //梯度方向
//    float rate = 0.0; // 线插比例
//    float *pAmp , *pnoMaxSuppress,*pgradientX,*pgradientY;
//    pAmp = (float *)amplitue->imageData;
//    pnoMaxSuppress = (float *)noMaxSupress->imageData;
//    pgradientX = (float *)gradientX->imageData;
//    pgradientY = (float *)gradientY->imageData;
//
//    for ( int i = 1 ; i <nheight -1 ; ++i)
//    {
//        for (int j = 1 ; j < nwidth -1 ; ++j)
//        {
//            amp =*( pAmp+ i *nwidth + j);
//            //如果幅度值为0 则为非边缘点
//            if (amp ==0)
//            {
//                *(pnoMaxSuppress + i * nwidth + j) = 0 ;
//                    continue;
//            }
//
//
//            Gx = *(pgradientX + i * nwidth + j);
//            Gy = *(pgradientY + i *nwidth + j);
//
//
//            angle = atan(1.0 * Gy / Gx);
//
//            //第一种情况
//            //                    temp1               
//            //temp3 amp temp2
//            //temp4
//            if ( (angle >=0 && angle < CV_PI/4) /* || (angle >=CV_PI && angle < 5.0/4 *CV_PI)*/ )
//            {
//                temp1 = *(pAmp + (i-1)*nwidth + j+1);
//                temp2 = *(pAmp + i *nwidth + j+1);
//                temp3 = *(pAmp + i *nwidth + j-1);
//                temp4 = *(pAmp + (i +1) * nwidth + j-1);
//                rate = fabs(1.0 *Gy)/fabs(1.0*Gx);
//                amp1 = temp1 * rate + temp2 * (1 -rate);
//                amp2 = temp3 * rate + temp4 * (1 -rate);
//            }
//            //第二种情况
//            // temp1 temp2
//            // amp
//            // temp3 temp4
//            else if ( ( angle >=CV_PI/4 && angle <= CV_PI/2)/* || (angle >=5.0/4 * CV_PI && angle <3.0/2 * CV_PI )*/) 
//            {
//                temp1 = *(pAmp + (i-1) * nwidth + j );
//                temp2 =*(pAmp + (i-1) * nwidth + j +1);
//                temp3 = *(pAmp + (i +1)*nwidth + j -1);
//                temp4 = *(pAmp + (i +1) * nwidth + j);
//                rate = fabs(1.0*Gx)/fabs(1.0*Gy);
//                amp1 = temp2 * rate + temp1 *(1- rate);
//                amp2 = temp3 * rate + temp4 *(1-rate);
//            }
//            //第三种情况
//            // temp1 temp2
//            // amp
//            // temp3 temp4
//            else if( ( angle >= -1.0 * CV_PI/2 && angle < -1.0 *CV_PI /4))
//            {
//                temp1 = *(pAmp + (i-1)*nwidth + j-1);
//                temp2 = *(pAmp + (i-1)*nwidth + j);
//                temp3 =*(pAmp + (i+1)*nwidth + j);
//                temp4 = *(pAmp + (i +1)*nwidth +j+1);
//                rate = fabs(1.0*Gx) / fabs(1.0*Gy);
//                amp1 = temp1 * rate + temp2 *(1 - rate);
//                amp2 = temp4 * rate + temp3 * (1- rate);
//            }
//            //第四种情况
//            // temp1     
//            // temp2 amp temp3
//            // temp4
//            else /*if ( angle >= -1.0 * CV_PI /4 && angle <0)*/
//            {
//                temp1 = *(pAmp + (i-1)*nwidth + j-1);
//                temp2 = *(pAmp + i * nwidth + j-1);
//                temp3 = *(pAmp + i * nwidth + j +1);
//                temp4 =*(pAmp + (i+1)*nwidth + j +1);
//                rate = fabs(1.0*Gy)/fabs(1.0*Gx);
//                amp1 = temp1 * rate + temp2 *( 1- rate);
//                amp2 = temp4 * rate + temp3* (1- rate);
//            }
//
//            //局部最大值判断 如果为最大值 128 否则0
//            if( amp >=amp1 && amp >= amp2)
//                *(pnoMaxSuppress + i * nwidth + j)=128;
//            else
//                 *(pnoMaxSuppress + i * nwidth + j)=0;
//
//        }
//    }
//}
//
///*
//    函数： EstimateThreshold
//    功能：估计上下阈值
//    参数：noMaxSuppress -- in 非极大值抑制处理结果 疑似边缘点图
//              amplitue -- in 幅度图
//              lowthreashold -- out 低阈值
//              highthreshold --out 高阈值
//*/
//void EstimateThreshold(IplImage * noMaxSuppressImg , IplImage * amplitue , float &lowthreahold ,float &highthreahold)
//{
//    int hist[1024]; // 幅度值最大不超过 sqrt (255^2 + 255^2)
//
//    for (int i = 0 ; i < 1024 ; ++i)
//    {
//        hist[i] = 0 ;
//    }
//
//    int nwidth = amplitue->width;
//    int nheight = amplitue->height;
//    float * pnoMaxSuppress ,*pAmp;
//
//    pnoMaxSuppress = (float *)noMaxSuppressImg->imageData ;
//    pAmp = (float *)amplitue->imageData;
//    //直方图统计 x轴幅度值 y 在该幅度值的点数
//
//    for (int i = 0 ; i <nheight ; ++i)
//    {
//        for (int j = 0 ; j <nwidth ; ++j)
//        {
//            if(*(pnoMaxSuppress + i * nwidth + j)==128)
//                hist[(int)*(pAmp +i*nwidth + j)]++;
//        }
//    }
//
//
//    //确定最大幅度值，统计疑似边缘点个数
//    int nEdgeNum = 0 ;
//    int nmaxAmp = 0 ;
//    for (int i = 0 ; i< 512 ; ++i)
//    {
//        if (hist[i]!=0)
//        {
//            nmaxAmp = i ;
//            nEdgeNum += hist[i];
//        }
//    }
//
//    //计算上下阈值
//    //将疑似边缘点的幅度值排序，79%处为高阈值，显然 hist直方图是按从小到大存储的，所以省略了排序步骤
//    int highcount = 0.79 * nEdgeNum + 0.5 ;
//    int nEdgeCount = 0 ; //计数
//    for (int i = 0 ; i <=nmaxAmp ; ++i)
//    {
//        nEdgeCount += hist[i] ; 
//        if (nEdgeCount >= highcount)
//        {
//            highthreahold = i ;
//            lowthreahold = 0.5 * highthreahold + 0.5;
//            break;
//        }
//    }
//
//}
//
//
///*
//    函数：hysteresis
//    功能：滞后阈值化
//    参数：amplitue -- in 幅度图
//              nomaxsuppress -- in 疑似边缘图
//              highthreshold -- in 高阈值
//              lowthreshold -- in 低阈值
//*/
//void hysteresis(IplImage * amplitue , IplImage * noMaxSuppressImg , float highthreshold , float lowthreshold)
//{
//    int nwidth = noMaxSuppressImg->width ; 
//    int nheight = noMaxSuppressImg->height ;
//    float * pnoMaxSuppress , *pAmp;
//    pnoMaxSuppress = (float *)noMaxSuppressImg->imageData;
//    pAmp = (float *)amplitue->imageData ;
//    for (int i = 0 ; i<nheight ; ++i)
//    {
//        for (int j = 0 ; j <nwidth ; ++j)
//        {
//            if ( *(pAmp + i * nwidth + j)>=highthreshold && *(pnoMaxSuppress + i * nwidth + j)==128)
//            {
//                *(pnoMaxSuppress + i * nwidth + j) =255;
//                trackEdge( j , i , lowthreshold , amplitue ,noMaxSuppressImg);
//            }
//        }
//    }
//
//
//
//    for (int i = 0 ; i <nheight ; ++i)
//    {
//        for (int j = 0 ; j < nwidth ;++j)
//        {
//            if (*(pnoMaxSuppress + i * nwidth + j)!=255)
//            {
//                *(pnoMaxSuppress + i * nwidth + j) = 0 ;
//            }
//        }
//    }
//
//}
//
//
///*
//    函数：trackEdge
//    功能：找八邻域满足条件的点
//    参数：x ，y -- in 中心点
//             lowthreshold -- in 低阈值
//             amplitue -- 幅度图
//             momaxsuppress -- 疑似边缘图
//*/
//
//void trackEdge(int x , int y , int lowthreshold, IplImage * amplitue , IplImage * noMaxSuppressImg)
//{
//    int xnum[8] = {-1 , 0 , 1 , -1 , 1, -1 ,0 , 1};
//    int ynum[8] = {-1, -1 ,-1 ,0 ,0 , 1, 1, 1};
//    int nwidth = noMaxSuppressImg->width;
//    int nheight = noMaxSuppressImg->height;
//
//    float * pAmp , * pnoMaxSuppress;
//    pAmp = (float *)amplitue->imageData ; 
//    pnoMaxSuppress = (float *)noMaxSuppressImg->imageData ;
//    for ( int i = 0 ; i < 8 ; ++i)
//    {
//        if ( *(pAmp + (y + ynum[i]) * nwidth + x + xnum[i]) >=lowthreshold &&
//            *(pnoMaxSuppress + (y + ynum[i]) * nwidth + x + xnum[i])==128)
//        {
//            *(pnoMaxSuppress + (y + ynum[i]) * nwidth + x + xnum[i])=255;
//            trackEdge(x+xnum[i] , y +ynum[i] , lowthreshold,amplitue , noMaxSuppressImg);
//        }
//    }
//
//}
//
//
//
//
//
///*
//    函数：canny
//    功能：canny边缘提取
//    参数：src -- in 原图
//             dst -- in 边缘图
//            sigma -- in 高斯核半径
//            size -- in 高斯核大小
//            highthreshold --- 高阈值  拓展用 任意值
//            lowthreshold -- 低阈值    拓展用 任意值
//*/
//void canny(IplImage * src, IplImage * dst ,float sigma , int size , float highthreshold, float lowthreshold)
//{
//    //高斯滤波
//    float *guassfilter = (float * )malloc( size * size * sizeof(float));
//    createGuassFilter(guassfilter,size,sigma);
//
//    CvMat mat_filter;
//    cvInitMatHeader(&mat_filter,size,size,CV_32FC1 , (float *)guassfilter);
//    IplImage * filterimg = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    cvFilter2D(src,filterimg,&mat_filter);
//
//    //计算梯度方向和幅度
//    IplImage * gradientx = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    IplImage * gradienty = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    IplImage * amplitue = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    calGradientAndAmplitue(filterimg , gradientx,gradienty ,amplitue);
//
//
//
//	 //IplImage *out=cvCreateImage(  
//  //          cvGetSize(src),//当前图像大小  
//  //          IPL_DEPTH_8U,//各通道每个像素点的类型  
//  //          3//通道总数  
//  //          );  
//	 //cvSmooth(src,out,CV_GAUSSIAN,3,3);  
//	 //IplImage* sobel=cvCreateImage(cvGetSize(out),out->depth,1);
//	 //cvSobel(sobel,sobel,1,0,3);
//
//    //非极大值抑制
//    IplImage * noMaxSuppressImg = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    cvZero(noMaxSuppressImg);
//    noMaxSuppress(gradientx,gradienty,amplitue,noMaxSuppressImg);
//
//
//    //确定上下阈值
//    float high , low;
//    EstimateThreshold(noMaxSuppressImg,amplitue,low,high);
//	//high = 53;
//	//low = 27;
//	std::cout<<"high"<<high<<std::endl;
//	std::cout<<"low"<<low<<std::endl;
//
//    //阈值滞后处理
//    hysteresis(amplitue,noMaxSuppressImg,high,low);
//
//    IplImage * canny = cvCreateImage(cvGetSize(src),8,1);
//    cvConvert(noMaxSuppressImg,canny);
//	cvNamedWindow("CANNY",CV_WINDOW_AUTOSIZE); 
//    cvShowImage("CANNY",canny);
//
//    memcpy(dst->imageData , noMaxSuppressImg->imageData,dst->imageSize);
//
//
//    cvReleaseImage(&filterimg);
//    cvReleaseImage(&gradientx);
//    cvReleaseImage(&gradienty);
//    cvReleaseImage(&amplitue);
//    cvReleaseImage(&noMaxSuppressImg);
//
//    free(guassfilter);
//}