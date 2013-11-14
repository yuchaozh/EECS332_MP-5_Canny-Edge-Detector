//// createGuassFilter.cpp : �������̨Ӧ�ó������ڵ㡣 
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
//    ������ : creatGuassFilter
//    ���ܣ����ɸ�˹�˲�ģ��
//    ������mat_GuassFilter --- out ���ڴ洢���ɵĸ�˹�˲�ģ��
//              nsize ----- in �˲����ڳߴ�
//              sigma ----- in ��˹�뾶  
//*/
//void createGuassFilter(float *GuassFilter, int nsize,float sigma)
//{
//        float distX,distY; //���������ĵľ���
//        int ncenter = nsize/2; //��������
//        float sigmaA = 1.0 / (sqrt(2 * CV_PI) * sigma) ;
//        float sum =0.0; 
//
//        //���sigma = 0 ���Զ�����sigmaֵ
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
//        //��һ��
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
//    ������calGradientAndAmplitue 
//    ���ܣ�����ͼ���ݶȷ��� �ͷ���
//    ������src -- in ԭͼ
//              gradientX Y -- out x��y�����ݶ�ֵ �������� int
//              amplitue -- out ����ֵ ��������double
//*/
//void calGradientAndAmplitue(IplImage * src , IplImage * gradientX,IplImage * gradientY ,IplImage * amplitue )
//{
//    ////���� x ��y �����ݶ�
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
//	    //���� x ��y �����ݶ�
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
//		  //  //���� x ��y �����ݶ�
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
//    //�������ֵ
//    int nwidth = src->width ; 
//    int nheight = src->height;
//    float Gx, Gy; //x y �����ݶ�
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
//    ������noMaxSuppress
//    ���ܣ��Ǽ���ֵ����
//*/
//void noMaxSuppress(IplImage * gradientX, IplImage *gradientY, IplImage *amplitue,IplImage * noMaxSupress )
//{
//    int nwidth = noMaxSupress->width ; 
//    int nheight = noMaxSupress->height;
//    //�Ǽ���ֵ����
//
//    float Gx ,Gy ; // x,y�ݶ�ֵ
//    float temp1,temp2,temp3,temp4; //���ڼ����ֵ�õ��ݶȷ����������ֵ���м����
//    float amp , amp1 ,amp2; // �õ����ֵ �õ��ݶȷ����ϵķ���ֵ
//    float angle = 0.0; //�ݶȷ���
//    float rate = 0.0; // �߲����
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
//            //�������ֵΪ0 ��Ϊ�Ǳ�Ե��
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
//            //��һ�����
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
//            //�ڶ������
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
//            //���������
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
//            //���������
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
//            //�ֲ����ֵ�ж� ���Ϊ���ֵ 128 ����0
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
//    ������ EstimateThreshold
//    ���ܣ�����������ֵ
//    ������noMaxSuppress -- in �Ǽ���ֵ���ƴ����� ���Ʊ�Ե��ͼ
//              amplitue -- in ����ͼ
//              lowthreashold -- out ����ֵ
//              highthreshold --out ����ֵ
//*/
//void EstimateThreshold(IplImage * noMaxSuppressImg , IplImage * amplitue , float &lowthreahold ,float &highthreahold)
//{
//    int hist[1024]; // ����ֵ��󲻳��� sqrt (255^2 + 255^2)
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
//    //ֱ��ͼͳ�� x�����ֵ y �ڸ÷���ֵ�ĵ���
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
//    //ȷ��������ֵ��ͳ�����Ʊ�Ե�����
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
//    //����������ֵ
//    //�����Ʊ�Ե��ķ���ֵ����79%��Ϊ����ֵ����Ȼ histֱ��ͼ�ǰ���С����洢�ģ�����ʡ����������
//    int highcount = 0.79 * nEdgeNum + 0.5 ;
//    int nEdgeCount = 0 ; //����
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
//    ������hysteresis
//    ���ܣ��ͺ���ֵ��
//    ������amplitue -- in ����ͼ
//              nomaxsuppress -- in ���Ʊ�Եͼ
//              highthreshold -- in ����ֵ
//              lowthreshold -- in ����ֵ
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
//    ������trackEdge
//    ���ܣ��Ұ��������������ĵ�
//    ������x ��y -- in ���ĵ�
//             lowthreshold -- in ����ֵ
//             amplitue -- ����ͼ
//             momaxsuppress -- ���Ʊ�Եͼ
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
//    ������canny
//    ���ܣ�canny��Ե��ȡ
//    ������src -- in ԭͼ
//             dst -- in ��Եͼ
//            sigma -- in ��˹�˰뾶
//            size -- in ��˹�˴�С
//            highthreshold --- ����ֵ  ��չ�� ����ֵ
//            lowthreshold -- ����ֵ    ��չ�� ����ֵ
//*/
//void canny(IplImage * src, IplImage * dst ,float sigma , int size , float highthreshold, float lowthreshold)
//{
//    //��˹�˲�
//    float *guassfilter = (float * )malloc( size * size * sizeof(float));
//    createGuassFilter(guassfilter,size,sigma);
//
//    CvMat mat_filter;
//    cvInitMatHeader(&mat_filter,size,size,CV_32FC1 , (float *)guassfilter);
//    IplImage * filterimg = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    cvFilter2D(src,filterimg,&mat_filter);
//
//    //�����ݶȷ���ͷ���
//    IplImage * gradientx = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    IplImage * gradienty = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    IplImage * amplitue = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    calGradientAndAmplitue(filterimg , gradientx,gradienty ,amplitue);
//
//
//
//	 //IplImage *out=cvCreateImage(  
//  //          cvGetSize(src),//��ǰͼ���С  
//  //          IPL_DEPTH_8U,//��ͨ��ÿ�����ص������  
//  //          3//ͨ������  
//  //          );  
//	 //cvSmooth(src,out,CV_GAUSSIAN,3,3);  
//	 //IplImage* sobel=cvCreateImage(cvGetSize(out),out->depth,1);
//	 //cvSobel(sobel,sobel,1,0,3);
//
//    //�Ǽ���ֵ����
//    IplImage * noMaxSuppressImg = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F,1);
//    cvZero(noMaxSuppressImg);
//    noMaxSuppress(gradientx,gradienty,amplitue,noMaxSuppressImg);
//
//
//    //ȷ��������ֵ
//    float high , low;
//    EstimateThreshold(noMaxSuppressImg,amplitue,low,high);
//	//high = 53;
//	//low = 27;
//	std::cout<<"high"<<high<<std::endl;
//	std::cout<<"low"<<low<<std::endl;
//
//    //��ֵ�ͺ���
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