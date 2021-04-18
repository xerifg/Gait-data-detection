#ifndef CAMSHIFT_H
#define CAMSHIFT_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "geometry_msgs/Point.h"
#include <deque>

using namespace cv;
using namespace std;
Mat image;
float v1_x,v1_y;
float  k = 0.4;  ///搜索框相比于初始框的缩小比例
///////////////////////////////////////////////////////////////////
int num = 0;
int p =0;
///////////////////////////////////////////////////////////////////
bool selectObject = false;
bool paused = false;
int trackObject = 0;
int count_track = 0;
bool calinit = true;
bool r_errinit = true;
Point origin;
Rect selection;                                             //ROI矩形区域
Rect standard_window;
float  prop;
float  width_stan=0 , high_stan=0;
int vmin = 10, vmax = 256, smin = 90;   ///   v:亮度
int data=4;
float r1;     ///点坐标
float diff_missing = 0;  ///目标丢失后，认为增加所搜索框放大尺度
float diff_premiss = 0;   ///目标即将丢失时，认为增加所搜索框放大尺度
float  ch_x,ch_y;
float expend_x,expend_y;
Rect trackWindow ;//搜索框矩形
int hsize = 60;                                              //在每一维上直方图的个数。如果是一维直方图，就是竖条(bin)的个数。
float hranges[] = {0,180};                                       //每一维数值的取值范围数组,因为色相的取值范围为（0，180）
const float* phranges = hranges;

float trackcenter1_x,trackcenter1_y;              ///搜索框的中心

Point a;                                   //区域质心
//geometry_msgs::Point aa,bb,cc,dd;                //质心坐标
FILE *fp;
Mat frame, hsv, hue, mask,hist;
Mat histimg = Mat::zeros(200, 320, CV_8UC3), backproj,backprojj;

Mat m_predict;

deque<Rect> dequeWindow1;

static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)//鼠标左键按下时，则条件为true，在if里面的代码块就是确定所选择的矩形区域selection
    {
        if (count_track == 1)
        {
            selection.x = MIN(x, origin.x);
            selection.y = MIN(y, origin.y);//矩形左上角顶点的坐标
            selection.width = std::abs(x - origin.x);//矩形宽
            selection.height = std::abs(y - origin.y);//矩形高
            selection &= Rect(0, 0, image.cols, image.rows);//用于确保所选的矩形区域在图片区域内

        }



    }

    switch (event)//鼠标事件
    {
        case EVENT_LBUTTONDOWN:
            origin = Point(x, y);
            if(count_track == 0)
                selection = Rect(x, y, 0, 0);//鼠标刚按下去时初始化了一个矩形区域
            selectObject = true;
            count_track ++;
            break;
        case EVENT_LBUTTONUP:
            selectObject = false;
            if (selection.width > 0 && selection.height > 0 && count_track == 1)
                trackObject = -1;
            break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////
//
//CV_IMPL RotatedRect
//CamShift2(Mat imgProb, CV_IN_OUT Rect& windowIn, TermCriteria criteria)
//
//{
//    const int TOLERANCE = 10;    ///算法自适应窗口变化时每次扩大的范围
//    Moments muu;    ///用于计算多边形的最高达三阶的所有矩以及用于查找多边形中心
//    double m00 = 0, m10, m01, mu20, mu11, mu02, inv_m00;  ///m00：表示0阶矩,m01：表示1阶水平矩,m10：表示一阶垂直矩
//    double a1, b1, c1, xc, yc;
//    double rotate_a, rotate_c;
//    double theta = 0, square;
//    double cs, sn;
//    double length = 0, width = 0;
//    RotatedRect itersUsed;     ///返回的旋转Rect（成员变量center,size,angle）
//
//   // CvConnectedComp comp;    ///保存运算结果，包括新的search window的位置和面积
//    Mat  cur_win, mat = imgProb;
//
//    CV_FUNCNAME( "cvCamShift2" );
//    {///Begin///
//        if (imgProb.empty())
//            cout<<"don't get image"<<endl; ///检验是否得到有效图像
//
//        CV_CALL( meanShift( mat, windowIn, criteria));  ///移动固定搜索框的中心到质心
//
//        windowIn.x -= TOLERANCE;   ///扩大搜索框
//        if( windowIn.x < 0 )
//            windowIn.x = 0;
//
//        windowIn.y -= TOLERANCE;
//        if( windowIn.y < 0 )
//            windowIn.y = 0;
//
//        windowIn.width += 2 * TOLERANCE;
//        if( windowIn.x + windowIn.width > mat.cols )
//            windowIn.width = mat.cols - windowIn.x;
//
//        windowIn.height += 2 * TOLERANCE;
//        if( windowIn.y + windowIn.height > mat.rows )
//            windowIn.height = mat.rows - windowIn.y;
//        {
//            Size patchsize;
//            patchsize.height = windowIn.height;
//            patchsize.width = windowIn.width;
//            Point2f centerpatch;
//            centerpatch.x = windowIn.x + windowIn.width * 0.5f;
//            centerpatch.y = windowIn.y + windowIn.height * 0.5f;
//            CV_CALL(getRectSubPix(mat, patchsize, centerpatch, cur_win));///把截取图像中需要的区域存入矩阵cur_win
//
//           ///  Calculating moments in new center mass
//            CV_CALL(muu = moments(cur_win));   ///计算0阶矩，1阶矩
//            m00 = muu.m00;   ///空间距
//            m10 = muu.m10;
//            m01 = muu.m01;
//            mu11 = muu.mu11;   ///中心距
//            mu20 = muu.mu20;
//            mu02 = muu.mu02;
//        }
//        if( fabs(m00) < DBL_EPSILON )
//            goto exit;
//
//        inv_m00 = 1. / m00;
//        xc = round( m10 * inv_m00 + windowIn.x );  ///搜索框的质心坐标
//        yc = round( m01 * inv_m00 + windowIn.y );
//        a1 = mu20 * inv_m00 ;
//        b1 = mu11 * inv_m00 ;
//        c1 = mu02 * inv_m00 ;
//
//        /* Calculating width & height */
//        square = sqrt( 4 * b1 * b1 + (a1 - c1) * (a1 - c1) );
//
//        /* Calculating orientation */
//        theta = atan2( 2 * b1, a1 - c1  +square);  ///物体的旋转角度(0 - 360)
//       // theta = theta*CV_PI/180.0;
//        /* Calculating width & length of figure */
//        cs = cos( theta );  ///输入的弧度
//        sn = sin( theta );
//
//        rotate_a = cs * cs * mu20 + 2 * cs * sn * mu11 + sn * sn * mu02;
//        rotate_c = sn * sn * mu20 - 2 * cs * sn * mu11 + cs * cs * mu02;
//        length = sqrt( rotate_a * inv_m00 ) * 4;        ///包围物体的最小矩形的长度
//        width = sqrt( rotate_c * inv_m00 ) * 4;
//
//        /* In case, when tetta is 0 or 1.57... the Length & Width may be exchanged */
//        if( length < width )
//        {
//            double t;
//
//            CV_SWAP( length, width, t );
//            CV_SWAP( cs, sn, t );
//            theta = CV_PI*0.5 - theta;
//        }
//
//        /* Saving results */
//        {
//            int t0, t1;
//            int _xc = round(xc);   ///质心x坐标
//            int _yc = round(yc);   ///质心y坐标
//
//            t0 = round(fabs(length * cs));
//            t1 = round(fabs(width * sn));
//
//            t0 = MAX(t0, t1) + 2;
//            windowIn.width = MIN(t0, (mat.cols - _xc) * 2);   ///改变搜索框的width大小
//
//            t0 = round(fabs(length * sn));
//            t1 = round(fabs(width * cs));
//
//            t0 = MAX(t0, t1) + 2;
//            windowIn.height = MIN(t0, (mat.rows - _yc) * 2);   ///改变搜索框的height大小
//
//            windowIn.x = MAX(0, _xc - windowIn.width / 2);
//            windowIn.y = MAX(0, _yc - windowIn.height / 2);
//
//            windowIn.width = MIN(mat.cols - windowIn.x, windowIn.width);
//            windowIn.height = MIN(mat.rows - windowIn.y, windowIn.height);
//
//        }
//
//        exit: ; } //////__END__;
//
//        itersUsed.size.height = (float)windowIn.height;
//        itersUsed.size.width = (float)windowIn.width;
//        itersUsed.angle = (float)(theta*180./CV_PI);   ///物体的旋转角度
//        itersUsed.center = cvPoint2D32f( windowIn.x + windowIn.width*0.5f,
//                                         windowIn.y + windowIn.height*0.5f);
//
//    return itersUsed;
//}

////////////////////////////  计算两个向量之间的夹角（0～pi）///////////////////////////

float vector_angle (Point po1,Point po2,Point po3,Point po4)
{
    float alfer;
    float  sn,cs;
    po2.x = po2.x - po1.x;
    po2.y = -(po2.y - po1.y);
    po4.x = po4.x - po3.x;
    po4.y = -(po4.y - po3.y);

    sn = po2.x*po4.x + po4.y*po2.y;
    cs = sqrt(po2.x*po2.x+po2.y*po2.y)*sqrt(po4.x*po4.x+po4.y*po4.y);

    alfer = abs(acos(sn/cs));

    alfer = alfer*180/CV_PI;
    return alfer;
}


#endif
