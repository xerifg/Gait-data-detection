#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "../include/cam_show/init.h"
#include "cstdio"   //用于数据存储到txt文件使用
#include "geometry_msgs/Point.h"
#include <fstream>
#include <cmath>
#include <ctime>
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>


ofstream outfile;   //NOLINT
ofstream outfile2;   //NOLINT
ofstream outfile3;   //NOLINT
ofstream outfileCenterPosition;   //NOLINT
ofstream outfileTime;   //NOLINT
float center_x[6],center_y[6];
using namespace std;
namespace enc = sensor_msgs::image_encodings;

/*准备再次发布的图像显示到本窗口*/
//static const char OUT_WINDOW[] = "Image Out";
/*读取订阅的图像并显示到本窗口*/
//static std::string IN_WINDOW = "Image In";

int time_count = 0;
double start_time;
double end_time;


int global = 0;         ///定义采集图像的频率

cv::KalmanFilter m_KF;  ///卡尔曼滤波器  //NOLINT
Mat m_measured;      ///测量值  //NOLINT
Mat m_estimated;     ///卡尔曼修正后的数值  //NOLINT


cv::KalmanFilter m_KF2;  ///卡尔曼滤波器  //NOLINT
Mat m_measured2;      ///测量值  //NOLINT
Mat m_estimated2;     ///卡尔曼修正后的数值  //NOLINT

cv::KalmanFilter m_KF3;  ///卡尔曼滤波器  //NOLINT
Mat m_measured3;      ///测量值  //NOLINT
Mat m_estimated3;     ///卡尔曼修正后的数值  //NOLINT

cv::KalmanFilter m_KF4;  ///卡尔曼滤波器  //NOLINT
Mat m_measured4;      ///测量值  //NOLINT
Mat m_estimated4;     ///卡尔曼修正后的数值  //NOLINT

cv::KalmanFilter m_KF5;  ///卡尔曼滤波器  //NOLINT
Mat m_measured5;      ///测量值  //NOLINT
Mat m_estimated5;     ///卡尔曼修正后的数值  //NOLINT

cv::KalmanFilter m_KF6;  ///卡尔曼滤波器  //NOLINT
Mat m_measured6;      ///测量值  //NOLINT
Mat m_estimated6;     ///卡尔曼修正后的数值  //NOLINT

//删除文件夹下的所有文件
void Getfilepath(const char *path, const char *filename,  char *filepath)
{
    strcpy(filepath, path);
    if(filepath[strlen(path) - 1] != '/')
        strcat(filepath, "/");
    strcat(filepath, filename);
    printf("path is = %s\n",filepath);
}

bool DeleteFile(const char* path)
{
    DIR *dir;
    struct dirent *dirinfo;
    struct stat statbuf;
    char filepath[256] = {0};
    lstat(path, &statbuf);

    if (S_ISREG(statbuf.st_mode))//判断是否是常规文件
    {
        remove(path);
    }
    else if (S_ISDIR(statbuf.st_mode))//判断是否是目录
    {
        if ((dir = opendir(path)) == NULL)
            return true;
        while ((dirinfo = readdir(dir)) != NULL)
        {
            Getfilepath(path, dirinfo->d_name, filepath);
            if (strcmp(dirinfo->d_name, ".") == 0 || strcmp(dirinfo->d_name, "..") == 0)//判断是否是特殊目录
                continue;
            DeleteFile(filepath);
            rmdir(filepath);
        }
        closedir(dir);
    }
    return false;
}



//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////时间计时-精确到毫秒///////////////////////////////////////////////////
double get_wall_time()
{
    struct timeval time ;//NOLINT
    if (gettimeofday(&time,NULL))//NOLINT
    {
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
///////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////      保存数据函数      //////////////////////////////////////
void writeLog(float data1 ,int num_x)    /////Point_<float>
{ outfile.open("/home/xrf/catkin_ws/src/cam_show/data1.txt", ios::app);//追加模式
  //  fstream file("/home/ethan/catkin_ws/src/cam_show/data1.txt", ios::out); //清空文件内容
    if (!outfile) //检查文件是否正常打开//不是用于检查文件是否存在
    {
        cout << "data1.txt can't open" << endl;
        abort(); //打开失败，结束程序
    } else {
        outfile << setiosflags(ios::fixed) << setprecision(2) << data1 << endl;
        outfile.close();
    }
}
void writeLog_2(float data2 ,int num_x)    /////Point_<float>
{
    outfile2.open("/home/xrf/catkin_ws/src/cam_show/data2.txt", ios::app);//追加模式
  //  fstream file("/home/ethan/catkin_ws/src/cam_show/data2.txt", ios::out);
    if (!outfile2) //检查文件是否正常打开//不是用于检查文件是否存在
    {
        cout << "data2.txt can't open" << endl;
        abort(); //打开失败，结束程序
    } else {
        outfile2 << setiosflags(ios::fixed) << setprecision(2) << data2 << endl;
        outfile2.close();
    }
}
void writeLog_3(float data3 ,int num_x)    /////Point_<float>
{
    outfile3.open("/home/xrf/catkin_ws/src/cam_show/data3.txt", ios::app);//追加模式
    //  fstream file("/home/ethan/catkin_ws/src/cam_show/data2.txt", ios::out);
    if (!outfile3) //检查文件是否正常打开//不是用于检查文件是否存在
    {
        cout << "data3.txt can't open" << endl;
        abort(); //打开失败，结束程序
    } else {
        outfile3 << setiosflags(ios::fixed) << setprecision(2) << data3 << endl;
        outfile3.close();
    }
}

void writeLog_outfileCenterPosition(float center_x[6],float center_y[6])    /////Point_<float>
{
    outfileCenterPosition.open("/home/xrf/catkin_ws/src/cam_show/CenterPosition.txt", ios::app);//追加模式
    if (!outfileCenterPosition) //检查文件是否正常打开//不是用于检查文件是否存在
    {
        cout << "CenterPosition.txt can't open" << endl;
        abort(); //打开失败，结束程序
    } else {
        outfileCenterPosition << setiosflags(ios::fixed) << setprecision(0) << center_x[0] << "," << center_y[0] << "," << center_x[1] << "," << center_y[1] << "," ;
	outfileCenterPosition << center_x[2] << "," << center_y[2] << "," << center_x[3] << "," << center_y[3] << "," << center_x[4] << "," << center_y[4] << "," << center_x[5] << "," << 		center_y[5] << endl;
        outfileCenterPosition.close();
    }
}

/////////////////////////////////////////////////////////////////////////////////////

////////////////////////////自定义一个窗口显示多图函数//////////////////////////////////
void imshowMany(const std::string& _winName, const vector<Mat>& ployImages,int size)
{
    int nImg = (int)ployImages.size();//获取在同一画布中显示多图的数目
    ///int size=800;
    int x, y;
    ///若要在OpenCV实现同一窗口显示多幅图片，图片要按矩阵方式排列，类似于Matlab中subplot();
    int w=3, h=2;  ///3x2
    float scale;///缩放比例
    int max;
    Mat white_img = Mat(Size(100 + size*w, 30 + size*h),CV_8UC3,Scalar(255, 255, 255));//根据图片矩阵w*h，创建画布，可线的图片数量为w*h
    for (int i = 0, m = 20, n = 20; i<nImg; i++, m += (20 + size))
    {
        x = ployImages[i].cols; //第(i+1)张子图像的宽度(列数)
        y = ployImages[i].rows;//第(i+1)张子图像的高度（行数）
        max = (x > y) ? x : y;//比较每张图片的行数和列数，取大值
        scale = (float)((float)max / (float)size);//计算缩放比例
        if (i%w == 0 && m != 20)
        {
            m = 20;
            n += (20 + size);
        }
        Mat imgROI = white_img(Rect(m, n, (int)((float)x / scale), (int)((float)y / scale))); //在画布dispImage中划分ROI区域
        resize(ployImages[i], imgROI, Size((int)((float)x / scale), (int)((float)y / scale))); //将要显示的图像设置为ROI区域大小
    }
    namedWindow(_winName);
    imshow(_winName, white_img);
}
//////////////////////////////////////////////////////////////////////

////////////////////////////单通道转化为3通道图像函数/////////////////////////////////
Mat convertTo3Channels(const Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
    vector<Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);
    return three_channel;
}
////////////////////////////////////////////////////////////////////////////////////////////////
class ImageConvertor
{
    ros::NodeHandle nh_;
  //  ros::Publisher pub_data = nh_.advertise<geometry_msgs::Point>("pre_data",1);
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
  //  image_transport::Publisher image_pub_;

public:
    ImageConvertor():it_(nh_){
        /*发布主题out*/
        //image_pub_  = it_.advertise("node_b", 1);
        /*订阅主题camera/image*/
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConvertor::ImageCb, this);
        ////////////////////////卡尔曼滤波器初始化///////////////////////
        {
            //float dert = 33;       //////////采集时间
            m_KF = cv::KalmanFilter(4, 2, 2);        ///卡尔曼滤波器的初始化
            m_measured = cv::Mat::zeros(2, 1, CV_32F);
            m_predict = cv::Mat::zeros(2, 1, CV_32F);
            m_KF.transitionMatrix = (cv::Mat_<float>(4, 4)
                    << 1, 0, dert, 0, 0, 1, 0, dert, 0, 0, 1, 0, 0, 0, 0, 1);   ///卡尔曼滤波器的状态转移矩阵的初始化,0.03为相邻状态更新的时间差
            cv::setIdentity(m_KF.measurementMatrix); ///测量矩阵 H  ///setIdentity :将行数和列数相等的元素设置为s，默认为1，其他元素设置为0。
            cv::setIdentity(m_KF.processNoiseCov, cv::Scalar::all(1e-2)); ////过程噪声 Q
            cv::setIdentity(m_KF.measurementNoiseCov, cv::Scalar::all(10));////测量噪声 R
            cv::setIdentity(m_KF.errorCovPost, cv::Scalar::all(1)); ////最小均方误差 P
            m_KF.controlMatrix = (cv::Mat_<float>(4, 2) << 0.5 * dert * dert, 0, 0, 0.5 *
                                                                                    dert, dert, 0, 0, dert);   ///控制矩阵

            float dert2 = 33;
            m_KF2 = cv::KalmanFilter(4, 2, 2);        ///卡尔曼滤波器的初始化
            m_measured2 = cv::Mat::zeros(2, 1, CV_32F);
            m_KF2.transitionMatrix = (cv::Mat_<float>(4, 4)
                    << 1, 0, dert2, 0, 0, 1, 0, dert2, 0, 0, 1, 0, 0, 0, 0, 1);   ///卡尔曼滤波器的状态转移矩阵的初始化,0.03为相邻状态更新的时间差
            cv::setIdentity(m_KF2.measurementMatrix); ///测量矩阵 H  ///setIdentity :将行数和列数相等的元素设置为s，默认为1，其他元素设置为0。
            cv::setIdentity(m_KF2.processNoiseCov, cv::Scalar::all(1e-2)); ////过程噪声 Q
            cv::setIdentity(m_KF2.measurementNoiseCov, cv::Scalar::all(10));////测量噪声 R
            cv::setIdentity(m_KF2.errorCovPost, cv::Scalar::all(1)); ////最小均方误差 P
            m_KF2.controlMatrix = (cv::Mat_<float>(4, 2) << 0.5 * dert2 * dert2, 0, 0, 0.5 *
                                                                                       dert2, dert2, 0, 0, dert2);   ///控制矩阵

            float dert3 = 33;
            m_KF3 = cv::KalmanFilter(4, 2, 2);        ///卡尔曼滤波器的初始化
            m_measured3 = cv::Mat::zeros(2, 1, CV_32F);
            m_KF3.transitionMatrix = (cv::Mat_<float>(4, 4)
                    << 1, 0, dert3, 0, 0, 1, 0, dert3, 0, 0, 1, 0, 0, 0, 0, 1);   ///卡尔曼滤波器的状态转移矩阵的初始化,0.03为相邻状态更新的时间差
            cv::setIdentity(m_KF3.measurementMatrix); ///测量矩阵 H  ///setIdentity :将行数和列数相等的元素设置为s，默认为1，其他元素设置为0。
            cv::setIdentity(m_KF3.processNoiseCov, cv::Scalar::all(1e-2)); ////过程噪声 Q
            cv::setIdentity(m_KF3.measurementNoiseCov, cv::Scalar::all(10));////测量噪声 R
            cv::setIdentity(m_KF3.errorCovPost, cv::Scalar::all(1)); ////最小均方误差 P
            m_KF3.controlMatrix = (cv::Mat_<float>(4, 2) << 0.5 * dert3 * dert3, 0, 0, 0.5 *
                                                                                       dert3, dert3, 0, 0, dert3);   ///控制矩阵

            float dert4 = 33;
            m_KF4 = cv::KalmanFilter(4, 2, 2);        ///卡尔曼滤波器的初始化
            m_measured4 = cv::Mat::zeros(2, 1, CV_32F);
            m_KF4.transitionMatrix = (cv::Mat_<float>(4, 4)
                    << 1, 0, dert4, 0, 0, 1, 0, dert4, 0, 0, 1, 0, 0, 0, 0, 1);   ///卡尔曼滤波器的状态转移矩阵的初始化,0.03为相邻状态更新的时间差
            cv::setIdentity(m_KF4.measurementMatrix); ///测量矩阵 H  ///setIdentity :将行数和列数相等的元素设置为s，默认为1，其他元素设置为0。
            cv::setIdentity(m_KF4.processNoiseCov, cv::Scalar::all(1e-2)); ////过程噪声 Q
            cv::setIdentity(m_KF4.measurementNoiseCov, cv::Scalar::all(10));////测量噪声 R
            cv::setIdentity(m_KF4.errorCovPost, cv::Scalar::all(1)); ////最小均方误差 P
            m_KF4.controlMatrix = (cv::Mat_<float>(4, 2) << 0.5 * dert4 * dert4, 0, 0, 0.5 *
                                                                                       dert4, dert4, 0, 0, dert4);   ///控制矩阵

            float dert5 = 33;
            m_KF5 = cv::KalmanFilter(4, 2, 2);        ///卡尔曼滤波器的初始化
            m_measured5 = cv::Mat::zeros(2, 1, CV_32F);
            m_KF5.transitionMatrix = (cv::Mat_<float>(4, 4)
                    << 1, 0, dert5, 0, 0, 1, 0, dert5, 0, 0, 1, 0, 0, 0, 0, 1);   ///卡尔曼滤波器的状态转移矩阵的初始化,0.03为相邻状态更新的时间差
            cv::setIdentity(m_KF5.measurementMatrix); ///测量矩阵 H  ///setIdentity :将行数和列数相等的元素设置为s，默认为1，其他元素设置为0。
            cv::setIdentity(m_KF5.processNoiseCov, cv::Scalar::all(1e-2)); ////过程噪声 Q
            cv::setIdentity(m_KF5.measurementNoiseCov, cv::Scalar::all(10));////测量噪声 R
            cv::setIdentity(m_KF5.errorCovPost, cv::Scalar::all(1)); ////最小均方误差 P
            m_KF5.controlMatrix = (cv::Mat_<float>(4, 2) << 0.5 * dert5 * dert5, 0, 0, 0.5 *
                                                                                       dert5, dert5, 0, 0, dert5);   ///控制矩阵

            float dert6 = 33;
            m_KF6 = cv::KalmanFilter(4, 2, 2);        ///卡尔曼滤波器的初始化
            m_measured6 = cv::Mat::zeros(2, 1, CV_32F);
            m_KF6.transitionMatrix = (cv::Mat_<float>(4, 4)
                    << 1, 0, dert6, 0, 0, 1, 0, dert6, 0, 0, 1, 0, 0, 0, 0, 1);   ///卡尔曼滤波器的状态转移矩阵的初始化,0.03为相邻状态更新的时间差
            cv::setIdentity(m_KF6.measurementMatrix); ///测量矩阵 H  ///setIdentity :将行数和列数相等的元素设置为s，默认为1，其他元素设置为0。
            cv::setIdentity(m_KF6.processNoiseCov, cv::Scalar::all(1e-2)); ////过程噪声 Q
            cv::setIdentity(m_KF6.measurementNoiseCov, cv::Scalar::all(10));////测量噪声 R
            cv::setIdentity(m_KF6.errorCovPost, cv::Scalar::all(1)); ////最小均方误差 P
            m_KF6.controlMatrix = (cv::Mat_<float>(4, 2) << 0.5 * dert6 * dert6, 0, 0, 0.5 *
                                                                                       dert6, dert6, 0, 0, dert6);   ///控制矩阵
        }
        }

    ~ImageConvertor()
    {
        cv::destroyWindow("Histogram");
        cv::destroyWindow("CamShift Demo");
        cv::destroyWindow("反向投影");
    }

     void ImageCb(const sensor_msgs::ImageConstPtr& msg)       ///回调函数   //NOLINT
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            /*转化成CVImage*/
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception is %s", e.what());
            return;
        }

        /*****************  对原图像进行算法处理 ********************************/
        if(!paused)
        {

                if (!paused)//不暂停
                {
                    frame = cv_ptr->image;//从摄像头抓取一帧图像并输入到frame中
                    if (frame.empty())
                        cout<<"don't get image"<<endl;
                }

                frame.copyTo(image);

                if (!paused)
                {
                    cvtColor(image, hsv, COLOR_BGR2HSV);//将rgb摄像头帧转化成hsv空间的

                    if (trackObject)//trackObject初始化为0，当鼠标单击松开后为-1
                    {
                        int _vmin = vmin, _vmax = vmax;
                        //inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以是多通道的,mask保存0通道的最小值，也就是h分量
                        //这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
                        //mask对应的那个点的值全为1(0xff)，否则为0(0x00).
                        inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
                                Scalar(180, 256, MAX(_vmin, _vmax)), mask);

                        int ch[] = { 0, 0};
                        hue.create(hsv.size(), hsv.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
                        mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue的第一个通道中，0索引数组

                        if (trackObject < 0)///鼠标选择区域松开后，该函数内部又将其赋值1；该函数体里的内容只在刚开始第一帧时运行，后面不再运行
                        {
                            trackObject = 1;
                            /////////////////得到ROI矩形区域的颜色直方图/////////////////
                            //此处的构造函数roi用的是Mat hue的矩阵头，且roi的数据指针指向hue，即共用相同的数据，select为其感兴趣的区域
                            Mat roi(hue, selection), maskroi(mask, selection);
                            Mat roi2(hue, selection2), maskroi2(mask, selection2);
                            Mat roi3(hue, selection3), maskroi3(mask, selection3);
                            Mat roi4(hue, selection4), maskroi4(mask, selection4);
                            Mat roi5(hue, selection5), maskroi5(mask, selection5);
                            Mat roi6(hue, selection6), maskroi6(mask, selection6);
                            //calcHist()函数第一个参数为输入矩阵序列，第2个参数表示输入的矩阵数目，第3个参数表示将被计算直方图维数通道的列表，第4个参数表示可选的掩码函数
                            //第5个参数表示输出直方图，第6个参数表示直方图的维数，第7个参数为每一维直方图数组的大小，第8个参数为每一维直方图bin的边界
                            calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小
                            calcHist(&roi2, 1, 0, maskroi2, hist2, 1, &hsize, &phranges);
                            calcHist(&roi3, 1, 0, maskroi3, hist3, 1, &hsize, &phranges);
                            calcHist(&roi4, 1, 0, maskroi4, hist4, 1, &hsize, &phranges);
                            calcHist(&roi5, 1, 0, maskroi5, hist5, 1, &hsize, &phranges);
                            calcHist(&roi6, 1, 0, maskroi6, hist6, 1, &hsize, &phranges);
                            normalize(hist, hist, 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0-255
                            normalize(hist2, hist2, 0, 255, CV_MINMAX);
                            normalize(hist3, hist3, 0, 255, CV_MINMAX);
                            normalize(hist4, hist4, 0, 255, CV_MINMAX);
                            normalize(hist5, hist5, 0, 255, CV_MINMAX);
                            normalize(hist6, hist6, 0, 255, CV_MINMAX);
                            trackWindow = selection;
                            trackWindow2 = selection2;
                            trackWindow3 = selection3;
                            trackWindow4 = selection4;
                            trackWindow5 = selection5;
                            trackWindow6 = selection6;

                            /////////////初始化calman的初始位置为selection////////////
                                m_KF.statePost = (cv::Mat_<float>(4, 1) << selection.x, selection.y, 0, 0);
                                m_KF2.statePost = (cv::Mat_<float>(4, 1) << selection2.x, selection2.y, 0, 0);
                                m_KF3.statePost = (cv::Mat_<float>(4, 1) << selection3.x, selection3.y, 0, 0);
                                m_KF4.statePost = (cv::Mat_<float>(4, 1) << selection4.x, selection4.y, 0, 0);
                                m_KF5.statePost = (cv::Mat_<float>(4, 1) << selection5.x, selection5.y, 0, 0);
                                m_KF6.statePost = (cv::Mat_<float>(4, 1) << selection6.x, selection6.y, 0, 0);
                                dequeWindow1.push_back(selection);
                                dequeWindow2.push_back(selection2);
                                dequeWindow3.push_back(selection3);
                                dequeWindow4.push_back(selection4);
                                dequeWindow5.push_back(selection5);
                                dequeWindow6.push_back(selection6);
                            ////////////////////绘制颜色直方图////////////////////////
                            histimg = Scalar::all(0);
                            histimg2 = Scalar::all(0);
                            histimg3 = Scalar::all(0);
                            histimg4 = Scalar::all(0);
                            histimg5 = Scalar::all(0);
                            histimg6 = Scalar::all(0);
                            int binW = histimg.cols / hsize;//histing是一个200*300的矩阵，hsize应该是每一个bin的宽度，也就是histing矩阵能分出几个bin出来
                            int binW2 = histimg2.cols / hsize;
                            int binW3 = histimg3.cols / hsize;
                            int binW4 = histimg4.cols / hsize;
                            int binW5 = histimg5.cols / hsize;
                            int binW6 = histimg6.cols / hsize;
                            Mat buf(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            Mat buf2(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            Mat buf3(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            Mat buf4(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            Mat buf5(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            Mat buf6(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            for (int i = 0; i < hsize; i++)//saturate_case函数为从一个初始类型准确变换到另一个初始类型
                            {
                                buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                                buf2.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                                buf3.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                                buf4.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                                buf5.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                                buf6.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                            }
                            cvtColor(buf, buf, CV_HSV2BGR);//hsv又转换成bgr
                            cvtColor(buf2, buf2, CV_HSV2BGR);
                            cvtColor(buf3, buf3, CV_HSV2BGR);
                            cvtColor(buf4, buf4, CV_HSV2BGR);
                            cvtColor(buf5, buf5, CV_HSV2BGR);
                            cvtColor(buf6, buf6, CV_HSV2BGR);

                            for (int i = 0; i < hsize; i++)
                            {
                                int val = saturate_cast<int>(hist.at<float>(i)*(float )histimg.rows / 255);//at函数为返回一个指定数组元素的参考值
                                int val2 = saturate_cast<int>(hist2.at<float>(i)*(float )histimg2.rows / 255);
                                int val3 = saturate_cast<int>(hist3.at<float>(i)*(float )histimg3.rows / 255);//at函数为返回一个指定数组元素的参考值
                                int val4 = saturate_cast<int>(hist4.at<float>(i)*(float )histimg4.rows / 255);
                                int val5 = saturate_cast<int>(hist5.at<float>(i)*(float )histimg5.rows / 255);//at函数为返回一个指定数组元素的参考值
                                int val6 = saturate_cast<int>(hist6.at<float>(i)*(float )histimg6.rows / 255);
                                //在一幅输入图像上画一个简单抽的矩形，指定左上角和右下角，并定义颜色，大小，线型等
                                rectangle(histimg, Point(i*binW, histimg.rows),
                                          Point((i + 1)*binW, histimg.rows - val),
                                          Scalar(buf.at<Vec3b>(i)), -1, 8);
                                rectangle(histimg2, Point(i*binW2, histimg2.rows),
                                          Point((i + 1)*binW2, histimg2.rows - val2),
                                          Scalar(buf2.at<Vec3b>(i)), -1, 8);
                                rectangle(histimg3, Point(i*binW3, histimg3.rows),
                                          Point((i + 1)*binW3, histimg3.rows - val3),
                                          Scalar(buf3.at<Vec3b>(i)), -1, 8);
                                rectangle(histimg4, Point(i*binW4, histimg4.rows),
                                          Point((i + 1)*binW4, histimg4.rows - val4),
                                          Scalar(buf4.at<Vec3b>(i)), -1, 8);
                                rectangle(histimg5, Point(i*binW5, histimg5.rows),
                                          Point((i + 1)*binW5, histimg5.rows - val5),
                                          Scalar(buf5.at<Vec3b>(i)), -1, 8);
                                rectangle(histimg6, Point(i*binW6, histimg6.rows),
                                          Point((i + 1)*binW6, histimg6.rows - val6),
                                          Scalar(buf6.at<Vec3b>(i)), -1, 8);
                            }
                        }

                        ////////////////////反向投影 ////////////////////////
                        //计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
                        calcBackProject(&hue, 1, 0, hist, backproj, &phranges);  //反向投影（概率分布图）函数
                        calcBackProject(&hue, 1, 0, hist2, backproj2, &phranges);
                        calcBackProject(&hue, 1, 0, hist3, backproj3, &phranges);
                        calcBackProject(&hue, 1, 0, hist4, backproj4, &phranges);
                        calcBackProject(&hue, 1, 0, hist5, backproj5, &phranges);
                        calcBackProject(&hue, 1, 0, hist6, backproj6, &phranges);
                        backproj &= mask;
                        backproj2 &= mask;
                        backproj3 &= mask;
                        backproj4 &= mask;
                        backproj5 &= mask;
                        backproj6 &= mask;
                        /////////////////单通道转化为3通道图像////////////////////////
                        backprojj = convertTo3Channels(backproj);
                        backprojj2 = convertTo3Channels(backproj2);
                        backprojj3 = convertTo3Channels(backproj3);
                        backprojj4 = convertTo3Channels(backproj4);
                        backprojj5 = convertTo3Channels(backproj5);
                        backprojj6 = convertTo3Channels(backproj6);
                        vector<Mat>Images(6);//模板类vector，用于放置4个类型为Mat的元素，即四张图片
                        Images[0] = backprojj;
                        Images[1] = backprojj2;
                        Images[2] = backprojj3;
                        Images[3] = backprojj4;
                        Images[4] = backprojj5;
                        Images[5] = backprojj6;

                        imshowMany("反向投影",Images,300);    ///多画面显示

                        /////////////////////////    卡尔曼预测        ////////////////////////////
                            Mat prediction = m_KF.predict();
                            Mat prediction2 = m_KF2.predict();
                            Mat prediction3 = m_KF3.predict();
                            Mat prediction4 = m_KF4.predict();
                            Mat prediction5 = m_KF5.predict();
                            Mat prediction6 = m_KF6.predict();
                        ///////////////////////////////遇到目标在两帧间间突然丢失的情况////////////////////
                            Moments mu1;
                            Mat cur_win1;
                            Size patchsize1;
                            patchsize1.height = trackWindow.height;
                            patchsize1.width = trackWindow.width;
                            Point2f centerpatch1;
                            centerpatch1.x = trackWindow.x + trackWindow.width * 0.5f;
                            centerpatch1.y = trackWindow.y + trackWindow.height * 0.5f;
                            getRectSubPix(backproj, patchsize1, centerpatch1, cur_win1);///把截取图像中需要的区域存入矩阵cur_win
                            mu1 = moments(cur_win1);   ///计算0阶矩，1阶矩

                            ///计算搜索框内的0阶矩
                            Moments mu2;
                            Mat cur_win2;
                            Size patchsize2;
                            patchsize2.height = trackWindow2.height;
                            patchsize2.width = trackWindow2.width;
                            Point2f centerpatch2;
                            centerpatch2.x = trackWindow2.x + trackWindow2.width * 0.5f;
                            centerpatch2.y = trackWindow2.y + trackWindow2.height * 0.5f;
                            getRectSubPix(backproj2, patchsize2, centerpatch2, cur_win2);///把截取图像中需要的区域存入矩阵cur_win
                            mu2 = moments(cur_win2);   ///计算0阶矩，1阶矩

                            Moments mu3;
                            Mat cur_win3;
                            Size patchsize3;
                            patchsize3.height = trackWindow3.height;
                            patchsize3.width = trackWindow3.width;
                            Point2f centerpatch3;
                            centerpatch3.x = trackWindow3.x + trackWindow3.width * 0.5f;
                            centerpatch3.y = trackWindow3.y + trackWindow3.height * 0.5f;
                            getRectSubPix(backproj3, patchsize3, centerpatch3, cur_win3);///把截取图像中需要的区域存入矩阵cur_win
                            mu3 = moments(cur_win3);   ///计算0阶矩，1阶矩

                            Moments mu4;
                            Mat cur_win4;
                            Size patchsize4;
                            patchsize4.height = trackWindow4.height;
                            patchsize4.width = trackWindow4.width;
                            Point2f centerpatch4;
                            centerpatch4.x = trackWindow4.x + trackWindow4.width * 0.5f;
                            centerpatch4.y = trackWindow4.y + trackWindow4.height * 0.5f;
                            getRectSubPix(backproj4, patchsize4, centerpatch4, cur_win4);///把截取图像中需要的区域存入矩阵cur_win
                            mu4 = moments(cur_win4);   ///计算0阶矩，1阶矩

                            Moments mu5;
                            Mat cur_win5;
                            Size patchsize5;
                            patchsize5.height = trackWindow5.height;
                            patchsize5.width = trackWindow5.width;
                            Point2f centerpatch5;
                            centerpatch5.x = trackWindow5.x + trackWindow5.width * 0.5f;
                            centerpatch5.y = trackWindow5.y + trackWindow5.height * 0.5f;
                            getRectSubPix(backproj5, patchsize5, centerpatch5, cur_win5);///把截取图像中需要的区域存入矩阵cur_win
                            mu5 = moments(cur_win5);   ///计算0阶矩，1阶矩

                            Moments mu6;
                            Mat cur_win6;
                            Size patchsize6;
                            patchsize6.height = trackWindow6.height;
                            patchsize6.width = trackWindow6.width;
                            Point2f centerpatch6;
                            centerpatch6.x = trackWindow6.x + trackWindow6.width * 0.5f;
                            centerpatch6.y = trackWindow6.y + trackWindow6.height * 0.5f;
                            getRectSubPix(backproj6, patchsize6, centerpatch6, cur_win6);///把截取图像中需要的区域存入矩阵cur_win
                            mu6 = moments(cur_win6);   ///计算0阶矩，1阶矩
                            ///////////////////////   目标丢失后如何处理   ///////////////////////
                            if (mu1.m00/255 < (float)standard_window.area()/10) {
                                //ch_x = (float) dequeWindow1[1].x - dequeWindow1[0].x;
                                //ch_y = (float) dequeWindow1[1].y - dequeWindow1[0].y;
                                ch_x = v1_x * dert;
                                ch_y = v1_y * dert;
                                expend_x = (float)0.5*standard_window.width+diff_missing;
                                expend_y = (float)0.5*standard_window.height+diff_missing;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<one_two)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(len12_x) - standard_window.width;
                                    expend_y = sqrt(len12_y) - standard_window.height;
                                }
                                trackWindow.x = trackWindow.x + ch_x - expend_x;
                                trackWindow.y = trackWindow.y + ch_y - expend_y;
                                trackWindow.width += 2*expend_x;
                                trackWindow.height += 2*expend_y;
                            }

                            if (mu2.m00/255 < (float)standard_window.area()/10) {
                               // ch_x = (float) dequeWindow2[1].x - dequeWindow2[0].x;
                               // ch_y = (float) dequeWindow2[1].y - dequeWindow2[0].y;
                                ch_x = v2_x * dert;
                                ch_y = v2_y * dert;
                                expend_x = (float)0.5*standard_window.width+diff_missing;
                                expend_y = (float)0.5*standard_window.height+diff_missing;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<one_two&&sqrt(expend_x*expend_x+expend_y*expend_y)<two_three)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len12_x,len23_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len12_y,len23_y)) - standard_window.height;
                                }
                                trackWindow2.x = trackWindow2.x + ch_x - expend_x;
                                trackWindow2.y = trackWindow2.y + ch_y - expend_y;
                                trackWindow2.width += 2*expend_x;
                                trackWindow2.height += 2*expend_y;
                            }

                            if (mu3.m00/255 < (float)standard_window.area()/10) {
                                //ch_x = (float) dequeWindow3[1].x - dequeWindow3[0].x;
                                //ch_y = (float) dequeWindow3[1].y - dequeWindow3[0].y;
                                ch_x = v3_x * dert;
                                ch_y = v3_y * dert;
                                expend_x = (float)0.5*standard_window.width+diff_missing;
                                expend_y = (float)0.5*standard_window.height+diff_missing;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<two_three&&sqrt(expend_x*expend_x+expend_y*expend_y)<three_four)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len34_x,len23_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len34_y,len23_y)) - standard_window.height;
                                }
                                trackWindow3.x = trackWindow3.x + ch_x  - expend_x;
                                trackWindow3.y = trackWindow3.y + ch_y - expend_y;
                                trackWindow3.width += 2*expend_x;
                                trackWindow3.height += 2*expend_y;

                            }

                            if (mu4.m00/255 < (float)standard_window.area()/10) {
                                //ch_x = (float) dequeWindow4[1].x - dequeWindow4[0].x;
                                //ch_y = (float) dequeWindow4[1].y - dequeWindow4[0].y;
                                ch_x = v4_x * dert;
                                ch_y = v4_y * dert;
                                expend_x = (float)0.5*standard_window.width+diff_missing;
                                expend_y = (float)0.5*standard_window.height+diff_missing;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<four_five&&sqrt(expend_x*expend_x+expend_y*expend_y)<three_four)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len34_x,len45_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len34_y,len45_y)) - standard_window.height;
                                }
                                trackWindow4.x = trackWindow4.x + ch_x - expend_x;
                                trackWindow4.y = trackWindow4.y + ch_y - expend_y;
                                trackWindow4.width += 2*expend_x;
                                trackWindow4.height += 2*expend_y;
                            }

                            if (mu5.m00/255 < (float)standard_window.area()/10) {
                                //ch_x = (float) dequeWindow5[1].x - dequeWindow5[0].x;
                                //ch_y = (float) dequeWindow5[1].y - dequeWindow5[0].y;
                                ch_x = v5_x * dert;
                                ch_y = v5_y * dert;
                                expend_x = (float)0.5*standard_window.width+diff_missing;
                                expend_y = (float)0.5*standard_window.height+diff_missing;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<four_five&&sqrt(expend_x*expend_x+expend_y*expend_y)<five_six)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len56_x,len45_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len56_y,len45_y)) - standard_window.height;
                                }
                                trackWindow5.x = trackWindow5.x + ch_x  - expend_x;
                                trackWindow5.y = trackWindow5.y + ch_y  - expend_y;
                                trackWindow5.width += 2*expend_x;
                                trackWindow5.height += 2*expend_y;
                            }
                            if (mu6.m00/255 < (float)standard_window.area()/10) {

                                //ch_x = (float) dequeWindow6[1].x - dequeWindow6[0].x;
                                //ch_y = (float) dequeWindow6[1].y - dequeWindow6[0].y;
                                ch_x = v6_x * dert;
                                ch_y = v6_y * dert;
                                expend_x = (float)0.5*standard_window.width+diff_missing;
                                expend_y = (float)0.5*standard_window.height+diff_missing;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<five_six)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(len56_x) - standard_window.width;
                                    expend_y = sqrt(len56_y) - standard_window.height;
                                }
                                trackWindow6.x = trackWindow6.x + ch_x - expend_x;
                                trackWindow6.y = trackWindow6.y + ch_y - expend_y;
                                trackWindow6.width += 2*expend_x;
                                trackWindow6.height += 2*expend_y;
                            }
                        ////////////////////利用camshift函数实现目标跟踪////////////////////////
                        RotatedRect trackBox2 = CamShift(backproj, trackWindow,
                                                        TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1)); //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则

                        RotatedRect trackBox22 = CamShift(backproj2, trackWindow2,
                                                         TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));

                        RotatedRect trackBox23 = CamShift(backproj3, trackWindow3,
                                                         TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1)); //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则
                        RotatedRect trackBox24 = CamShift(backproj4, trackWindow4,
                                                          TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));

                        RotatedRect trackBox25 = CamShift(backproj5, trackWindow5,
                                                          TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1)); //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则
                        RotatedRect trackBox26 = CamShift(backproj6, trackWindow6,
                                                          TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
                        /////////////////////////////////////////////////////
//                        dequeWindow1.push_back(trackWindow);
//                        dequeWindow2.push_back(trackWindow2);
//                        dequeWindow3.push_back(trackWindow3);
//                        dequeWindow4.push_back(trackWindow4);
//                        dequeWindow5.push_back(trackWindow5);
//                        dequeWindow6.push_back(trackWindow6);
//                        if(dequeWindow1.size() == 3)
//                        {
//                            dequeWindow1.pop_front();
//                            dequeWindow2.pop_front();
//                            dequeWindow3.pop_front();
//                            dequeWindow4.pop_front();
//                            dequeWindow5.pop_front();
//                            dequeWindow6.pop_front();
//
//                        }
                        ///////////////////////   计算临近点之间的距离  ////////////////////////////
                        if(p<=9) {
                            len12_x = (trackBox2.center.x - trackBox22.center.x) *
                                      (trackBox2.center.x - trackBox22.center.x);
                            len12_y = (trackBox2.center.y - trackBox22.center.y) *
                                      (trackBox2.center.y - trackBox22.center.y);
                            one_two = sqrt(len12_x + len12_y) - MAX(standard_window.width, standard_window.height);

                            len23_x = (trackBox23.center.x - trackBox22.center.x) *
                                      (trackBox23.center.x - trackBox22.center.x);
                            len23_y = (trackBox23.center.y - trackBox22.center.y) *
                                      (trackBox23.center.y - trackBox22.center.y);
                            two_three = sqrt(len23_x + len23_y) - MAX(standard_window.width, standard_window.height);

                            len34_x = (trackBox23.center.x - trackBox24.center.x) *
                                      (trackBox23.center.x - trackBox24.center.x);
                            len34_y = (trackBox23.center.y - trackBox24.center.y) *
                                      (trackBox23.center.y - trackBox24.center.y);
                            three_four = sqrt(len34_x + len34_y) - MAX(standard_window.width, standard_window.height);

                            len45_x = (trackBox25.center.x - trackBox24.center.x) *
                                      (trackBox25.center.x - trackBox24.center.x);
                            len45_y = (trackBox25.center.y - trackBox24.center.y) *
                                      (trackBox25.center.y - trackBox24.center.y);
                            four_five = sqrt(len45_x + len45_y) - MAX(standard_window.width, standard_window.height);

                            len56_x = (trackBox25.center.x - trackBox26.center.x) *
                                      (trackBox25.center.x - trackBox26.center.x);
                            len56_y = (trackBox25.center.y - trackBox26.center.y) *
                                      (trackBox25.center.y - trackBox26.center.y);
                            five_six = sqrt(len56_x + len56_y) - MAX(standard_window.width, standard_window.height);
                        }
                        //////////////////////////////////////////////////////////////////////////////////////////////

                        ////////////////////     卡尔曼修正     /////////////////////////////
                            if (trackWindow.area() / (float) standard_window.area() <k && trackWindow.area() >1)   ///当搜索窗口变得比较小时，将calman的预测值作为calman滤波器的观测值进行修正
                            {
                                m_predict.at<float>(0) = prediction.at<float>(0);
                                m_predict.at<float>(1) = prediction.at<float>(1);
                                m_estimated = m_KF.correct(m_predict);
                                ///放大搜索框
                                prop = trackWindow.area() / (float) standard_window.area();
                                expend_x = (1 - prop)*standard_window.width+diff_premiss;
                                expend_y = (1 - prop)*standard_window.height+diff_premiss;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<one_two)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(len12_x) - standard_window.width;
                                    expend_y = sqrt(len12_y) - standard_window.height;
                                }
                                trackWindow.x = trackWindow.x - expend_x;
                                trackWindow.y = trackWindow.y - expend_y;
                                trackWindow.width += 2*expend_x;
                                trackWindow.height += 2*expend_y;

                            } else {
                                m_measured.at<float>(0) = trackBox2.center.x;
                                m_measured.at<float>(1) = trackBox2.center.y;
                                m_estimated = m_KF.correct(m_measured);

                            }

                            trackBox2.center.x = m_estimated.at<float>(0);
                            trackBox2.center.y = m_estimated.at<float>(1);
                            //////////////////将下一帧搜索框的中心改为目标中心////////////////////////
                            trackcenter1_x = trackWindow.x + trackWindow.width * 0.5f;
                            trackcenter1_y = trackWindow.y + trackWindow.height * 0.5f;
                            float move1_x;
                            float move1_y;
                            move1_x = trackBox2.center.x - trackcenter1_x;
                            move1_y = trackBox2.center.y - trackcenter1_y;
                            trackWindow.x = trackWindow.x + move1_x;
                            trackWindow.y = trackWindow.y + move1_y;
                            ///////////////////////////////////////////////////////////////////////////
                            if (trackWindow2.area() / (float) standard_window.area() <k && trackWindow2.area() >1)
                            {
                                m_predict.at<float>(0) = prediction2.at<float>(0);
                                m_predict.at<float>(1) = prediction2.at<float>(1);
                                m_estimated2 = m_KF2.correct(m_predict);
                                prop = trackWindow2.area() / (float) standard_window.area();
                                expend_x = (1 - prop)*standard_window.width+diff_premiss;
                                expend_y = (1 - prop)*standard_window.height+diff_premiss;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<one_two&&sqrt(expend_x*expend_x+expend_y*expend_y)<two_three)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len12_x,len23_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len12_y,len23_y)) - standard_window.height;
                                }
                                trackWindow2.x = trackWindow2.x - expend_x;
                                trackWindow2.y = trackWindow2.y - expend_y;
                                trackWindow2.width += 2*expend_x;
                                trackWindow2.height += 2*expend_y;

                            } else {
                                m_measured2.at<float>(0) = trackBox22.center.x;
                                m_measured2.at<float>(1) = trackBox22.center.y;
                                m_estimated2 = m_KF2.correct(m_measured2);

                            }

                            trackBox22.center.x = m_estimated2.at<float>(0);
                            trackBox22.center.y = m_estimated2.at<float>(1);
                            //////////////////将下一帧搜索框的中心改为目标中心////////////////////////
                            trackcenter2_x = trackWindow2.x + trackWindow2.width * 0.5f;
                            trackcenter2_y = trackWindow2.y + trackWindow2.height * 0.5f;
                            float move2_x;
                            float move2_y;
                            move2_x = trackBox22.center.x - trackcenter2_x;
                            move2_y = trackBox22.center.y - trackcenter2_y;
                            trackWindow2.x = trackWindow2.x + move2_x;
                            trackWindow2.y = trackWindow2.y + move2_y;
                            ///////////////////////////////////////////////////
                            if (trackWindow3.area() / (float) standard_window.area() <k && trackWindow3.area()>1)
                            {
                                m_predict.at<float>(0) = prediction3.at<float>(0);
                                m_predict.at<float>(1) = prediction3.at<float>(1);
                                m_estimated3 = m_KF3.correct(m_predict);
                                prop = trackWindow3.area() / (float) standard_window.area();
                                expend_x = (1 - prop)*standard_window.width+diff_premiss;
                                expend_y = (1 - prop)*standard_window.height+diff_premiss;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<two_three&&sqrt(expend_x*expend_x+expend_y*expend_y)<three_four)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len34_x,len23_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len34_y,len23_y)) - standard_window.height;
                                }
                                trackWindow3.x = trackWindow3.x - expend_x;
                                trackWindow3.y = trackWindow3.y - expend_y;
                                trackWindow3.height += 2*expend_y;    ///增大搜索框
                                trackWindow3.width += 2*expend_x;

                            } else {
                                m_measured3.at<float>(0) = trackBox23.center.x;
                                m_measured3.at<float>(1) = trackBox23.center.y;
                                m_estimated3 = m_KF3.correct(m_measured3);

                            }

                            trackBox23.center.x = m_estimated3.at<float>(0);
                            trackBox23.center.y = m_estimated3.at<float>(1);
                            //////////////////将下一帧搜索框的中心改为目标中心////////////////////////
                            trackcenter3_x = trackWindow3.x + trackWindow3.width * 0.5f;
                            trackcenter3_y = trackWindow3.y + trackWindow3.height * 0.5f;
                            float move3_x;
                            float move3_y;
                            move3_x = trackBox23.center.x - trackcenter3_x;
                            move3_y = trackBox23.center.y - trackcenter3_y;
                            trackWindow3.x = trackWindow3.x + move3_x;
                            trackWindow3.y = trackWindow3.y + move3_y;
                            ///////////////////////////////////////////////////
                            if (trackWindow4.area() / (float) standard_window.area() < k && trackWindow4.area()  > 1)
                            {
                                m_predict.at<float>(0) = prediction4.at<float>(0);
                                m_predict.at<float>(1) = prediction4.at<float>(1);
                                m_estimated4 = m_KF4.correct(m_predict);
                                prop = trackWindow4.area() / (float) standard_window.area();
                                expend_x = (1 - prop)*standard_window.width+diff_premiss;
                                expend_y = (1 - prop)*standard_window.height+diff_premiss;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<four_five&&sqrt(expend_x*expend_x+expend_y*expend_y)<three_four)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len34_x,len45_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len34_y,len45_y)) - standard_window.height;
                                }
                                trackWindow4.x = trackWindow4.x - expend_x;
                                trackWindow4.y = trackWindow4.y - expend_y;
                                trackWindow4.height += 2*expend_y;    ///增大搜索框
                                trackWindow4.width += 2*expend_x;

                            } else {
                                m_measured4.at<float>(0) = trackBox24.center.x;
                                m_measured4.at<float>(1) = trackBox24.center.y;
                                m_estimated4 = m_KF4.correct(m_measured4);

                            }

                            trackBox24.center.x = m_estimated4.at<float>(0);
                            trackBox24.center.y = m_estimated4.at<float>(1);
                            trackcenter4_x = trackWindow4.x + trackWindow4.width * 0.5f;
                            trackcenter4_y = trackWindow4.y + trackWindow4.height * 0.5f;
                            float move4_x;
                            float move4_y;
                            move4_x = trackBox24.center.x - trackcenter4_x;
                            move4_y = trackBox24.center.y - trackcenter4_y;
                            trackWindow4.x = trackWindow4.x + move4_x;
                            trackWindow4.y = trackWindow4.y + move4_y;

                            if (trackWindow5.area() / (float) standard_window.area() <k && trackWindow5.area() >1)
                            {
                                m_predict.at<float>(0) = prediction5.at<float>(0);
                                m_predict.at<float>(1) = prediction5.at<float>(1);
                                m_estimated5 = m_KF5.correct(m_predict);
                                prop = trackWindow5.area() / (float) standard_window.area();
                                expend_x = (1 - prop)*standard_window.width+diff_premiss;
                                expend_y = (1 - prop)*standard_window.height+diff_premiss;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<four_five&&sqrt(expend_x*expend_x+expend_y*expend_y)<five_six)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(MIN(len56_x,len45_x)) - standard_window.width;
                                    expend_y = sqrt(MIN(len56_y,len45_y)) - standard_window.height;
                                }
                                trackWindow5.x = trackWindow5.x - expend_x;
                                trackWindow5.y = trackWindow5.y - expend_y;
                                trackWindow5.height += 2*expend_y;    ///增大搜索框
                                trackWindow5.width += 2*expend_x;

                            } else {
                                m_measured5.at<float>(0) = trackBox25.center.x;
                                m_measured5.at<float>(1) = trackBox25.center.y;
                                m_estimated5 = m_KF5.correct(m_measured5);

                            }

                            trackBox25.center.x = m_estimated5.at<float>(0);
                            trackBox25.center.y = m_estimated5.at<float>(1);
                            //////////////////将下一帧搜索框的中心改为目标中心////////////////////////
                            trackcenter5_x = trackWindow5.x + trackWindow5.width * 0.5f;
                            trackcenter5_y = trackWindow5.y + trackWindow5.height * 0.5f;
                            float move5_x;
                            float move5_y;
                            move5_x = trackBox25.center.x - trackcenter5_x;
                            move5_y = trackBox25.center.y - trackcenter5_y;
                            trackWindow5.x = trackWindow5.x + move5_x;
                            trackWindow5.y = trackWindow5.y + move5_y;
                            ///////////////////////////////////////////////////
                            if (trackWindow6.area() / (float) standard_window.area() <k && trackWindow6.area()>1)
                            {
                                m_predict.at<float>(0) = prediction6.at<float>(0);
                                m_predict.at<float>(1) = prediction6.at<float>(1);
                                m_estimated6 = m_KF6.correct(m_predict);
                                prop = trackWindow6.area() / (float) standard_window.area();
                                expend_x = (1 - prop)*standard_window.width+diff_premiss;
                                expend_y = (1 - prop)*standard_window.height+diff_premiss;
                                if(sqrt(expend_x*expend_x+expend_y*expend_y)<five_six)
                                {
                                    expend_x = expend_x;
                                    expend_y = expend_y;
                                }
                                else
                                {
                                    expend_x = sqrt(len56_x) - standard_window.width;
                                    expend_y = sqrt(len56_y) - standard_window.height;
                                }
                                trackWindow6.x = trackWindow6.x - expend_x;
                                trackWindow6.y = trackWindow6.y - expend_y;
                                trackWindow6.height += 2*expend_y;    ///增大搜索框
                                trackWindow6.width += 2*expend_x;

                            } else {
                                m_measured6.at<float>(0) = trackBox26.center.x;
                                m_measured6.at<float>(1) = trackBox26.center.y;
                                m_estimated6 = m_KF6.correct(m_measured6);

                            }

                            trackBox26.center.x = m_estimated6.at<float>(0);
                            trackBox26.center.y = m_estimated6.at<float>(1);
                            //////////////////将下一帧搜索框的中心改为目标中心////////////////////////
                            trackcenter6_x = trackWindow6.x + trackWindow6.width * 0.5f;
                            trackcenter6_y = trackWindow6.y + trackWindow6.height * 0.5f;
                            float move6_x;
                            float move6_y;
                            move6_x = trackBox26.center.x - trackcenter6_x;
                            move6_y = trackBox26.center.y - trackcenter6_y;
                            trackWindow6.x = trackWindow6.x + move6_x;
                            trackWindow6.y = trackWindow6.y + move6_y;

                            ////////////////////   所有目标运动的速度   /////////////////
                            v1_x = m_KF.statePost.at<float>(2);
                            v1_y = m_KF.statePost.at<float>(3);
                            v2_x = m_KF2.statePost.at<float>(2);
                            v2_y = m_KF2.statePost.at<float>(3);
                            v3_x = m_KF3.statePost.at<float>(2);
                            v3_y = m_KF3.statePost.at<float>(3);
                            v4_x = m_KF4.statePost.at<float>(2);
                            v4_y = m_KF4.statePost.at<float>(3);
                            v5_x = m_KF5.statePost.at<float>(2);
                            v5_y = m_KF5.statePost.at<float>(3);
                            v6_x = m_KF6.statePost.at<float>(2);
                            v6_y = m_KF6.statePost.at<float>(3);
                        ///////////////////////////////////////////////////
                            cout<<"center1:"<<trackBox2.center<<endl;
                            cout<<"center2:"<<trackBox22.center<<endl;
                            cout<<"center3:"<<trackBox23.center<<endl;
                            cout<<"center4:"<<trackBox24.center<<endl;
                            cout<<"center5:"<<trackBox25.center<<endl;
                            cout<<"center6:"<<trackBox26.center<<endl;
                            
                            		       //将每个点的质心位置数据存储在一个数组中，并输出到文件中

                            center_x[0]=trackBox2.center.x;
                            center_y[0]=trackBox2.center.y;
                            center_x[1]=trackBox22.center.x;
                            center_y[1]=trackBox22.center.y;
                            center_x[2]=trackBox23.center.x;
                            center_y[2]=trackBox23.center.y;
                            center_x[3]=trackBox24.center.x;
                            center_y[3]=trackBox24.center.y;
                            center_x[4]=trackBox25.center.x;
                            center_y[4]=trackBox25.center.y;
                            center_x[5]=trackBox26.center.x;
                            center_y[5]=trackBox26.center.y;


			                writeLog_outfileCenterPosition(center_x,center_y);
                        //////////////记录中心坐标////////////////////////
                             a = trackBox2.center;
                             b = trackBox22.center;
                             c = trackBox23.center;
                             d = trackBox24.center;
                             e = trackBox25.center;
                             f = trackBox26.center;
                         ///////////////////坐标转化为关节旋转角度///////////////////////
                         ///髋关节旋转角
                         r1 = atan2((b.x - a.x),(b.y - a.y));
                         r1 = (float)(180*r1/CV_PI);
                         ///膝关节旋转角
                         r2 = vector_angle(a,b,c,d);
                         ///踝关节旋转角
                         r3 = vector_angle(c,d,e,f);

                         p ++;

                        if(p>9 && p<15)
                        {
                          r1_err += r1;   ///记录第一帧的角度误差
                          r2_err += r2;
                          r3_err += r3;
                          width_stan += trackWindow.width;
                          high_stan += trackWindow.height;
                        }
                        if(p == 15)
                        {
                            r1_err = r1_err/5;
                            r2_err = r2_err/5;
                            r3_err = r3_err/5;
                            standard_window.width = width_stan /5;
                            standard_window.height = high_stan /5;

                        }

                        ////////////////////  矫正角度  /////////////////
                        if(p>50)
                        {
                            r1 = r1 - r1_err;
                            r2 = r2 - r2_err;
                            r3 = r3 - r3_err;
                        }

                        ////////////////////////////////////////////////
                        ///////////   显示每次循环时间   /////////////////
                            if (time_count == 0) {

                                start_time = get_wall_time();

                            }

                            if (time_count == 1) {
                                time_count = -1;
                                end_time = get_wall_time();
                                cout << "数据间隔时间****************************: " << end_time - start_time << "s" << endl;
                                start_time = 0.0;
                                end_time = 0.0;

                            }
                        ///////////////////////////////////////////////
                         cout<<"angle1: "<<r1<<endl;
                         cout<<"angle2: "<<r2<<endl;
                         cout<<"angle3: "<<r3<<endl;

                        writeLog(r1,1);    //存储数据
                        writeLog_2(r2,2);    //存储数据
                        writeLog_3(r3,3);    //存储数据

                        time_count ++;

                        /////////////////////////目标逐渐丢失后如何处理////////////////////////////////////
                       if (trackWindow.area() <= 1)
                        {
                            cout<<"trackwindow1.area<=1"<<endl;
                            expend_x = sqrt(len12_x);
                            expend_y = sqrt(len12_y);
                            trackWindow = Rect(trackWindow.x - expend_x, trackWindow.y - expend_y,
                                                trackWindow.x + 2*expend_x, trackWindow.y + 2*expend_y) ;
                        }

                       if ((float )trackWindow2.area() <= 1)
                        {
                            cout<<"trackwindow2.area<=1"<<endl;
                            expend_x = sqrt(MIN(len12_x,len23_x));
                            expend_y = sqrt(MIN(len12_y,len23_y));
                            trackWindow2 = Rect(trackWindow2.x - expend_x, trackWindow2.y - expend_y,
                                                trackWindow2.x + 2*expend_x, trackWindow2.y + r2*expend_y);
                        }

                       if ((float )trackWindow3.area() <= 1)
                        {
                            cout<<"trackwindow3.area<=1"<<endl;
                            expend_x = sqrt(MIN(len34_x,len23_x));
                            expend_y = sqrt(MIN(len34_y,len23_y));
                            trackWindow3 = Rect(trackWindow3.x - expend_x, trackWindow3.y - expend_y,
                                               trackWindow3.x + 2*expend_x, trackWindow3.y + 2*expend_y) ;
                        }

                         if ((float )trackWindow4.area() <= 1)
                        {
                            cout<<"trackwindow4.area<=1"<<endl;
                            expend_x = sqrt(MIN(len34_x,len45_x));
                            expend_y = sqrt(MIN(len34_y,len45_y));
                            trackWindow4 = Rect(trackWindow4.x - expend_x, trackWindow4.y - expend_y,
                                                trackWindow4.x + 2*expend_x, trackWindow4.y + 2*expend_y) ;
                        }
                         if ((float )trackWindow5.area() <= 1)
                        {
                            cout<<"trackwindow5.area<=1"<<endl;
                            expend_x = sqrt(MIN(len56_x,len45_x));
                            expend_y = sqrt(MIN(len56_y,len45_y));
                            trackWindow5 = Rect(trackWindow5.x - expend_x, trackWindow5.y - expend_y,
                                                trackWindow5.x + 2*expend_x, trackWindow5.y + 2*expend_y);
                        }
                         if ((float )trackWindow6.area() <= 1)
                        {
                            cout<<"trackwindow6.area<=1"<<endl;
                            expend_x = sqrt(len56_x);
                            expend_y = sqrt(len56_y);
                            trackWindow6 = Rect(trackWindow6.x - expend_x , trackWindow6.y - expend_y,
                                                trackWindow6.x + 2*expend_x , trackWindow6.y + 2*expend_y) ;
                        }

                        /////////////////////////////////////////////////////////////////////////////////////////////

                    ///////////////////////画出可视框///////////////////////////////////////////////
                        ellipse(image, trackBox2, Scalar(0, 0, 255), 3, LINE_AA);
                        ellipse(image, trackBox22, Scalar(0, 0, 255), 3, LINE_AA);//跟踪的时候以椭圆为代表目标
                        ellipse(image, trackBox23, Scalar(0, 0, 255), 3, LINE_AA);
                        ellipse(image, trackBox24, Scalar(0, 0, 255), 3, LINE_AA);
                        ellipse(image, trackBox25, Scalar(0, 0, 255), 3, LINE_AA);
                        ellipse(image, trackBox26, Scalar(0, 0, 255), 3, LINE_AA);

                        rectangle (image,trackWindow2,Scalar(255, 0, 0),2);
                        rectangle (image,trackWindow,Scalar(255, 0, 0),2);
                        rectangle (image,trackWindow3,Scalar(255, 0, 0),2);
                        rectangle (image,trackWindow4,Scalar(255, 0, 0),2);
                        rectangle (image,trackWindow5,Scalar(255, 0, 0),2);
                        rectangle (image,trackWindow6,Scalar(255, 0, 0),2);

                        ///////////////////////画出点与点之间的连线///////////////////////////////////////////////
                        Point b_ext,c_ext,d_ext,e_ext;
                        double lon_x, lon_y;
                        lon_x = b.x - a.x;
                        lon_y = b.y - a.y;
                        b_ext.x = b.x + lon_x;
                        b_ext.y = b.y + lon_y;
                        lon_x = d.x - c.x;
                        lon_y = d.y - c.y;
                        c_ext.x = c.x - lon_x;
                        c_ext.y = c.y - lon_y;
                        d_ext.x = d.x + lon_x;
                        d_ext.y = d.y + lon_y;
                        lon_x = f.x - e.x;
                        lon_y = f.y - e.y;
                        e_ext.x = e.x - lon_x;
                        e_ext.y = e.y - lon_y;

                        arrowedLine(image, a, b_ext, Scalar(255, 0, 0),2);     //显示坐标点之间的连线
                        arrowedLine(image, c_ext, d_ext, Scalar(0, 255, 0),2);     //显示坐标点之间的连线
                        arrowedLine(image, e_ext, f, Scalar(0, 0, 255),2);     //显示坐标点之间的连线

                        string imagestore = "/home/xrf/catkin_ws/picture/";
                        num++;
                        string str;
                        str = str + to_string(num);
                        imagestore = imagestore + str + ".jpg";
                        cout << imagestore << endl;
                        imwrite(imagestore,image);

                    }
                }
                else if (trackObject < 0)
                    paused = false;
                //////////////////////////可视化选择窗口///////////////////////////////////////
                if (selectObject && selection.width > 0 && selection.height > 0) {
                    Mat roi(image, selection);
                    bitwise_not(roi, roi);// bitwise_not为将每一个bit位取反
                }
                if (selectObject && selection2.width > 0 && selection2.height > 0) {
                    Mat roi2(image, selection2);
                    bitwise_not(roi2, roi2);// bitwise_not为将每一个bit位取反
                }
                if (selectObject && selection3.width > 0 && selection3.height > 0) {
                    Mat roi3(image, selection3);
                    bitwise_not(roi3, roi3);// bitwise_not为将每一个bit位取反
                }
                if (selectObject && selection4.width > 0 && selection4.height > 0) {
                    Mat roi4(image, selection4);
                    bitwise_not(roi4, roi4);// bitwise_not为将每一个bit位取反
                }
                if (selectObject && selection5.width > 0 && selection5.height > 0) {
                    Mat roi5(image, selection5);
                    bitwise_not(roi5, roi5);// bitwise_not为将每一个bit位取反
                }
                if (selectObject && selection6.width > 0 && selection6.height > 0) {
                    Mat roi6(image, selection6);
                    bitwise_not(roi6, roi6);// bitwise_not为将每一个bit位取反
                }
                ///////////////////////////////////////////////////////////////////////////////////////
                imshow("CamShift Demo", image);

                vector<Mat>Images_histimg(6);//模板类vector，用于放置4个类型为Mat的元素，即四张图片
                Images_histimg[0] = histimg;
                Images_histimg[1] = histimg2;
                Images_histimg[2] = histimg3;
                Images_histimg[3] = histimg4;
                Images_histimg[4] = histimg5;
                Images_histimg[5] = histimg6;

                imshowMany("Histogram",Images_histimg,200);    ///多画面直方图显示
                cv::waitKey(3);
                //image_pub_.publish(cv_ptr->toImageMsg());   //发布cv_ptr->image图像数据
                }
    }

        /*****************************  END ********************************/

    };

int main(int argc, char** argv)
{

    fstream file("/home/xrf/catkin_ws/src/cam_show/data1.txt", ios::out); //清空文件内容
    fstream file2("/home/xrf/catkin_ws/src/cam_show/data2.txt", ios::out);
    fstream file3("/home/xrf/catkin_ws/src/cam_show/data3.txt", ios::out);
    fstream file4("/home/xrf/catkin_ws/src/cam_show/CenterPosition.txt", ios::out);

    char* path = "/home/xrf/catkin_ws/picture";//删除文件夹里面保存的图片
    DeleteFile(path);

    cv::namedWindow("CamShift Demo", 0);
    cv::namedWindow("反向投影", 0);
    cv::namedWindow("Histogram", 0);
    setMouseCallback("CamShift Demo", onMouse, 0);//鼠标响应事件
    createTrackbar("Vmin", "CamShift Demo", &vmin, 256, 0);//createTrackbar函数的功能是在对应的窗口创建滑动条，滑动条Vmin,vmin表示滑动条的值，最大为256
    createTrackbar("Vmax", "CamShift Demo", &vmax, 256, 0);//最后一个参数为0代表没有调用滑动拖动的响应函数
    createTrackbar("Smin", "CamShift Demo", &smin, 256, 0);//vmin（亮度的最小值）,vmax（亮度的最大值）,smin（饱和度的最小值）初始值分别为10，256，30

    ros::init(argc, argv, "object_capture");

    ImageConvertor ic;

    ros::spin();            //等待订阅的话题

    return 0;
}
