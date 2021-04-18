#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "../include/cam_show/init_one.h"
#include "geometry_msgs/Point.h"
#include <ctime>
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include <dirent.h>

float center_x[1],center_y[1];
using namespace std;
namespace enc = sensor_msgs::image_encodings;
/*准备再次发布的图像显示到本窗口*/
//static const char OUT_WINDOW[] = "Image Out";
/*读取订阅的图像并显示到本窗口*/
//static std::string IN_WINDOW = "Image In";

int time_count = 0;
double start_time;
double end_time;

cv::KalmanFilter m_KF;  ///卡尔曼滤波器  //NOLINT
Mat m_measured;      ///测量值  //NOLINT
Mat m_estimated;     ///卡尔曼修正后的数值  //NOLINT

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

/////////////////////////////////////////////////////////////////////////////////////

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
    ros::NodeHandle nh2_;
  //  ros::Publisher pub_data = nh_.advertise<geometry_msgs::Point>("pre_data",1);
    image_transport::ImageTransport it2_;
    image_transport::Subscriber image_sub2_;
  //  image_transport::Publisher image_pub_;

public:
    ImageConvertor():it2_(nh2_){
        /*发布主题out*/
        //image_pub_  = it_.advertise("node_b", 1);
        /*订阅主题camera/image*/
        image_sub2_ = it2_.subscribe("/camera/color/image_raw", 1, &ImageConvertor::ImageCb, this);
        ////////////////////////卡尔曼滤波器初始化///////////////////////
        {
            float dert = 33;       //////////采集时间
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
        }
        }

    ~ImageConvertor()
    {
        cv::destroyWindow("CamShift Demo");
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
                        inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
                                Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                        int ch[] = { 0, 0};
                        hue.create(hsv.size(), hsv.depth());
                        mixChannels(&hsv, 1, &hue, 1, ch, 1);
                        if (trackObject < 0)
                        {
                            trackObject = 1;
                            /////////////////得到ROI矩形区域的颜色直方图/////////////////
                            Mat roi(hue, selection), maskroi(mask, selection);
                            calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                            trackWindow = selection;
                            /////////////初始化calman的初始位置为selection////////////
                                m_KF.statePost = (cv::Mat_<float>(4, 1) << selection.x, selection.y, 0, 0);
                            ////////////////////绘制颜色直方图////////////////////////
                            histimg = Scalar::all(0);
                            int binW = histimg.cols / hsize;//histing是一个200*300的矩阵，hsize应该是每一个bin的宽度，也就是histing矩阵能分出几个bin出来
                            Mat buf(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
                            for (int i = 0; i < hsize; i++)//saturate_case函数为从一个初始类型准确变换到另一个初始类型
                            {
                                buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);

                            }
                            cvtColor(buf, buf, CV_HSV2BGR);//hsv又转换成bgr

                            for (int i = 0; i < hsize; i++)
                            {
                                int val = saturate_cast<int>(hist.at<float>(i)*(float )histimg.rows / 255);//at函数为返回一个指定数组元素的参考值

                                //在一幅输入图像上画一个简单抽的矩形，指定左上角和右下角，并定义颜色，大小，线型等
                                rectangle(histimg, Point(i*binW, histimg.rows),
                                          Point((i + 1)*binW, histimg.rows - val),
                                          Scalar(buf.at<Vec3b>(i)), -1, 8);
                            }
                        }

                        ////////////////////反向投影 ////////////////////////
                        calcBackProject(&hue, 1, 0, hist, backproj, &phranges);  //反向投影（概率分布图）函数
                        backproj &= mask;
                        /////////////////单通道转化为3通道图像////////////////////////
                        backprojj = convertTo3Channels(backproj);
                        vector<Mat>Images(1);
                        Images[0] = backprojj;
                        /////////////////////////    卡尔曼预测        ////////////////////////////
                        Mat prediction = m_KF.predict();
                        cout<<"xxxxxxxxxxxxxxxx:  "<<prediction.at<float>(2)<<"      "<<"yyyyyyyyyyy:   "<<prediction.at<float>(3)<<endl;
                        ////////////////////利用camshift函数实现目标跟踪////////////////////////
                        RotatedRect trackBox2 = CamShift(backproj, trackWindow,
                                                        TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1)); //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则

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
                            ////////////////////  目标运动的速度   /////////////////
                            v1_x = m_KF.statePost.at<float>(2);
                            v1_y = m_KF.statePost.at<float>(3);
                            ///////////////////////////////////////////////////
                            cout<<"center1:"<<trackBox2.center<<endl;
                            cout<<"v1_x:  "<<v1_x<<"    "<<"v1_y:  "<<v1_y<<endl;
                            //////////////////////////////////////////////
			                center_x[0]=trackBox2.center.x;
			                center_y[0]=trackBox2.center.y;

                            if(p>9 && p<15)
                            {
                              width_stan += trackWindow.width;
                              high_stan += trackWindow.height;
                            }
                            if(p == 15)
                            {
                                standard_window.width = width_stan /5;
                                standard_window.height = high_stan /5;

                            }

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

                        time_count ++;
                        /////////////////////////目标逐渐丢失后如何处理////////////////////////////////////
                       if (trackWindow.area() <= 1)
                        {
                            expend_x = 100;
                            expend_y = 100;
                            trackWindow = Rect(trackWindow.x - expend_x, trackWindow.y - expend_y,
                                                trackWindow.x + 2*expend_x, trackWindow.y + 2*expend_y) ;
                        }

                        ellipse(image, trackBox2, Scalar(0, 0, 255), 3, LINE_AA);
                        rectangle (image,trackWindow,Scalar(255, 0, 0),2);

                    }
                }
                else if (trackObject < 0)
                    paused = false;
                //////////////////////////可视化选择窗口///////////////////////////////////////
                if (selectObject && selection.width > 0 && selection.height > 0) {
                    Mat roi(image, selection);
                    bitwise_not(roi, roi);// bitwise_not为将每一个bit位取反
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                imshow("CamShift Demo", image);

                vector<Mat>Images_histimg(1);//模板类vector，用于放置4个类型为Mat的元素，即四张图片
                Images_histimg[0] = histimg;
                cv::waitKey(3);
                //image_pub_.publish(cv_ptr->toImageMsg());   //发布cv_ptr->image图像数据
                }
    }

        /*****************************  END ********************************/

    };

int main(int argc, char** argv)
{
    cv::namedWindow("CamShift Demo", 0);
    setMouseCallback("CamShift Demo", onMouse, 0);//鼠标响应事件
    createTrackbar("Vmin", "CamShift Demo", &vmin, 256, 0);//createTrackbar函数的功能是在对应的窗口创建滑动条，滑动条Vmin,vmin表示滑动条的值，最大为256
    createTrackbar("Vmax", "CamShift Demo", &vmax, 256, 0);//最后一个参数为0代表没有调用滑动拖动的响应函数
    createTrackbar("Smin", "CamShift Demo", &smin, 256, 0);//vmin（亮度的最小值）,vmax（亮度的最大值）,smin（饱和度的最小值）初始值分别为10，256，30

    ros::init(argc, argv, "one");

    ImageConvertor ic;

    ros::spin();            //等待订阅的话题

    return 0;
}
