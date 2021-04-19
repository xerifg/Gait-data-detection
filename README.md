---

---

# Gait-data-detection
Gait data detection based on improved *<u>camshif</u>*t algorithm

In this project. l used improved <u>*camshift*</u> algorithm and added a *<u>kalman</u>* to track six targets precisely. The test found that you even can run ,and the targets won't miss in the future.

Then l will calculate three angles through the six targets coordinates in the program, they are the hip, knee and ankle joints separately.

# Development Environment

1. **Ubuntu 18.04**
2. **ROS Melodic**
3. **Realsense SDK 2.44**
4. **D400 series(D435 is used in this project)**

# Start

## Prerequisites

Download the source codes to your computer

```c++
git clone https://github.com/xerifg/Gait-data-detection.git
```



### step 1 Compilation Workspace

```c++
cd Gait-data-detection

catkin_make
```

### step 2 Runing 

```c++
cd src

cd cam_show

roslaunch begin.launch
```

### step 3 Testing

**select six object via mouse**

then the project will run,and you can see some data in your terminal

if you want to stop, you can `ctrl + c`

you will find four txt files:data1.txt(the first angle), data2.txt(the secend angle), data3.txt(the third angle), centerposition.txt(six object x-y location coordinates ).

### Note:

You **must** change five paths to yours in **object_capture.cpp**

```c++
125: outfile.open("/home/xrf/catkin_ws/src/cam_show/data1.txt", ios::app);

138: outfile2.open("/home/xrf/catkin_ws/src/cam_show/data2.txt", ios::app);

151: outfile3.open("/home/xrf/catkin_ws/src/cam_show/data3.txt", ios::app);

165: outfileCenterPosition.open("/home/xrf/catkin_ws/src/cam_show/CenterPosition.txt", ios::app);

1241: string imagestore = "/home/xrf/catkin_ws/picture/";
```

You need to create a folder named `pricture` if you want to save all process pictures.

```c++
 cd Gait-data-detection

mkdir picture
```

You should preferably use a clean background (no color similar to the color of the tracking target) in order to make the experiment accurate. This is really important.

Then you preferably  affix the targets to your leg firmly, like in this picture below, **importantly,**because of a bug in my codes,you should have a Initial angle like the picture below around your knee .

# Experimental results

![image](https://github.com/xerifg/Gait-data-detection/tree/master/Results/result.jpg)