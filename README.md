# UAS-Project-STM32
下位机STM32端无人船/无人车程序，配合树莓派、英伟达Jetson Nano等上位机端程序使用
# 说明
* STM32型号为ZET6，由于使用了定时器8，因此必须使用大容量产品。
* 上位机负责和地面站通信，路线规划等等，可以移植到ROS上。  
下位机就用比较简单的架构，大体上就是把姿态解算、电机控制等任务分5，10，50，100Hz的频率运行。
* 树莓派使用Python脚本在后台以进程的形式运行（如果使用的是非ROS版上位机程序的话）
* 建议使用非ROS版上位机程序，原因：开发环境容易配置（只需要Python），并且稳定性好。
* [上位机端非ROS版Github地址](https://github.com/matreshka15/raspberry-pi-USV-program)
* [上位机端ROS版Github地址](https://github.com/matreshka15/ROS-based-unmanned-vehicle-project)
* [姿态解算算法的解释与实机演示视频](https://zhuanlan.zhihu.com/p/82973264)
## 文件格式：
* Periphrals文件夹存储STM32的外设硬件部分代码
* Software文件夹存储STM32软件部分代码，即PID算法等
* CONFIGURATION.h头文件用于设置、条件编译等
* initialize.c为STM32初始化时运行的代码
## 功能(配合上位机)
* 语音提示GPS连接情况、机体运行情况等。（只需要给上位机接一个小音箱）
* 使用遥控器控制
* 采集GPS坐标：此时在使用遥控器控制的同时，上位机在后台记录当前GPS坐标点
* 输入GPS坐标，自主导航。（输入坐标的方法可用Google Earth采点或使用之前采集的GPS坐标）
# 开发者的话
* 如有意向共同开发，请联系作者
* Email:8523429@qq.com
