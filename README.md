# UAS-Project-STM32
STM32端无人船程序，配合Raspi上位机端程序使用
# 说明
* 基于树莓派与STM32的无人船控制系统/ STM32型号为ZET6，由于使用了定时器8，因此必须使用大容量产品。
* 使用Keil5编译
* 树莓派使用Python脚本在后台以进程的形式运行
## 文件格式：
* Periphrals文件夹存储STM32的外设硬件部分代码
* Software文件夹存储STM32软件部分代码，即PID算法等
* CONFIGURATION.h头文件用于设置、条件编译等
* initialize.c为STM32初始化时运行的代码
## 功能(配合上位机)
* 采集GPS坐标
* 输入GPS坐标，自主导航。（输入坐标的方法有：Google Earth采点与实地采点）
# 开发者的话
* 正在将此上位机程序移植至ROS平台,连接：https://github.com/matreshka15/uas-raspi-ros
* 如有意向共同开发，请联系作者
* Estello.club-- A self-supported newly-born geek club
* Email:8523429@qq.com
