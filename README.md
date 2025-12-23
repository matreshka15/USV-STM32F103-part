# UAS-Project-STM32
下位机STM32端无人船/无人车程序，配合树莓派、英伟达Jetson Nano等上位机端程序使用

## 亮点概要：
1. **系统架构方面**：
   * STM32采用类操作系统式框架，任务可分不同频率被调用。优点是该框架可以提高系统灵活性，降低系统耦合性。
   * 下位机工程可选择编译，只需要改动一个宏定义，系统即可进行相应改变，自动选择传感器元件。主要应用于对GY901与MPU6050+HML5883两套惯性测量装置同时兼容，无缝切换。
   * 传感器上电自动自检，自动校准，保障数据精度
   * GPS模块采用先进的UBX协议，系统上电前自动配置GPS输出必要信息，滤除不必要信息，大幅提高数据处理效率

2. **系统核心方面**：
   * 上、下位机内部数据缓冲区采用先进的环形缓冲区，大幅降低数据高速传输时的丢包率，提高上下位机对数据处理速度
   * STM32采用有限状态机设计，可通过遥控器进入不同状态，执行不同操作，提高系统灵活性
   * 采用先进的Madgwick梯度下降姿态解算算法，较以前的卡尔曼滤波方法大幅降低了下位机对数据迭代的次数，采用此方法仅需10Hz的采样速度即可收敛至当前载具的姿态
   * GPS模块由于需要实时接收大量数据，严重占用MCU任务处理时间，因此加入了DMA。并且在工程中配置为DMA与中断处理两种可选方式选择编译，提高了效率、灵活性

3. **功能拓展方面**：
   * 可设置的载具模型(汽车模型、海上模型)可最优化使用GPS输出数据，GPS输出数据将根据所选模型进行适配
   * 可在Google Map上标点直接导出为载具轨迹

## 硬件配置
### 核心控制器
* STM32型号：STM32F103ZET6（必须使用大容量产品，因为使用了定时器8）
* 时钟频率：72MHz
* 开发库：STM32标准库（Standard Peripheral Library），**不基于STM32 HAL库**

### 传感器配置
* GPS模块：Ublox M8（可选自带HMC5883磁力计的版本）
* 姿态传感器：MPU6050 + HMC5883L（或GY901）
* 遥控器：WFR07/WFT07
* 电机驱动：有刷电机驱动模块
* 舵机：用于转向控制

### 车体配置
* 前两个轮子：使用舵机转向
* 后两个轮子：各用一个电机驱动（使用PID控制两电机转速基本相同）

## 软件架构
### 系统框架
* 采用类操作系统式的任务调度框架
* 任务按不同频率执行：1Hz、5Hz、10Hz、50Hz、100Hz、200Hz
* 有限状态机设计，支持多种运行模式

### 文件夹结构
* **Libraries/**：STM32标准库和CMSIS库
* **User/**：用户应用代码
  * **COMPASS/**：指南针模块驱动
  * **DATAFUSION/**：数据融合算法（姿态解算）
  * **Delay/**：延时函数
  * **EXT/**：外部中断处理
  * **FILTER/**：滤波算法
  * **GPS/**：GPS模块驱动
  * **GeneralFunction/**：通用函数
  * **IIC/**：IIC总线驱动
  * **IMU/**：惯性测量单元驱动
  * **INPUT_CAPTURE/**：输入捕获（遥控器信号处理）
  * **LoopSequence/**：任务调度核心
  * **Motor/**：电机驱动
  * **NRF24L01/**：无线通信模块
  * **PID/**：PID控制算法
  * **PWM/**：PWM输出
  * **PathPlanning/**：路径规划算法
  * **RingBuff/**：环形缓冲区
  * **TD/**：数据传输模块
  * **Usart/**：串口通信
  * **WDG/**：看门狗
  * **WirelessUSART/**：无线串口通信
* **CONFIGURATION.h**：配置头文件，用于设置、条件编译等
* **initialize.c**：STM32初始化时运行的代码

## 配置说明
### CONFIGURATION.h 主要配置项
* **USE_BRUSHED_MOTOR**：是否使用有刷电机（1：使用，0：不使用）
* **USE_USART3_GPS_DMA_RX**：是否使用DMA接收GPS数据（1：使用，0：不使用）
* **USE_USART1_COMM_DMA_TX**：是否使用DMA发送串口通信数据（1：使用，0：不使用）
* **GY901_nMPU6050**：选择IMU模块（1：使用GY901，0：使用MPU6050）
* **MODEL_nVEHICLE_SEA**：选择载具模型（1：海洋模型，0：陆地模型）

### 任务调度频率
* **1Hz**：系统状态更新、看门狗喂狗、LED闪烁
* **5Hz**：数据发送、上位机通信
* **10Hz**：遥控器信号处理、传感器数据采集、系统状态机更新
* **50Hz**：姿态解算（Madgwick算法）
* **100Hz**：预留
* **200Hz**：预留

## 使用方法
### 环境准备
1. 安装Keil MDK-ARM V5或更高版本（推荐V5.30+）
2. 安装STM32F1系列标准库（Standard Peripheral Library）
3. 准备烧录工具（如J-Link、ST-Link或串口下载器）

### 项目导入与配置
1. 将项目文件下载到本地文件夹
2. 打开Keil5，点击"Project" -> "Open Project"，选择Project文件夹下的工程文件（.uvprojx后缀）
3. 打开User/CONFIGURATION.h文件，根据实际硬件配置修改以下关键宏定义：
   - `USE_BRUSHED_MOTOR`：是否使用有刷电机（1：使用，0：不使用）
   - `USE_USART3_GPS_DMA_RX`：是否使用DMA接收GPS数据（1：使用，0：不使用）
   - `GY901_nMPU6050`：选择IMU模块（1：使用GY901，0：使用MPU6050+HMC5883L）
   - `MODEL_nVEHICLE_SEA`：选择载具模型（1：海洋模型，0：陆地模型）

### 编译与烧录
4. 点击Keil工具栏中的"Build"按钮编译工程，确保无错误无警告
5. 编译成功后，在Output文件夹中生成.hex文件
6. 连接烧录工具到STM32开发板
7. 在Keil中配置烧录工具（点击"Flash" -> "Configure Flash Tools"）
8. 点击"Download"按钮将.hex文件下载到STM32中

### 硬件连接与测试
9. 按照【STM32资源使用情况.xlsx】表格，将各传感器和执行器连接到STM32对应引脚
10. 连接电源，系统将自动进行传感器自检和校准
11. 通过上位机软件或遥控器测试系统功能

## 运行模式
1. **手动遥控模式**：通过遥控器直接控制载具运动
2. **自动导航模式**：根据上位机发送的路径点进行自主导航
3. **GPS采点模式**：记录当前GPS坐标，用于后续路径规划
4. **调试模式**：用于系统调试，输出调试信息

## 通信协议
* 上位机与下位机通信：使用自定义协议，通过串口传输
* 数据帧格式：22字节，包含姿态数据、GPS数据、控制指令等
* 通信速率：921600bps

## 上位机程序
* [上位机端非ROS版Github地址](https://github.com/matreshka15/raspberry-pi-USV-program)
* [上位机端ROS版Github地址](https://github.com/matreshka15/ROS-based-unmanned-vehicle-project)

## 开发日志和资料
* [开发无人船过程中参考的传感器手册以及算法资料](https://github.com/matreshka15/unmanned-ship-datasheets)
* [姿态解算算法的解释与实机演示视频](https://zhuanlan.zhihu.com/p/82973264)

## 注意事项
* 确保使用正确的STM32型号（大容量产品，如ZET6）
* 严格按照连线图连接传感器和执行器
* 首次使用时，确保传感器正确校准
* 上位机程序需要与下位机程序版本匹配
* 调试时建议先在调试模式下测试各模块功能

## 开发者的话
* 如有意向共同开发，请联系作者
* Email:matreshka999@icloud.com
* 项目支持链接：[AFDIAN](https://afdian.com/a/hankli)

---

## 从标准库转换到HAL库的方法

本项目当前基于STM32标准库开发。如果您希望将其转换为使用HAL库，可以按照以下步骤进行：

### 1. 环境准备
- 安装最新版本的STM32CubeMX
- 安装STM32CubeF1 HAL库包
- 更新Keil MDK-ARM到支持HAL库的版本

### 2. 创建新的HAL库项目
1. 打开STM32CubeMX，选择芯片型号为STM32F103ZET6
2. 配置时钟树，设置为72MHz（与原项目保持一致）
3. 配置GPIO引脚：根据原项目的【STM32资源使用情况.xlsx】表格，将所有IO口重新配置到HAL库项目中
4. 配置外设：
   - 串口（USART1用于通信，USART3用于GPS）
   - I2C（用于传感器通信）
   - SPI（如果使用NRF24L01）
   - 定时器（TIM8用于PWM输出，其他定时器用于输入捕获等）
   - DMA（用于GPS数据接收等）
5. 生成Keil MDK-ARM项目

### 3. 代码迁移
1. 将原项目中的核心算法和业务逻辑代码迁移到新的HAL库项目中：
   - 姿态解算算法（Madgwick）
   - PID控制算法
   - 路径规划算法
   - 环形缓冲区实现
   - 任务调度框架
   - 有限状态机逻辑
2. 替换所有标准库函数调用为对应的HAL库函数：
   - GPIO操作：`GPIO_SetBits()` → `HAL_GPIO_WritePin()`
   - 串口操作：`USART_SendData()` → `HAL_UART_Transmit()`
   - I2C操作：`I2C_Start()` → `HAL_I2C_Master_Transmit()`
   - 定时器操作：`TIM_SetCompare1()` → `__HAL_TIM_SET_COMPARE()`
   - DMA操作：`DMA_Cmd()` → `HAL_DMA_Start()`
3. 重写中断处理函数，使用HAL库的中断回调机制
4. 调整时钟配置和系统初始化代码

### 4. 测试与调试
1. 编译新的HAL库项目，解决编译错误
2. 逐步测试各个模块的功能：
   - 传感器数据采集
   - 姿态解算
   - 电机控制
   - 通信功能
3. 与上位机程序联调，确保数据格式兼容
4. 进行实机测试，验证系统稳定性

### 5. 注意事项
- HAL库的API设计与标准库有较大差异，需要仔细查阅HAL库文档
- HAL库的初始化流程与标准库不同，需要重新设计系统启动流程
- HAL库的中断处理机制是基于回调函数的，需要重新组织中断相关代码
- 转换过程中可能会遇到性能差异，需要进行性能测试和优化
- 建议先在小范围模块上进行转换测试，确保可行性后再全面转换

### 6. 资源参考
- [STM32CubeMX官方文档](https://www.st.com/en/development-tools/stm32cubemx.html)
- [STM32F1 HAL库用户手册](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f1-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [HAL库与标准库函数对照表](https://blog.csdn.net/weixin_42405819/article/details/105043137)
