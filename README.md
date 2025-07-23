# 25EDC - STM32F407 电子设计竞赛项目

## 项目简介

本项目基于STM32F407VG(1MB Flash, 128KB SRAM,具体自查DATASHEET)，为2025年电子设计竞赛而开发

作为general版本，秉持着“大而美”的原则，这一branch包含了所有的功能模块

实际食用应因地制宜，根据需求增减

> BTW直接用导致的爆RAM|爆栈可别怪我没提醒你呦🤪

---


## 项目结构

```
25EDC/
├── Main/                    # 主程序文件
│   ├── main.c              # 主程序入口
│   ├── stm32f4xx_it.c      # 中断服务程序
│   └── stm32f4xx_it.h      # 中断头文件
├── USER/                    # 用户应用代码
│   ├── src/                # 源文件
│   │   ├── adc.c           # ADC驱动
│   │   ├── dac.c           # DAC驱动
│   │   ├── lcd.c           # LCD驱动
│   │   ├── fft.c           # FFT处理
│   │   ├── spi.c           # SPI通信
│   │   ├── uart.c          # 串口通信
│   │   ├── i2c.c           # I2C通信
│   │   ├── key.c           # 按键处理
│   │   ├── led.c           # LED控制
│   │   ├── timer.c         # 定时器
│   │   ├── pwm.c           # PWM输出
│   │   ├── dma.c           # DMA传输
│   │   ├── w25qxx.c        # Flash存储
│   │   ├── AD9833.c        # DDS信号发生器
│   │   ├── xpt2046.c       # 触摸屏控制器
│   │   ├── touch.c         # 触摸处理
│   │   ├── pid.c           # PID控制器（已集成至DSP库）
│   │   ├── freqmeas.c      # 频率测量
│   │   ├── convolution.c   # 卷积运算
│   │   ├── wavetest.c      # 波形分析
│   │   └── arm_cmsis_dsp.c # 自封装ARM DSP库
│   └── inc/                # 头文件
│       ├── *.h             # 对应源文件的头文件
│       ├── font.h          # 字体文件
│       └── arm_cmsis_dsp.h # ARM DSP库头文件
├── STM32F4_FWLIB/          # STM32F4固件库
│   ├── inc/                # 库头文件
│   └── src/                # 库源文件
├── Startup_config/         # 启动配置
│   ├── startup_stm32f40_41xxx.s  # 启动文件
│   ├── stm32f4xx.h         # 芯片定义
│   ├── stm32f4xx_conf.h    # 配置文件
│   └── system_stm32f4xx.c  # 系统配置
├── DSP_LIB/                # DSP库
│   ├── Include/            # DSP库头文件
│   └── arm_cortexM4lf_math.lib  # DSP库文件
├── Common/                 # 公共文件
│   ├── common.c            # 公共函数
│   └── common.h            # 公共头文件
└── Project/                # 项目文件
    ├── *.uvprojx          # Keil项目文件
    ├── *.hex              # 编译输出文件
    └── Objects/           # 编译对象文件
```

## 功能模块详解

### 基本外设

一些基础但蛋疼的配置，采用**标准库**

#### TIM

> 配置文件：`timer.h`| `timer.c`

- 共配置了5个TIM
  
1. **TIM2**：ADC1和ADC3时钟源  
     - 使能时钟84MHz  
     - 可通过`TIM2_Init`函数设置采样率：
     ```c
     //  auto_data: 自动重装值
     //  fractional: 时钟预分频数
     void TIM2_Init(uint16_t auto_data, uint16_t fractional)
     ```
     > 如`TIM2_Init(2,41)` → Fs = 1.024MHz
2. **TIM6**：DAC1时钟源  
     - 使能时钟84MHz
     - 调用方法同TIM2

3. **TIM4**：通用定时器，闲置

4. **TIM1&TIM7**：分别作为频率计的计数器和基准时钟，无需特别注意

---

#### UART

> 配置文件：`uart.h`| `uart.c`

- 其中**UART4**可用，所复用GPIO为**PC10|PC11**(TX/RX对应的引脚请自试)
- 初始化时需要设置波特率，如`uart4_init(115200)`
- 单片机→上位机通信：
    ```c
    printf("KEY1 Pressed~\n"); 
    ```

---

#### SPI

> 配置文件：`spi.h`| `spi.c`

- **SPI3**已用于**W25QXX**，无需特别配置

- **SPI2**用于DDS，引脚配置为`PB11/PB12(CS)  B13(SCK)  B15(MOSI)`

---

#### DMA

> 配置文件：`dma.h`| `dma.c`

- .h中的全局参数：
  ```c
    extern u8 DMA_FLAG;	//采样完成标志，中断中 put，main里手动clear
    extern volatile int8_t ADC_Data_Ready_Buffer; //DMA状态标志，默认`-1`，半传输完成置`0`，全传输完成置`1`，需手动复位
    ```

- 使用**DMA2**
- 默认循环模式，即`DMA_InitStructure.DMA_Mode = DMA_Mode_Circular`
- 与其他外设配合食用，对应的DMA`Channel|stream`见下图：

![DMA配置](https://cdn.jsdelivr.net/gh/dixiLOG/blogStatic/PixPin_2025-07-23_14-51-09.jpg)


- 初始化函数有`USER_DMA_Config`(配置)与`USER_DMA_Enable`(使能)，前者已为ADC1设置中断，且为ping-pong模式（存在半传输中断）

- 修改配置时注意`ndtr`(buffer长度)在`USER_DMA_Config`与`USER_DMA_Enable`函数中的一致性。具体来说：
    ```c
    // 注意修改其中的【DataSize】参数
    /* 配置DMA */
	USER_DMA_Config(DMA1_Stream5,DMA_Channel_7,DMA_DIR_MemoryToPeripheral,(u32)&DAC->DHR12R1,(u32)&DAC_Data_Tx,DataSize);	//DAC1输出固定值
    //USER_DMA_Config(DMA1_Stream5,DMA_Channel_7,DMA_DIR_MemoryToPeripheral,(u32)&DAC->DHR12R1,(u32)DAC1_Data_Tx,DataSize);	//DAC1连续输出正弦波（波形可改）
	
	/* 配置ADC中的DMA */
	DAC_DMACmd(DAC_Channel_1,ENABLE);	//使能DAC1的DMA
	
	/* 使能DMA */
	USER_DMA_Enable(DMA1_Stream5,DataSize);	//使能DMA1的Stream5
	
    ```

---

#### ADC

> 配置文件：`adc.h`| `adc.c`

- .h中的**全局参数**(重要)：
  ```c
    #define Fs 1024	//采样率(kHz),全工程同一
    #define ChannelSize 2	//ADC3通道数
    #define Sampl_Times 4096	//FFT_N,全工程同一
    extern u16 ADC1_Data_Rx[2*Sampl_Times];	// ADC1缓存buffer<-- ping-pong模式下，缓冲区大小加倍
    extern u16 ADC3_Data_Rx[ChannelSize*Sampl_Times];
    ```

- **ADC1**：
  - 单通道(引脚为PA0)
  - 采样频率可由**TIM2**配置，默认1024KHz
  - DMA2传输，缓冲区大小为`2*Sampl_Times`，带中断

- **ADC3**：
  - 双通道，buffer中为**交替**采样(引脚为PA2|PA3)
  - Fs也由**TIM2**配置
  - DMA2传输，缓冲区大小为`ChannelSize*Sampl_Times`，不带中断

- ADC初始化函数(记得开TIM)：
    ```c
    void ADC1_Init(void);
    ```

- 中断与处理：
    数据传输完成后进DMA中断：
    ```c
    //DMA2_Stream0中断处理函数
    void  DMA2_Stream0_IRQHandler(void)
    {
    //	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0)!=RESET){
    //		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);	//清除DMA中断标志
    //		DMA_FLAG = 1;	//FFT完成标志
    //	}
        // 检查是否是半传输完成中断
        if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0) != RESET)
        {
            DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0); // 清除中断标志
            
            ADC_Data_Ready_Buffer = 0; // 标记前半段缓冲区(Ping)数据准备就绪
            DMA_FLAG = 1;              // 通知主循环有数据要处理
        }

        // 检查是否是传输完成中断
        if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
        {
            DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0); // 清除中断标志
            
            ADC_Data_Ready_Buffer = 1; // 标记后半段缓冲区(Pong)数据准备就绪
            DMA_FLAG = 1;              // 通知主循环有数据要处理
        }
    }
    ```
    则可在`mian.c`中清标志位，并做下一步运算

- ADC采样时需要注意：
    - 信号需要偏置，`0~3.3V`
    - 基准电压确定
    - 输入阻抗不超过`50KΩ`，尽可能小
    - 采样信号频率不超过`2M`，且随采样率升高而降低

---

#### DAC

> 配置文件：`dac.h`| `dac.c`


- .h中的全局参数：
  ```c
    #define DataSize 1	//输出固定值时取消注释，并将下面的DataSize注释
    extern u16 DAC_Data_Tx;	//DAC输出固定值，可随时在主函数while(1)中更改
    ```

- **DAC1**：
    - 由**TIM6**触发，可输出电平/波形(注意DMA的DataSize传参)
    - 有专门的IO口输出
    - `DMA1_Stream5`传输

- DAC初始化函数(记得开TIM)：
    ```c
    void DAC1_Init(void);
    ```

> 题外话：IO口输出若需要带后级电路，建议改为 *漏极开路+外部电阻上拉*

---


#### LCD

> 配置文件：`lcd.h`| `lcd.c`

- 液晶显示文字（注意字号大小）、图形，实现数据可视化
- `x`为水平方向，`y`为垂直方向

- 初始化函数：
    ```c
    void LCD_Init(void);
    ```

- 常用接口函数：
    ```c
    void LCD_DrawPoint(u16 x,u16 y);//画点
    void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);//画线
    void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);//画矩形
    void LCD_Color_DrawPoint(u16 x,u16 y,u16 color);//颜色画点
    void LCD_Color_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);//颜色画线
    void LCD_Color_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);//颜色画矩形
    u16  LCD_GetPoint(u16 x,u16 y);//读点
    ```

- 示例 | 显示数据：
    ```c
    sprintf(String, "Mag_max = %.3f",Mag_max);
    LCD_DisplayString(10, 130, 12, (u8*)String);
    ```

- 复杂图像绘制可见
  - `main.c`→`FFT_SHOW()`函数
  - `arm_cmsis_dsp.c`→`DSP_LMS_DRAW()`函数（LMS滤波前后对比图）
  - `arm_cmsis_dsp.c`→`DSP_PID_DRAW()`函数（PID控制算法可视化）

---

### 自增功能

对下述功能的实现原理与使用方法进行简要说明

#### 基于AD9833的DDS

> 配置文件：`AD9833.h`| `AD9833.c`

- **功能概述**

	考虑到F4自带DAC的可玩性不够高，利用集成电路的AD9833芯片实现DDS信号输出
- **涉及原理**

	用就完了
- **使用方法**
    - 接口引脚为：`PB11/PB12(CS)  B13(SCK)  B15(MOSI)`，两个CS对应双通道
    - 根据芯片板载晶振调整`.h`中的宏定义：
        ```c
        /* 波形标识 */
        #define SQU_WAVE    1	// 方波
        #define TRI_WAVE 	2	// 三角波
        #define SIN_WAVE 	3	// 正弦波

        /* 寄存器常量 */
        #define F_mclk 25000000.0f	// 9833外接频率25Mhz←此处修改
        #define M_mclk 268435456	// 2的28次方
        #define P_mclk 4096			// 2的12次方
        ```
    - 初始化函数：
        ```c
        void AD9833_Init(void); //等效为SPI2初始化
        ```
    - 调用输出：
        ```c
        // DDS测试
		AD9833_WaveOut(SIN_WAVE,25000,0,2);// 波形类型，频率，相位，通道
        ```


---

#### 基于高精度差值补偿的FFT

> 配置文件：`fft.h`| `fft.c`

- **功能概述**

	在ARM CMSIS-DSP库的cFFT基础上，增加了差值补偿算法。用于单音信号的高精度频率和幅值恢复，解决FFT点数有限导致的频率分辨率不足问题
- **涉及原理**

	关于FT、DFT、FFT此处不再赘述
    FFT的分辨率受限于采样率与FFT点数，即：

  	$$\Delta = \frac{f_s}{N}$$

	对于F4，ARM的CMSIS-DSP库最高点数为4096
    如果不手搓FFT，想让分辨率低于100Hz是困难的
    换一个角度，若能补偿低于分辨率（非周期）带来的频谱泄露，则可以提高分辨率，同时恢复信号幅值
    综上，基于Candan-Abe-Suzuki插值算法，我们实现了高效的差值补偿FFT
    幅值补偿表达式：

  	$$a_o=\frac{2\pi r(1-r^2)\left|Z_1\right|}{N\cdot\sin(\pi r)}$$

  	其中，a_o为补偿后的幅值。计算差值

  	$$Z_1=X[k]-\frac{X[k-1]+X[k+1]}{2}$$

  	$$Z_2=X[k+1]-\frac{X[k]+X[k+2]}{2}$$

  	取模后得到频偏估计因子：

  	$$k_a=\frac{|Z_1|}{|Z_2|},\quad r=\frac{2-k_a}{1+k_a}$$

  	则有偏估计频率为：

  	$$f=(k+r)\cdot\frac{f_s}{N}$$

  	**实现逻辑**：
    1. 数据采入与预处理：将ADC采样值送入buffer，虚部取0，去直流分量
    > 为规避DMA与CPU处理冲突，采样值送入两个buffer，交替送入，缓冲区大小为`2*Sampl_Times`，即ping-pong模式
    2. FFT计算与单边谱峰值计算：ARM CMSIS-DSP库的cFFT计算
    3. 峰值查找：忽略 DC 分量，查找高于门限的极大值并记录
    4. 差值补偿：对峰值进行差值补偿，提高分辨率
    > 对于频偏极小的频率不再进行频率补偿以增强Robustness
    5. 输出：输出前三个峰值的频率与幅值
- **使用方法**
    - 接口引脚为：`PA0`(ADC1输入)
    - `.h`文件中的宏定义
        ```c
        #define FFT_N 4096  //FFT点数

        // --- 外部变量声明 (注意类型的变化) ---
        extern float32_t FFT_Data[FFT_N << 1];
        extern float32_t Mag[FFT_N];
        extern float32_t Mag_Single[FFT_N >> 1];

        // 幅值变量保持不变
        extern float32_t Mag_max;
        extern float32_t Mag_secmax;
        extern float32_t Mag_thirmax;

        // 索引变量类型已更改，以存储精确频率！
        // 我们也可以选择将它们重命名为 Freq_max, Freq_secmax 等，但为了保持变量名不变，仅改变类型。
        extern float32_t Mag_max_index;    // <-- 类型从 u32 变为 float32_t
        extern float32_t Mag_secmax_index; // <-- 类型从 u32 变为 float32_t
        extern float32_t Mag_thirmax_index;// <-- 类型从 u32 变为 float32_t
        ```
    - 正确初始化ADC1后即可调用`main.c`中的`FFT_SHOW()`函数
---

#### 混合波形频率判断(可能失效)

> 配置文件：`wavetest.h`| `wavetest.c`

- **功能概述**

	支持10K~100KHz（5KHz步进）下正弦与三角波混合波形的分离与识别，并通过LCD显示结果
- **涉及原理**
    - 根本遵循是傅里叶变换后，三角波的基波、三次、五次谐波幅值特性
    
	**实现逻辑**
    
	- 抓取FFT中的峰值
- **使用方法**

	因为修改过了FFT，所以此功能可能不能开箱即用，但总体逻辑是对的，适当修改即可
    - 在FFT正常使用的前提下，调用`wave_testv2()`函数即可

---

#### 触摸屏设置虚拟按钮

> 配置文件：`touch.h`| `touch.c`

- **功能概述**

	在LCD触摸屏上实现虚拟按键区域
- **涉及原理**

	用就完了
- **使用方法**
    - 初始化函数：
        ```c
        void Touch_Init(void);
        ```
    - 在`main.c`中调用`LCD_TOUCH_TEST()`函数，并在if句中填充你希望的东东：
        ```c
        void LCD_TOUCH_TEST(){
            //初始化，设定四个按钮
            // Clear_Screen();
            //扫描屏幕
            XPT2046_Scan(0);
            
            // 判断落点与输出
            judge_point = judge_button();
            for(u8 i=0;i<button_cnt;i++){
                // 此处DIY
                if(judge_point == button_centerX[i]){
                    // 模拟按键，修改值
                    flagForTouch = i;
                    // 这里可以做一些其他操作，比如切换页面等
                }
            }
        }
        ```
---

#### 高精度频率测量

> 配置文件：`freqmeas.h`| `freqmeas.c`

- **功能概述**

	实现高精度方波/脉冲信号频率测量。0~40 MHz 范围内的方波频率，典型误差为 0.1%
- **涉及原理**
    - 闸门法（测定时间内的脉冲个数），并通过 双定时器架构 实现高分辨率
- **使用方法**
    - 接口引脚为：`PA12`(TIM1外部触发输入)
    - 调用`freqMeasurement()`函数即可

---

#### 数字信号处理模块（DSP）

> 配置文件：`freqarm_cmsis_dspmeas.h`| `arm_cmsis_dsp.c` | `math_helper.h` | `math_helper.c`

基于ARM CMSIS-DSP库的二次封装，提供丰富的数字信号处理函数，涵盖基础运算、矩阵运算、FIR滤波、卷积、相关性、自适应滤波、PID控制、统计分析、插值等


**特别注意**：

```c

/*
    数组变量输入函数调用如下：
    DSP_ADD(pSrcA,pSrcB,pDst,blockSize);
    【单个数字变量】调用方式如下：（需要引用符）
    DSP_ADD(&pSrcA,&pSrcB,&pDst,blockSize);
*/

/* 【basic math】 的单输入运算允许输入输出为同一个变量，即【缓存同址复用】 */

/* 【fast math】 输入为【单个变量】，一进一出 */

/* 【complex math】 数据交错存储，偶数存real,奇数存img*/

/* 对数字滤波器而言，高阶的IIR滤波器很难稳定，高阶的FIR滤波器会使计算量大增 ; FIR 为线性相位差 */

/* 【矩阵运算】需要初始化，在【指针】下运算 ;  矩阵都为【一维数组】 */

/* 【LMS滤波器】的两个输入一定要软件滤除直流分量，否则其强相关性会使得结果趋向0；
			- 注意去噪模式下，pSrc为参考信号，Ref为混叠信号； 
			- 目前的函数无记忆，若一次学习后效果不好（比如频差过近），DMA连续搬运下
				可修改函数，将结构体定义为全局，初始化放在main的init中，并删除函数中这两部分，
				再在while中调用即可。注意，将errorOutput迭代输入是没有意义的，因为特征值已然不明显了
				> 具体参见 `25EDCForAdaF` 项目*/
```

- **功能概述**：基于ARM CMSIS-DSP库的二次封装，提供丰富的数字信号处理函数，涵盖基础运算、矩阵运算、滤波、卷积、相关性、自适应滤波、PID控制、统计分析、插值等。
- **主要功能类别**：
  - 基础数学运算：加法、减法、乘法、取绝对值、缩放、偏移、点积、区间剪裁、移位等
  - 复数运算：共轭、点积、取模、模平方、复数乘法、复实数乘法
  - 矩阵运算：初始化、加减乘、比例缩放、求逆、转置
  - 滤波与卷积：FIR滤波、卷积、区域卷积、相关性、下采样/上采样FIR、LMS/NLMS自适应滤波
  - 控制算法：PID控制器（支持仿真与可视化）
  - 统计分析：最大/最小值、均值、方差、标准差、均方根、平方和
  - 插值：线性插值
  - 支持函数：数组复制、填充、FIFO滤波
- **调用方式举例**：
  - 数组运算：`DSP_ADD(pSrcA, pSrcB, pDst, blockSize);`
  - 单值运算：`DSP_ABS(&a, &b, 1);`
  - 矩阵运算：`DSP_MAT_ADD(pSrcA, pSrcB, pDst, nRows, nColumns);`
  - FIR滤波：`DSP_FIR_FIL(pSrc, pDst, sampleP, blockSize, pCoeffs, numTaps);`
  - LMS自适应滤波：`DSP_LMS_FILTER(pSrc, pRef, pDst, err, inputLen, pCoeffs, numTaps, mu, state);`
  - PID控制：
    ```c
    DSP_PID_Controller my_pid;
    DSP_PID_Init(&my_pid, Kp, Ki, Kd, tol, upper, lower);
    float32_t adjust = DSP_PID_Compute(&my_pid, error);
    ```
- **详细注释与用法**：见USER/inc/arm_cmsis_dsp.h头文件说明。
- **参考文档**：
  - [ARM CMSIS-DSP 官方文档](https://arm-software.github.io/CMSIS_6/latest/DSP/index.html)
  - [CMSIS-DSP GitHub](https://github.com/ARM-software/CMSIS-DSP)
  - [CMSIS-DSP 示例](https://arm-software.github.io/CMSIS-DSP/v1.10.1/group__groupExamples.html)


---


## 许可证

本项目采用MIT许可证，详见 [LICENSE](LICENSE) 文件
