# 大连交通大学TOE战队RoboMaster2023英雄电控程序开源

本人环境配置教程：<https://robomaster.ones.pro/wiki/#/team/T6gsU22v/space/oqJvJWK4/page/2okLKyCo>

注意：链接可能会挂，打开ones后，有一个配置教程模块，vscode开发STM32配置教程在电控配置里。

如果有疑问，可以给我发邮箱问问题:<l2356838399@163.com>，也可以邮件发你的联系方式，看到了会回。

VsCode工程缺点：

没有一个十分好用的debug，无法像别的IDE一样能够在debug的时候实时查看变量信息

所以建议使用JLink，它官方自带的Ozone非常好用（比keil好用），弥补了vscode不能debug的缺陷。

如果你想在Ozone中实时查看FreeRTOS中每个任务占据的内存大小的话，需要你在 `Inc\FreeRTOSConfig.h` 文件中增加

```C
   #define configRECORD_STACK_HIGH_ADDRESS          1
```

这一行代码，(CubeMX中是可以点一点就能开启这个选项的)，你可以先检查一下自己的程序中是不是有。

## 注意事项

1. 如果你要使用CubeMX重新生成程序，有可能会导致makefile清空，建议重新生成前把makefile添加bak后缀，防止被覆盖掉。

   如果你在cubemx中添加了一些额外的外设(一般来讲不会，因为我基本把能打开的都打开了，你新增PWM或者GPIO都不会新增文件的)，然后使用备份的makefile去编译出错了，可以根据报错信息在makefile中添加你增加的源文件和头文件路径

2. 文件夹名字中绝不可以有空格！！！

3. 那些烧录工具尽量添加到系统环境变量里，否则要在`.vscode/tasks.json`中修改路径

4. 想换车必须要在`Mode/Mode_Switch.h`中修改宏定义`ROBOT_ID`，只要你有多台车，就不可以把程序分成两个工程。

5. 有关换车编译：
   * 在 `Mode_Switch.h` 中修改ROBOT_ID宏定义，会由于链接问题导致修改后再build不识别，所以需要make remake；

   * 至于为什么不把这个宏定义放到 `Mode_Switch.c` 中，因为vscode打开的其它文件找不到这个宏定义了，就没有高亮了。

   * 这个高亮其实可以在`.vscode/c_cpp_properties.json` 中增加宏定义`"ROBOT_ID=SUN"`，或者`"ROBOT_ID=MOON"`，

   * 只是文件中实际有宏定义的话会覆盖掉json文件中的高亮。

6. 电机的ID两辆车必须一样，除非有连电机型号都不一样的地方。

## 一、学习路线

### 1. 嵌入式基础学习

先从CubeMX创建一个现成的IDE开始（Keil、CubeIDE、sw4stm32等），学习创建工程、IDE的使用，如何使用HAL库函数的API，

学着自己去写一个bsp板级支持包；遇到不会的一定要补基础，一定要看明白总线每个参数或者寄存器什么意思，最起码用到的要眼熟掌握，可以用过往学长的bsp或者一些程序或者官方程序去学习，因为对我们的比赛调车是直接性帮助。

### 2. 学习编译的一些基本原理，最起码gcc生成一个可执行文件的流程要清楚（VsCode要掌握）

学习gcc命令，Makefile语法，学着自己去创建一个多文件c工程然后使用gcc和makefile编译；

然后就可以学着去搭建自己的工具链，当然了解后还是建议使用成品IDE，因为简单好用，让你的精力集中在优化你的代码上，

而不是天天配环境。

### 3. CMake、xmake等编译工具的使用

可以依靠VsCode，去用插件来辅助，编译一些简单C\C++工程，就足够了，CMake是常用工具，建议学一下。

## 二、程序结构

咱的.c和.h文件放在同一个路径下了，单纯为了方便，因为.h文件也有好多有关配置的，方便好找就放在一起了，你可以.c一个文件夹.h一个文件夹，看个人喜好.

![image](.instruction/总工程.png)

![image](.instruction/官方C板工程.png)

### 0. 前缀./文件夹

* `.instruction`文件夹里面有一些说明文档和教程，有一些程序框图；

* `.JLink`文件夹主要是与`task.json`+`Jlink.exe`配合来使用jlink烧录程序，等于是`Jlink.exe`的脚本；

* `.Ozone`文件夹是用来存放Ozone调试用的`FreeRTOSPlugin_CM4.js`文件和Ozone工程配置文件；

* `.Serial_debug_tools`里面是两个串口调试工具(免安装)(Vofa+没有，自己去官网下)；

* `.vscode`中主要存放vscode配置文件：

  * `c_cpp_properties.json`用来配置当前工作空间的各文件或者变量索引(仅与高亮有关)；
  * `keybindings.json`这个文件是快捷键配置，这个文件放在这个文件夹是不生效的，需要放到系统的`keybindings.json`中；
  * `launch.json`我基本都弃用了，这个文件用来配置vscode中debug的，但是只能打断点调试，后来不用了；
  * `settings.json`是相对当前工作空间与系统配置不同的设置的修改；
  * `tasks.json`是比较重要的文件，这个文件放在这个文件夹是对当前工作空间生效的，用来烧录程序，编译程序等

### 1. ./application 里为应用层代码

![image](.instruction/应用层.png)

有一个`robot`文件用来管理不同的机器人；

以英雄为例，`Hero`文件夹内都是有关英雄控制、数据接收、任务调度等功能；

如果你想加入自己的机器人，比如步兵，

文件夹结构应该是：

```txt
   Infantry:.
   │ bsp # 用来实例化can中断回调函数，一些兵种独立的can发送接口
   | Devices # 用来控制额外设备，比如PC交互、红外激光控制、裁判系统数据
   | Mode # 主要的机器人运动控制模块，里面有各种模式
   | Thread # 线程运行
   |  └─ConfigTask # 线程管理与创建
   | Variables # 全局变量管理，如电机结构体变量、pid结构体变量、ramp结构体变量
   | ... #你可以创建自己的文件夹，需要在 makefile 中添加 .c 文件和 .h 的路径
   └─Infantry_control.h # 用来管理上面这些文件夹的头文件，方便统一extern
```

文件夹的结构也可以自己定义，我在这里只建立了`bsp、Mode、Thread、Variables、Devices`等模块；

其中`None_robot`模块是为了在不需要机器人控制情况下，`makefile`修改`ROBOT`参数的值为这个文件夹名，就可以屏蔽机器人模块；

  接下来介绍模块中每个组成：

* `bsp`主要是放一些兵种特殊需要的外设接口，比如can数据接收函数我就单独拿出来放在`hero_can`里了；

* `Mode`主要放功能控制和自检文件：

  * `Mode_Switch`用来模式控制;
  * `Monitor`用来计算帧率和获取机器人状态，其中有蜂鸣器报警部分；
  * `Chassis.c`文件用来解算底盘运动，然后计算pid，最后给电机发送电流；
  * `Gimbal.c`文件用来解算云台，并发送电流；
  * `Shoot.c`文件用来解算发射机构电机电流,令拨弹盘电机转一步的控制框图如下：

    ![image](.instruction/shoot.png)

* `Variables`主要是用来管理一些重要的全局变量，比如电机结构体变量、pid、ramp(斜坡函数)等

* `Devices`主要是设备控制的代码：

  * `nuc_interface`主要是处理PC信息的文件，并在其中计算自瞄目标值
  * `laser`为RM官方红外激光控制的文件
  * `RM_Judge`为RM裁判系统数据管理，没写太多

* `Thread`主要放具体功能要运行的线程;

  * `ConfigTask`文件夹主要用来创建和管理线程，最终的.h文件被`Hero_control.h`包含，最后被`robot.c`调用，其中`RobotTaskInit()`函数用来创建线程，这个函数最终放在`freertos.c`的`MX_FREERTOS_Init`中。

PS:`Chassis_task`、`Gimbal_task`、`Shoot_task`这三个线程需要`osPriorityHigh`优先级,不然`can2`发送会返回`HAL_ERROR`

**<u><font color='red'> (这是个待解决bug，不知道为什么) </font></u>**

除了线程需要打开CubeMX修改外（因为我是基于英雄重构的代码），步兵需要使能需要的pwm引脚、平衡步兵需要dwt库等，我这里并没有按所有兵种需求去增加东西，主打个人DIY的灵活度（狗头）。

### 2. ./bsp 里有基础的bsp板级支持包文件

* `bsp_can.c`文件是最重要的文件，里面写了打开CAN外设的接口和一些CAN发送数据的API;

* `bsp_remote.c`文件是照着官方移植的遥控器数据串口接收程序，使用了双缓冲DMA，可以仔细看看源码，我标注了详细的注释，里面有一个fps_remote_count用来计算遥控器帧率;

* `bsp_usart.c`文件中主要使用C板4pin口的那个`USART1`和`USART6`，用来无线调参和数据打印;

* `bsp_timer.c`文件暂时没写东西，因为pwm有关的都可以写到`devices`文件里去直接控制;

* `bsp_adc.c`文件中有计算电池电压和芯片温度的接口；

* 其余的bsp文件都是官方自己的。

### 3. ./Thread 里有C板官方的线程文件(led、adc等)

* adc计算电压线程放在`adc_task`中;

* 陀螺仪数据处理程序在`INS_task`中;

* 上电自检陀螺仪烧写flash等程序在`calibrate_task`和`detect_task`中;

* 闪灯程序在`led_flow_task`中;

* 空闲线程是`idle_task`,这个没有放在文件夹里。

### 4 ./components文件说明

![image](.instruction/组件.png)

#### 1. ./components/devices 是 stm32 外接设备相关的

其中的设备有：BMI088、IST8310、串口发送（printf）、电机、STMGood上位机数据处理。

* `printf.c`为串口重定向格式化文件，这个是arm-gcc工程专有的; 如果你用Keil，简单重定向就好了，参考实验室祖传代码或者正点原子的教程
* `dji_motor.c`中为电机编码器解算，头文件中有不同减速箱3508的精确减速比
* `STMGood`上位机在`STMGood.c`里，已经开启了所有通道的参数输入

#### 2. ./components/controller 文件为空，没想好要放什么文件

#### 3. ./components/algorithm 里是有关控制算法的文件

* `pid.c`文件是一些pid解算的接口；

* `CRC_Check.c`是官方给的CRC校验，用来校验串口数据；

* `ramp.c`是计算斜坡函数的，是有关斜坡函数的一些接口；

* `filter.c`文件是陈澍学长写的低通滤波接口，跟`INS_task.c`文件中写的滤波一摸一样，是为了调用方便，后来也没再用了；

* `kalman_filter.c`是卡尔曼滤波，只有一阶和二阶；

* `user_lib.c`是一些常用快速算法，比如快速开方之类的；

* `MahonyAHRS.c`、`AHRS_middleware.c` 、`AHRS.h`都是有关陀螺仪解算的接口。

### 5. ./lib 里有一些官方的静态库文件和arm的数学静态库文件

* `libAHRS.a` 和 `libarm_cortexM4lf_math.a`:

   一个是官方陀螺仪解算，一个是STM32官方的库文件，这个文件我是在cubemx下载的stm32f407ig包里找到的。

### 6. ./build 中为二进制文件

* `*.o 、 *.d 、 *.lst` 这些编译的中间文件放在 `./build/temp` 里了，这个是在makefile中修改的生成路径;

* `工程名.bin、工程名.elf、工程名.hex、工程名.map` 这些文件放在`./build` 中。

PS: 上传git时不需要上传`build`文件夹，在初始化仓库时就需要在工程路径下创建`.gitignore`文件去忽略该路径。

.gitignore:

```gitignore
# 忽略build目录
/build
# 忽略一切.log后缀的文件
*.log
# 忽略cortex-debug.registers.state.json文件,一个中间文件，没必要上传
.vscode/.cortex-debug.registers.state.json
```

### 7. UI的程序是用keil构建的，这个懒得重构了，写了一堆屎山代码，造了点轮子，凑合用吧

## 三、程序设计思路

* 在基础差不多学完以后，就该开始设计一个自己熟悉的程序框架了，我的框架是基于丁羽昊学长的框架，在其上修修补补的

* 首先所有的解算与功能放在一起，用标志位去控制模式，再用遥控器或者按键去改变标志位，要有一个习惯，硬件上的操作
基本都是操作标志位，然后通过标志位来改变你想要的功能，在中断里去处理数据是非常耗费性能的，但是有时候为了不出
现一些奇怪的bug，还是在中断里去处理了，那是极少的一部分。

* 我们与PC数据传输的方式是使用虚拟USB，随便搜一个教程，在CubeMX里点点就可以配置成功了，只不过注意，要接收消息
必须要把接收消息的函数写在`usbd_cdc_if.c`文件里的`CDC_Receive_FS`函数中; 发送消息只需要调用`CDC_Transmit_FS`函数即可。
