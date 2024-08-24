# Engineer2024_Custom_Contoller
2024赛季工程机器人自定义控制器嵌入式代码开源

## 写在前面

RoboWalker战队的工程机器人在区域赛的时候取得了不错的战绩，从区域赛开始基本上稳定单场黄金矿工，并且获得了区域赛东部赛区场均经济最高、单场经济最高。在复活赛，各个强队的工程机器人水平迎头赶上，而我们的工程机器人也出现了一些机械、电路上的战损情况，导致机器人在赛场上的表现有所下滑，不过每场的表现也还算可以接受。现开源工程机器人自定义控制器代码，希望和各队伍多多交流。

上位机代码仓库：https://github.com/RoboWalker/Engineer2024_ROS2_Controller

嵌入式代码仓库：https://github.com/RoboWalker/Engineer_2024_Embedding

自定义控制器代码架构和下位机类似。不过，代码中主要比较有开创性的一点（也可以认为是比较抽象的一点），是作者塞了一个完整的`Eigen`库在里面做矩阵运算用。这虽然极大的方便了代码开发，但也导致编译的时候需要开编译优化，否则会爆Flash。

关于自定义控制器的数学推导，见[技术文档](./中国科学技术大学RoboWalker_2024工程机器人技术文档.pdf)

## 工具链

开发板为达妙F4板，使用`CubeMX`+`cmake`+`armgcc`+`openocd`工具链，使用`Clion`作为IDE开发。

Clion作为一个成熟的IDE，相比于Keil开发环境，具有代码补全功能丰富、编译超级快等显著优点，同时也可以在Linux系统上配置环境。

## 代码架构

代码主要构建在User目录下。代码层次为如下。

### 0-MWL(Middleware Layer)

主要是算法中间件，包括CRC校验、PID、自定义的一些数学库、Eigen。

### 1-APL(Application Layer)

代码最顶层，包含轮询执行的函数，和中断回调函数。

### 2-FML(Function Module Layer)

包括底盘、云台、机械臂三个功能模块的类。相关控制函数在类中封装。

### 3-HDL(Hardware Driver Layer)

底层传感器、执行器等驱动代码，包括控制电机的代码、解析电机数据的代码、读取陀螺仪的代码等。每一个硬件驱动用类来封装。

### 4-HAL(Hardware Abstract Layer)

自己对stm32 HAL库中的CAN、UART等接口进行再封装。
