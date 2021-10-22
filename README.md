# 第十六届智能车大赛——全向行进组

Author：北京邮电大学OmniRun队
Date：2021/06

本项目开源地址：https://gitee.com/SimonKenneth/bupt_smartcar_omni
**使用代码时请联系作者授权！**

---
## 队员信息
| 姓名             | QQ         | 负责内容|
| --- | --- | --- |
| 司马宽宽（队长） | 229795357  | 1. 主单片机程序设计<br />2. 运动控制（电机速度环、偏航角闭环控制等）<br />3. 麦克纳姆轮运动学解算 <br />4. IMU数据处理 <br />5. 串口命令行实现<br />6. 比赛状态机编程 |
| 周弋迪           |  1253140016 | 1. 从单片机程序设计<br />2. 摄像头图像处理(车道线识别、特殊元素识别等)<br />3. 陀螺仪环岛处理<br />4. 电路焊接 |
| 艾诺舟           |  2963950664 | 1. 双核主控PCB<br />2. 直流电机双驱驱动板PCB<br />3. 电路焊接 |

## 目录说明

1. MasterChip内为主单片机工程目录

   ```c
   |- MasterChip
      |- CODE
         |- cmd.c // 串口dma配置与串口命令行实现
         |- config.h // 智能车参数配置
         |- mecanum_chassis.c // 麦克纳姆轮运动学解算与底盘状态机实现
         |- motor.c // 电机控制相关
         |- path_following.c // 巡线状态机
         |- pid.c // PID控制器实现
         |- sci_compute.c // 科学计算相关
         |- slave_comm.c // 从单片机通信相关
         |- valuepack.c // 虚拟串口示波器相关
      |- USER
         |- main.c
         |- isr.c // 所有中断服务函数
   ```

2. SlaveChip内为协单片机工程目录
   ```c
   |- SlaveChip
      |- CODE
         imageprocessing.c // 摄像头图像处理
         inducercontrol.c // 电感处理
      |- USER
         |- main.c
         |- isr.c // 所有中断服务函数
   ```

3. Docs内为比赛相关文档
   [硬件资源分配.md](./Docs/硬件资源分配.md)详细说明了引脚分配及定义。
   [巡线效果.mp4](./Docs/巡线效果.mp4)演示了巡线效果，仅供参考。

