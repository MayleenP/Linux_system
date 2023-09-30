# Linux_system
## Watchdog System 看门狗系统
1. Linux Watchdog driving system 看门狗驱动系统

- 基于飞腾FT 2000/4 开发板设计的一种Linux看门狗驱动系统。需编译进Linux内核里，在Linux内核里创建新C语言文件。原理是当一定数值超过超时值时，就会使开发板“死机”，之后再复位重启。

2. Linux automated Watchdog application system 一个无人值守的Linux应用系统

- 基于飞腾FT 2000/4 开发板设计的一种Linux看门狗应用系统。应用系统主要是一个进程不断的喂狗。如果系统死机了，这个进程得不到调度，无法喂狗，就会复位整个系统。

3. Linux Long Range Detection System (RADAR) Linux远距离探测系统

- 基于飞腾FT 2000/4 开发板设计一种远距离探测系统。比如：雷达与两物体三点共线,两物体相距600米,雷达的脉冲宽度为7us,线性信号带宽为6MHz,下限截止频率为222MHz,各个脉冲初始相位为0,且在电路中信号会通过225±3MHz的带通滤波器,ADC采样率为20MHz.
 
