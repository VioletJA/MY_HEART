## 心跳跟随心形灯

### 硬件部分
心脏的外壳采用紫铜丝或黄铜丝焊接，1mm的铜丝较硬，适合完成外部框架的搭建，0.7mm的铜丝可塑性较好，适合焊接内部的WS2812。<br><br>
整个过程中吗，焊接难度较大，电子器件的固定难度也很大。<br><br>


### 软件部分
软件使用stm32控制的，库是用stm32cubeMX生成的HAL库<br><br>
因为我想利用FreeRTOS学习一下实际项目编写，所以将主控STM32F103C8T6，<br><br>
当然编写了两套代码，先编写了一套裸机开发，测试能够完成所有功能、之后移植了一套FreeRTOS的，都可以运行。<br><br>

![image](https://github.com/VioletJA/MY_HEART/blob/main/prg_img/IMG_20230804_234926.jpg)<br>

详细步骤见：https://blog.csdn.net/zerokingwang/article/details/132117993?spm=1001.2014.3001.5501
