# 1

也可以查看.vscode路径下task.json，里面我也写了一些说明

参考链接：

官网指令大全：https://wiki.segger.com/J-Link_Commander

Jlink commander使用方法（附指令大全）：https://blog.csdn.net/qq_30095921/article/details/128311887

编写批处理脚本使用JLink.exe / ST-LINK_CLI.exe烧写STM32F103RC系列芯片:https://blog.csdn.net/hpf247/article/details/118799555

在工程路径下的JLink文件夹保存了JLink.exe需要的指令，我不知道为什么不能在命令行直接输入这些指令，他们都用txt那我也用txt了
注意，txt中只能有指令，不能有我这些中文

```
usb 0 ----连接到usb（如果电脑只接有一个JLink下），貌似这个usb应该输入设备地址，但我没用到
si 1 ----工作在SWD模式（si 0则工作在JTAG模式下）
speed 4000 ----速度为4000KHz
device STM32F407IG ----目标下载器件为STM32F407IG系列
r ----重新复位目标
h ----单片机停机，内核停止运行,也就是halt
erase ----擦除flash的内容
//下面这两个任选一个，我建议使用hex和elf，因为bin文件是纯粹的二进制文件，需要指定起始地址，而hex、elf等带地址信息的不用指定。下载会判断地址范围，自动擦除扇区，详情可以看ST-LinkCLI指令教程.md
loadbin xxxx.bin 0x08000000 ----将xxxx.bin下载到芯片起始地址为0x08000000
loadfile xxxx.elf
r ----重新复位目标
q ----退出
```