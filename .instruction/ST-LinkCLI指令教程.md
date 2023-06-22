# 1

也可以查看.vscode路径下task.json，里面我也写了一些说明

烧录中用到的ST-Link仿真器是有唯一ID的，如果你想使用多个stlink烧录，输入指令时记得用设备sn码指定link

下面这个回答讲了一些烧录指令，最好先把这个看一遍走一遍烧录成功
LabVIEW实现ST-Link自动烧录MCU：https://zhuanlan.zhihu.com/p/620885189

STM32_Programmer_CLI.exe指令与ST-LinkCLI.exe指令类似，最大的不同是烧录指令ST-LinkCLI.exe用-P,和全片擦写指令

STM32_Programmer_CLI.exe基本命令介绍：https://blog.csdn.net/yxy244/article/details/108453398

```
"label": "ST-link Download",
    "type": "shell",
    "command": "D:\\ProgramFiles\\stlink_utility\\ST-LINK Utility\\ST-LINK_CLI.exe",
    "args": [
        "-c",//连接命令
        "SN=55FF64066570564811412587",//提供烧录器的ID或SN信息，ID从[0..9]，根据连接的烧录器数量递增，SN信息可以通过-List命令获取；
        "SWD",//选择使用的接口协议类别，是用JTAG还是SWD，默认使用的是JTAG，这里我选用SWD；
        "UR",//设置复位模式，UR（Connect to the target under reset）， HOTPLUG（Connect to the target without halt or reset），这里我选用UR;
        "LPM",//激活在低功耗模式下调试
        "-ME",//擦除整个芯片
        "-P",//下载固件到Flash,-P <File_Path> [<Address>] 命令进行操作，其中地址是可选的，如果没有特定要求可以不指定，STM32的Flash映射地址是从0x08000000开始的，固件文件格式支持3种：.bin, .hex, .srec；如果文件路径中有空格，需要包含在双引号中
        "build/${workspaceFolderBasename}.hex" ,//文件路径
        "-V",//如果需要验证烧录是否成功，需要使用 -V ["while_programming"/"after_programming"] 命令，一种是在烧录中进行验证，另一种是在烧录完后进行,注意添加后缀的时候"while_programming"或者"after_programming"要加上双引号
        "-Rst"//复位MCU
    ],
```

以上是使用vscode的task功能配置，如果在命令行展开的话是下面这样的（注意要把ST-LinkCLI.exe所在路径添加到系统环境变量path中）
```
ST-LINK_CLI.exe -c SN=55FF64066570564811412587 SWD UR LPM -ME -P build/${workspaceFolderBasename}.hex -V -Rst
```

还有一些其它的指令，比如开关读写保护，阅读板子中的程序，咱们基本用不上，在这里提一嘴而已
```
//清空读写保护
ST-LINK_CLI.exe -c SN=55FF64066570564811412587 SWD UR -OB RDP=0 IWDG_SW=1 nRST_STOP=1 nRST_STDBY=1 Data0=0xFF Data1=0xFF WRP=0xFFFFFFFF
//添加读写保护
ST-LINK_CLI.exe -c SN=55FF64066570564811412587 SWD UR -OB RDP=1 IWDG_SW=1 nRST_STOP=1 nRST_STDBY=1 Data0=0xFF Data1=0xFF WRP=0xFFFFFFE0
```