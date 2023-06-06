/******************************************************************************/
/** @file STMgood.c
 *  @version 1.0
 *  @date 2018.12.8
 *
 *  @brief Deal the data from the Hoster when we use usart1 to debug
 *
 *  @author
 *
 */
/*****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "STMGood.h"
#include "stdio.h"
#include "usart.h"
#include "bsp_usart.h"
#include "printf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//-----------------------------------变量保护区（以下变量不能改变）
char Count = 0;
char Cmd1[20];
char Cmd2[100];
char Data[100];
char Str1[100];
double xx[100];
//-----------------------------------变量保护区（以上变量不能改变）

double P1 = 0.0f;
double I1 = 0.0f;
double D1 = 0.0f;
double S1 = 0.0f;

double P2 = 0.0f;
double I2 = 0.0f;
double D2 = 0.0f;
double S2 = 0.0f;

double P3 = 0.0f;
double I3 = 0.0f;
double D3 = 0.0f;
double S3 = 0.0f;

double P4 = 0.0f;
double I4 = 0.0f;
double D4 = 0.0f;
double S4 = 0.0f;

double P5 = 0.0f;
double I5 = 0.0f;
double D5 = 0.0f;
double S5 = 0.0f;

double P6 = 0.0f;
double I6 = 0.0f;
double D6 = 0.0f;
double S6 = 0.0f;

double P7 = 0.0f;
double I7 = 0.0f;
double D7 = 0.0f;
double S7 = 0.0f;

double P8 = 0.0f;
double I8 = 0.0f;
double D8 = 0.0f;
double S8 = 0.0f;

double P9 = 0.0f;
double I9 = 0.0f;
double D9 = 0.0f;
double S9 = 0.0f;
/* Private functions ---------------------------------------------------------*/
//-------------------------------------------------------------------------------
int cmd(char* Cmd, int n)
{
    //生成的代码填写在此函数内

    if(CompStr(Cmd, "q"))
    {
        return 0;
    }
    if(CompStr(Cmd, "w"))
    {
        return 0;
    }
    if(CompStr(Cmd, "e"))
    {
        return 0;
    }
    if(CompStr(Cmd, "a"))
    {
        return 0;
    }
    if(CompStr(Cmd, "s"))
    {
        return 0;
    }
    if(CompStr(Cmd, "d"))
    {
        return 0;
    }
    if(CompStr(Cmd, "r"))
    {
        return 0;
    }

    if(CompStr(Cmd, "u"))
    {
        return 0;
    }
    if(CompStr(Cmd, "i"))
    {
        return 0;
    }
    if(CompStr(Cmd, "o"))
    {
        return 0;
    }
    if(CompStr(Cmd, "j"))
    {
        return 0;
    }
    if(CompStr(Cmd, "k"))
    {
        return 0;
    }
    if(CompStr(Cmd, "l"))
    {
        return 0;
    }
    if(CompStr(Cmd, "p"))
    {
        return 0;
    }
    return 0;
}

void multi1(int n)
{
    P1 = xx[1];
    I1 = xx[2];
    D1 = xx[3];
    S1 = xx[4];
    printf("P1=%f  I1=%f  D1=%f  S1=%lf\r\n", P1, I1, D1, S1);
}
void multi2(int n)
{
    P2 = xx[1];
    I2 = xx[2];
    D2 = xx[3];
    S2 = xx[4];
    printf("P2=%f  I2=%f  D2=%f  S2=%lf\r\n", P2, I2, D2, S2);
}
void multi3(int n)
{
    P3 = xx[1];
    I3 = xx[2];
    D3 = xx[3];
    S3 = xx[4];
    printf("P3=%f  I3=%f  D3=%f  S3=%lf\r\n", P3, I3, D3, S3);
}
void multi4(int n)
{
    P4 = xx[1];
    I4 = xx[2];
    D4 = xx[3];
    S4 = xx[4];
    printf("P4=%f  I4=%f  D4=%f  S4=%lf\r\n", P4, I4, D4, S4);
}
void multi5(int n)
{
    P5 = xx[1];
    I5 = xx[2];
    D5 = xx[3];
    S5 = xx[4];
    printf("P5=%f  I5=%f  D5=%f  S5=%lf\r\n", P5, I5, D5, S5);
}
void multi6(int n)
{
    P6 = xx[1];
    I6 = xx[2];
    D6 = xx[3];
    S6 = xx[4];
    printf("P6=%f  I6=%f  D6=%f  S6=%lf\r\n", P6, I6, D6, S6);
}
void multi7(int n)
{
    P7 = xx[1];
    I7 = xx[2];
    D7 = xx[3];
    S7 = xx[4];
    printf("P7=%f  I7=%f  D7=%f  S7=%lf\r\n", P7, I7, D7, S7);
}
void multi8(int n)
{
    P8 = xx[1];
    I8 = xx[2];
    D8 = xx[3];
    S8 = xx[4];
    printf("P8=%f  I8=%f  D8=%f  S8=%lf\r\n", P8, I8, D8, S8);
}
void multi9(int n)
{
    P9 = xx[1];
    I9 = xx[2];
    D9 = xx[3];
    S9 = xx[4];
    printf("P9=%f  I9=%f  D9=%f  S9=%lf\r\n", P9, I9, D9, S9);
}

void part1(int xx)
{

}
void part2(int xx)
{

}
void part3(int xx)
{

}
void part4(int xx)
{

}
void part5(int xx)
{

}
void part6(int xx)
{

}

void go()
{

}
void back()
{

}
void left()
{

}
void right()
{

}
void stopcar()
{

}
void goleft()
{

}
void goright()
{

}
void backleft()
{

}
void backright()
{

}


void go1()
{

}
void back1()
{

}
void left1()
{

}
void right1()
{

}
void goleft1()
{

}
void goright1()
{

}
void backleft1()
{

}
void backright1()
{

}

//-----------------------------------------------------------------------



int Command(char* Cmd, int n)
{
//------------------------------以下为多个参数-----------------------------
    if(CompStr(Cmd, "*1"))
    {
        multi1(n);
        return 0;
    }
    if(CompStr(Cmd, "*2"))
    {
        multi2(n);
        return 0;
    }
    if(CompStr(Cmd, "*3"))
    {
        multi3(n);
        return 0;
    }
    if(CompStr(Cmd, "*4"))
    {
        multi4(n);
        return 0;
    }
    if(CompStr(Cmd, "*5"))
    {
        multi5(n);
        return 0;
    }
    if(CompStr(Cmd, "*6"))
    {
        multi6(n);
        return 0;
    }
    if(CompStr(Cmd, "*7"))
    {
        multi7(n);
        return 0;
    }
    if(CompStr(Cmd, "*8"))
    {
        multi8(n);
        return 0;
    }
    if(CompStr(Cmd, "*9"))
    {
        multi9(n);
        return 0;
    }
//------------------------------以下为部件控制，参数为xx[1]-------------------
    if(CompStr(Cmd, "#1"))
    {
        part1(xx[1]);
        return 0;
    }
    if(CompStr(Cmd, "#2"))
    {
        part2(xx[1]);
        return 0;
    }
    if(CompStr(Cmd, "#3"))
    {
        part3(xx[1]);
        return 0;
    }
    if(CompStr(Cmd, "#4"))
    {
        part4(xx[1]);
        return 0;
    }
    if(CompStr(Cmd, "#5"))
    {
        part5(xx[1]);
        return 0;
    }
    if(CompStr(Cmd, "#6"))
    {
        part6(xx[1]);
        return 0;
    }
//------------------------------以下为小车控制-----------------------------
    if(CompStr(Cmd, "@1"))
    {
        //前进
        go();
        return 0;
    }
    if(CompStr(Cmd, "@2"))
    {
        //后退
        back();
        return 0;
    }
    if(CompStr(Cmd, "@3"))
    {
        //左转
        left();
        return 0;
    }
    if(CompStr(Cmd, "@4"))
    {
        //右转
        right();
        return 0;
    }
    if(CompStr(Cmd, "@5"))
    {
        //停止
        stopcar();
        return 0;
    }
    if(CompStr(Cmd, "@6"))
    {
        //左前
        goleft();
        return 0;
    }
    if(CompStr(Cmd, "@7"))
    {
        //右前
        goright();
        return 0;
    }
    if(CompStr(Cmd, "@8"))
    {
        //左后
        backleft();
        return 0;
    }
    if(CompStr(Cmd, "@9"))
    {
        //右后
        backright();
        return 0;
    }


    if(CompStr(Cmd, "$1"))
    {
        //前进(按下功能键)
        go1();
        return 0;
    }
    if(CompStr(Cmd, "$2"))
    {
        //后退(按下功能键)
        back1();
        return 0;
    }
    if(CompStr(Cmd, "$3"))
    {
        //左转(按下功能键)
        left1();
        return 0;
    }
    if(CompStr(Cmd, "$4"))
    {
        //右转(按下功能键)
        right1();
        return 0;
    }
    if(CompStr(Cmd, "$6"))
    {
        //左前(按下功能键)
        goleft1();
        return 0;
    }
    if(CompStr(Cmd, "$7"))
    {
        //右前(按下功能键)
        goright1();
        return 0;
    }
    if(CompStr(Cmd, "$8"))
    {
        //左后(按下功能键)
        backleft1();
        return 0;
    }
    if(CompStr(Cmd, "$9"))
    {
        //右后(按下功能键)
        backright1();
        return 0;
    }
    cmd(Cmd, n);
    return 0;
    //------------------------------------------
}


//--------------------------------------------以下为自动处理函数，无需改变-------------------------

void Dealdata(int Rx)                  //处理上位机发送过来的数据
{
    //printf("Rx = %d/r/n",Rx);
    if(Rx == '(' && Count == 0)
    {
        Data[Count++] = Rx;
    }
    else if(Count > 0)
    {
        Data[Count++] = Rx;
    }
    if(Rx == ')')
    {
        char i;
        Data[Count] = '\0';
        Count = 0;
        for(i = 0; i < Strlen(Data) - 1; i++)
        {
            Data[i] = Data[i + 1];
        }
        Data[Strlen(Data) - 2] = '\0';
        SplitStr(Data, Cmd1, Cmd2);
        Command(Cmd1, DealStr(Cmd2));
        for(i = 0; i < 100; i++)
        {
            Cmd2[i] = 0;
        }
    }
}

//-----------------------------------------------

int DealStr(char* Str)
{
    int i;
    int m = 0;
    int n = 1;
    int len = Strlen(Str);
    if(len == 0)
    {
        return 0;
    }
    for(i = 0; i < len; i++)
    {
        if(CompStr(SubStr(Str, i, i + 1), " "))
        {
            xx[n] = StrToFloat(SubStr(Str, m, i));
            n++;
            m = i + 1;
        }
        else
        {
            if(len - i == 1)
            {
                xx[n] = StrToFloat(SubStr(Str, m, len));
            }
            continue;
        }
    }
    return n;
}

char* SubStr(char* Str, int start, int final)          //获取子字符串
{
    int i;
    for(i = start; i < final; i++)
    {
        *(Str1 + i - start) = *(Str + i);
    }
    *(Str1 + final - start) = '\0';
    return Str1;
}

void SplitStr(char* Str, char* Str1, char* Str2)         //分割字符串
{
    int i;
    int k = FirstSpace(Str);
    int len = Strlen(Str);
    for(i = 0; i < k; i++)
    {
        Str1[i] = Str[i];
    }
    Str1[k] = '\0';
    for(i = k + 1; i < len; i++)
    {
        Str2[i - k - 1] = Str[i];
    }
    Str2[len] = '\0';
    if(k == -1)
    {
        CopyStr(Str, Str1);
        Str2[0] = '\0';
    }
}

int FirstSpace(char* Str)//寻找字符串中第一个空格位置
{
    int i;
    for(i = 0; i < 100; i++)
    {
        if(Str[i] == ' ')
        {
            return i;
        }
        else if(Str[i] == '\0')
        {
            return -1;
        }

    }
    return -1;
}

int Strlen(char* Str)//获取字符串长度
{
    int len = 0;
    while(*(Str + len++) != '\0');
    return --len;
}

int CompStr(char* Str1, char* Str2)//比较两个字符串是否相同
{
    int aStr = Strlen(Str1);
    int bStr = Strlen(Str2);
    if(aStr == bStr)
    {
        for(int i = 0; i < aStr; i++)
        {
            if(*(Str1 + i) == *(Str2 + i))
                continue;
            else
                return 0;
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

int CopyStr(char* Str1, char* Str2)//复制字符串，源字符串：Str1，目标：Str2
{
    int i;
    for(i = 0; i < 100; i++)
    {
        if(Str1[i] != '\0')
        {
            Str2[i] = Str1[i];
        }
        else
        {
            Str2[i] = Str1[i];
            return 1;
        }
    }
    return 1;
}

int StrToInt(char* Str)                                 //字符串转化为整数
{
    return (int)StrToFloat(Str);
}

float StrToFloat(char* Str)                              //字符串转化为小数
{
    char i, j, k, negative = 0;
#define s_temp Str
    double result = 0, result_1 = 0;
    for(i = 0; i < 10; i++)
    {
        j = Str[i];
        if(j == 0 || ((j < '0' || j > '9') && (j != '.') && (j != '-')))
            break;
    }
    k = j = i;
    for(i = 0; i < j; i++)
    {
        if(s_temp[i] == '.')
            break;
    }

    for(j = 0; j < i; j++)
    {
        if(s_temp[j] == '-')
        {
            negative = 1;
            continue;
        }
        result = result * 10 + (s_temp[j] - '0');
    }
    j++;
    i = j;
    for(; j < k; j++)
    {
        if(s_temp[j] < '0' || s_temp[j] > '9')
            break;
        result_1 = result_1 * 10 + (s_temp[j] - '0');
    }
    for(j = 0; j < (k - i); j++)
        result_1 *= 0.1;
    result += result_1;

    if(negative)result = -result;
    return result;
}
//------------------------------------传感器发送小数函数
void senddouble1(double x)
{

}
void senddouble2(double x)
{

}
void senddouble3(double x)
{

}
void senddouble4(double x)
{

}
void senddouble5(double x)
{

}
void senddouble6(double x)
{

}
void senddouble7(double x)
{

}
void senddouble8(double x)
{

}
void senddouble9(double x)
{

}
void senddouble10(double x)
{

}
//---------------------------传感器发送整数函数
void sendint1(int x)
{

}
void sendint2(int x)
{

}
void sendint3(int x)
{

}
void sendint4(int x)
{

}
void sendint5(int x)
{

}
void sendint6(int x)
{

}
void sendint7(int x)
{

}
void sendint8(int x)
{

}
void sendint9(int x)
{

}
void sendint10(int x)
{

}
