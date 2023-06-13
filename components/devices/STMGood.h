#ifndef __STMGood_H__
#define __STMGood_H__

#include "stm32f4xx.h"
#include "stdint.h"
int cmd(char *Cmd,int n);

void multi1(int n);
void multi2(int n);
void multi3(int n);
void multi4(int n);
void multi5(int n);
void multi6(int n);
void multi7(int n);
void multi8(int n);
void multi9(int n);

void part1(int xx);
void part2(int xx);
void part3(int xx);
void part4(int xx);
void part5(int xx);
void part6(int xx);

void go(void);
void back(void);
void left(void);
void right(void);
void goleft(void);
void goright(void);
void backleft(void);
void backright(void);
void stopcar(void);

void go1(void);
void back1(void);
void left1(void);
void right1(void);
void goleft1(void);
void goright1(void);
void backleft1(void);
void backright1(void);

void Dealdata(int Rx);
void sendint1(int x);
void sendint2(int x);
void sendint3(int x);
void sendint4(int x);
void sendint5(int x);
void sendint6(int x);
void sendint7(int x);
void sendint8(int x);
void sendint9(int x);
void sendint10(int x);
void senddouble1(double x);
void senddouble2(double x);
void senddouble3(double x);
void senddouble4(double x);
void senddouble5(double x);
void senddouble6(double x);
void senddouble7(double x);
void senddouble8(double x);
void senddouble9(double x);
void senddouble10(double x);
int Command(char *Cmd,int n);
void SplitStr(char* Str, char* Str1, char* Str2);
int Strlen(char* Str);
int CompStr(char* Str1, char* Str2);
int CopyStr(char* Str1, char* Str2);
int FirstSpace(char* Str);
int DealStr(char *Str);
int StrToInt(char *Str);
float StrToFloat(char* Str);
char *SubStr(char* Str, int start, int final);

extern double	P1,I1,D1,S1,P2,I2,D2,S2,P3,I3,D3,S3,P4,I4,D4,S4,P5,I5,D5,S5,P6,I6,D6,S6,P7,I7,D7,S7,P8,I8,D8,S8,P9,I9,D9,S9;
#endif
