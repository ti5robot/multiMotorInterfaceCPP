#ifndef _TI5BASIC_H_
#define _TI5BASIC_H_

#include <iostream>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <string>


#include "can/SingleCaninterface.h"
#include "can/motortypehelper.h"
#include "can/tcontrolcanfactor.h"

#include "Ti5LOGIC.h"
#include "tool.h"

using namespace std;

extern class robotArm1 TH;

extern char LogInfo[100]; // 存储写入log文件的信息

extern float AG; //>=4*USLEEPTIME   USLEEPTIME是和电机通信的时间(1个电机假如通信100ms，6个就是600ms)
extern float n2p;
extern float scale;
extern float j2p;
extern float NMAX;

extern int8_t *canidList_new;
extern uint8_t *canidList;

extern uint32_t *reg_position_kp; // 位置环比例
extern uint32_t *reg_position_kd; // 位置环微分

extern uint32_t *reg_speed_kp; // 速度环比例
extern uint32_t *reg_speed_ki; // 速度环积分

extern uint32_t *reg_max_curr_value; // 最大正电流
extern uint32_t *reg_min_curr_value; // 最小负电流

extern uint32_t *reg_max_app_speed; // 最大正向允许速度
extern uint32_t *reg_min_app_speed; // 最小负向允许速度

extern uint32_t *reg_max_app_position; // 电机最大正向位
extern uint32_t *reg_min_app_position; // 电机最大负向位

extern uint32_t *MotorCurrent;        // 电流值
extern uint32_t *electric_machinery; // 电机错误状态
extern uint32_t *reg_fault_clear;    // 清除电机错误
extern uint32_t *ampere;             // 电机电流值

extern uint32_t *ele_status; // 电机状态
extern uint32_t *MotorSpeed;  // 电机速度

extern uint32_t *MotorPosition; // 电机当前位置
extern int32_t *MotorPosition_new; // 电机目标位置

extern "C"
{
    // 动态分配内存
    void allocate_variable(int size);

    // 释放内存
    void deallocate_variable();

    /*初始化can设备
        参数：无
        返回值：成功返回true，失败返回false
    */
    bool Start(); 

    /*断开can设备，也可以用作清除can设备占用
        参数：无
        返回值：成功返回true，失败返回false
    */
    bool Exit();               

    // bool brake(int size); 

    /*获取电机错误状态
        参数：无
        返回值：为电机错误数组
            0：无错误
            1：软件错误
            2：过压
            4：欠压
            16：启动错误
    */
    uint32_t* get_motor_error_status();

    /*清除电机错误
        参数：无
        返回值：无
    */
    void clear_motor_errors();

    // 写入调试信息到文件
    void writeDebugInfoToFile(const char *func_name, const char *info);

    // 输出数组的调试信息
    void printArrayDebugInfo(float arr[], int size, const char *arr_name);

    /*设置电机位置
        参数:
            motorIds 要设置的对应电机
            motorCount 电机数量
            targetPositions 电机目标位置
        返回值：成功返回true，失败返回false
    */
    bool set_motor_position(int* motorIds,  int motorCount,uint32_t* targetPositions);

    /*读取电机电流
        参数：无
        返回值：电机电流数组
    */
    uint32_t* get_motor_current();

    /*
    获取电机位置
        参数：无
        返回值：电机位置数组( 转化为减速机角度公式：(返回值/65536/减速比)*360 )
    */
    uint32_t* get_motor_position();
    // int32_t* get_motor_position_new();//改为有符号类型的getParameter

    /*获取电机速度
        参数：无
        返回值：电机速度数组
    */
    uint32_t* get_motor_speed();

    /*设置电机为电流模式并设置电流
        参数:
            motorIds 要设置的对应电机
            motorCount 电机数量
            targetPositions 电机目标电流，单位：毫安ma
        返回值：成功返回true，失败返回false
    */
    bool set_motor_current(int* motorIds,  int motorCount,uint32_t* current);

    /*设置电机速度
        参数:
            motorIds 要设置的对应电机
            motorCount 电机数量
            targetPositions 电机目标速度，单位：rpm(下发参数为：(目标转速（度每秒）*减速比*100)/360)
        返回值：成功返回true，失败返回false
    */
    bool set_motor_speed(int* motorIds,  int motorCount,uint32_t* speed);

    /*查询can设备号
        参数：无
        返回值：can设备序列号
    */
    std::vector<std::string> query_can();
}
#endif
