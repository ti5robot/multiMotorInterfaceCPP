#ifndef _TI5BASIC_H_
#define _TI5BASIC_H_

#include <iostream>
#include <unistd.h>
#include <cstring>
#include <vector>

#include "can/SingleCaninterface.h"
#include "can/motortypehelper.h"

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

// extern const int IDNUM;
// extern uint8_t canidList[IDNUM];

// extern uint32_t reg_position_kp[IDNUM]; // 位置环比例
// extern uint32_t reg_position_kd[IDNUM]; // 位置环微分

// extern uint32_t reg_speed_kp[IDNUM]; // 速度环比例
// extern uint32_t reg_speed_ki[IDNUM]; // 速度环积分

// extern uint32_t reg_max_curr_value[IDNUM]; // 最大正电流
// extern uint32_t reg_min_curr_value[IDNUM]; // 最小负电流

// extern uint32_t reg_max_app_speed[IDNUM]; // 最大正向允许速度
// extern uint32_t reg_min_app_speed[IDNUM]; // 最小负向允许速度

// extern uint32_t reg_max_app_position[IDNUM]; // 电机最大正向位
// extern uint32_t reg_min_app_position[IDNUM]; // 电机最大负向位

// extern uint32_t electricity[IDNUM];        // 电流值
// extern uint32_t electric_machinery[IDNUM]; // 电机错误状态
// extern uint32_t reg_fault_clear[IDNUM];    // 清除电机错误
// extern uint32_t ampere[IDNUM];             // 电机电流值

// extern uint32_t ele_status[IDNUM]; // 电机状态
// extern uint32_t ele_speed[IDNUM];  // 电机速度

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

extern uint32_t *electricity;        // 电流值
extern uint32_t *electric_machinery; // 电机错误状态
extern uint32_t *reg_fault_clear;    // 清除电机错误
extern uint32_t *ampere;             // 电机电流值

extern uint32_t *ele_status; // 电机状态
extern uint32_t *ele_speed;  // 电机速度

extern "C"
{
    // 动态分配内存
    void allocate_variable(int size);

    // 释放内存
    void deallocate_variable();

    bool login();                // 连接can设备
    bool logout();               // 断开can设备
    void get_elc_info(int size); // 获取信息
    bool brake(int size);        // 刹车

    /*设置电机参数
      参数:
        elc_parameterlist 要设置的对应电机
        elc_value 要设置的n个电机
        parameterType 要设置的项目
        elc_value 新值
    */
    void set_elc_info(uint32_t *elc_parameterlist, int elc_num, int parameterType, uint32_t elc_value);

    /*获取电机错误状态
    返回值：为电机错误
        0：无错误
        1：软件错误
        2：过压
        4：欠压
        16：启动错误
    */
    int get_elektrische_Maschinen_status(int size);

    /*清除电机错误*/
    void clear_elc_error(int size);

    // 写入调试信息到文件
    void writeDebugInfoToFile(const char *func_name, const char *info);

    // 输出数组的调试信息
    void printArrayDebugInfo(float arr[], int size, const char *arr_name);

    bool move_to_joint();
    void plan_move();
    void ACTmove(float *a, float *b, float T0); // 实际运动
    // void setn(int npL[IDNUM]);
    void setn(int* npL);
    /*机械臂关节运动
    参数：
        *arr：存放角度值的数组
    返回值：
        1：成功
        0：失败
    */
    bool joint_movement(const float *arr);
}
#endif