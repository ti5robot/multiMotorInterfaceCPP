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

extern uint32_t *motor_current_position; // 电机当前位置

extern "C"
{
    // 动态分配内存
    void allocate_variable(int size);

    // 释放内存
    void deallocate_variable();

    bool start();                // 连接can设备
    bool logout();               // 断开can设备
    // void get_elc_info(int size); // 获取信息
    bool brake(int size);        // 刹车

    /*获取电机错误状态
        参数：
            size：电机数量
        返回值：为电机错误
        0：无错误
        1：软件错误
        2：过压
        4：欠压
        16：启动错误
    */
    int get_motor_error_status(int size);

    /*清除电机错误
        参数：
            size：电机数量
    */
    void clear_motor_errors(int size);

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

    /*读取电机电流*/
    std::vector<float> get_motor_current();
    // float get_motor_current_1(int motor_index);//读取指定电机电流

    /*
    获取电机位置
    参数：
        size：电机数量
    */
    void get_motor_current_position(int size);

    /*获取电机速度*/
    std::vector<float> get_motor_speed();

    /*设置电机为电流模式并设置电流
        参数:
            motorIds 要设置的对应电机
            motorCount 电机数量
            targetPositions 电机目标电流
        返回值：成功返回true，失败返回false
    */
    bool set_motor_current(int* motorIds,  int motorCount,uint32_t* current);

    /*设置电机速度
        参数:
            motorIds 要设置的对应电机
            motorCount 电机数量
            targetPositions 电机目标速度(下发参数为：(目标转速（度每秒）*减速比*100)/360)
        返回值：成功返回true，失败返回false
    */
    bool set_motor_speed(int* motorIds,  int motorCount,uint32_t* speed);

    /*std::string query_can();
    查询can设备号
    */
    std::vector<std::string> query_can();
}
#endif
