/*多电机，位置接口，状态接口，暂停接口，恢复接口 C++版*/
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include<sstream>
#include<fstream>
#include<cmath>
#include<iostream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include "Ti5BASIC.h"

using namespace std;

int main()
{
    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty()) {
        cout <<"未找到任何 USB 设备，请插入设备后重试！" << endl;
        exit(0);
    } else {
        cout << "找到的 CAN 设备序列号：" ;
        for (const string& serialNumber : productSerialNumbers) {
            cout <<serialNumber << endl;
        }
    }

    setIDNUM(2);
    cout<<"IDNUM="<<IDNUM<<endl;


    if(start()){
        cout<<"start success"<<endl;
    }

    // get_motor_current_position(IDNUM);
    get_motor_speed();
    // set_motor_speed(0,oripos2,1,278);
    // set_motor_speed(1,oripos2,1,278);
    get_motor_current_position(2);
    get_motor_speed();
    int num[1]={1};
    uint32_t newpos[2]={0};
    set_motor_position(num,1,newpos);
    sleep(3);
    int num2[1]={2};
    uint32_t newpos2[2]={0};
    set_motor_position(num2,1,newpos2);
    sleep(3);
    int num3[2]={1,2};
    uint32_t newpos3[2]={4000000,4000000};
    set_motor_position(num3,2,newpos3);
    sleep(3);
    get_motor_current_position(2);
    if(logout()){
        cout<<"logout success"<<endl;
    }

    return 0;
}

// bool joint_movement(uint32_t *oripos)
// {
//     static uint8_t canidList[10]; // 确保canidList已经初始化并填充数据
//     SCanInterface *canInterface = SCanInterface::getInstance();
    
//     // 调用setParameter以控制所有10个电机
//     canInterface->setParameter(canidList, oripos, MotorTypeHelper::REG_TARGET_POSITION, 10);
    
//     return true;
// }
// 在这个示例中，setParameter函数会一次性处理所有10个电机。确保canidList和oripos都已经正确初始化并包含相应的电机ID和目标位置。

// 要单独控制第3个电机（假设电机从0开始索引），你需要从canidList中提取第3个电机的canid，并传递相应的目标位置。可以使用临时数组来存储第3个电机的canid和目标位置，然后调用setParameter函数。以下是如何实现的代码示例：
// bool control_single_motor(uint32_t *oripos)
// {
//     static uint8_t canidList[10]; // 确保canidList已经初始化并填充数据
//     SCanInterface *canInterface = SCanInterface::getInstance();
    
//     // 选择第3个电机（索引2）
//     uint8_t motorId = 2; // 第3个电机的索引
//     uint8_t motorCanid = canidList[motorId];
//     uint32_t motorPosition = oripos[motorId];
    
//     // 创建临时数组存储单个电机的canid和目标位置
//     uint8_t singleMotorCanidList[1] = { motorCanid };
//     uint32_t singleMotorParameterList[1] = { motorPosition };
    
//     // 调用setParameter控制第3个电机
//     canInterface->setParameter(singleMotorCanidList, singleMotorParameterList, MotorTypeHelper::REG_TARGET_POSITION, 1);
    
//     return true;
// }
// 在这个代码中，我们:

// 选择第3个电机: 从canidList中获取对应的canid，并从oripos中获取目标位置。
// 创建临时数组: singleMotorCanidList和singleMotorParameterList分别用于存储单个电机的canid和目标位置。
// 调用setParameter: 传递临时数组和其他必要的参数以控制第3个电机。
// 这样，你可以精确地控制指定的电机，而不影响其他电机。


// bool control_motors(uint32_t *oripos, int motorIndex = -1)
// {
//     static uint8_t canidList[10]; // 确保canidList已经初始化并填充数据
//     SCanInterface *canInterface = SCanInterface::getInstance();

//     if (motorIndex == -1) // 控制所有电机
//     {
//         canInterface->setParameter(canidList, oripos, MotorTypeHelper::REG_TARGET_POSITION, 10);
//     }
//     else // 控制单个电机
//     {
//         uint8_t singleMotorCanidList[1] = { canidList[motorIndex] };
//         uint32_t singleMotorParameterList[1] = { oripos[motorIndex] };
//         canInterface->setParameter(singleMotorCanidList, singleMotorParameterList, MotorTypeHelper::REG_TARGET_POSITION, 1);
//     }

//     return true;
// }
// 在这个代码中：

// motorIndex: 如果motorIndex是-1，则控制所有电机；否则，控制指定的单个电机。
// 控制所有电机: 调用setParameter函数，传递所有电机的ID和目标位置。
// 控制单个电机: 创建临时数组，只包含指定电机的ID和目标位置，然后调用setParameter。
// 你可以通过传递motorIndex值来选择是控制所有电机还是单个电机。

// #include <iostream>

// class SCanInterface
// {
// public:
//     static SCanInterface* getInstance() {
//         static SCanInterface instance;
//         return &instance;
//     }

//     void setParameter(uint8_t* canidList, uint32_t* parameters, int parameterType, int numMotors) {
//         // 实际的控制逻辑
//         for (int i = 0; i < numMotors; ++i) {
//             std::cout << "Setting motor with CAN ID " << static_cast<int>(canidList[i])
//                       << " to position " << parameters[i] << std::endl;
//         }
//     }
// };

// namespace MotorTypeHelper {
//     const int REG_TARGET_POSITION = 1; // 示例值
// }

// bool control_motors(uint32_t* oripos, int size, int motorIndex = -1)
// {
//     // 动态分配和初始化 canidList
//     uint8_t* canidList = new uint8_t[size];
//     for (int i = 0; i < size; ++i)
//     {
//         canidList[i] = i + 1;
//     }

//     SCanInterface* canInterface = SCanInterface::getInstance();

//     if (motorIndex == -1) // 控制所有电机
//     {
//         canInterface->setParameter(canidList, oripos, MotorTypeHelper::REG_TARGET_POSITION, size);
//     }
//     else if (motorIndex >= 0 && motorIndex < size) // 控制单个电机
//     {
//         uint8_t singleMotorCanidList[1] = { canidList[motorIndex] };
//         uint32_t singleMotorParameterList[1] = { oripos[motorIndex] };
//         canInterface->setParameter(singleMotorCanidList, singleMotorParameterList, MotorTypeHelper::REG_TARGET_POSITION, 1);
//     }
//     else
//     {
//         std::cerr << "Error: motorIndex is out of range." << std::endl;
//     }

//     delete[] canidList; // 释放动态分配的内存
//     return true;
// }

// int main() {
//     int size = 10; // 电机的数量
//     uint32_t oripos[10];
//     for (int i = 0; i < size; ++i)
//     {
//         oripos[i] = (i + 1) * 100; // 示例目标位置
//     }

//     // 控制所有电机
//     control_motors(oripos, size);

//     // 控制单个电机（例如第3个电机）
//     control_motors(oripos, size, 2);

//     return 0;
// }
// 要将 set_motor_position 函数修改为接受 int* idnum 类型的参数，并根据传入的数组长度来控制多个电机，你可以按照以下步骤进行修改：

// 修改参数类型为 int* idnum 和另一个参数来表示 idnum 数组的长度。
// 在函数内部，使用一个循环来遍历 idnum 数组，并相应地设置电机参数。
// 下面是修改后的代码示例：

// cpp
// bool set_motor_position(int* idnum,  int idnum_length,uint32_t* elc_value)
// {
//     SCanInterface *canInterface = SCanInterface::getInstance();
//     uint32_t *elc_parameterlist= new uint32_t[IDNUM]();
//     for (int i = 0; i < idnum_length; ++i)
//     {
//         int current_idnum = idnum[i] -1 ;
//         if (current_idnum >= 0 && current_idnum < IDNUM) // 验证 idnum 范围
//         {
//             elc_parameterlist[current_idnum] = elc_value[i];
//             uint32_t *new_elc_parameterlist = &elc_parameterlist[current_idnum];
//             uint8_t *new_canidList = &canidList[current_idnum];
            
//             canInterface->setParameter(new_canidList, new_elc_parameterlist, MotorTypeHelper::REG_TARGET_POSITION, idnum_length);
//         }
//         else
//         {
//             cout << "Error: idnum " << current_idnum << " out of range." << endl;
//             delete[] elc_parameterlist; // 清理内存
//             return false; // 处理无效的 idnum
//         }
//     }
//     delete[] elc_parameterlist;
//     elc_parameterlist = nullptr;
//     return true;
// }
// 关键变化：
// 参数更改：idnum 改为 int* idnum，并添加 int idnum_length 来表示数组长度。
// 控制电机的逻辑：使用循环遍历 idnum 数组，并对每个电机进行设置。
// 这样，你就可以传入不同长度的 idnum 数组来控制多个电机了。