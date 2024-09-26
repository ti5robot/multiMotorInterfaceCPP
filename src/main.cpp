/*多电机*/
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

    setmotorTotal(2);
    cout<<"IDNUM="<<IDNUM<<endl;


    if(Start()){
        cout<<"start success"<<endl;
    }

    get_motor_speed();

    get_motor_position();
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
    get_motor_position();

    // 调用 get_motor_position 函数
    uint32_t* get_positions = get_motor_position();

    // 检查返回的指针是否为 nullptr
    if (get_positions != nullptr)
    {
        // 使用 positions 数组
        for (int i = 0; i < 2; i++)
        {
            std::cout << "main函数中电机" << (i + 1) << "的位置: " << static_cast<int32_t>(get_positions[i]) << std::endl;
        }
    }
    else
    {
        std::cerr << "main函数中获取电机位置失败！" << std::endl;
    }

    uint32_t* current_values = get_motor_current();
    if(current_values != nullptr)
    {
        for (int i = 0; i < 2; i++)
        {
            std::cout << "main函数中电机" << (i + 1) << "的电流: " << static_cast<int32_t>(current_values[i]) << std::endl;
        }
    }
    else{
        std::cerr << "main函数中获取电机电流失败！" << std::endl;
    }

    uint32_t* speed_values = get_motor_speed();
    if(speed_values != nullptr)
    {
        for (int i = 0; i < 2; i++)
        {
            std::cout << "main函数中电机" << (i + 1) << "的速度: " << static_cast<int32_t>(speed_values[i]) << std::endl;
        }
    }
    else{
        std::cerr << "main函数中获取电机速度失败！" << std::endl;
    }

    uint32_t* errorStatuses = get_motor_error_status();
    for (int i = 0; i < IDNUM; i++) {
        cout << "电机 " << i + 1 << " 状态码: " << errorStatuses[i] << endl;
    }
    clear_motor_errors();
    if(Exit()){
        cout<<"exit success"<<endl;
    }

    return 0;
}
