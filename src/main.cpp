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

    get_motor_speed();

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
