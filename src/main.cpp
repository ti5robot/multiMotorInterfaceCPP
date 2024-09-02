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
    setIDNUM(2);
    cout<<"IDNUM="<<IDNUM<<endl;
    allocate_variable(IDNUM);

    if(login()){
        cout<<"login success"<<endl;
    }
    
    //get_elc_info(IDNUM);
    //int q = get_elektrische_Maschinen_status(IDNUM);
    //clear_elc_error(IDNUM);
    //set_elc_info(ampere, 0, MotorTypeHelper::REG_TARGET_CURRENT, 0);
    //TH.j[0]=1.2;
    //move_to_joint();
    float arr2[2]={2.2,2.2};
    joint_movement(arr2);
    deallocate_variable();
    if(logout()){
        cout<<"logout success"<<endl;
    }

    return 0;
}
