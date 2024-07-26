---
sort: 1
---

# SDK介绍

机械臂控制的代码code中，分别是`include`，`src`，`log`以及`usrlib`。

+ [include](https://github.com/mrhouse-sweet/mechanical_arm_SDK-docs/tree/main/code/include) 存储着机械臂所需的头文件。
+ [src](https://github.com/mrhouse-sweet/mechanical_arm_SDK-docs/tree/main/code/src) 一般控制机械臂的文件放在此处，其中`main.cpp`是一个示例程序。
+ [log](https://github.com/mrhouse-sweet/mechanical_arm_SDK-docs/tree/main/code/log) sdk中存放log的文件夹。
+ [usrlib](https://github.com/mrhouse-sweet/mechanical_arm_SDK-docs/tree/main/code/usrlib)包含SDK所需的so文件

## 1. include

除以下提到的文件外，用户无需查看该文件夹下的其他文件。

### 1.1 mathfunc.h
机械臂的数学模型函数


### 1.2 Ti5BASIC.h

机械臂控制基础库，包含了基本控制以及信息，用户在使用时需要根据自身使用方式自行选择调用。

+ void allocate_variable(int size);
  ```
  函数功能：动态分配内存
  返回值：true停止，false未停止
  参数：无
  示例：bool result=inspect_brake()
  ```

+ void writeDebugInfoToFile(const char *func_name, const char *info);
  ```
  函数功能：将信息写入log中
  返回值：无
  参数：
      *func_name：函数名字
      *info：要写入log的信息内容
  示例：
      getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, IDNUM);
      for (int i = 0; i < 6; i++)
      {
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      }
      cout << endl;
  ```

+ void login();
  ```
  函数功能：登录can设备
  返回值：无
  参数：无
  示例：
      login();
      getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, IDNUM);
      for (int i = 0; i < 6; i++)
      {
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      }
      cout << endl;
  ```

+ void logout();//函数功能：登出can设备
    ```
  函数功能：登录can设备
  返回值：无
  参数：无
  示例：
      login();
      getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, IDNUM);
      for (int i = 0; i < 6; i++)
      {
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      }
      cout << endl;
      logout();
  ```



+ void get_elc_info(int size);
  ```
  函数功能：获取电机参数
  返回值：无
  参数：电机总个数
  示例：
      int main()
      {
          login();
          get_elc_info(6);
          logout();
          return 0;
      }
  ```

+ void set_elc_info(uint32_t *elc_parameterlist,int elc_num, int parameterType,uint32_t elc_value);
   ```
   函数功能：设置电机参数
   返回值：
   参数：
       elc_parameterlist 要设置的对应电机，
       elc_value 要设置的第n个电机，
       parameterType 要设置的项目，
       elc_value 新值
   示例：
       int main()
       {
           login();
           set_elc_info(reg_position_kp,0,MotorTypeHelper::REG_POSITION_KP, 500);
           set_elc_info(reg_position_kp,1,MotorTypeHelper::REG_POSITION_KP, 700);
           set_elc_info(reg_position_kp,2,MotorTypeHelper::REG_POSITION_KP, 800);
           set_elc_info(reg_position_kp,3,MotorTypeHelper::REG_POSITION_KP, 900);
           set_elc_info(reg_position_kp,4,MotorTypeHelper::REG_POSITION_KP, 1000);
           set_elc_info(reg_position_kp,5,MotorTypeHelper::REG_POSITION_KP, 1100);
           logout();
           return 0;
       }
   ```

+ int get_elektrische_Maschinen_status(int size);
  ```
  函数功能：获取电机错误状态
  返回值：
      0：无错误
      1：软件错误
      2：过压
      4：欠压
      16：启动错误
  参数：电机总个数
  示例：
      int main()
      {
           login();
           int num = get_elektrische_Maschinen_status();
           logout();
           return 0;
       }
  ```
+ void clear_elc_error(int size);
  ```
  函数功能：清除电机错误
  返回值：无
  参数：点击总个数
  示例：
      int main()
      {
          login();
          if(get_electricity_status() != true)
          {
              cout << "电机异常！" << endl;
          }
          clear_elc_erro();
          logout();
          return 0;
       }
  ```

+ void joint_movement(const float *arr);
   ```
  函数功能：机械臂关节运动
  返回值：无
  参数：各个关节值
  示例：
     int main()
      {
          login();
          mechanical_arm_origin();
          float arr[6]={2.3,3.2,2.2,0,0,0};
          joint_movement(arr);
          logout();
          return 0;
       }
  ```


### 1.3 Ti5LOGIC.h

该文件是机械臂的算法库
使用方法：根据需求在规划机械臂运动的时候可以调用该库的函数。




### 1.4 tool.h

该文件是一些`tool`，具体函数使用及参数请查看该文件。

### 1.5 can
该文件夹包含`can`通讯的头文件，机械臂是通过can通讯与控制机联通的，具体函数功能及参数请查看里面所包含的文件中注释了解函数作用。


## 2.src
### 2.1 main.cpp

该文件是一个简单的示例程序

### 2.2 gcc.sh

该文件中的内容是编译命令，编译的时候可以使用该命令直接编译，也可以使用g++命令+对应参数直接编译

### 2.3 编译

最后执行`gcc.sh`文件进行编译或通过以下命令进行编译生成可执行文件`move_sov`。(注意：以下路径是默认路径，如果修改了路径要替换成自己的)
```
g++ main.cpp  Ti5BASIC.cpp Ti5LOGIC.cpp  mathfunc.cpp  tool.cpp -L../include/can -lmylibscan -lcontrolcan -o multi_motor
```
**运行**:
```
sudo ./multi_motor
```
注意机械臂处在一个安全的环境中

## 开发须知
（1）代码中一切角大小相关的量采用弧度制（bais由角度制自动转向弧度制）。

（2）机械臂的一切控制基于robotArm类，传参方式为直接改变TH.j或TH.pos的值。

（3）TH.j与TH.pos代表数学模型的理论值，需要借由bais校准到实际发送的理论值。

（4）相邻两次指令发送时间低于USLEEPTIME可能信息堵塞。

（5）由于电机内部的算法，指令发送的值与实际执行的值有微小误差。

（6）位置环与速度环都是直接控制电机内圈，因此从外到内涉及scale的放缩。“位置环参数”单位为“步”，“速度环参数/100”单位为“圈/秒”。一圈65536步。
