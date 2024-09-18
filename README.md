---
sort: 1
---

# SDK介绍

机械臂控制的代码中，分别是`include`，`src`，`log`以及`usrlib`。

+ include存储着机械臂所需的头文件。
+ src一般控制机械臂的文件放在此处，其中`main.cpp`是一个示例程序。
+ logsdk中存放log的文件夹。
+ usrlib包含SDK所需的so文件

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
  参数：动态分配内存长度
  示例：
      int main()
      {
          allocate_variable(6);

          login();
          cout << "登录成功" << endl;
          get_elc_info(6);
          int q = get_elektrische_Maschinen_status(6);
          clear_elc_error(6);
          set_elc_info(ampere, 0, MotorTypeHelper::REG_TARGET_CURRENT, 0);
          TH.j[0]=1.2;
          move_to_joint();
          deallocate_variable();
          logout();

          return 0;
      }
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
      for (int i = 0; i < IDNUM; i++)
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
      for (int i = 0; i < IDNUM; i++)
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
      for (int i = 0; i < IDNUM; i++)
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

      下面是目标速度和目标位置的parameterType：
        REG_TARGET_SPEED 设置目标速度  下发参数为： (目标转速（度每秒）*减速比*100)/360
        REG_TARGET_POSITION 设置目标位置  下发参数为： (减速机目标角度/360)*减速比*65536
       例如：
        set_elc_info(reg_position_kp,5,MotorTypeHelper::REG_TARGET_SPEED, 目标转速);
        set_elc_info(reg_position_kp,5,MotorTypeHelper::REG_TARGET_POSITION, 目标位置);
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
           int num = get_elektrische_Maschinen_status(6);
           logout();
           return 0;
       }
  ```
+ void clear_elc_error(int size);
  ```
  函数功能：清除电机错误
  返回值：无
  参数：电机总个数
  示例：
      int main()
      {
          login();
          if(get_electricity_status() != true)
          {
              cout << "电机异常！" << endl;
          }
          clear_elc_erro(6);
          logout();
          return 0;
       }
  ```

+ void joint_movement(const float *arr);
   ```
  函数功能：电机运动
  返回值：无
  参数：各个关节值
  示例：
     int main()
      {
          login();
          float arr[6]={2.3,3.2,2.2,0,0,0};
          joint_movement(arr);
          logout();
          return 0;
       }
  ```

+  bool set_motor_position(int idnum,uint32_t *elc_parameterlist, int elc_num, uint32_t elc_value);
   ```
    函数功能：设置电机位置
    返回值：成功true，失败false
    参数：
      idnum 要设置的对应电机(如果值为-1则设置所有电机，同时elc_value要传入电机总个数)
      elc_parameterlist 要设置的对应电机
      elc_value 要设置的n个电机
      elc_value 新值(下发参数为：(减速机目标角度/360)*减速比*65536)
    示例：
     int main()
      {
          setIDNUM(2);
          cout<<"IDNUM="<<IDNUM<<endl;
          allocate_variable(IDNUM);

          if(login()){
            cout<<"login success"<<endl;
          }
          uint32_t oripos[IDNUM]={0};
          uint32_t oripos2[IDNUM]{0};
          set_motor_position(0,oripos,1,1000000);

          sleep(2);
          set_motor_position(1,oripos2,1,-1000000);
          deallocate_variable();
          if(logout()){
            cout<<"logout success"<<endl;
          }
          return 0;
       }
    ```

   +  void set_motor_current(int idnum,uint32_t *elc_parameterlist, int elc_num, uint32_t elc_value);
   ```
    函数功能：设置电机为电流模式并设置电流
    返回值：无
    参数：
      idnum 要设置的对应电机(如果值为-1则设置所有电机，同时elc_value要传入电机总个数)
      elc_parameterlist 要设置的对应电机
      elc_value 要设置的n个电机
      elc_value 新值
    示例：
     int main()
      {
          setIDNUM(2);
          cout<<"IDNUM="<<IDNUM<<endl;
          allocate_variable(IDNUM);

          if(login()){
            cout<<"login success"<<endl;
          }
          uint32_t oripos[IDNUM]={0};
          uint32_t oripos2[IDNUM]{0};
          set_motor_current(0,oripos,1,10);
          set_motor_current(1,oripos2,1,10);
          deallocate_variable();
          if(logout()){
            cout<<"logout success"<<endl;
          }
          return 0;
       }
    ```

   +  void set_motor_speed(int idnum,uint32_t *elc_parameterlist, int elc_num, uint32_t elc_value);
   ```
    函数功能：设置电机速度
    返回值：无
    参数：
      idnum 要设置的对应电机(如果值为-1则设置所有电机，同时elc_value要传入电机总个数)
      elc_parameterlist 要设置的对应电机
      elc_value 要设置的n个电机
      elc_value 新值(下发参数为：(目标转速（度每秒）*减速比*100)/360)
    示例：
     int main()
      {
          setIDNUM(2);
          cout<<"IDNUM="<<IDNUM<<endl;
          allocate_variable(IDNUM);

          if(login()){
            cout<<"login success"<<endl;
          }
          uint32_t oripos[IDNUM]={0};
          uint32_t oripos2[IDNUM]{0};
          set_motor_speed(0,oripos,1,278);

          sleep(2);
          set_motor_speed(1,oripos2,1,278);
          deallocate_variable();
          if(logout()){
            cout<<"logout success"<<endl;
          }
          return 0;
       }
    ```
    
    

### 1.3 Ti5LOGIC.h

该文件是算法库，不需要使用。


### 1.4 tool.h

该文件是一些`tool`，具体函数使用及参数请查看该文件。
+ void setIDNUM(int newIDNUM);
```
函数功能：设置IDNUM
返回值：无
参数：newIDNUM - 新的IDNUM即电机个数
示例：
  int main()
{
    setIDNUM(2);//两个IDNUM
    cout<<"IDNUM="<<IDNUM<<endl;
    allocate_variable(IDNUM);

    if(login()){
        cout<<"login success"<<endl;
    }
    float arr2[2]={2.2,2.2};
    joint_movement(arr2);
    deallocate_variable();
    if(logout()){
        cout<<"logout success"<<endl;
    }

    return 0;
}
```

### 1.5 can
该文件夹包含`can`通讯的头文件，机械臂是通过can通讯与控制机联通的，具体函数功能及参数请查看里面所包含的文件中注释了解函数作用。


## 2.src
### 2.1 main.cpp

该文件是一个简单的示例程序

### 2.2 gcc.sh

该文件中的内容是编译命令，编译的时候可以使用该命令直接编译，也可以使用g++命令+对应参数直接编译

### 2.3 依赖库安装
+ `将usrlib中的libcontrolcan.so  libmylibscan.so  libmylibti5_multi_motor.so文件拷贝到/usr/lib/下`

```bash
cd ~/multi_motor/usrlib
sudo cp * /usr/lib
```

### 2.4 编译

最后执行`gcc.sh`文件进行编译或通过以下命令进行编译生成可执行文件`multi_motor`。(注意：以下路径是默认路径，如果修改了路径要替换成自己的)
```
export CPLUS_INCLUDE_PATH=~/multiMotorInterfaceCPP/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/multiMotorInterfaceCPP/include/can
g++ main.cpp -L./include -lmylibti5_multi_motor -L../include/can -lmylibscan -lcontrolcan -o multi_motor
```
**运行**:
```
sudo ./multi_motor
```
