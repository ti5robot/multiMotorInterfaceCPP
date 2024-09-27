---
sort: 1
---

# SDK介绍

运行环境ubuntu22.04；can分析仪为：can分析仪（创芯科技）linux版

机械臂控制的代码中，分别是`include`，`src`，`log`以及`usrlib`。

+ include存储着机械臂所需的头文件。
+ src一般控制机械臂的文件放在此处，其中`main.cpp`是一个示例程序。
+ logsdk中存放log的文件夹。
+ usrlib包含SDK所需的so文件

## 1. include

除以下提到的文件外，用户无需查看该文件夹下的其他文件。

### 1.1 Ti5BASIC.h

机械臂控制基础库，包含了基本控制以及信息，用户在使用时需要根据自身使用方式自行选择调用。

+ void writeDebugInfoToFile(const char *func_name, const char *info);
  ```
  函数功能：将信息写入log中
  返回值：无
  参数：
      *func_name：函数名字
      *info：要写入log的信息内容
  示例：
      writeDebugInfoToFile(__func__, "LogInfo");//调用writeDebugInfoToFile将内容写入log文件中
  ```

+ bool Start();
  ```
  函数功能：初始化can设备
  返回值：成功返回true，失败返回false
  参数：无
  示例：
      int main()
      {
        int num_of_motors=2;
        setmotorTotal(num_of_motors);  //设置电机总个数为2个
        if(Start()){  //初始化can设备
             cout<<"Start success"<<endl;
         }
        cout<<"程序开始"<<endl;
        if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
        }
        return 0;
      }
  ```

+ bool Exit();
    ```
  函数功能：断开can设备，也可以用作清除can设备占用
  返回值：成功返回true，失败返回false
  参数：无
  示例：
      int main()
      {
        int num_of_motors=2;
        setmotorTotal(num_of_motors);  //设置电机总个数为2个
        if(Start()){  //初始化can设备
             cout<<"Start success"<<endl;
         }
        cout<<"程序开始"<<endl;
        if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
        }
        return 0;
      }
  ```

+ uint32_t* get_motor_error_status();
  ```
  函数功能：获取电机错误状态
  返回值：
      0：无错误
      1：软件错误
      2：过压
      4：欠压
      16：启动错误
  参数：无
  示例：
      int main()
      {
           int num_of_motors=2;
           setmotorTotal(num_of_motors);  //设置电机总个数为2个
           Start();  //初始化can设备
           uint32_t* errorStatuses = get_motor_error_status();//调用get_motor_error_status函数获取所有电机状态码
           for (int i = 0; i < num_of_motors; i++) {
              cout << "电机 " << i + 1 << " 状态码: " << errorStatuses[i] << endl;
           }
           Exit();  //断开can设备连接
           return 0;
       }
  ```
+ void clear_motor_errors();
  ```
  函数功能：清除电机错误
  返回值：无
  参数：无
  示例：
      int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          Start();  //初始化can设备
          if(get_motor_error_status() != true) //调用get_motor_error_status获取电机错误状态
          {
              cout << "电机异常！" << endl;
          }
          clear_motor_errors();//清除电机的错误
          Exit();  //断开can设备连接
          return 0;
       }
  ```

+  bool set_motor_position(int* motorIds,  int motorCount,uint32_t* targetPositions);
   ```
    函数功能：设置电机位置
    返回值：成功true，失败false
    参数：
      motorIds 要设置的对应电机
      motorCount 电机数量
      targetPositions 电机目标位置(下发参数为：(减速机目标角度/360)*减速比*65536)
    示例：
     int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          if(Start()){  //初始化can设备
            cout<<"Start success"<<endl;
          }
          int num[1]={1};
          uint32_t newpos[2]={0};
          set_motor_position(num,1,newpos);//设置一个电机
          sleep(3);
          int num3[2]={1,2};
          uint32_t newpos3[2]={4000000,4000000};
          set_motor_position(num3,2,newpos3);//设置两个电机
          if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
          }
          return 0;
       }
    ```

   +   bool set_motor_current(int* motorIds,  int motorCount,uint32_t* current);
   ```
    函数功能：设置电机为电流模式并设置电流
    返回值：成功返回true，失败返回false
    参数：
      motorIds 要设置的对应电机
      motorCount 电机数量
      current 电机目标电流,单位：毫安ma
    示例：
     int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          if(Start()){  //初始化can设备
            cout<<"Start success"<<endl;
          }
          int num[1]={1};
          uint32_t newcurrent[2]={0};
          set_motor_current(num,1,newcurrent);//设置一个电机
          int num3[2]={1,2};
          uint32_t newcurrent3[2]={100,100};
          set_motor_current(num3,2,newcurrent3);//设置两个电机
          if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
          }
          return 0;
       }
    ```

   +  bool set_motor_speed(int* motorIds,  int motorCount,uint32_t* speed);
   ```
    函数功能：设置电机速度
    返回值：无
    参数：
      motorIds 要设置的对应电机
      motorCount 电机数量
      speed 电机目标速度,单位：rpm(下发参数为：(目标转速（度每秒）*减速比*100)/360)
    示例：
     int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          if(Start()){  //初始化can设备
            cout<<"Start success"<<endl;
          }
          int num[1]={1};
          uint32_t newspeed[2]={0};
          set_motor_speed(num,1,newspeed);//设置一个电机
          sleep(3);
          int num3[2]={1,2};
          uint32_t newspeed3[2]={100,100};
          set_motor_speed(num3,2,newspeed3);//设置两个电机
          if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
          }
          return 0;
       }
    ```

    + uint32_t* get_motor_current();
    ```
    函数功能：读取电机电流
    返回值：return MotorCurrent; // 返回所有电机的电流值
    参数：无
    示例：
      int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          if(Start()){  //初始化can设备
            cout<<"Start success"<<endl;
          }
          uint32_t* current_values = get_motor_current();  //调用get_motor_current函数获取所有电机电流值
          if(current_values != nullptr)
          {
            for (int i = 0; i < num_of_motors; i++)
            {
              std::cout << "main函数中电机" << (i + 1) << "的电流: " << static_cast<int32_t>(current_values[i]) << std::endl;
            }
          }
          else{
            std::cerr << "main函数中获取电机电流失败！" << std::endl;
          }
          if(Exit()){   //断开can设备连接
            cout<<"Exit success"<<endl;
          }
          return 0;
      }
    ```

    + uint32_t* get_motor_position();
    ```
    函数功能：获取电机位置
    返回值：return MotorPosition; // 返回电机位置数组
    参数：无
    示例：
      int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          if(Start()){  //初始化can设备
            cout<<"Start success"<<endl;
          }
          // 调用 get_motor_position 函数
          uint32_t* get_positions = get_motor_position();  //调用get_motor_position函数获取所有电机位置

          if (get_positions != nullptr)
          {
              for (int i = 0; i < num_of_motors; i++)
              {
                  std::cout << "main函数中电机" << (i + 1) << "的位置: " << static_cast<int32_t>(get_positions[i]) << std::endl;
              }
          }
          else
          {
              std::cerr << "main函数中获取电机位置失败！" << std::endl;
          }
          if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
          }
          return 0;
      }
    ```

    + uint32_t* get_motor_speed();
    ```
    函数功能：获取电机速度
    返回值：return MotorSpeed; // 返回所有电机的速度
    参数：无
    示例：
      int main()
      {
          int num_of_motors=2;
          setmotorTotal(num_of_motors);  //设置电机总个数为2个
          if(Start()){  //初始化can设备
            cout<<"Start success"<<endl;
          }
          uint32_t* speed_values = get_motor_speed();  //调用get_motor_speed函数获取所有电机速度
          if(speed_values != nullptr)
          {
              for (int i = 0; i < num_of_motors; i++)
              {
                  std::cout << "main函数中电机" << (i + 1) << "的速度: " << static_cast<int32_t>(speed_values[i]) << std::endl;
              }
          }
          else{
              std::cerr << "main函数中获取电机速度失败！" << std::endl;
          }

          if(Exit()){  //断开can设备连接
            cout<<"Exit success"<<endl;
          }
          return 0;
      }
    ```
    
    

### 1.2 Ti5LOGIC.h

该文件是算法库，不需要使用。


### 1.3 tool.h

该文件是一些`tool`，具体函数使用及参数请查看该文件。
+ void setmotorTotal(int Total);
```
函数功能：设置电机总个数
返回值：无
参数：Total 电机总个数
示例：
  int main()
{
    setmotorTotal(2);  //设置电机总个数为2个
    if(Start()){  //初始化can设备
        cout<<"login success"<<endl;
    }
    if(Exit()){  //断开can设备连接
        cout<<"Exit success"<<endl;
    }

    return 0;
}
```

### 1.4 can
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
