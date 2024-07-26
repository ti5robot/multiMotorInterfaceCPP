/*多电机，位置接口，状态接口，暂停接口，恢复接口 C++版*/
#include "Ti5BASIC.h"

using namespace std;
class robotArm1 TH;

char LogInfo[100];//存储写入log文件的信息

float n2p = 655.36;
float AG = 0.005; //>=4*USLEEPTIME   USLEEPTIME是和电机通信的时间(1个电机假如通信100ms，6个就是600ms)
float scale = 101;
float j2p = scale * 65536 / 2 / pi;
float NMAX = 2000;

// const int IDNUM = 6;
// uint8_t canidList[IDNUM];

// uint32_t reg_position_kp[IDNUM]; // 位置环比例
// uint32_t reg_position_kd[IDNUM]; // 位置环微分

// uint32_t reg_speed_kp[IDNUM]; // 速度环比例
// uint32_t reg_speed_ki[IDNUM]; // 速度环积分

// uint32_t reg_max_curr_value[IDNUM]; // 最大正电流
// uint32_t reg_min_curr_value[IDNUM]; // 最小负电流

// uint32_t reg_max_app_speed[IDNUM]; // 最大正向允许速度
// uint32_t reg_min_app_speed[IDNUM]; // 最小负向允许速度

// uint32_t reg_max_app_position[IDNUM]; // 电机最大正向位
// uint32_t reg_min_app_position[IDNUM]; // 电机最大负向位

// uint32_t electricity[IDNUM];        // 电流值
// uint32_t electric_machinery[IDNUM]; // 电机错误状态
// uint32_t reg_fault_clear[IDNUM];    // 清除电机错误
// uint32_t ampere[IDNUM];             // 电机电流值

// uint32_t ele_status[IDNUM]; // 电机状态
// uint32_t ele_speed[IDNUM];  // 电机速度

uint8_t* canidList;
uint32_t* reg_position_kp; // 位置环比例
uint32_t* reg_position_kd; // 位置环微分

uint32_t* reg_speed_kp; // 速度环比例
uint32_t* reg_speed_ki; // 速度环积分

uint32_t* reg_max_curr_value; // 最大正电流
uint32_t* reg_min_curr_value; // 最小负电流

uint32_t* reg_max_app_speed; // 最大正向允许速度
uint32_t* reg_min_app_speed; // 最小负向允许速度

uint32_t* reg_max_app_position; // 电机最大正向位
uint32_t* reg_min_app_position; // 电机最大负向位

uint32_t* electricity;        // 电流值
uint32_t* electric_machinery; // 电机错误状态
uint32_t* reg_fault_clear;    // 清除电机错误
uint32_t* ampere;             // 电机电流值

uint32_t* ele_status; // 电机状态
uint32_t* ele_speed;  // 电机速度

void allocate_variable(int size) {
    canidList = new uint8_t[size];

    for (int i = 0; i < size; i++)
    {
        canidList[i] = i + 1;
        cout << "canlist=" << static_cast<int>(canidList[i]) << endl;
    }
    reg_position_kp = new uint32_t[size];
    reg_position_kd = new uint32_t[size];
    reg_speed_kp = new uint32_t[size];
    reg_speed_ki = new uint32_t[size];
    reg_max_curr_value = new uint32_t[size];
    reg_min_curr_value = new uint32_t[size];
    reg_max_app_speed = new uint32_t[size];
    reg_min_app_speed = new uint32_t[size];
    reg_max_app_position = new uint32_t[size];
    reg_min_app_position = new uint32_t[size];
    electricity = new uint32_t[size];
    electric_machinery = new uint32_t[size];
    reg_fault_clear = new uint32_t[size];
    ampere = new uint32_t[size];
    ele_status = new uint32_t[size];
    ele_speed = new uint32_t[size];

}

void deallocate_variable() {
    delete[] canidList;
    canidList = nullptr;

    delete[] reg_position_kp;
    reg_position_kp = nullptr;

    delete[] reg_position_kd;
    reg_position_kd = nullptr;

    delete[] reg_speed_kp;  
    reg_speed_kp = nullptr;

    delete[] reg_speed_ki;
    reg_speed_ki = nullptr;

    delete[] reg_max_curr_value;
    reg_max_curr_value = nullptr;

    delete[] reg_min_curr_value;
    reg_min_curr_value = nullptr;

    delete[] reg_max_app_speed;
    reg_max_app_speed = nullptr;

    delete[] reg_min_app_speed;
    reg_min_app_speed = nullptr;

    delete[] reg_max_app_position;
    reg_max_app_position = nullptr;

    delete[] reg_min_app_position;  
    reg_min_app_position = nullptr;

    delete[] electricity;
    electricity = nullptr;

    delete[] electric_machinery;
    electric_machinery = nullptr;

    delete[] reg_fault_clear;
    reg_fault_clear = nullptr;

    delete[] ampere;
    ampere = nullptr;

    delete[] ele_status;
    ele_status = nullptr;

    delete[] ele_speed;
    ele_speed = nullptr;
}

void writeDebugInfoToFile(const char *func_name, const char *info)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];
    char filename_buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(filename_buffer, sizeof(filename_buffer), "%Y-%m-%d", timeinfo); // 只获取日期部分作为文件名
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);          // 获取完整的时间戳

    // 构造文件名
    char filename[100];
    // sprintf(filename, "%s.log", filename_buffer);
    sprintf(filename, "../log/%s.log", filename_buffer); // 指定log文件的位置

    // 判断文件是否存在
    if (access(filename, F_OK) == -1)
    {
        // 文件不存在，创建文件
        FILE *fp = fopen(filename, "w");
        if (fp == NULL)
        {
            printf("无法创建文件。\n");
            return;
        }
        fclose(fp);
    }

    // 打开文件
    FILE *fp;
    fp = fopen(filename, "a"); // 使用追加模式以便多次调用时不覆盖之前的内容

    if (fp == NULL)
    {
        printf("无法打开文件。\n");
        return;
    }

    // 将时间戳、函数名和调试信息写入文件
    fprintf(fp, "[%s] [%s] %s\n", buffer, func_name, info);

    // 关闭文件
    fclose(fp);

    // printf("调试信息已写入到 %s 文件中。\n", filename);
}

// 输出数组的调试信息
void printArrayDebugInfo(float arr[], int size, const char *arr_name)
{
    char info[256]; // 用于存储数组转换后的字符串
    sprintf(info, "%s数组内容:[", arr_name);
    for (int i = 0; i < size; ++i)
    {
        char num_str[16]; // 用于存储整数转换后的字符串
        sprintf(num_str, "%f", arr[i]);
        strcat(info, num_str);
        if (i < size - 1)
        {
            strcat(info, ", ");
        }
    }
    strcat(info, "]");

    // 输出调试信息到文件
    writeDebugInfoToFile(__func__, info);
}

void clear_elc_error(int size)
{
    SCanInterface *canInterface = SCanInterface::getInstance();
    if (canInterface->setParameter(canidList, reg_fault_clear, MotorTypeHelper::REG_FAULT_CLEAR, size))
    {
        cout << "电机错误清除成功";
        // for (int i = 0; i < 6; i++)
        // {
        // 	cout<<"电机"<<i<<"最大正向允许速度: "<< static_cast<int32_t>(reg_max_app_speed[i])<<endl;
        // }
    }
    cout << endl;
}

int get_elektrische_Maschinen_status(int size)
{
    int result = 0;
    uint32_t parameterList[size] = {0};
    SCanInterface *elektrische_Maschinen_status = SCanInterface::getInstance();
    elektrische_Maschinen_status->getParameter(canidList, electric_machinery, MotorTypeHelper::REG_STATUS, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i + 1 << "状态: " << static_cast<int32_t>(electric_machinery[i]) << endl;
        if (electric_machinery[i] == 1)
        {
            cout << "软件错误" << endl;
            result = 1;
            // return result;
        }
        if (electric_machinery[i] == 2)
        {
            cout << "过压" << endl;
            result = 2;
            // return result;
        }
        if (electric_machinery[i] == 4)
        {
            cout << "欠压" << endl;
            result = 4;
            // return result;
        }
        if (electric_machinery[i] == 16)
        {
            cout << "启动错误" << endl;
            result = 16;
            // return result;
        }
    }
    cout << endl;
    return result;
}

void set_elc_info(uint32_t *elc_parameterlist, int elc_num, int parameterType, uint32_t elc_value)
{
    SCanInterface *canInterface = SCanInterface::getInstance();
    elc_parameterlist[elc_num] = elc_value;
    // cout << "sadsad==" << elc_parameterlist[elc_num] << endl;202438修改
    // canInterface->setParameter(canidList, elc_parameterlist, parameterType, IDNUM);
    canInterface->setParameter(canidList, elc_parameterlist, parameterType, elc_num); // elc_num代表几个电机
}

bool brake(int size)
{
    SCanInterface *canInterface = SCanInterface::getInstance();
    uint32_t unpL[size];
    uint32_t unpL1[size];
    uint32_t unpL2[size];
    for (int t = 0; t < size; t++)
    {
        unpL[t] = 0;
        unpL1[t] = 1000;
        unpL2[t] = -1000;
    }
    canInterface->setParameter(canidList, unpL1, MotorTypeHelper::REG_MAX_APP_SPEED, size);
    canInterface->setParameter(canidList, unpL2, MotorTypeHelper::REG_MIN_APP_SPEED, size);
    canInterface->setParameter(canidList, unpL, MotorTypeHelper::REG_TARGET_SPEED, size);
    writeDebugInfoToFile(__func__, "机械臂停止.");//写入日志
    return true;
}

bool login()
{
    SCanInterface *canInterface = SCanInterface::getInstance();
    bool isOpen = canInterface->open();
    if (isOpen)
    {
        writeDebugInfoToFile(__func__, "CAN设备连接正常！"); // 写入日志
    }
    // if(isOpen) cout<<"qwert"<<endl;
    // 返回连接状态
    return isOpen;
}

bool logout()
{
    SCanInterface *canInterface = SCanInterface::getInstance();
    bool isclose = canInterface->close();
    if (isclose)
    {
        writeDebugInfoToFile(__func__, "CAN设备退出正常！");
    }
    return isclose;
}

void get_elc_info(int size)
{
    cout << "dadad" << endl;
    for (int i = 0; i < size; i++)
    {
        canidList[i] = i + 1;
        cout << "canlist=" << static_cast<int>(canidList[i]) << endl;
    }
    SCanInterface *getCurrentposition = SCanInterface::getInstance();

    getCurrentposition->getParameter(canidList, reg_max_app_position, MotorTypeHelper::REG_MAX_APP_POSITION, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "最大正向位置: " << static_cast<int32_t>(reg_max_app_position[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_max_app_position[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_position_kp, MotorTypeHelper::REG_POSITION_KP, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "位置环比例: " << static_cast<int32_t>(reg_position_kp[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_position_kp[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_position_kd, MotorTypeHelper::REG_POSITION_KD, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "位置环微分: " << static_cast<int32_t>(reg_position_kd[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_position_kd[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_speed_kp, MotorTypeHelper::REG_SPEED_KP, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "速度环比例: " << static_cast<int32_t>(reg_speed_kp[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_speed_kp[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_speed_ki, MotorTypeHelper::REG_SPEED_KI, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "速度环积分: " << static_cast<int32_t>(reg_speed_ki[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_speed_ki[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_max_curr_value, MotorTypeHelper::REG_MAX_CURR_VALUE, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "最大正电流: " << static_cast<int32_t>(reg_max_curr_value[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_max_curr_value[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_min_curr_value, MotorTypeHelper::REG_MIN_CURR_VALUE, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "最小负电流: " << static_cast<int32_t>(reg_min_curr_value[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_curr_value[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_max_app_speed, MotorTypeHelper::REG_MAX_APP_SPEED, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "最大正向允许速度: " << static_cast<int32_t>(reg_max_app_speed[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_max_app_speed[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, reg_min_app_speed, MotorTypeHelper::REG_MIN_APP_SPEED, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "最小负向允许速度: " << static_cast<int32_t>(reg_min_app_speed[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_speed[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    // getCurrentposition->getParameter(canidList, electricity, 4, 6);
    getCurrentposition->getParameter(canidList, electricity, MotorTypeHelper::REG_I_Q, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "电流值: " << static_cast<int32_t>(electricity[i]) << endl;

        sprintf(LogInfo, "电机%d的电流值是: %d", i, static_cast<int32_t>(electricity[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;

    getCurrentposition->getParameter(canidList, ele_status, MotorTypeHelper::REG_STATUS, size);
    for (int i = 0; i < size; i++)
    {
        cout << "电机" << i << "状态: " << static_cast<int32_t>(ele_status[i]) << endl;

        sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(electric_machinery[i]));
        writeDebugInfoToFile(__func__, LogInfo);
    }
    cout << endl;
}

bool move_to_joint()
{
	bool token;
	// show_value("calcu pos:",TH.pos);
	token=TH.forward_move();
	if (token)
	{
		// show_value("calcu pos:", TH.pos);
		// show_value("calcu J:", TH.j);
		plan_move();
		//show_value("th.j:",TH.j);
	}else cout<<"unsolvable fwdmv"<<endl;
	writeDebugInfoToFile(__func__, "关节运动.");
	return token;
}

void plan_move()
{
	uint32_t curt_jv[IDNUM];//将原来的局部变量修改成全局变量
	
	int p = 0, gap[IDNUM];
	float C, k, tm = 0;
	int i, t, im = 0;
	
	get_canidlist(canidList);
	SCanInterface *canInterface = SCanInterface::getInstance();
	if (canInterface->getParameter(canidList, curt_jv, MotorTypeHelper::REG_CURRENT_POSITION, IDNUM))
	{

		float a[IDNUM],b[IDNUM];
		for (i = 0; i < IDNUM; i++)
		{
			gap[i] = TH.j[i]*j2p-int(curt_jv[i]);
			a[i]=gap[i]/2.0;
			b[i]=(TH.j[i]*j2p+int(curt_jv[i]))/2.0;
		}
		for (i = 1; i < IDNUM; i++)
			if (abs(gap[i]) > abs(gap[im]))
				im = i;
		float T0 = pi*abs(gap[im])/NMAX/n2p;
		ACTmove(a,b,T0);
	}
	else
		cout << "recieve curt_jv failed." << endl;
}

void ACTmove(float *a,float *b,float T0)//实际运动
{
	float t=0;
	int k,i;
	int AT=int(T0/2/AG);
	uint32_t goto_jv[AT+2][IDNUM];
	int npl[AT+2][IDNUM];
	for(k=0;k<AT+1;k++){
		for(i=0;i<IDNUM;i++){
			goto_jv[k][i]=int(a[i]*sin(2*pi*k*AG/T0-pi/2)+b[i]);
			if(k>0){
				npl[k][i]=int((int(goto_jv[k][i])-int(goto_jv[k-1][i]))/AG/n2p);
			}
		}
		// if(k>0){
		// 	for(int f=0;f<6;f++){
		// 		cout<<npl[k][f]<<" ";
		// 	}
		// 	cout<<endl;
		// }

		//show_value("plan_j",goto_jv[k]);
	}
	for(i=0;i<IDNUM;i++){
		goto_jv[AT+1][i]=int(a[i]+b[i]);
		npl[AT+1][i]=int((int(goto_jv[AT+1][i])-int(goto_jv[AT][i]))/AG/n2p);
	}
	for(int f=0;f<6;f++){
		// cout<<npl[k][f]<<" ";//该打印信息由下面添加到log文件中

		// 将整数转换为字符串
		std::string value_str = std::to_string(npl[k][f]);
		// 调用 writeDebugInfoToFile 函数，传递转换后的字符串作为参数
		writeDebugInfoToFile(__func__, value_str.c_str());
	}
	cout<<endl;
	//show_value("plan_j",goto_jv[AT+1]);
	SCanInterface *canInterface = SCanInterface::getInstance();
	for(k=1;k<AT+2;k++){
		setn(npl[k]);
		canInterface->setParameter(canidList, goto_jv[k], MotorTypeHelper::REG_TARGET_POSITION, IDNUM);
		usleep(AG*1000000);
	}
	// cout<<"AT="<<AT<<endl;
}

void setn(int npL[IDNUM])
{
	uint32_t unpL1[IDNUM], unpL2[IDNUM];
	for (int i = 0; i < IDNUM; i++)
	{
		unpL1[i] = abs(npL[i]);
		unpL2[i] = -abs(npL[i]);
	}
	SCanInterface *canInterface = SCanInterface::getInstance();
	canInterface->setParameter(canidList, unpL1, MotorTypeHelper::REG_MAX_APP_SPEED, IDNUM);
	canInterface->setParameter(canidList, unpL2, MotorTypeHelper::REG_MIN_APP_SPEED, IDNUM);
}

// 机械臂关节运动
bool joint_movement(const float *arr)
{
	for (int i = 0; i < IDNUM; i++)
	{
		TH.j[i] = arr[i];
		// cout<<"j="<<TH.j[i];
	}
	return move_to_joint();
}