#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iomanip>
#include <cstdint>

#include "tool.h"

extern uint8_t new_arr[15];

// 将字节数组转换回浮点数组
void convert_bytes_to_float(const uint8_t* bytes, float* arr, size_t size);
int init_convert(float originalArr[6]);

/*socket client函数
    返回值：成功返回0，失败返回1
    参数：
        serverIP：服务器IP地址
        port：服务器端口
        input[15]：要发送的数据
        input_size：数据长度
*/
int socket_client(std::string& serverIP,int port,uint8_t input[15],size_t input_size);

#endif