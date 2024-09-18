#ifndef SINGLECANINTERFACE_H
#define SINGLECANINTERFACE_H
#include <memory>
#include <iostream>

class TControlCanFactor;

class SCanInterface
{
public:
    static SCanInterface* getInstance();
    bool open() const;
    bool close() const;

//parameterList：存放参数值 parameterType: 参数类型（见下表）  nodeCount:电机个数 （关节长度）nodeList：存放电机（关节）具体节点,id:1-6
    bool getParameter(uint32_t *parameterList, int parameterType,int nodeCount);
    bool getParameter(uint8_t *nodeList,uint32_t *parameterList, int parameterType,int nodeCount);
    bool setParameter(uint32_t *parameterList, int parameterType,int nodeCount);
    bool setParameter(uint8_t *nodeList,uint32_t *parameterList, int parameterType,int nodeCount);    // 设置Reduction ratio减速比
    /***
     * 设置kt电机扭矩常数
     * 设置kp
     * 设置kd
     * 设置position机器人期望位置
     **/
     
     bool setRobotParameter(float *parameterList,int parameterType,int nodeCount);
     bool setRobotParameter(uint8_t *nodeList,float *parameterList,int parameterType,int nodeCount);
     bool getRobotParameter(float *parameterList, int parameterType,int nodeCount);
    bool getRobotParameter(uint8_t *nodeList,float *parameterList, int parameterType,int nodeCount);

 // bool saveParameter();
private:
    SCanInterface();
    static SCanInterface* instance;
    std::shared_ptr<TControlCanFactor>  controlCAN;
};


#endif // SINGLECANINTERFACE_H
