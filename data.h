#ifndef DATA_H_
#define DATA_H_

class data {
public:
    data();
    virtual ~data();
    static double GPStime;
    static double lat; //转换之后的纬度坐标
    static double lng;//转换之后的经度坐标
    static char satelite_num[16];//卫星数目
    static char num[16];     //差分状态信息，RTK固定为4
    static double altitude;  //海拔高度
    static double north;     //与真北方向夹角
    static double velocity;  //车的速度
    static char stop[16];    //停止指令
    static double theta;     //车轮转角信息
    static double car_length; //车身长度信息

    static double distance;   //实时横向偏差
    static double dNorth;     //实时航向角偏差
    static double calculFai;  //实时期望转角

    static double xInitial; //设定AB线时的初始坐标
    static double yInitial;
    static double northInitial;  //设定AB线时的初始航向角
    static double deInitial;   //初始横向偏差
    static double dNorthInitial;   //初始航向角偏差
    static double thetaABLine;
    static double bABLine;
    static double Ld;

    static int motor_vel;
};

#endif /* DATA_H_ */
