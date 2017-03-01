/*
 * recvthread.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "recvthread.h"

recvthread::recvthread() {

    degree=0;
    stopped=false;
    serial485=new serial();
    serialGPS=new serial();
    f485=0;
    fGPS=0;

    pilotalg = new algorithm();
    this->pilotflag = 0;

}

recvthread::~recvthread() {
    // TODO Auto-generated destructor stub
}
void recvthread::run(){

    char recv_buff[128] = "0";
    char buff[]="0";

    // set the GPS
    if((fGPS=serialGPS->open_port(fGPS,2))<0) {
        perror("open_port error");
    }
    // 设置串口的波特率，校验位等
    if(serialGPS->set_opt(fGPS,38400,8,'N',1)<0) {
        perror("set_opt error");
    }

    //  set the RS485
    if((f485=serial485->open_port(f485,4))<0) {
        perror("open_port error");
    }
    if((serial485->set_opt(f485,9600,8,'E',1))<0) {
        perror("set_opt error");
    }

    while(!stopped) {
        numGPS = 0, num485 = 0;
        memset(recv_buff,0,sizeof(recv_buff));
        memset(buff,0,sizeof(buff));

        while ((num485=serial485->Recv(f485,buff,8)) >0 ) {
//        num485=serial485->Recv(f485,buff,8);
            degree=serial485->buffToDouble(buff);

            if(degree>=180) {
                degree=-degree+360;
            }
            else {
                degree=-degree;
            }
            data::theta=degree;
        }

        while((numGPS = serialGPS->Recv(fGPS, recv_buff, 128)) > 0) {

            if((strstr(recv_buff, "GPGGA") != NULL)) {
                data_handle(recv_buff);
                /* show the data of received */
                emit send_theta();
                emit send_data();
                if (pilotflag == 1) {
                    pilotalg->autopilot();
                }
                else if (pilotflag == 2){
                    pilotalg->stop();
                }
            }
            else if((strstr(recv_buff, "PASHR") != NULL)) {
                data_handle(recv_buff);
            }
            else if((strstr(recv_buff, "GPVTG") != NULL)) {

                data_handle(recv_buff);
            }
        }
    }
    stopped=false;
}
/* this is the pilot slot function */
void recvthread::pilotFlagSlot(int flag) {
    this->pilotflag = flag;
}

void recvthread::data_handle(char data[])
{
    const char *sep = ",";    //define the split symbol
    int len=0;
    char *temp[20];
    char *pt;
    char gps_temp[16];

    pt = strtok(data, sep);
    while(pt) {
        temp[len] = pt;
        pt = strtok(NULL, sep);
        len++;
    }
    sprintf(gps_temp, "%s", temp[0]);

    if((strstr(gps_temp, "GPGGA") != NULL) && (len == 15)) {
        double x,y;
        sscanf(temp[2], "%lf", &x);
        x = x / 100.0;
        int lat_int = (int)x;
        x = lat_int + (x-lat_int)*100 / 60;

        sscanf(temp[4], "%lf", &y);
        y = y/100.0;
        int lng_int = (int)y;
        y= lng_int + (y - lng_int) * 100 /60;
        double *coordinate;
        coordinate = change(x, y);
        data::lat=coordinate[1];
        data::lng=coordinate[0];

        sscanf(temp[1], "%lf", &data::GPStime);
        sprintf(data::num, "%s", temp[6]);
        sprintf(data::satelite_num, "%s", temp[7]);
        sscanf(temp[9], "%lf", &data::altitude);
    }
    else if((strstr(gps_temp, "PASHR") != NULL) && (len == 11)) {
        sscanf(temp[2], "%lf", &data::north);
        data::north = -data::north+360;
    }
    else if((strstr(gps_temp, "GPVTG") != NULL) && ( len ==10 )) {
        sscanf(temp[7], "%lf", &data::velocity);
    }
}

/* calculate the coordinate */
double* recvthread::change(double B,double L)
{
    double L0 = 120;
    double a = 6378137;//a为椭球的长半轴:a=6378.137km
    double b = 6356752.3142;//b为椭球的短半轴:a=6356.7523141km
    //double e = sqrt(1 - pow(b, 2) / pow(a, 2));
    double e2 = 1 - pow(b, 2) / pow(a, 2);
    double ee = pow(a, 2) / pow(b, 2) - 1;
    double C1 =1.0 + 3.0 / 4.0 * e2 + 45.0 / 64.0 * pow(e2, 2) + 175.0 / 256.0 * pow(e2, 3) + 11025.0 / 16384.0 * pow(e2,4);
    double C2 = 3.0 / 4.0 * e2 + 15.0 / 16.0 * pow(e2, 2) + 525.0 / 512.0 * pow(e2, 3) + 2205.0 / 2048.0 * pow(e2, 4);
    double C3 = 15.0 / 64.0 * pow(e2, 2) + 105.0 / 256.0 * pow(e2, 3) + 2205.0 / 4096.0 * pow(e2, 4);
    double C4 = 35.0 / 512.0 * pow(e2, 3) + 315.0 / 2048.0 * pow(e2, 4);
    double C5 = 315.0 / 131072.0 * pow(e2, 4);
    double *x = (double *)malloc(16);

    B = B / 180 * M_PI;
    L = L / 180 * M_PI;
    L0 = L0 / 180 * M_PI;
    double l1 = L - L0;
    double t = tan(B);
    double n2 = ee*pow(cos(B), 2);
    double X = a*(1 - e2)*(C1*B - 1.0 / 2.0 * C2*sin(2 * B) + 1.0/ 4.0 * C3*sin(4 * B) - 1.0/6.0 * C4*sin(6 * B) + C5*sin(8 * B));
    double N = a / sqrt(1 - e2*pow(sin(B), 2));
    x[0]=X;
    x[0]+=(N*sin(B)*cos(B)*pow(l1,2)) / 2.0;
    x[0]+= (N*sin(B)*(pow(cos(B), 3))*(5.0 - pow(t, 2) + 9.0 * n2 + 4.0 * pow(n2, 2))*pow(l1, 4)) / 24.0;
    x[0]+=(N*sin(B)*(pow(cos(B), 5))*(61.0 - 58.0 * pow(t, 2) + pow(t, 4))*pow(l1, 6)) / 720.0;
    x[1] = N*cos(B)*l1;
    x[1]+= N*(pow(cos(B), 3)) * (1 - pow(t, 2) + n2)*pow(l1, 3) / 6;
    x[1]+= N*(pow(cos(B), 5)* (5 - 18 * t *t + pow(t, 4) - 14 * n2 - 58 * n2*(pow(t, 2)))*pow(l1, 5)) / 120;
    x[1]+=500000;

    return x;
}
void recvthread::stop() {
    /* stop the motor */
    pilotalg->stop();
    pilotflag = 2;
    serialGPS->Close(fGPS);
    serial485->Close(f485);
//    serialBlue->Close(fBlue);
    stopped=true;
}
