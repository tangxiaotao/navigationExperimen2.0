/*
 * algorithm.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "algorithm.h"
#include "data.h"
#include "fuzzy.h"

algorithm::algorithm() {

    can=new can0();
    fuzzyCompute = new fuzzy();
    l=1.06;
    expected=0;
    thetaAB = data::thetaABLine;
    b = data::bABLine;
}

algorithm::~algorithm() {
    // TODO Auto-generated destructor stub
}

void algorithm::autopilot() {
    /* this just need to be calculated a time */

    expected = calculateFai()*180/M_PI;
    double comp = (-1.7*(data::theta+77.14)) + expected;
    qDebug("zhuanjiao=%lf",data::theta);

    /* 20 is jiansubi, time is 0.1s */
    int a = 0;
    a = (int)(comp*16*100/18)*37.5;
    data::motor_vel=a;
    qDebug() << QString::number(a);

    if(a>32000) {
        can->can_send(32000);
    }
    else if( a<-32000) {
        can->can_send(-32000);
    }
    else if ( (a<500 ) && (a>-500) ) {
        can->can_send(0);
    }
    else {
        can->can_send(a);
    }
}

/* calculate the Distance */
double algorithm::calculateD(){

    double x=data::lat;
    double y=data::lng;
    thetaAB = data::thetaABLine/180*M_PI;
    b = data::bABLine;
    double d=0;
    while((thetaAB<-M_PI/2) || (thetaAB>3*M_PI/2))
    {
        if (thetaAB<-M_PI/2)
        {
            thetaAB = thetaAB + 2*M_PI;
        }
        else
        {
            thetaAB = thetaAB - 2*M_PI;
        }
    }
    if( (thetaAB<M_PI/2) && (thetaAB>-M_PI/2) ) {
            double k=tan(thetaAB);
            d=(k*x+b-y)/sqrt(1+k*k);
            qDebug()<<"1.the d="<<d;
        }
    else if( (thetaAB>M_PI/2) && (thetaAB<3*M_PI/2) ) {
        double k=tan(thetaAB);
        d =-(k*x + b-y)/sqrt(1+k*k);
        qDebug()<<"2.the d="<<d;
    }
    else if( thetaAB == (M_PI/2) ){
        d=x-data::xInitial;
        qDebug()<<"3.the d="<<d;
    }
    else if( thetaAB == (3*M_PI/2) ) {
        d=data::xInitial-x;
        qDebug()<<"4.the d="<<d;
    }

    data::distance = d;
    return d;
}
double algorithm::calculateFai()
{	//x and y are the location of tractor
    double d=calculateD();
    qDebug()<<"the d="<<d;

    /*if((abs(d))>=data::Ld){
        data::Ld = 3*abs(d);
    }
    else{
        data::Ld = 3.5;  // the initial value
    }*/
    data::Ld = fuzzyCompute->fuzzyOutput(d, data::velocity/3.6);
    qDebug()<<"LD="<<data::Ld;
    double northTheta = data::north;
    data::dNorth = data::north - thetaAB*180/M_PI;
    northTheta = northTheta/180*M_PI;
    double fai=0;
    fai = atan(-2*l*(-d*cos(northTheta-thetaAB) + sqrt((data::Ld)*(data::Ld) - d*d)*sin(northTheta-thetaAB))/data::Ld/data::Ld);

    if(fai>FAILIMIT) {
        fai=FAILIMIT;
    }
    else if(fai<-FAILIMIT) {
        fai=-FAILIMIT;
    }

    qDebug("fai=%f\n",fai);
    data::calculFai = fai;
    return fai;
}
/* stop the motor */
void algorithm::stop() {
    can->can_send(8);
}
