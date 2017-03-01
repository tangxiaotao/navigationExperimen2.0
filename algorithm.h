#ifndef ALGORITHM_H_
#define ALGORITHM_H_
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include<QFile>
#include<QTextStream>
#include <qthread.h>
#include "data.h"
#include "can0.h"
#include <qthread.h>
#include <QDateTime>
#include <QFile>
#include <QDir>
#include <QDebug>
#include "fuzzy.h"

#define FAILIMIT (M_PI/3)	//FAILIMIT is the limit of front steering

class algorithm {

public:
    algorithm();
    virtual ~algorithm();
    //double atan2(double, double);//-pi/2----3*pi/2
    double calculateD();
    double calculateFai();
    void autopilot();
    void stop();

    double l;
    double thetaAB;
    double b;
    double expected;

    can0 *can;
    fuzzy *fuzzyCompute;
};

#endif /* ALGORITHM_H_ */
