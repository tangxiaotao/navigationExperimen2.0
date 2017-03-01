/*
 * recvthread.h
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#ifndef RECVTHREAD_H_
#define RECVTHREAD_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <qthread.h>

#include "serial.h"
#include "algorithm.h"
#include "data.h"

#include <QDir>
#include <QTextStream>
#include <QFile>
#include <QDateTime>
#include <QDebug>


class recvthread: public QThread {
    Q_OBJECT
public:
    recvthread();
    virtual ~recvthread();
    void run();
    void data_handle(char data[]);
    double* change(double B,double L);
    void stop();
    double degree;
    int pilotflag;

signals:
    void send_theta();
    void send_data();

private:
    volatile bool stopped;

    /* serial settings */
    serial *serialGPS;
    serial *serial485;
    int fGPS;
    int f485;
    int num485;
    int numGPS;

    algorithm *pilotalg;

private slots:
    void pilotFlagSlot(int);

};

#endif /* RECVTHREAD_H_ */
