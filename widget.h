#ifndef WIDGET_H
#define WIDGET_H

#include <QtGui/QWidget>
#include <ui_widget.h>

#include "data.h"
#include "recvthread.h"
#include "algorithm.h"

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = 0);
    int pilotFlag;

private:
    Ui::widgetClass *ui;
    recvthread *t1;//接收线程

    QString filename;
    QFile fname;
    int filenum;

public slots:
    void receive_data();//开始接受串口数据，开始接收线程
    void show_theta();//显示车轮转角值
    void show_data();//show the distance,dNorth and fai
    void setABLine();
    void start_autopilot(); //开始导航线程

signals:
    void pilotFlagSig(int);

};

#endif // WIDGET_H
