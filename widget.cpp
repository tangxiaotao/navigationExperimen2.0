#include "widget.h"


Widget::Widget(QWidget *parent) :
    QWidget(parent), ui(new Ui::widgetClass)
{
    ui->setupUi(this);
    ui->pilotButton->setText("Start Autopilot");
    ui->deInitialLine->setText(QString::number(data::deInitial));
    ui->dNorthInitialLine->setText(QString::number((data::dNorthInitial)));

    t1 = new recvthread();

    this->pilotFlag=0;

    int filenum = 0;
    filename = QDir::currentPath() + "/data/" + QString::number(filenum) + "GPS.txt";
    fname.setFileName(filename);

    while (fname.exists())
    {
        filenum++;
        filename = QDir::currentPath() + "/data/" + QString::number(filenum) + "GPS.txt";
        fname.setFileName(filename);
    }
}

void Widget::receive_data()
{
    static bool flag2 = 0;
    if (flag2 == 0)
    {
        t1->start();//aim
        ui->recvButton->setText("Stop receive");
        flag2 = 1;
        ui->recvButton->setStyleSheet("background-color:rgb(0, 255, 0); border-radius: 3px;");
    }
    else
    {
        t1->stop();
        ui->recvButton->setText("Start receive");
        flag2 = 0;
        ui->recvButton->setStyleSheet("background-color: rgb(154, 154, 154); border-radius: 3px;");
    }

    QObject::connect(t1, SIGNAL(send_theta()), this, SLOT(show_theta()));
    QObject::connect(this, SIGNAL(pilotFlagSig(int)), t1, SLOT(pilotFlagSlot(int)));
}

void Widget::show_theta()
{
    ui->xLine->setText(QString::number(data::lat, 'g', 12));
    ui->yLine->setText(QString::number(data::lng, 'g', 12));
    ui->norLine->setText(QString::number(data::north));
    ui->thetaLine->setText(QString::number(data::theta));
    ui->velocityLine->setText(QString::number((data::velocity/3.6)));

    //save the data
    if(this->pilotFlag == 1) {
        fname.open(QIODevice::Append | QIODevice::Text);

        QTextStream out(&fname);

        out << "TIME:" << QString::number(data::GPStime, 'g', 10) << "\r\n";
        out << "x:" << QString::number(data::lat, 'g', 12) << "\r\n";
        out << "y:" << QString::number(data::lng, 'g', 12) << "\r\n";
        out << "north:" << QString::number(data::north) << "\r\n";
        out << "theta:" <<QString::number(data::theta) << "\r\n";
        out << "velocity:" << QString::number(data::velocity/3.6) << "\r\n";
        out << "distance:" << QString::number(data::distance) <<"\r\n";
        out << "dNorth:" << QString::number(data::dNorth) <<"\r\n";
        out << "calculatedfai:" << QString::number(data::calculFai) << "\r\n";
        out << "deInitial:" << QString::number(data::deInitial) << "\r\n";
        out << "dNorthInitial:" << QString::number(data::dNorthInitial) << "\r\n";
        out << "xInitial:" << QString::number(data::xInitial, 'g', 12) << "\r\n";
        out << "yInitial:" << QString::number(data::yInitial, 'g', 12) << "\r\n";
        out << "northInitial:" << QString::number(data::northInitial) << "\r\n";
        out << "thetaABLine:" << QString::number(data::thetaABLine) << "\r\n";
        out << "bABLine:" << QString::number(data::bABLine, 'g', 12) << "\r\n";
        out << "Ld" << QString::number(data::Ld) << "\r\n";
        out << "motorVel:"<<QString::number(data::motor_vel)<<"\r\n";

        out << "\r\n";
        fname.close();
    }
}

void Widget::start_autopilot()//开始导航线程
{

    if (ui->pilotButton->text() =="Start Autopilot") {
        this->pilotFlag = 1;
        emit pilotFlagSig(this->pilotFlag);
        ui->pilotButton->setText("Stop Autopilot");
        ui->pilotButton->setStyleSheet("background-color:rgb(0, 255, 0); border-radius: 3px;");
    }
    else if (ui->pilotButton->text() == "Stop Autopilot") {
        this->pilotFlag = 2;
        emit pilotFlagSig(this->pilotFlag);
        ui->pilotButton->setText("Start Autopilot");
        ui->pilotButton->setStyleSheet("background-color: rgb(154, 154, 154); border-radius: 3px;");
    }

}

void Widget::setABLine()
{
    data::xInitial = data::lat;
    data::yInitial = data::lng;
    data::northInitial = data::north;
    data::thetaABLine = data::northInitial - data::dNorthInitial;
    double thetaAB = (data::thetaABLine)/180*M_PI;
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
        data::bABLine = data::deInitial*sqrt(1+k*k) + data::yInitial - k*data::xInitial;
    }
    else if( (thetaAB>M_PI/2) && (thetaAB<3*M_PI/2) ) {
        double k=tan(thetaAB);
        data::bABLine = -data::deInitial*sqrt(1+k*k) + data::yInitial - k*data::xInitial;
    }
    else if( thetaAB == (M_PI/2) ){
        data::bABLine = 12345678;
    }
    else if( (thetaAB == (3*M_PI/2))||(thetaAB == (-M_PI)/2) ) {
        data::bABLine = 12345678;
    }

    static bool setABLineFlag = 0;
    if (setABLineFlag == 0)
    {
        setABLineFlag = 1;
        ui->setABLineButton->setStyleSheet("background-color:rgb(0, 255, 0); border-radius: 3px;");
    }
    else
    {
        setABLineFlag = 0;
        ui->setABLineButton->setStyleSheet("background-color: rgb(154, 154, 154); border-radius: 3px;");
    }
    QObject::connect(t1, SIGNAL(send_data()), this, SLOT(show_data()));
}
void Widget::show_data()
{
    ui->deLine->setText(QString::number(data::distance));
    ui->dNorthLine->setText(QString::number(data::dNorth));
    ui->faiLine->setText(QString::number(data::calculFai*180/M_PI));
}
