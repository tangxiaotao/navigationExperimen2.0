#include "traversalcontrol.h"
#define IF_INTEGRATE 1
#define FAILIMIT M_PI/4
#define LD0 2

traversalcontrol::traversalcontrol()
{
    l = 0;
    d = 0;
    dnorth = 0;
    distanceintegrate = 0;
    pidfactor->pfactor = 0.2;
    pidfactor->ifactor = 0.01;
    pidfactor->dfactor = 1.2;
    pidfactor->error = 0;
    pidfactor->lasterror = 0;
    pidfactor->sumerror = 0;

    ppursuitfactor->Ld = LD0;

    likepdfactor->kp = 0.255;
    likepdfactor->kd = 1;
    likepdfactor->ki = 0;
}

void traversalcontrol::algorithmchose(int a, double carlength, double distance, dobuel dnorth) {
    this->l = carlength;
    this->d = distance;
    this->dnorth = dnorth;
    if (fabs(this->dnorth) > 3.14) {
        printf("Please check the unit of degree!");
    }
    switch(a) {
    case 0:
        pid();
        break;
    case 1:
        purepursuit();
        break;
    case 2:
        likepd();
        break;
    default:
        purepursuit();
        break;
    }
}

double traversalcontrol::pid() {
    pidfactor->error = this->d;
    pidfactor->sumerror += this->d;
    double fai = 0;
    fai = pidfactor->pfactor*pidfactor->error + pidfactor->ifactor*pidfactor->sumerror + pidfactor->dfactor*(pidfactor->error-pidfactor->lasterror);
    pidfactor->lasterror = pidfactor->error;
    return fai;
}

double traversalcontrol::purepursuit() {
    if((abs(this->d))>=this->Ld){
        this->Ld = 3*abs(this->d);
    }
    else{
        this->Ld = LD0;  // the initial value
    }

    //data::Ld = fuzzyCompute->fuzzyOutput(d, data::velocity/3.6);

    double northTheta = data::north;
    data::dNorth = data::north - thetaAB*180/M_PI;
    northTheta = northTheta/180*M_PI;
    double fai=0;

    fai = atan(-2*this->l*(-this->d*cos(this->dnorth) + sqrt((ppursuitfactor->Ld)*(ppursuitfactor->Ld) - this->d*this->d)*sin(this->dnorth))/ppursuitfactor->Ld/ppursuitfactor->Ld)+ this->disintegrate()*ppursuitfactor->integratefactor*IF_INTEGRATE;
    if ((fai<=FAILIMIT)&&(fai>=-FAILIMIT))
    {
        fai=fai;
    }
     if(fai>FAILIMIT) {
        fai=FAILIMIT;
    }
    if(fai<-FAILIMIT) {
        fai=-FAILIMIT;
    }
    return fai;
}

double traversalcontrol::likepd() {
    double fai = atan(this->l*cos(this->dnorth)*cos(this->dnorth)*cos(this->dnorth)*(-likepdfactor->kd*tan(this->dnorth)+likepdfactor->kp*this->d)) + this->disintegrate()*likepdfactor->ki*IF_INTEGRATE;
    return fai;
}

double traversalcontrol::disintegrate() {
    if (this->distanceintegrate > 100) {
        if(this->d > 0)
        {
            this->distanceintegrate += 0;
        }
        else
        {
            this->distanceintegrate += this->d;
        }
    }
    else if(this->distanceintegrate < -100)
    {
        if(this->d < 0)
        {
            this->distanceintegrate += 0;
        }
        else
        {
            this->distanceintegrate += this->d;
        }
    }
    else {
        this->distanceintegrate += this->d;
    }
    return this->distanceintegrate;
}
