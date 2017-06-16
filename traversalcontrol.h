#ifndef TRAVERSALCONTROL_H
#define TRAVERSALCONTROL_H

#include<math.h>

#define M_PI 3.1415926

typedef struct PIDs
{
    double pfactor;
    double ifactor;
    double dfactor;
    double lasterror;
    double error;
    double sumerror;
} PIDfactor;

typedef struct PPursuit {
    double Ld;
    double integratefactor;
} Ppursuitfactor;

typedef struct LikePD {
    double kp;
    double kd;
    double ki;
} LikePDfactor;


class traversalcontrol
{
public:
    traversalcontrol();
    void algorithmchose(int a, double carlength, double distance, double dnorth);

private:
    double l;
    double d;
    double dnorth;
    double distanceintegrate;
    double pid();
    double purepursuit();
    double likepd();
    double disintegrate();

    PIDfactor *pidfactor;
    Ppursuitfactor *ppursuitfactor;
    LikePDfactor *likepdfactor;
};

#endif // TRAVERSALCONTROL_H
