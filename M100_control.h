#include<stdio.h>

#define POSI_THRESHOLD 0.01

enum{
    stay = 0,
    go = 1,
    stop = 2,
    ckp = 3,
    climit = 4,
};

class PID{
public:
    PID():Kp(0.0005),Kd(0),Ki(0),max_out(1),min_out(-1),errlast(0),errnow(0),integrate(0),out(0){}

    PID(double kp, double ki, double kd):Kp(kp),Kd(kd),Ki(ki){}
    double UFO_PID_Control(double value)
    {
        integrate +=value;
        errnow = value;
        out= Kp* errnow+ Kd*( errnow- errlast)+Ki*integrate;
        errlast= errnow;

        if(out > max_out) out=max_out;
        if(out < min_out) out=min_out;
        return	out;
    }

    void pid_set_gains(double kp, double ki, double kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    double getValue(){
        return out;
    }

    void setValue(double value){
        out = value;
    }

    void setKp(double value){
        Kp = value;
    }

    void setParam(double kp,double ki,double kd){
        Kp=kp;
        Ki=ki;
        Kd=kd;
    }

    void setLimit(double limit){
        max_out = limit;
        min_out = -limit;
    }

private:
    double Kp;
    double Kd;
    double Ki;
    double max_out;
    double min_out;
    double errlast;
    double errnow;
    double integrate;
    double out;
};
