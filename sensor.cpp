#include "sensor.h"
#include "math.h"

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2
#define TYPE_CIRCLE 0
#define TYPE_LINE 1

#define MIN(a,b) ((a) < (b) ? (0) : (1))

std::vector<double> linspace(double start_in, double end_in, int num_in)
{
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0)
    {
        return linspaced;
    }
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }
    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
    return linspaced;
}
Sensor::Sensor(int num_sensors, double max_dist, double radius, int max_quark)
{
    ip.sparam.num_sensors=num_sensors;
    ip.sparam.max_dist=max_dist;
    ip.sparam.radius=radius;

    ip.qparam.max_quark=max_quark;

    is=new info_sensor[num_sensors];
}

Sensor::Sensor(const info_param in)
{
    ip=in;
    is=new info_sensor[ip.sparam.num_sensors];
}

Sensor::Sensor(const Sensor& sen)
{
    ip=sen.ip;
    is=new info_sensor[ip.sparam.num_sensors];

    rpos[INDEX_X]=sen.rpos[INDEX_X];
    rpos[INDEX_Y]=sen.rpos[INDEX_Y];
    rpos[INDEX_Q]=sen.rpos[INDEX_Q];
    //copy obs vector
    std::copy(sen.vc.begin(), sen.vc.end(), std::back_inserter(vc));
    std::copy(sen.vl.begin(), sen.vl.end(), std::back_inserter(vl));
    std::copy(sen.vq.begin(), sen.vq.end(), std::back_inserter(vq));
    setMap(sen.m_map,rpos);
}

Sensor::~Sensor()
{
    if(is!=nullptr)
    {
        delete is;
        is=nullptr;
    }
}

void Sensor::setMap(std::vector<BYTE> map,double* src)
{
    M2C(map);
    mLimit[0]=-4.5;//limit left top x
    mLimit[1]=4.5;//limit left top y
    mLimit[2]=4.5;//limit right bottom x
    mLimit[3]=-4.5;//limit right bottom y
}

void Sensor::updateSensorPos()
{
    int sensor=ip.sparam.num_sensors;
    double radius=ip.sparam.radius;
    std::vector<double> v_sensor = linspace(0, 2.0*M_PI, sensor+1);
    for(int i=0;i<sensor;i++)
    {
        double q=v_sensor[i];
        double rx,ry,rq;

        rx=radius*cos(q);
        ry=radius*sin(q);
        rq=q;
        q=rpos[INDEX_Q];

        is[i].pos.x=rx*cos(q)-ry*sin(q)+rpos[INDEX_X];
        is[i].pos.y=rx*sin(q)+ry*cos(q)+rpos[INDEX_Y];
        is[i].pos.q=rq+q;
    }
}
#include <random>
#include <algorithm>
void Sensor::senseQuark()
{
    if(!(vq.size()>0)) return;

    int quark=vq.size();
    double rx=rpos[INDEX_X];
    double ry=rpos[INDEX_Y];

    for(int i=0;i<quark;i++)
    {
        double qx=vq[i].pos.x;
        double qy=vq[i].pos.y;
        double diff_x=qx-rx;
        double diff_y=qy-ry;
        double dist=sqrt(pow(diff_x,2)+pow(diff_y,2));
        double vx=diff_x/dist;
        double vy=diff_y/dist;
        vq[i].sense.dist=dist;
        vq[i].sense.vx=vx;
        vq[i].sense.vy=vy;
    }
}

void Sensor::senseCObs(int sensor)
{
    int cobs=vc.size();

    double sx=is[sensor].pos.x;
    double sy=is[sensor].pos.y;
    double rx=rpos[INDEX_X];
    double ry=rpos[INDEX_Y];

    for(int i=0;i<cobs;i++)
    {
        double cx=vc[i].pos.x;
        double cy=vc[i].pos.y;
        double r =vc[i].param.radius;

        double m=(sy-ry)/(sx-rx);
        double coeff=sy-m*sx;

        double a=1.0+m*m;
        double b=2.0*(m*(coeff-cy)-cx);
        double c=cx*cx+pow(coeff-cy,2)-r*r;

        double det=b*b-4.0*a*c;

        if(det<0)
        {
            s[i].dist=-1.0;
            s[i].vx=0;
            s[i].vy=0;
            continue;
        }
        double x1=(-b+sqrt(det))/(2.0*a);
        double x2=(-b-sqrt(det))/(2.0*a);
        double y1=m*x1+coeff;
        double y2=m*x2+coeff;

        double dist1=sqrt(pow(x1-sx,2)+pow(y1-sy,2));
        double dist2=sqrt(pow(x2-sx,2)+pow(y2-sy,2));

        if(MIN(dist1,dist2)==0)
        {
            double dot=(x1-sx)*(sx-rx)+(y1-sy)*(sy-ry);
            if(dot<0)
            {
                s[i].dist=-1.0;
                s[i].vx=0.0;
                s[i].vy=0.0;
                continue;
            }
            double d=dist1<ip.sparam.max_dist?dist1:-1.0;
            s[i].dist=d;
            s[i].vx=d>=0?(x1-sx)/d:0.0;
            s[i].vy=d>=0?(y1-sy)/d:0.0;
        }
        else
        {
            double dot=(x2-sx)*(sx-rx)+(y2-sy)*(sy-ry);
            if(dot<0)
            {
                s[i].dist=-1.0;
                s[i].vx=0.0;
                s[i].vy=0.0;
                continue;
            }
            double d=dist2<ip.sparam.max_dist?dist2:-1.0;
            s[i].dist=d;
            s[i].vx=d>=0?(x2-sx)/d:0.0;
            s[i].vy=d>=0?(y2-sy)/d:0.0;
        }
    }
}

void Sensor::senseLObs(int sensor)
{
    int lobs=vl.size();
    int cobs=vc.size();

    double sx=is[sensor].pos.x;
    double sy=is[sensor].pos.y;
    double rx=rpos[INDEX_X];
    double ry=rpos[INDEX_Y];

    double m1=(sy-ry)/(sx-rx);
    double c1=sy-m1*sx;
    for(int i=0;i<lobs;i++)
    {
        double m2=vl[i].unknown.m;
        double c2=vl[i].unknown.c;

        double x=(c2-c1)/(m1-m2);
        double y=m1*x+c1;

        double lx=vl[i].pos.x1;
        double ly=vl[i].pos.y1;
        double ux=vl[i].pos.x2;
        double uy=vl[i].pos.y2;

        bool isInsideX=(std::min(lx,ux)<=x)&&(x<=std::max(lx,ux));
        bool isInsideY=(std::min(ly,uy)<=y)&&(y<=std::max(ly,uy));

        if(!(isInsideX&&isInsideY))
        {
            s[i+cobs].dist=-1.0;
            s[i+cobs].vx=0.0;
            s[i+cobs].vy=0.0;
            continue;
        }
        double dot=(x-sx)*(sx-rx)+(y-sy)*(sy-ry);
        if(dot<0)
        {
            s[i+cobs].dist=-1.0;
            s[i+cobs].vx=0.0;
            s[i+cobs].vy=0.0;
            continue;
        }
        double d=sqrt(pow(x-sx,2)+pow(y-sy,2));
        s[i+cobs].dist=d<ip.sparam.max_dist?d:-1.0;
        s[i+cobs].vx=d>=0?(x-sx)/d:0.0;
        s[i+cobs].vy=d>=0?(y-sy)/d:0.0;
    }
}

void Sensor::senseMap(int sensor,bool bVirtual)//change pixel coord
{
    int sx,sy;
    int lx,ly;
    double sq=is[sensor].pos.q;
    double rx=rpos[INDEX_X];//local robot pos
    double ry=rpos[INDEX_Y];

    if(!bVirtual)
        M2P(0,0,sx,sy);
    else
        M2P(rx,ry,sx,sy);
    if(!bVirtual)
        M2P(4.5*cos(sq),4.5*sin(sq),lx,ly);
    else
        M2P(rx+4.5*cos(sq),ry+4.5*sin(sq),lx,ly);

    int dx = abs(lx-sx);
    int step_x = sx<lx?1:-1;
    int dy = -abs(ly-sy);
    int step_y = sy<ly?1:-1;
    int error = dx+dy;
    int limit_l_x,limit_l_y;
    int limit_r_x,limit_r_y;
    int prx=0;
    int pry=0;

    M2P(mLimit[0],mLimit[1],limit_l_x,limit_l_y);
    M2P(mLimit[2],mLimit[3],limit_r_x,limit_r_y);
    if(bVirtual)
        M2P(rx,ry,prx,pry);

    bool bRun=true;
    while(true)
    {
        if(!bRun)
        {
            is[sensor].sense.dist=-1.0;
            is[sensor].sense.vx=0.0;
            is[sensor].sense.vy=0.0;
            break;
        }
        if(sx<(limit_l_x+1)||sx>(limit_r_x-1)||sy>(limit_l_y-1)||sy<(limit_r_y+1))
        {
            bRun=false;
            continue;
        }
        if(senseCell(sx,sy))
        {
            double mx,my;
            P2M(sx,sy,mx,my);
            is[sensor].sense.dist=sqrt((mx)*(mx)+(my)*(my));
            is[sensor].sense.vx=(mx)/is[sensor].sense.dist;
            is[sensor].sense.vy=(my)/is[sensor].sense.dist;
            if(bVirtual)
            {
                is[sensor].sense.dist=sqrt((mx-rx)*(mx-rx)+(my-ry)*(my-ry));
                is[sensor].sense.vx=(mx-rx)/is[sensor].sense.dist;
                is[sensor].sense.vy=(my-ry)/is[sensor].sense.dist;
            }
            break;
        }
        if(sx==lx && sy==ly)
        {
            bRun=false;
            continue;
        }
        double error2=2*error;
        if(error2>=dy)
        {
            if(sx==lx)
            {
                //bRun=false;
                continue;
            }
            error+=dy;
            sx+=step_x;
        }
        if(error2<=dx)
        {
            if(sy==ly)
            {
                //bRun=false;
                continue;
            }
            error+=dx;
            sy+=step_y;
        }
    }
}

void Sensor::selectDist(int sensor)
{
    int cobs=vc.size();
    int lobs=vl.size();
    int iMin=0;
    double minDist=-1.0;
    minDist=s[iMin].dist;
    for(int i=0;i<cobs+lobs;i++)
    {
        double d=s[i].dist;
        if((minDist<0.0)&&(d>0.0))
        {
            minDist=d;
            iMin=i;
        }
        else if((minDist>0.0)&&(d>0.0))
        {
            if(minDist>d)
            {
                minDist=d;
                iMin=i;
            }
        }
    }
    if(minDist<0.0)
    {
        is[sensor].sense.dist=-1.0;
        is[sensor].sense.vx=0.0;
        is[sensor].sense.vy=0.0;
    }
    else
    {
        is[sensor].sense.dist=s[iMin].dist;
        is[sensor].sense.vx=s[iMin].vx;
        is[sensor].sense.vy=s[iMin].vy;
    }
}

void Sensor::M2C(std::vector<BYTE> map)
{
    if(map.empty())
        return;
    int w = 180;
    int h = 180;
    int index=w*(h-1);
    std::vector<BYTE> temp(w*h);
    for(int i =0;i<h;i++)
    {
        std::copy(&map[w*i],&map[w*i+w],&temp[index]);
        index-=w;
    }
    m_map=temp;
}

void Sensor::M2P(double x, double y, int &px, int &py)
{
    double mtp = 0.05; // meter to pixel
    double mx=x/mtp;
    double my=y/mtp;

    px=90+mx;//w/2=30.0
    py=90+my;
}

void Sensor::P2M(double x, double y, double &mx, double &my)
{
    double mtp=0.05;
    mx=x-90.0;
    my=y-90.0;
    mx*=mtp;
    my*=mtp;
}
#include <iostream>
bool Sensor::senseCell(int x, int y)
{
    if(m_map.empty())
        return false;

    int index=y*180+x;
    if(index<0)
    {
        std::cout<< "index underflow!!";
        return false;
    }
    else if(index>32400)
    {
        std::cout<< "index overflow!!";
        return false;
    }

    return m_map[index]>50;
}

void Sensor::sense(double *pos,bool bVirtual)
{
    rpos[INDEX_X]=(double)pos[INDEX_X];
    rpos[INDEX_Y]=(double)pos[INDEX_Y];
    rpos[INDEX_Q]=(double)pos[INDEX_Q];
    int cobs=vc.size();
    int lobs=vl.size();
    int sensor=ip.sparam.num_sensors;

    updateSensorPos();
    for(int i=0;i<sensor;i++)
    {
//        s=new info_sensor::sensor_sense[cobs+lobs];
//        senseCObs(i);
//        senseLObs(i);
//        selectDist(i);
//        delete[] s;
        senseMap(i,bVirtual);
    }
    senseQuark();
}

void Sensor::addQuark(double x, double y)
{
    info_quark q;
    q.pos.x=x;
    q.pos.y=y;

    if(!(vq.size()<ip.qparam.max_quark))
    {
        vq.erase(vq.begin());
    }
    vq.push_back(q);
}

void Sensor::addCObs(double x, double y, double radius)
{
    info_cobs c;
    c.pos.x=x;
    c.pos.y=y;
    c.param.radius=radius;

    vc.push_back(c);
}

void Sensor::addLObs(double x1, double y1, double x2, double y2)
{
    info_lobs l;
    l.pos.x1=x1;
    l.pos.y1=y1;
    l.pos.x2=x2;
    l.pos.y2=y2;
    l.unknown.m=(y2-y1)/(x2-x1);
    l.unknown.c=y1-l.unknown.m*x1;

    vl.push_back(l);
}
