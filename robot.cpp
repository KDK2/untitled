#include "robot.h"
#include "math.h"
#include <iostream>
#include "stdafx.h"
Robot::Robot():
    m_running(false)
{
    Actuator::info_rparam ia;
    Sensor::info_param is;
    Generator::info ig;

    ia.d=0.08;
    ia.m=10.0;
    ia.dt=0.1;
    ia.radius=0.35;

    is.qparam.max_quark=50;
    is.sparam.max_dist=4.5;
    is.sparam.num_sensors=99;
    is.sparam.radius=ia.radius;

    ig.f_param.aparam.k_vg=1.0;
    ig.f_param.aparam.q_g=5.0;
    ig.f_param.mparam.q_v=-1.0;
    ig.f_param.rparam.d_o=0.85;
    ig.f_param.rparam.d_oq=0.4;
    ig.f_param.rparam.k_vo=2.0/(static_cast<double>(is.sparam.num_sensors));
    ig.f_param.rparam.q_o=-1.0;
    ig.f_param.qparam.k_vq=5.0*ig.f_param.rparam.k_vo;//d_o와 묶여있는 변수다.
    ig.f_param.qparam.q_q=-1.0;

    ig.p_param.lparam.delta=0.1;
    ig.p_param.lparam.lam=4.5;
    ig.p_param.lparam.lam_stagnation=0.25*ig.p_param.lparam.lam;
    ig.p_param.lparam.radius=0.4*ig.p_param.lparam.lam_stagnation;

    ig.m_param.eparam.theta_max=45.0*M_PI/180.0;
    ig.m_param.eparam.tolorance=0.2;
    ig.m_param.vparam.v_max=0.2;
    ig.m_param.vparam.w_max=0.5;

    double pos[SIZE_STATE];
    double cgoal[SIZE_STATE];

    con.getPos(pos);
    //con.addGoal(2.8,3.4,0.0);//add global goal
    //con.addGoal(0.0,5.0,0.0);//add global goal
    con.addGoal(3.5,0.0,0.0);//add global goal
    //con.addGoal(2.5,2.0,0.0);//add global goal
    con.getGoal(cgoal,true);

    sen=new Sensor(is);
    // sen->addLObs(1.8,2.52,1.81,2.57);
    // sen->addLObs(1.8,2.52,2.5,2.5);
    // sen->addLObs(2.5,2.5,2.48,1.8);
    // sen->addLObs(2.48,1.8,2.53,1.801);

    // sen->addCObs(0.0,3.0,0.4);
    // sen->addCObs(-1.0,3.0,0.4);
    // sen->addCObs(1.0,3.0,0.4);

//    sen->addCObs(1.5,-0.5,0.3);
//    sen->addCObs(1.5,-0.35,0.3);
//    sen->addCObs(1.5,-0.2,0.3);
//    sen->addCObs(1.5,-0.05,0.3);
//    sen->addCObs(1.5, 0.1,0.3);
//    sen->addCObs(1.5, 0.25,0.3);
//    sen->addCObs(1.5, 0.4,0.3);


//    sen->addCObs(4.0,6.5,0.5);
//    sen->addCObs(2.5,6.5,0.5);
//    sen->addCObs(5.0,3.5,0.5);
//    sen->addCObs(6.5,3.5,0.5);
//    sen->addCObs(8.0,3.5,0.5);

    // sen->addLObs(1.7,1.7,1.65,1.75);
    // sen->addLObs(1.7,1.7,2.0,2.0);
    // sen->addLObs(2.0,2.0,2.4,1.6);
    // sen->addLObs(2.4,1.6,2.1,1.3);
    // sen->addLObs(2.1,1.3,2.15,1.25);

    act=new Actuator(ia);
    gen=new Generator(ig,*sen,pos,cgoal);

    con.setSensor(sen);
    con.setActuator(act);
    con.setGenerator(gen);
}

Robot::~Robot()
{
    stop();
    delete gen;
    delete act;
    delete sen;
}

void Robot::initRos(int argc, char **argv)
{
    tflisten.SetFPS(100);
    tflisten.TF("gmap,gbase_footprint");
    tflisten.Init(argc,argv,"pe","tf_global",this);

    lmap.onReceive  = onROSLocalMap;
    lmap.SetFPS(100);
    lmap.SetReceiver("/costmap_node/costmap/costmap");
    lmap.Init(argc,argv,"robot_local",this);
}

void Robot::start()
{
    if(!m_running)
    {
        m_running = true;
        m_worker = std::thread(&Robot::updateLoop, this);
    }
}

void Robot::run()
{
    m_running = true;
}

void Robot::stop()
{
    m_running=false;
    if(m_worker.joinable())
    {
        m_worker.join();
    }
}

std::vector<int> Robot::getiData()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_iData;
}

std::vector<double> Robot::getdData()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_dData;
}

std::vector<BYTE> Robot::getbData()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_bData;
}
#include <QElapsedTimer>
#include <QDebug>
void Robot::updateLoop()
{
    while(m_running)
    {
        dhMat h = tflisten.Get();
        dhVector o  = h.O();
        dhVector rpy= h.RPY();
        gPos[0]=o.x;
        gPos[1]=o.y;
        gPos[2]=rpy.z;
        std::vector<BYTE> map=m_bData;
        sen->setMap(map,gPos);
        con.control(gPos);
        updateData();
        {
            setDataUpdated(true);
        }
        m_condition.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool Robot::isDataUpdated()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_update;
}

void Robot::setDataUpdated(bool updated)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_update = updated;
}

void Robot::onROSLocalMap(nav_msgs::OccupancyGrid &msg, void *pArg)
{
    Robot *pMF = (Robot*)pArg;
    int w   = msg.info.width;
    int h   = msg.info.height;

    std::vector<BYTE> bData(w*h);
    int index=w*(h-1);
    for (int i=0;i<h;i++)
    {
        std::copy(&msg.data[w*i],&msg.data[w*i+w],&bData[index]);
        index-=w;
    }
    pMF->m_bData=bData;

    float x = msg.info.origin.position.x;
    float y = msg.info.origin.position.y;
    dhQuat q;
    q.x     = msg.info.origin.orientation.x;
    q.y     = msg.info.origin.orientation.y;
    q.z     = msg.info.origin.orientation.z;
    q.q     = msg.info.origin.orientation.w;
    dhMat H(q);
    dhVector rpy    = H.RPY();
    pMF->lPos[0]=x;
    pMF->lPos[1]=-y;
    pMF->lPos[2]=rpy.z;
    pMF->l_height=h;
    pMF->l_width=w;
}
#include <QDebug>
void Robot::updateData()
{
    m_update=false;
    int path_size=con.m_bArrived?con.b_path.size():0;
    std::vector<int> iData;
    std::vector<double> dData;

    iData.push_back(con.s->vc.size());//cobstacle size
    iData.push_back(con.s->vl.size());//lobstacle size
    iData.push_back(con.s->vq.size());//quark size
    iData.push_back(con.g->rPath.size());//future path size
    iData.push_back(con.s->ip.sparam.num_sensors);//sensor size
    iData.push_back(con.o.size());//optimized pos size
    iData.push_back(con.optimized_path.size());//optimized path size
    iData.push_back(path_size);
    iData.push_back(l_height);
    iData.push_back(l_width);

    for(int i=0;i<con.s->ip.sparam.num_sensors;i++)//sensor pos, data
    {
        dData.push_back(con.s->is[i].pos.x);
        dData.push_back(con.s->is[i].pos.y);
        dData.push_back(con.s->is[i].pos.q);
        dData.push_back(con.s->is[i].sense.dist);
        dData.push_back(con.s->is[i].sense.vx);
        dData.push_back(con.s->is[i].sense.vy);
    }
    for(int i=0;i<con.s->vc.size();i++)//cobs pos, radius
    {
        dData.push_back(con.s->vc[i].pos.x);
        dData.push_back(con.s->vc[i].pos.y);
        dData.push_back(con.s->vc[i].param.radius);
    }
    for(int i=0;i<con.s->vl.size();i++)//lobs pos
    {
        dData.push_back(con.s->vl[i].pos.x1);
        dData.push_back(con.s->vl[i].pos.y1);
        dData.push_back(con.s->vl[i].pos.x2);
        dData.push_back(con.s->vl[i].pos.y2);
    }
    for(int i=0;i<con.s->vq.size();i++)//quark pos
    {
        dData.push_back(con.s->vq[i].pos.x);
        dData.push_back(con.s->vq[i].pos.y);
    }
    for(int i=0;i<con.g->rPath.size();i++)//path pos
    {
        dData.push_back(con.g->rPath[i].px);
        dData.push_back(con.g->rPath[i].py);
    }
    for(int i=0;i<con.optimized_path.size();i++)//path pos
    {
        dData.push_back(con.optimized_path[i].px);
        dData.push_back(con.optimized_path[i].py);
    }
    for(int i=0;i<con.o.size();i++)
    {
        dData.push_back(con.o[i].x);
        dData.push_back(con.o[i].y);
    }
    for(int i=0;i<path_size;i++)
    {
        dData.push_back(con.b_path[i].px);
        dData.push_back(con.b_path[i].py);
        dData.push_back(con.b_path[i].pq);
    }

    dData.push_back(con.s->ip.sparam.max_dist);

    double rpos[SIZE_STATE];//robot pos
    con.getPos(rpos);
    dData.push_back(rpos[0]);
    dData.push_back(rpos[1]);
    dData.push_back(rpos[2]);

    double cgoal[SIZE_STATE];//robot goal
    con.getGoal(cgoal,false);
    dData.push_back(cgoal[0]);
    dData.push_back(cgoal[1]);

    dData.push_back(con.a->ip.radius);//robot radius
    dData.push_back(con.a->ip.dt);//robot dt

    dData.push_back(con.con_vel[0]);//control linear vel;
    dData.push_back(con.con_vel[1]);//control angular vel;

    dData.push_back(lPos[0]);//local map x
    dData.push_back(lPos[1]);//local map y
    dData.push_back(lPos[2]);//local map q

    m_iData = iData;
    m_dData = dData;
}
