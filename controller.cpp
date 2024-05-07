#include "controller.h"
#include "math.h"
#include "string.h"
#define INDEX_REF_V 0
#define INDEX_REF_Q 1

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2
static std::ofstream file("data.csv");
static int iter=0;
Controller::Controller():
    s(nullptr),
    a(nullptr),
    g(nullptr),
    m_bArrived(false),
    temporary({0,0,0,0,true}),
    temporary_o({0,0,0,0,true}),
    rPos({0.0,0.0,0}),
    eold(0.0),
    kp(2.2),
    kd(0.8),
    minLoss(0.0),
    state(idle)
{
}

Controller::~Controller()
{
    file.close();
    delete g;
    delete a;
    delete s;
}

void Controller::setSensor(Sensor *sensor)
{
    if(s!=nullptr)
    {
        delete s;
    }
    s=sensor;
}

void Controller::setActuator(Actuator *actuator)
{
    if(a!=nullptr)
    {
        delete a;
    }
    a=actuator;
}

void Controller::setGenerator(Generator *generator)
{
    if(g!=nullptr)
    {
        delete g;
    }
    g=generator;
}

void Controller::setTemporaryGoal(double x, double y, double theta, double d)
{
    if(idle==state)
    {
        if(!(d>temporary.d)) return;
    }
    else if(optimized==state)
    {
//        Generator* g_temp=new Generator(*g,rPos);
//        double go[3]={x,y,theta};
//        g_temp->setPos(rPos);
//        g_temp->setGoal(go);
//        g_temp->gen(Generator::prediction);
//        double lastp[3]={g_temp->getPath().back().px,g_temp->getPath().back().py,g_temp->getPath().back().pq};
//        g_temp->calcTemporaryGoal();
//        double temp_go[3];
//        g_temp->getTemporaryGoal(temp_go);
//        temporary.x=temp_go[0];
//        temporary.y=temp_go[1];
//        temporary.theta=temp_go[2];
//        temporary.arrived=false;
//        temporary.d=d;
//        eold=0.0;
        return;
    }
    temporary.x=x;
    temporary.y=y;
    temporary.theta=theta;
    temporary.arrived=false;
    temporary.d=d;
    eold=0.0;
}

void Controller::setOptimizedTemporaryGoal(double x, double y, double theta)
{
    temporary_o.arrived=false;
    temporary_o.x=x;
    temporary_o.y=y;
    temporary_o.theta=theta;
    eold=0.0;
}

void Controller::addGoal(double x, double y, double theta)
{
    goal g;
    g.x=x;
    g.y=y;
    g.theta=theta;
    g.arrived=false;
    goals.push_back(g);
}

void Controller::checkMaxVelocity(double vel, double vel_max, double &dst)
{
    if(abs(vel)<=vel_max)
    {
        dst=vel;
    }
    else
    {
        dst=vel_max*vel/abs(vel);
    }
}
#include <algorithm>
void Controller::optimize(const double *pos, double *param, double *dst, double* cst1, double* cst2, double& loss)
{
    double delta=0.1;

    std::vector<Generator::path> pfPath;
    std::vector<Generator::path> paPath;
    std::vector<Generator::path> mfPath;
    std::vector<Generator::path> maPath;

    double gradient[2]={0.0,0.0};
    double learning_rate=0.05;

    std::vector<double> cost1;
    std::vector<double> cost2;
    std::vector<double> l;
    for(int i=0;i<2;i++)
    {
        Generator *pFuture;
        Generator *pAdd;
        Generator *mFuture;
        Generator *mAdd;

        double pPos[3]={pos[0],pos[1],pos[2]};
        double mPos[3]={pos[0],pos[1],pos[2]};

        pPos[i]+=delta;
        mPos[i]-=delta;
        pFuture=new Generator(*g,pPos);
        pFuture->setPos(pPos);
        mFuture=new Generator(*g,mPos);
        pFuture->setPos(mPos);
        pFuture->gen(Generator::oprediction);
        mFuture->gen(Generator::oprediction);

        pfPath=pFuture->getPath();
        mfPath=mFuture->getPath();

        double paPos[3]={pfPath.back().px,pfPath.back().py,pfPath.back().pq};
        double maPos[3]={mfPath.back().px,mfPath.back().py,mfPath.back().pq};

        pAdd=new Generator(*g,paPos);
        pAdd->setPos(paPos);
        mAdd=new Generator(*g,maPos);
        mAdd->setPos(maPos);

        pAdd->gen(Generator::stagnation);
        mAdd->gen(Generator::stagnation);

        paPath=pAdd->getPath();
        maPath=mAdd->getPath();
        double pc[2];//plus delta cost1,2
        double mc[2];//minus delta cost1,2
        double ploss;
        double mloss;
        double pl=cost(pfPath,paPath,pc,ploss);
        double ml=cost(mfPath,maPath,mc,mloss);
        gradient[i]=(pl-ml)/(2.0*delta);
        cost1.push_back(pc[0]);
        cost1.push_back(mc[0]);
        cost2.push_back(pc[1]);
        cost2.push_back(mc[1]);
        l.push_back(ploss);
        l.push_back(mloss);
    }
    for(int i=0;i<cost1.size();i++)
    {
        cst1[i]=cost1[i];
        cst2[i]=cost2[i];
    }
    auto min_itr=std::min_element(l.begin(),l.end());
    loss=*min_itr;
    dst[0]=pos[0]-learning_rate*gradient[0];
    dst[1]=pos[1]-learning_rate*gradient[1];
    param[0]=dst[0];
    param[1]=dst[1];
}

double Controller::cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double* cst, double& loss)
{
    double w1,w2;//weight
    double cost1,cost2;
    double mean_x,mean_y;
    double varianceX=0.0;
    double varianceY=0.0;

    w1=0.8;
    w2=10.0;

    //normalization
    int arg=0;
    cost1=normalizeCross(path,arg);

    mean_x=std::accumulate(aPath.begin(),aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.px;});
    mean_x/=aPath.size();
    mean_y=std::accumulate(aPath.begin(),aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.py;});
    mean_y/=aPath.size();

    double cost3=0.0;
    for(int i=0;i<aPath.size();i++)
    {
        varianceX+=pow(aPath.at(i).px-mean_x,2);
        varianceY+=pow(aPath.at(i).py-mean_y,2);
        double dx=stag_pos[0]-aPath.at(i).px;
        double dy=stag_pos[1]-aPath.at(i).py;
        double dist=sqrt(dx*dx+dy*dy);
        if(dist<0.5)
            cost3-=0.5;
    }
    varianceX/=aPath.size();
    varianceY/=aPath.size();
    cost2=varianceX+varianceY;
    cst[0]=w1*cost1;
    cst[1]=w2*cost2+cost3;
    double px,py;
    px=path.front().px;
    py=path.front().py;
    if(-(w1*cost1+w2*cost2)<-0.6)
        std::cout<<px<<", "<<py<<" : "<<w1*cost1<<", "<<w2*cost2<<", "<<-(w1*cost1+w2*cost2)<<std::endl;
    loss=-(w1*cost1+w2*cost2);
    return -(w1*cost1+w2*cost2);
}

double Controller::normalizeCross(std::vector<Generator::path> path, int& argFeature)
{
    std::vector<double> x(path.size());
    std::vector<double> y(path.size());
    std::vector<double> diff_x(path.size()-1);
    std::vector<double> diff_y(path.size()-1);
    std::vector<double> distance(path.size()-1);
    std::vector<double> normalized_distance(path.size()-1);
    std::vector<double> normalized_x(path.size()-1);
    std::vector<double> normalized_y(path.size()-1);
    double total_distance=0.0;
    for(int i=0;i<path.size();i++)
    {
        x[i]=path.at(i).px;
        y[i]=path.at(i).py;
    }
    for(int i=0;i<path.size()-1;i++)
    {
        diff_x[i]=x[i+1]-x[i];
        diff_y[i]=y[i+1]-y[i];
    }
    for(int i=0;i<path.size()-1;i++)
    {
        distance[i]=sqrt(pow(x[i+1]-x[i],2)+pow(y[i+1]-y[i],2));
    }
    for(int i=0;i<path.size()-1;i++)
    {
        total_distance+=distance[i];
    }
    for(int i=0;i<path.size()-1;i++)
    {
        normalized_distance[i]=distance[i]/total_distance;
    }
    for(int i=0;i<path.size()-1;i++)
    {
        normalized_x[i]=normalized_distance[i]*diff_x[i]/distance[i];
        normalized_y[i]=normalized_distance[i]*diff_y[i]/distance[i];
    }
    double cum_sum_x=0.0;
    double cum_sum_y=0.0;
    double ret=0.0;
    for(int i=0;i<path.size()-1;i++)
    {
        cum_sum_x+=normalized_x[i];
        cum_sum_y+=normalized_y[i];
        x[i]=cum_sum_x;
        y[i]=cum_sum_y;
    }
    bool bFirst=true;
    for (int i=0;i<path.size()-2;i++)
    {
        double cross_product =x[i]*y[i+1]-x[i+1]*y[i];
        ret+=abs(cross_product);
        if(bFirst)
        {
            if(abs(cross_product)>0.01)
            {
                argFeature=i;
                //bFirst=false;
            }
        }
    }
    return ret;
}

void Controller::checkGoal()
{
    double goal[3];
    getGoal(goal,false);

    double dx=goal[INDEX_X]-rPos[INDEX_X];
    double dy=goal[INDEX_Y]-rPos[INDEX_Y];
    double d=sqrt(dx*dx+dy*dy);
    double tolorance=g->ip.m_param.eparam.tolorance;
    if(!temporary.arrived)
    {
        if(d<tolorance)
        {
            temporary.arrived=true;
            temporary.d=0.0;
            //s->vq.clear();
        }
    }
    for(int i=0;i<goals.size();i++)
    {
        if(goals[i].arrived)
            continue;

        dx=goals[i].x-rPos[INDEX_X];
        dy=goals[i].y-rPos[INDEX_Y];
        d =sqrt(dx*dx+dy*dy);

        if(d<tolorance)
            goals[i].arrived=true;
    }
}

bool Controller::isArrived()
{
    bool ret=true;
    for(int i=0;i<goals.size();i++)
    {
        ret&=goals[i].arrived;
    }
    return ret;
}

bool Controller::checkGoal(std::vector<Generator::path> path,bool bGlobal)
{
    double goal[3];
    getGoal(goal,bGlobal);
    for(int i=0;i<path.size();i++)
    {
        double dx=path[i].px-goal[INDEX_X];
        double dy=path[i].py-goal[INDEX_Y];
        double d=sqrt(dx*dx+dy*dy);
        if(d<g->ip.m_param.eparam.tolorance) return true;
    }
    return false;
}

void Controller::velocity(double *src, double &v, double &w)
{
    double v_max=g->ip.m_param.vparam.v_max;
    double w_max=g->ip.m_param.vparam.w_max;
    double v_ref=src[INDEX_REF_V];
    double q_ref=src[INDEX_REF_Q];
    double e;
    g->normalizeAngle(q_ref-rPos[INDEX_Q],e);

    checkMaxVelocity(v_ref,v_max,v);
//    checkMaxVelocity(kp*e+ki*esum,w_max,w);
    checkMaxVelocity(kp*e+kd*(e-eold),w_max,w);
    eold=e;
}
#define RAD(x) ((x)*M_PI/180.0)
#include <chrono>
#include <QDebug>
void Controller::control(double* src)
{
    bool bDetect=false;
    if(isArrived())
    {
        double v[2]={0.0,0.0};
        setOutput(v);
        m_bArrived=true;
        return;
    }
    initConState(src);
    b_path.push_back({rPos[0],rPos[1],rPos[2]});
    detectLocalminimum(bDetect);
    setState(bDetect);
    planing();
    moveGoal();
    updateGenerator();
}

void Controller::detectLocalminimum(bool& bLocalminimum)
{
    Generator* pRef=nullptr;
    Generator* pLocal=nullptr;
    Generator* pGen;
    double target[3];
    double lastPredict[3];
    double refPos[3];
    if(idle==state)
    {
        memcpy(refPos,rPos,sizeof(double)*3);
        pRef=g;
    }
    else if(localminimum==state)
    {
//        double tg[3];
//        getGoal(tg,false);//get temporary goal
//        memcpy(refPos,tg,sizeof(double)*3);
//        pLocal=new Generator(*g,refPos);
//        pRef=pLocal;
        memcpy(refPos,rPos,sizeof(double)*3);
        pRef=g;
    }
    else if(optimized==state)
    {
        memcpy(refPos,rPos,sizeof(double)*3);
        pRef=g;
    }
    getGoal(target,true);
    pRef->setPos(refPos);
    pRef->setGoal(target);
    pRef->gen(Generator::prediction);
    std::vector<Generator::path> temp;
    temp=pRef->getPath();
    lastPredict[0]=temp.back().px;
    lastPredict[1]=temp.back().py;
    lastPredict[2]=temp.back().pq;

    pGen=new Generator(*pRef,lastPredict);
    pGen->gen(Generator::stagnation);
    pGen->getStagPos(stag_pos);
//    stag_pos[0]=lastPredict[0];
//    stag_pos[1]=lastPredict[1];
    std::vector<Generator::path> stag_path;
    stag_path=pGen->getPath();
    if(!checkGoal(stag_path,true))
    {
        if(pGen->isLocalmin())
        {
            bLocalminimum=true;
            if(localminimum==state)
            {
                //s->addQuark(stag_pos[0],stag_pos[1]);
            }
        }
        else
        {
            bLocalminimum=false;
        }
    }
    else
    {
        bLocalminimum=false;
    }
    for(int i=0;i<stag_path.size();i++)
    {
        pRef->rPath.push_back(stag_path[i]);
    }
}

void Controller::setState(bool bLocalminimum)
{
    if(bLocalminimum)
    {
        if     (idle==state)         state=localminimum;
        else if(localminimum==state) state=localminimum;
        else if(optimized==state)    state=optimized;
    }
    else
    {
        if     (idle==state)         state=idle;
        //else if(localminimum==state) state=optimized;
        else if(optimized==state)
        {
            if(temporary.arrived)
                state=idle;
        }
    }
}

void Controller::planing()
{
    //optimize when detect localminimum
    //this function run when state is idle or localminimum
    //set temporary goal
    Generator* ref=nullptr;
    double d=0.0;
    double tg[3];
    if(idle==state)
    {
        //calc temporary goal
        //set temporary goal
        ref=g;
        d=ref->calcTemporaryGoal();
        ref->getTemporaryGoal(tg);
        setTemporaryGoal(tg[0],tg[1],tg[2],d);
        return;
    }
    else if(localminimum==state)
    {
        o.clear();
        optimized_path.clear();
        int sgd_iter=200;
        double opos[3]={rPos[0],rPos[1],rPos[2]};
        double param[2]={rPos[0],rPos[1]};
        //(x-a)^2+(y-b)^2=r^2
        //a= rpos_x, b= rpos_y, r=norm(stag-rpos)+d_o
        std::vector<optimized_data>temp_o(sgd_iter);
        double dx=stag_pos[0]-rPos[0];
        double dy=stag_pos[1]-rPos[1];
        double radius=sqrt(dx*dx+dy*dy)+g->ip.f_param.rparam.d_o;
        for(int i=0;i<sgd_iter;i++)
        {
            double temp[2]={opos[0],opos[1]};
//            double temp[2]={rPos[0],rPos[1]};
//            g->genSample(temp,radius,temp);
            temp[0]=g->addNoise(temp[0],0.1);
            temp[1]=g->addNoise(temp[1],0.1);
            double dst[2];
            std::cout<<"opos!! = "<<opos[0]<<", "<<opos[1]<<std::endl;
            optimize(temp,param,dst,temp_o[i].cost1,temp_o[i].cost2,temp_o[i].loss);
            opos[0]=dst[0];
            opos[1]=dst[1];
            temp_o[i].x=opos[0];
            temp_o[i].y=opos[1];
            o.push_back(temp_o[i]);
        }
        if(o.size()>0)
        {
            std::vector<optimized_data> to;
            for(int i=0;i<o.size();i++)
            {
                if(o[i].loss<-1.2)
                    to.push_back(o[i]);
            }
            if(to.size()==0)
            {
                std::cout<<"no optimized data"<<std::endl;
                o.clear();
                return;
            }
            auto minIt = std::min_element(to.begin(), to.end(),
                                          [](const optimized_data& a, const optimized_data& b){return a.loss < b.loss;});
            int minIndex = std::distance(to.begin(), minIt);
            if(minLoss>to[minIndex].loss)
            {
                minLoss=to[minIndex].loss;
                o.clear();
                o.push_back(to[minIndex]);
            }
            else
            {
                o.clear();
                return;
            }
            double op_pos[3]={to[minIndex].x,to[minIndex].y,0.0};
//            Generator* g_temp = new Generator(*g,rPos);
//            g_temp->setGoal(op_pos);
//            g_temp->gen(Generator::oprediction);
//            g_temp->calcTemporaryGoal();
//            Generator::path l_path=g_temp->getPath().back();
//            double op[SIZE_STATE]={l_path.px,l_path.py,l_path.pq};
//            g_temp->getTemporaryGoal(op);
//            Generator* g_stag = new Generator(*g,rPos);
//            g_stag->setPos(op);
//            g_stag->gen(Generator::stagnation);
//            if(g_stag->isLocalmin())
//            {
//                minLoss=0.0;
//                o.clear();
//                return;
//            }
            ref=new Generator(*g,rPos);
            ref->setPos(op_pos);
            ref->gen(Generator::oprediction);
            double lp[3]={ref->getPath().back().px,ref->getPath().back().py,ref->getPath().back().pq};
            ref->calcTemporaryGoal();
            ref->getTemporaryGoal(lp);
            Generator *sg=new Generator(*g,rPos);
            sg->setPos(lp);
            //sg->ip.p_param.lparam.lam_stagnation=10.0;
            sg->gen(Generator::stagnation);
            if(sg->isLocalmin())
            {
                minLoss=0.0;
                o.clear();
                return;
            }
            optimized_path=ref->getPath();
            std::cout<<"optimized first : "<<optimized_path.at(0).px<<", "<<optimized_path.at(0).py<<std::endl;
            int argValid=-1;
            normalizeCross(optimized_path,argValid);
            std::vector<Generator::path> sp=sg->getPath();
            std::copy(sp.begin(), sp.end(), std::back_inserter(optimized_path));
            //find argmax cross product
            if(argValid==-1)
            {
                minLoss=0.0;
                o.clear();
                return;
            }
            for(int i=argValid;i<optimized_path.size();i++)
            {
                g->rPath.push_back(optimized_path[i]);
            }
            std::cout<<"rPos x : "<<rPos[0]<<", rPos y : "<<rPos[1]<<std::endl;
            Generator* calcTG = new Generator(*g,rPos);
            calcTG->rPath=g->rPath;
            double d =calcTG->calcTemporaryGoal();
            calcTG->getTemporaryGoal(tg);
            setTemporaryGoal(tg[0],tg[1],tg[2],d);
            state=optimized;
            std::cout<<"loss : "<<to[minIndex].loss<<std::endl;
            std::cout<<"goal: "<<tg[0]<<", "<<tg[1]<<std::endl;
            std::cout<<"min_opos: "<<to[minIndex].x<<", "<<to[minIndex].y<<std::endl;
            std::cout<<"last_opos: "<<param[0]<<", "<<param[1]<<std::endl;
            std::cout<<"last_loss: "<<temp_o[sgd_iter-1].loss<<std::endl;
            //setOptimizedTemporaryGoal(tg[0],tg[1],tg[2]);
            //temporary=temporary_o;
        }
        //run optimize
        //select best loss
        //calc temporary goal
        //set temporary goal
        return;
    }
    else if(optimized==state)
    {
        return;
    }
}

void Controller::moveGoal()
{
    //this function run when state is idle or optimized
    if(localminimum==state)
        return;

    double tg[3];
    double v_ref,q_ref,v,w;
    double ref[2];
    getGoal(tg,false);
    g->setGoal(tg);
    g->gen(Generator::reference);
    g->getRef(v_ref,q_ref);
    ref[0]=v_ref;
    ref[1]=q_ref;
    velocity(ref,v,w);
    con_vel[0]=v;
    con_vel[1]=w;
}

void Controller::getPos(double *dst)
{
    dst[INDEX_X]=rPos[INDEX_X];
    dst[INDEX_Y]=rPos[INDEX_Y];
    dst[INDEX_Q]=rPos[INDEX_Q];
}

void Controller::getGoal(double *dst,bool bGlobal)
{
    if(bGlobal)
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
    if(!temporary.arrived)
    {
        dst[INDEX_X]=temporary.x;
        dst[INDEX_Y]=temporary.y;
        dst[INDEX_Q]=temporary.theta;
        return;
    }
    else
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
}

void Controller::initConState(double *pos)
{
    rPos[INDEX_X]=pos[INDEX_X];
    rPos[INDEX_Y]=pos[INDEX_Y];
    rPos[INDEX_Q]=pos[INDEX_Q];
}
#define INDEX_VEL    0
#define INDEX_LINEAR 1
void Controller::setOutput(double *v)
{
    con_vel[INDEX_VEL]=v[INDEX_VEL];
    con_vel[INDEX_LINEAR]=v[INDEX_LINEAR];
}

void Controller::updateGenerator()
{
    g->updateSensor(*s);
    checkGoal();
}
