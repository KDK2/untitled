#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "actuator.h"
#include "sensor.h"
#include "generator.h"
#include <fstream>
#include <iostream>
class Controller
{
public:
    Controller();
    ~Controller();
    struct goal
    {
        double x;
        double y;
        double theta;
        double d;
        bool   arrived;
    };
    struct optimized_data
    {
        double x;
        double y;
        double cost1[4];
        double cost2[4];
        double loss;
    };

    enum con_state
    {
        idle,
        localminimum,
        optimized
    };
    void setSensor(Sensor* sensor);//original pointer
    void setActuator(Actuator* actuator);//original pointer
    void setGenerator(Generator* generator);//original pointer
    void setTemporaryGoal(double x, double y, double theta, double d);
    void setOptimizedTemporaryGoal(double x, double y, double theta);
    void addGoal(double x, double y, double theta);
    void checkMaxVelocity(double vel, double vel_max, double& dst);
    void optimize(const double *pos, double *param,double* dst, double* cst1, double* cst2, double& loss);
    double cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double* cst, double& loss);
    double normalizeCross(std::vector<Generator::path> path, int& argFeature);
    void checkGoal();
    bool isArrived();
    bool checkGoal(std::vector<Generator::path> path,bool bGlobal);

    void velocity(double* src, double& v,double& w);
    void control(double* src);
    void detectLocalminimum(bool& bLocalminimum);
    void setState(bool bLocalminimum);
    void planing();
    void moveGoal();

    void getPos(double* dst);
    void getGoal(double* dst,bool bGlobal);
    void getConState(con_state& dst);

    Sensor* s;
    Actuator* a;
    Generator* g;

    std::vector<optimized_data> o;
    std::vector<Generator::path> optimized_path;
    std::vector<Generator::path> b_path;
    double stag_pos[2];
    double con_vel[2];
    bool m_bArrived;
protected:
    void initConState(double* pos);
    void setOutput(double* v);
private:
    void updateGenerator();
    goal temporary;
    goal temporary_o;
    std::vector<goal> goals;

    double rPos[3];//robot state
    double eold;

    double kp,kd;
    double minLoss;
    con_state state;
};

#endif // CONTROLLER_H
