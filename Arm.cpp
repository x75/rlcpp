#ifndef ARM
#define ARM
# include <ctime>
# include <cstdlib>
# include <iostream>
# include <math.h>
# include "Arm.h"
#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

using namespace std;

Arm::Arm() {
    srand((unsigned)time(0));
    MINACTION = -1.0;
    MAXACTION = 1.0;

    continuousActions   = true ;
    discreteActions     = true ;
    continuousStates    = true ;
    discreteStates      = false ;

    stateDimension  = 4;
    actionDimension = 3;
    numberOfActions = 21;

    cnt = 0;

    char * p=0;
    int argc=0;
    ros::init(argc, &p, getName());

    ros::NodeHandle n;
    pub_reset = n.advertise<std_msgs::Int32>("/robot/0/reset", 1);
    pub_target = n.advertise<std_msgs::Float32MultiArray>("/robot/0/target", 1);
    pub_motor = n.advertise<std_msgs::Float32MultiArray>("/robot/0/motors", 1);
    pub_reward = n.advertise<std_msgs::Float32MultiArray>("/robot/0/perf", 1);
    sub_sensor  = n.subscribe("/robot/0/sensors", 1, &Arm::sensorCallback, this);
    sensorValues = (sensor*)malloc(sizeof(sensor)*actionDimension);
    memset(sensorValues,0,sizeof(double)*actionDimension);
    // ros::R
    usleep(1000000);
    // loop_rate = new ros::Rate(10);
    
    gotsensor = false;

    // Mc = 1.0;
    // mp = 0.1;
    // l = 0.5;
    // t = 0.02;
    // g = 9.81;

    // totalM = Mc + mp;
    // ml = mp*l;

    // maxPhi = M_PI / 15.0;
    // maxX = 2.4;
    // maxDPhi = M_PI / 15.0;
    // maxDX = 2.4;
    t = 0.;

    // target
    tx = 0.;
    ty = 0.;

    // l0 = 0.69230769;
    // l1 = 0.23076923;
    // l2 = 0.07692308;

    reset() ;
}

Arm::~Arm() {
}

void Arm::sensorCallback(const std_msgs::Float32MultiArray::ConstPtr& sensormsg) {
  // std::cerr << "got something: [" << sensormsg->data[0] << ", " << sensormsg->data[1] << "]" << std::endl;
  // std::cerr << "data size: " << sensormsg->data.size() << ", " << stateDimension << std::endl;
  int len=std::min((int)sensormsg->data.size(), stateDimension);
  for(int k=0;k<len;k++){
    sensorValues[k] = sensormsg->data[k];
    // std::cout << "sensors: " << sensorValues[k] << std::endl;
  }
  // error is the input
  sensorValues[0] -= tx;
  sensorValues[1] -= ty;
  gotsensor = true;
  // cout << "gotsensor? cb " << gotsensor << endl;
}

void Arm::settarget() {
    double rnd = ((double) rand()/RAND_MAX);
    double phi = rnd * 0.5 * M_PI;
    double r = 0.8 + rnd * 0.2;
    tx = r * cos(phi);
    ty = r * sin(phi);
    cout << "rnd = " << rnd << ", phi = " << phi << ", r = " << r << ", tx " << tx << ", ty = " << ty << endl;

    std_msgs::Float32MultiArray msg_target;
    msg_target.data.clear();
    msg_target.data.push_back(tx);
    msg_target.data.push_back(ty);
    msg_target.data.push_back(0.);
    pub_target.publish(msg_target);
}

void Arm::reset() {
    std_msgs::Int32 msg;
    msg.data = 23;
    pub_reset.publish(msg);

    settarget();
    // cout << "tx " << tx << ", ty = " << ty << endl;
    cnt = 0;
    // motor
    eoe     = true ;
}

void Arm::step( double h, double *force ) {
  std_msgs::Float32MultiArray msg;
  //   msg.layout.dim_length = 1;
  msg.data.clear();
  for(int k=0;k<actionDimension;k++){
    msg.data.push_back(force[k]);
    // cout << "msg.data " << k << ", " << msg.data[k] << endl;
  }
  // msg.data = std::vector<double>(sensors[0], sensors[sensornumber-1]);
  // memcpy(msg.data,sensors, msg.data);
  // for (int i = 0; i < 10; i++) {
  // cout << "publishing " << i << endl;
  pub_motor.publish(msg);
  // cout << "published " << i << endl;
  // ros::spinOnce();
  // cout << "spun " << i << endl;
  // loop_rate->sleep();
  // }
  // spin once so to harvest incoming data
  // cout << "step" << endl;
}

double Arm::reward() {
    std_msgs::Float32MultiArray msg_reward;
    double reward = 1.0;
    double err = 0., xerr = 0, yerr = 0;
    // double tx = 1.;
    // double ty = 0.;

    xerr = fabs(sensorValues[0] - tx);
    yerr = fabs(sensorValues[1] - ty);
    err = sqrt(pow(xerr + yerr, 2));
    // cout << "sensors: " << sensorValues[0] << ", " << sensorValues[1] << endl;
    // cout << "errx " << fabs(sensorValues[0] - tx) <<  ", " << "erry " << fabs(sensorValues[1] - ty) << endl;
    // if (err < 0.2) {
    // 	reward = 1.0;
    // }
    // else {
    //   reward = -1.;
    // }
    // if (reward > 0) {
    //   // cout << "reward = " << reward << endl;
    //   cout << reward << flush;
    // }
    reward = -err;

    msg_reward.data.clear();
    msg_reward.data.push_back(reward);
    pub_reward.publish(msg_reward);

    // cout << "reward" << endl;
    return reward;
}

void Arm::update(double deltaTime, double *force) {
  int n = 1;
  double h = deltaTime/n;
  gotsensor = false;
  for( int i = 0 ; i < n ; i++ ) {
    step(h, force);
  }
  // cout << "gotsensor? pre" << gotsensor << endl;
  while (!gotsensor && ros::ok()) {
    // wait
    // cout << "gotsensor? inner " << gotsensor << endl;
    ros::spinOnce();
    usleep(100);
    // // loop_rate->sleep();
  }
  // cout << "gotsensor? post" << gotsensor << endl;
  std_msgs::Float32MultiArray msg_target;
  msg_target.data.clear();
  msg_target.data.push_back(tx);
  msg_target.data.push_back(ty);
  msg_target.data.push_back(0.);
  pub_target.publish(msg_target);
  cnt++;

  if (cnt % 500 == 0)
    settarget();
  // cout << "update" << endl;
}

double Arm::act( Action * action ) {

    double force = 0.0 ;

    if ( action->continuous ) {
      // cout << "a0: " << action->continuousAction[0] << ", a1: " << action->continuousAction[1] << endl;
        // This version of cart pole scales continuous actions in [-1.0, 1.0] to the nearest
        // discrete action in the set { -10, -9, ..., 9, 10 }

        // force = action->continuousAction[0] ;

        // if ( force < -1 ) {
        //     force = -1.0 ;
        // } else if ( force > 1 ) {
        //     force = 1.0 ;
        // }

        // force += 1.0 ;                  // range: [0.0, 2.0]
        // force *= 0.5 ;                  // range: [0.0, 1.0]
        // force *= numberOfActions - 1 ;  // range: [0.0, numberOfActions - 1]
        // force = floor( force + 0.5 ) ;    // set: { 0, 1, ..., numberOfActions - 1 }

    } else if ( action->discrete ) {

        force = action->discreteAction ;
    }

    // //Scale force from { 0, 1, ..., numberOfActions - 1 } to { -10, 10 }:
    // force /= numberOfActions - 1 ;      // set: {0.0, 1/(numberOfActions-1), ..., 1.0 }
    // force *= MAXACTION - MINACTION ;    // set: {0.0, 20.0/(numberOfActions-1), ..., 20.0 }
    // force += MINACTION ;                // set: {-10.0, -10.0 + 20.0/(numberOfActions-1), ..., 10.0 }

    // return act( force ) ;
    update(t, action->continuousAction);
    // cout << "act" << endl;
    double r = reward();

    // if (r <= -1.5) {
    //   reset();
    // }
    // else {
    //   eoe = false;
    // }
    return r;
}

// double Arm::act( double force ) {
//   // send out command
//     update( t, force );
//     double r = reward();

//     if (r < 0) {
//         reset();
//     } else {
//         eoe = false ;
//     }
//     return r;
// }

bool Arm::endOfEpisode() {
    return eoe ;
}

void Arm::getState( State * state ) {
  int i;
  for (i = 0; i < stateDimension; i++) {
    state->continuousState[i] = sensorValues[i];
  // state->continuousState[1] = 0.;
  // state->continuousState[2] = 0.;
  // state->continuousState[3] = 0.;
  }
  // cout << "getState" << endl;

}

void Arm::setState( State * state ) {

  // phi     = state->continuousState[0];
  // dphi    = state->continuousState[1];
  // x       = state->continuousState[2];
  // dx      = state->continuousState[3];

}

bool Arm::getDiscreteStates() {
    return false ;
}

bool Arm::getContinuousStates() {
    return true ;
}

int  Arm::getStateDimension() {
    return stateDimension ;
}

bool Arm::getDiscreteActions() {
    return true ;
}

bool Arm::getContinuousActions() {
    return true ;
}

int  Arm::getActionDimension() {
    return actionDimension ;
}

int  Arm::getNumberOfActions() {
    return numberOfActions;
}

const char * Arm::getName() {
    return "Arm" ;
}

#endif //ARM
