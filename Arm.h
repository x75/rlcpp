#ifndef ARM_H
#define ARM_H
# include "State.h"
# include "StateActionUtils.h"
# include "World.h"
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

typedef float motor;
typedef float sensor;

class Arm : public World {
    public:
        Arm();
        ~Arm();

        double act( Action * );
        void getState( State * );
        void setState( State * );
        bool endOfEpisode();
        void reset();

        bool getDiscreteStates() ;
        bool getContinuousStates() ;
        int  getStateDimension() ;
        bool getDiscreteActions() ;
        bool getContinuousActions() ;
        int  getActionDimension() ;
        int  getNumberOfActions() ;

        const char * getName() ;
	// ros
	void sensorCallback(const std_msgs::Float64MultiArray::ConstPtr& motormsg);

    private:
        double act( double ) ;
        void accel();
        void step( double, double* );
        void update(double , double* );
        double reward();
        // double phi, dphi, x, dx;
        // double maxPhi, maxDPhi, maxX, maxDX;
        double MINACTION, MAXACTION;
	double t;
        // double Mc, mp, l, t, g;
        // double totalM, ml, NORMANGLE;
	double theta0, theta1, theta2;
	double l0, l1, l2;
        bool eoe ;
	// ros
	// int number_sensors;
	// int number_motors;
	bool gotsensor;
	ros::Publisher motor_pub;
	ros::Subscriber sensor_sub;
	// ros::Rate *loop_rate;
	motor* motorValues;
	sensor* sensorValues;

};

#endif // ARM_H
