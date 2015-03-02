#ifndef CARTPOLE_H
#define CARTPOLE_H
# include "State.h"
# include "StateActionUtils.h"
# include "World.h"
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

class CartPole : public World {
    public:
        CartPole();
        ~CartPole();

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

    private:
        double act( double ) ;
        void accel();
        void step( double, double );
        void update(double , double );
        double reward();
        double phi, dphi, x, dx;
        double maxPhi, maxDPhi, maxX, maxDX;
        double MINACTION, MAXACTION;
        double Mc, mp, l, t, g;
        double totalM, ml, NORMANGLE;
        bool eoe ;
	ros::Rate *loop_rate;
	ros::Publisher pub_motor;
	ros::Publisher pub_sensor1;
	ros::Publisher pub_sensor2;
};

#endif // CARTPOLE_H
