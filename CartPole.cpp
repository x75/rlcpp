#ifndef CARTPOLE
#define CARTPOLE
# include <ctime>
# include <cstdlib>
# include <iostream>
# include <math.h>
# include "CartPole.h"

using namespace std;

CartPole::CartPole() {
    srand((unsigned)time(0));
    MINACTION = -10.0;
    MAXACTION = 10.0;

    continuousActions   = true ;
    discreteActions     = true ;
    continuousStates    = true ;
    discreteStates      = false ;

    stateDimension  = 4;
    actionDimension = 1;
    numberOfActions = 21 ;

    Mc = 1.0;
    mp = 0.1;
    l = 0.5;
    t = 0.02;
    g = 9.81;

    totalM = Mc + mp;
    ml = mp*l;

    maxPhi = M_PI / 15.0;
    maxX = 2.4;
    maxDPhi = M_PI / 15.0;
    maxDX = 2.4;

    reset() ;
}

CartPole::~CartPole() {
}

void CartPole::reset() {
    double rnd = ((double) rand()/RAND_MAX);

    phi     = 0.1*rnd - 0.05;
    dphi    = 0.0;
    x       = 0.0;
    dx      = 0.0;

    eoe     = true ;
}

void CartPole::step( double h, double force ) {
    double cos_phi = cos( phi );
    double sin_phi = sin( phi );
    double dphi_sq = dphi*dphi;
    double cos_phi_sq = cos_phi*cos_phi;

    double ddphi = force*cos_phi - totalM*g*sin_phi + ml*(cos_phi*sin_phi)*dphi*dphi ;
    ddphi /= ml*cos_phi*cos_phi - totalM*l ;

    double ddx = force + ml*sin_phi*dphi_sq - mp*g*cos_phi*sin_phi ;
    ddx /= totalM - mp*cos_phi_sq ;

    phi     += h*dphi;
    dphi    += h*ddphi;
    x       += h*dx;
    dx      += h*ddx;
}

double CartPole::reward() {
    double reward = 1.0;

    if (phi < -maxPhi) {
        reward = -1.0;
    } else if (phi > maxPhi) {
        reward = -1.0;
    } else if (x > maxX) {
        reward = -1.0;
    } else if (x < -maxX) {
        reward = -1.0;
    }
    return reward;
}

void CartPole::update(double deltaTime, double force) {
    int n = 10;
    double h = deltaTime/n;
    for( int i = 0 ; i < n ; i++ ) {
        step(h, force);
    }
}

double CartPole::act( Action * action ) {

    double force = 0.0 ;

    if ( action->continuous ) {
        // This version of cart pole scales continuous actions in [-1.0, 1.0] to the nearest
        // discrete action in the set { -10, -9, ..., 9, 10 }

        force = action->continuousAction[0] ;

        if ( force < -1 ) {
            force = -1.0 ;
        } else if ( force > 1 ) {
            force = 1.0 ;
        }

        force += 1.0 ;                  // range: [0.0, 2.0]
        force *= 0.5 ;                  // range: [0.0, 1.0]
        force *= numberOfActions - 1 ;  // range: [0.0, numberOfActions - 1]
        force = floor( force + 0.5 ) ;    // set: { 0, 1, ..., numberOfActions - 1 }

    } else if ( action->discrete ) {

        force = action->discreteAction ;
    }

    //Scale force from { 0, 1, ..., numberOfActions - 1 } to { -10, 10 }:
    force /= numberOfActions - 1 ;      // set: {0.0, 1/(numberOfActions-1), ..., 1.0 }
    force *= MAXACTION - MINACTION ;    // set: {0.0, 20.0/(numberOfActions-1), ..., 20.0 }
    force += MINACTION ;                // set: {-10.0, -10.0 + 20.0/(numberOfActions-1), ..., 10.0 }

    return act( force ) ;

}

double CartPole::act( double force ) {

    update( t, force );
    double r = reward();

    if (r < 0) {
        reset();
    } else {
        eoe = false ;
    }
    return r;
}

bool CartPole::endOfEpisode() {
    return eoe ;
}

void CartPole::getState( State * state ) {

    state->continuousState[0] = phi /maxPhi;
    state->continuousState[1] = dphi/maxDPhi;
    state->continuousState[2] = x   /maxX;
    state->continuousState[3] = dx  /maxDX;

}

void CartPole::setState( State * state ) {

    phi     = state->continuousState[0]*maxPhi;
    dphi    = state->continuousState[1]*maxDPhi;
    x       = state->continuousState[2]*maxX;
    dx      = state->continuousState[3]*maxDX;

}

bool CartPole::getDiscreteStates() {
    return false ;
}

bool CartPole::getContinuousStates() {
    return true ;
}

int  CartPole::getStateDimension() {
    return stateDimension ;
}

bool CartPole::getDiscreteActions() {
    return true ;
}

bool CartPole::getContinuousActions() {
    return true ;
}

int  CartPole::getActionDimension() {
    return actionDimension ;
}

int  CartPole::getNumberOfActions() {
    return numberOfActions;
}

const char * CartPole::getName() {
    return "CartPole" ;
}

#endif //CARTPOLE
