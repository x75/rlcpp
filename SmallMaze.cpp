#ifndef SMALLMAZE
#define SMALLMAZE
# include "SmallMaze.h"

using namespace std;

// The 'maze' looks as follows:
//
//  ___ ___ ___
// |   |   |   |
// | 2 | 5 | 8 |
// |___|___|___|
// |   |   |   |
// | 1 | 4 | 7 |
// |___|___|___|
// |   |   |   |
// | 0 | 3 | 6 |
// |___|___|___|
//
// where '0' is the starting state
// and '8' is the goal state. Any
// action leading to state 8 ends
// an episode and yields reward 0.
// All other actions yield reward
// -1.0.

SmallMaze::SmallMaze() {
    state = 0 ;

    discreteStates      = true ;
    numberOfStates      = 9 ;

    discreteActions     = true ;
    numberOfActions     = 4 ;

    //For continuous actions:
    continuousActions   = true ;
    actionDimension     = 2 ;
}

SmallMaze::~SmallMaze() {

}

double SmallMaze::act( Action * action ) {

    int direction ;

    if ( action->continuous ) {

        // The continuous action is interpreted as a vector and is mapped to the closest of the available vectors:
        double d1 = action->continuousAction[0] ; //up-down dimension
        double d2 = action->continuousAction[1] ; //left-right dimension

        if ( d1*d1 > d2*d2 ) { //if up-down is the largest dimension

            if ( d1 > 0 ) { //determine up or down
                direction = 0 ; //up
            } else {
                direction = 2 ; //down
            }

        } else {

            if ( d2 > 0 ) {
                direction = 3 ; //right
            } else {
                direction = 1 ; //left
            }

        }

        //cout << d1 << " " << d2 << ": " << direction << endl ;

    } else {

        direction = action->discreteAction ;

    }

    double reward = -1.0 ;

    eoe = false ; //We assume the episode does not end.

    if ( direction == 0 ) {             // up

        if ( state % 3 < 2 ) {
            state++ ;
        }

    } else if ( direction == 1 ) {      // left

        if ( state > 2 ) {
            state -= 3 ;
        }

    } else if ( direction == 2 ) {      // down

        if ( state % 3 > 0 ) {
            state-- ;
        }

    } else {                            // right

        if ( state < 6 ) {
            state += 3 ;
        }

    }

    if ( state == 8 ) {
        // If state == 8, end episode and return to 0.
        reward = 0.0 ;
        eoe = true ;

        state = 0 ;
    }

    return reward ;
}

void SmallMaze::getState( State * s ) {

    s->discreteState = state ;

}

void SmallMaze::setState( State * s ) {

    state = s->discreteState ;

}

bool SmallMaze::endOfEpisode() {

    return eoe ;

}

const char * SmallMaze::getName() {

    return "SmallMaze" ;

}

#endif //SMALLMAZE
