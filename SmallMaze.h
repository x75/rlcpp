#ifndef SMALLMAZE_H
#define SMALLMAZE_H
# include <cstdlib>
# include <iostream>
# include <math.h>
# include "State.h"
# include "Action.h"
# include "World.h"

using namespace std;

class SmallMaze : public World {

    public:
        SmallMaze() ;
        ~SmallMaze() ;
        double act( Action * action ) ;
        void getState( State * state ) ;
        void setState( State * state ) ;
        bool endOfEpisode() ;
        const char * getName() ;

        bool getDiscreteStates() { return discreteStates ; }
        int getNumberOfStates() { return numberOfStates ; }

        bool getDiscreteActions() { return discreteActions ; }
        int getNumberOfActions() { return numberOfActions ; }

        bool getContinuousActions() { return continuousActions ; }
        int getActionDimension() { return actionDimension ; }


    private:
        bool eoe ;
        int state ;

};

#endif //SMALLMAZE_H
