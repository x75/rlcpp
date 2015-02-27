#ifndef WORLD_H
#define WORLD_H

# include "State.h"
# include "Action.h"

class World {
    public:
        World() {
            continuousStates = false ;
            discreteStates = false ;
            continuousActions = false ;
            discreteActions = false ;
        }
        virtual ~World() { }
        virtual bool getContinuousStates() { return continuousStates ; }
        virtual bool getDiscreteStates() { return discreteStates ; }
        virtual int  getStateDimension() { return stateDimension ; }
        virtual int  getNumberOfStates() { return numberOfStates ; }
        virtual bool getContinuousActions() { return continuousActions ; }
        virtual bool getDiscreteActions() { return discreteActions ; }
        virtual int  getActionDimension() { return actionDimension ; }
        virtual int  getNumberOfActions() { return numberOfActions ; }
        virtual void reset() {}
        virtual bool endOfEpisode() { return false ; }

        virtual double act( Action * ) =0;

        virtual const char * getName() =0;
        virtual void getState( State * ) =0 ;
        virtual void setState( State * ) =0 ;

    protected:
        bool discreteStates, discreteActions, continuousStates, continuousActions ;
        int stateDimension, actionDimension, numberOfStates, numberOfActions ;

};

#endif //WORLD_H
