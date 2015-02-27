#ifndef ALGORITHM_H
#define ALGORITHM_H

# include <iostream>
# include <string>
# include <fstream>
# include <math.h>
# include <vector>
# include <stdlib.h>

# include "State.h"
# include "Action.h"
# include "World.h"

using namespace std ;

class Algorithm {
    public:
        Algorithm() ;
        virtual ~Algorithm() ;
        double max( double * array, int n ) ;
        int argmax( double * array, int n ) ;
        std::vector< int > argmaxAll( double * array, int n ) ;
        void egreedy( State * state, Action * action, double epsilon ) ;

        virtual void getMaxAction( State * state, Action * action ) =0 ;
        virtual void getRandomAction( State * state, Action * action ) =0 ;
        virtual void explore( State * state, Action * action, double explorationRate, std::string explorationType, bool endOfEpisode ) =0 ;
        virtual void update( State * st, Action * action, double rt, State * st_, bool endOfEpisode, double * learningRate, double gamma  ) =0 ;
        virtual unsigned int getNumberOfLearningRates() =0;
        virtual bool getContinuousStates() =0;
        virtual bool getDiscreteStates() =0;
        virtual bool getContinuousActions() =0;
        virtual bool getDiscreteActions() =0;
        virtual const char * getName() =0;

    protected:
        void read_moveTo( ifstream *, string ) ;

        bool continuousStates, discreteStates, continuousActions, discreteActions ;
        int stateDimension, numberOfStates, actionDimension, numberOfActions ;
        double X, maxX ;
        int maxI ;
};

#endif //ALGORITHM_H
