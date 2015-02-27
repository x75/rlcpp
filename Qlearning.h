#ifndef QLEARNING_H
#define QLEARNING_H

# include "StateActionAlgorithm.h"

class Qlearning : public StateActionAlgorithm {
    public:
        Qlearning( const char * parameterFile, World * w ) ;
        ~Qlearning() ;
        void readParameterFile( const char * parameterFile ) ;
        void update( State * st, Action * action, double rt, State * st_, bool endOfEpisode, double * learningRate, double gamma  ) ;
        unsigned int getNumberOfLearningRates() ;
        const char * getName() ;

};

#endif //QLEARNING_H
