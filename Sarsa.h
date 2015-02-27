#ifndef SARSA_H
#define SARSA_H

# include "StateActionAlgorithm.h"

class Sarsa : public StateActionAlgorithm {
    public:
        Sarsa( const char * parameterFile, World * w ) ;
        ~Sarsa() ;
        void readParameterFile( const char * parameterFile ) ;
        void update( State * st, Action * actions, double rt, State * st_, bool endOfEpisode, double * learningRate, double gamma  ) ;
        unsigned int getNumberOfLearningRates() ;
        const char * getName() ;

};

#endif //SARSA_H
