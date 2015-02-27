#ifndef ACLA_H
#define ACLA_H

# include "StateActionAlgorithm.h"

class Acla : public StateActionAlgorithm {
    public:
        Acla( const char * parameterFile, World * w ) ;
        ~Acla() ;
        void readParameterFile( const char * parameterFile ) ;
        void update( State * st, Action * action, double rt, State * st_, bool endOfEpisode, double * learningRate, double gamma  ) ;
        unsigned int getNumberOfLearningRates() ;
        const char * getName() ;

    private:
        int nHiddenV ;
        double * V ;
        cNeuralNetwork * VNN ;
        double * VTarget ;

};

#endif //ACLA_H
