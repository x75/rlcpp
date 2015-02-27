#ifndef QVLEARNING_H
#define QVLEARNING_H

# include "StateActionAlgorithm.h"

class QVlearning : public StateActionAlgorithm {
    public:
        QVlearning( const char * parameterFile, World * w ) ;
        ~QVlearning() ;
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

#endif //QVLEARNING_H
