#ifndef EXPERIMENT_H
#define EXPERIMENT_H

# include <iostream>
# include <string>
# include <sstream>
# include <vector>

# include "State.h"
# include "Action.h"
# include "World.h"
# include "Algorithm.h"

# include "Qlearning.h"
# include "QVlearning.h"
# include "Sarsa.h"
# include "Acla.h"
# include "Cacla.h"

class Experiment {
    
    public:
        virtual ~Experiment() ;
        bool initializeState( State * state, Algorithm * algorithm, World * world ) ;
        bool initializeAction( Action * action, Algorithm * algorithm, World * world ) ;
        void getParametersAndRunExperiments( World * world ) ;
        
    protected:
        double * runExperiment( World * world ) ;
        void explore( State * state, Action * action ) ;
        void storeRewards( const char *, double * , int ) ;
        void setAlgorithm( string algorithmName, World * world ) ;
        void readParameterFile( ) ;
        void read_moveTo( ifstream *, string ) ;
        vector< double > read_doubleArray( string ) ;
        
        Algorithm * algorithm ;
        World * world ;
        
        State * state ;
        State * nextState ;
        Action * action ;
        Action * nextAction ;
        
        int nExperiments, nAlgorithms ;
        int nSteps, nEpisodes, nMaxStepsPerEpisode, nResults ;
        int nTrainSteps, nTrainEpisodes, nMaxStepsPerTrainEpisode, nTrainResults, trainStorePer ;
        int nTestSteps, nTestEpisodes, nMaxStepsPerTestEpisode, nTestResults, testStorePer ;
        int stateDimension, actionDimension ;
        bool discreteStates, discreteActions, endOfEpisode ;
        bool storePerStep, storePerEpisode ;
        bool boltzmann, egreedy, gaussian ;
        int nLearningRates ;
        
        const char * parameterFile ;
        string parameterPath ;
        string algorithmName ;
        vector< string > algorithms ;
        vector< double > taus ;
        vector< double > epsilons ;
        vector< double > sigmas ;
        string learningRateDecreaseType ;
        double * learningRate ;
        vector< vector < double > >  learningRates ;
        vector< double >  gammas ;
        double tau, epsilon, sigma, gamma ;
        
        bool train ;
        
    
};

#endif //EXPERIMENT_H

