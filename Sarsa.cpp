#ifndef SARSA
#define SARSA

# include "Sarsa.h"


Sarsa::Sarsa( const char * parameterFile, World * w ) {

    discreteStates      = w->getDiscreteStates() ;

    if ( not w->getDiscreteActions() ) {

        cout << "Sarsa does not support continuous actions." << endl ;
        cout << "Please check which MDP you are using it on." << endl ;
        exit(0) ;

    } else {

        numberOfActions = w->getNumberOfActions() ;

    }

    srand48( clock() ) ;

    readParameterFile( parameterFile ) ;

    if ( discreteStates ) {

        numberOfStates = w->getNumberOfStates() ;

        Q = new double*[ numberOfStates ] ;

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            Q[s] = new double[ numberOfActions ] ;

            for ( int a = 0 ; a < numberOfActions ; a++ ) {
                Q[s][a] = 0.0 ;
            }

        }

    } else {

        stateDimension      = w->getStateDimension() ;

        int layerSizesA[] = { stateDimension, nHiddenQ, 1 } ;

        for ( int a = 0 ; a < numberOfActions ; a++ ) {

            QNN.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ;

        }

        Qs = new double[ numberOfActions ] ;

    }

    QTarget = new double[ 1 ] ;
    policy = new double[ numberOfActions ] ;

}

Sarsa::~Sarsa() {

    if ( discreteStates ) {

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            delete [] Q[s] ;

        }

        delete [] Q ;

    } else {

        for ( int a = 0 ; a < numberOfActions ; a++ ) {

            delete QNN[a] ;

        }

        QNN.clear() ;

        delete [] Qs ;

    }

    delete [] QTarget ;
    delete [] policy ;

}

void Sarsa::readParameterFile( const char * parameterFile ) {

    if ( not discreteStates ) {

        ifstream ifile ;

        ifile.open( parameterFile, ifstream::in ) ;

        read_moveTo( &ifile, "nn" ) ;

        read_moveTo( &ifile, "nHiddenQ" ) ;
        ifile >> nHiddenQ ;

        ifile.close() ;

    }

}

void Sarsa::update( State * state, Action * actions, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma  ) {

    int at  = actions[0].discreteAction ;
    int at_ = actions[1].discreteAction ;

    if ( state->discrete ) {

        int st  = state->discreteState ;
        int st_ = nextState->discreteState ;

        if ( endOfEpisode ) {

            Q[ st ][ at ] += learningRate[0]*( rt - Q[ st ][ at ] ) ;

        } else {

            Q[ st ][ at ] += learningRate[0]*( rt + gamma*Q[ st_ ][ at_ ] - Q[ st ][ at ] ) ;

        }

    } else {

        double * st = state->continuousState ;
        double * st_ = nextState->continuousState ;

        if ( endOfEpisode ) {

            QTarget[ 0 ] = rt ;

        } else {

            double Qsa_ = QNN[ at_ ]->forwardPropagate( st_ )[0] ;

            QTarget[ 0 ] = rt + gamma*Qsa_ ;

        }

        QNN[ at ]->backPropagate( st, QTarget, learningRate[0] ) ;

    }

}

unsigned int Sarsa::getNumberOfLearningRates() {

    return 1 ;

}

const char * Sarsa::getName() {

    return "Sarsa" ;

}

#endif //SARSA
