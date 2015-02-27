#ifndef QVLEARNING
#define QVLEARNING

# include "QVlearning.h"


QVlearning::QVlearning( const char * parameterFile, World * w ) {

    discreteStates      = w->getDiscreteStates() ;
    stateDimension      = w->getStateDimension() ;

    if ( not w->getDiscreteActions() ) {

        cout << "QV-learning does not support continuous actions." << endl ;
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
        V = new double[ numberOfStates ] ;

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            Q[s] = new double[ numberOfActions ] ;
            V[s] = 0.0 ;

            for ( int a = 0 ; a < numberOfActions ; a++ ) {
                Q[s][a] = 0.0 ;
            }

        }

    } else {

        int layerSizesA[] = { stateDimension, nHiddenQ, 1 } ;
        int layerSizesV[] = { stateDimension, nHiddenV, 1 } ;

        for ( int a = 0 ; a < numberOfActions ; a++ ) {
            QNN.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ;
        }
        VNN = new cNeuralNetwork( 1, layerSizesV ) ;


        VTarget = new double[ 1 ] ;
        QTarget = new double[ 1 ] ;

        Qs = new double[ numberOfActions ] ;

    }

    policy = new double[ numberOfActions ] ;

}

QVlearning::~QVlearning() {

    if ( discreteStates ) {

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            delete [] Q[s] ;

        }

        delete [] Q ;
        delete [] V ;

    } else {

        for ( int a = 0 ; a < numberOfActions ; a++ ) {
            delete QNN[a] ;
        }
        QNN.clear() ;
        delete VNN ;

        delete [] QTarget ;
        delete [] VTarget ;

        delete [] Qs ;

    }

    delete [] policy ;

}


void QVlearning::readParameterFile( const char * parameterFile ) {

    if ( not discreteStates ) {

        ifstream ifile ;

        ifile.open( parameterFile, ifstream::in ) ;

        read_moveTo( &ifile, "nn" ) ;

        read_moveTo( &ifile, "nHiddenQ" ) ;
        ifile >> nHiddenQ ;
        read_moveTo( &ifile, "nHiddenV" ) ;
        ifile >> nHiddenV ;

        ifile.close() ;

    }

}

void QVlearning::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma  ) {

    int at = action->discreteAction ;

    if ( state->discrete ) {

        int st = state->discreteState ;
        int st_ = nextState->discreteState ;

        if ( endOfEpisode ) {

            Q[ st ][ at ] += learningRate[0]*( rt - Q[ st ][ at ] ) ;
            V[ st ]       += learningRate[1]*( rt - V[ st ]       ) ;

        } else {

            Q[ st ][ at ] += learningRate[0]*( rt + gamma*V[ st_ ] - Q[ st ][ at ] ) ;
            V[ st ]       += learningRate[1]*( rt + gamma*V[ st_ ] - V[ st ]       ) ;

        }

    } else {

        double * st = state->continuousState ;
        double * st_ = nextState->continuousState ;

        if ( endOfEpisode ) {

            QTarget[ 0 ] = rt ;
            VTarget[ 0 ] = rt ;

        } else {

            double Vs_   = VNN->forwardPropagate( st_ )[0] ;

            QTarget[ 0 ] = rt + gamma*Vs_ ;
            VTarget[ 0 ] = rt + gamma*Vs_ ;

        }

        QNN[ at ]->backPropagate( st, QTarget, learningRate[0] ) ;
        VNN->backPropagate( st, VTarget, learningRate[1] ) ;

    }

}

unsigned int QVlearning::getNumberOfLearningRates() {

    return 2 ;

}

const char * QVlearning::getName() {

    return "QVlearning" ;

}

#endif //QVLEARNING
