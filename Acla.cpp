#ifndef ACLA
#define ACLA

# include "Acla.h"


Acla::Acla( const char * parameterFile, World * w ) {

    discreteStates      = w->getDiscreteStates() ;
    stateDimension      = w->getStateDimension() ;

    if ( not w->getDiscreteActions() ) {

        cout << "Acla does not support continuous actions." << endl ;
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

Acla::~Acla() {

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


void Acla::readParameterFile( const char * parameterFile ) {

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

void Acla::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma  ) {

    int at = action->discreteAction ;

    if ( state->discrete ) {

        int st = state->discreteState ;
        int st_ = nextState->discreteState;

        double Vt = V[ st ] ;

        if ( endOfEpisode ) {

            V[ st ]       += learningRate[1]*( rt - V[ st ]       ) ;

        } else {

            V[ st ]       += learningRate[1]*( rt + gamma*V[ st_ ] - V[ st ]       ) ;

        }

        if ( V[ st ] > Vt ) {

            Q[ st ][ at ] += learningRate[0]*( 1.0 - Q[ st ][ at ] ) ;

        } else {

            Q[ st ][ at ] += learningRate[2]*( 0.0 - Q[ st ][ at ] ) ;

        }

    } else {

        double * st = state->continuousState ;
        double * st_ = nextState->continuousState ;

        if ( endOfEpisode ) {

            VTarget[ 0 ] = rt ;

        } else {

            double Vs_   = VNN->forwardPropagate( st_ )[0] ;

            VTarget[ 0 ] = rt + gamma*Vs_ ;

        }

        double Vt = VNN->forwardPropagate( st )[0] ;

        VNN->backPropagate( st, VTarget, learningRate[1] ) ;

        if ( VTarget[0] > Vt ) {

            QTarget[ 0 ] = 1.0 ;
            QNN[ at ]->backPropagate( st, QTarget, learningRate[0] ) ;

        } else {

            QTarget[ 0 ] = 0.0 ;
            QNN[ at ]->backPropagate( st, QTarget, learningRate[2] ) ;

        }
    }
}

unsigned int Acla::getNumberOfLearningRates() {

    return 3 ;

}

const char * Acla::getName() {

    return "Acla" ;

}

#endif //ACLA
