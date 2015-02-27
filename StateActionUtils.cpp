#ifndef STATEACTIONUTILS
#define STATEACTIONUTILS

# include "StateActionUtils.h"


void copyAction( Action * aFROM, Action * aTO ) {

    if ( aFROM->discrete and aTO->discrete ) {

        aTO->discreteAction = aFROM->discreteAction ;

    }

    if ( aFROM->continuous and aTO->continuous and ( aFROM->actionDimension == aTO->actionDimension ) ) {

        for ( int a = 0 ; a < aFROM->actionDimension ; a++ ) {

            aTO->continuousAction[a] = aFROM->continuousAction[a] ;

        }

    }

}

void copyState( State * sFROM, State * sTO ) {

    if ( sFROM->discrete and sTO->discrete ) {

        sTO->discreteState = sFROM->discreteState ;

    }

    if ( sFROM->continuous and sTO->continuous and ( sFROM->stateDimension == sTO->stateDimension ) ) {

        for ( int s = 0 ; s < sFROM->stateDimension ; s++ ) {

            sTO->continuousState[s] = sFROM->continuousState[s] ;

        }

    }

}

#endif //STATEACTIONUTILS
