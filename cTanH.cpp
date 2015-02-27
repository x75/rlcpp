#ifndef CTANH
#define CTANH

using namespace std;
//# include <math.h>
# include "cTanH.h"

cTanH::cTanH() : cFunction() {
    //~ cout << "constructor TanH\n" ;
}

cTanH::~cTanH() {
    //~ cout << "destructor TanH\n" ;
}

int cTanH::printName() {
    cout << "cTanH\n" ;
    return 0;
}

double cTanH::output( double input ) {
    //~ //cout << "tanh in;" ;
    double output = -1 ;
    if (input > 1.92032) {
        output = 0.96016;
    } else if (input > 0.0) {
        output = -0.260373271*input*input + input;
    } else if (input > -1.92032) {
        output = 0.260373271*input*input + input;
    } else {
        output = -0.96016;
    }
    return output;
    //~ return tanh( input ) ;
}

void cTanH::output( double * input , double * output , int n ) {
    
    for ( int i = 0 ; i < n ; i++ ) {
        output[i] = -1 ;
        if (input[i] > 1.92032) {
            output[i] = 0.96016;
        } else if (input[i] > 0.0) {
            output[i] = -0.260373271*input[i]*input[i] + input[i];
        } else if (input[i] > -1.92032) {
            output[i] = 0.260373271*input[i]*input[i] + input[i];
        } else {
            output[i] = -0.96016;
        }
    }
}

double cTanH::derivative( double input, double output ) {
    return 1.0 - (output*output) ;
}

void cTanH::derivative( double * input, double * output, double * derivative, int n ) {
    for ( int i = 0 ; i < n ; i++ ) {
        derivative[i] = 1.0 - (output[i]*output[i]) ;
    }
}

#endif //CTANH
