#ifndef CLINEAR
#define CLINEAR

#include "cLinear.h"
using namespace std;

cLinear::cLinear() : cFunction() {
    //~ cout << "constructor Linear\n" ;
}

cLinear::~cLinear() {
    //~ cout << "destructor Linear\n" ;
}

int cLinear::printName() {
    cout << "cLinear\n" ;
    return 0 ;
}

double cLinear::output( double input ) {
    return input ;
    
}

void cLinear::output( double * inp, double * outp, int n) {
    for ( int i = 0 ; i < n ; i++ ) {
        outp[i] = inp[i] ;
    }
}

double cLinear::derivative( double input, double output ) {
    return 1.0 ;
}

void derivative( double * input, double * output, double * derivative, int n ) {
    for ( int i = 0 ; i < n ; i++ ) {
        derivative[i] = 1.0 ;
    }
}


#endif //CLINEAR
