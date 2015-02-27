#ifndef CTHRESHOLD
#define CTHRESHOLD

#include "cThreshold.h"
using namespace std;

cThreshold::cThreshold() : cFunction() {
    //~ cout << "constructor Threshold\n" ;
}

cThreshold::~cThreshold() {
    //~ cout << "destructor Threshold\n" ;
}

int cThreshold::printName() {
    cout << "cThreshold\n" ;
    return 0 ;
}

double cThreshold::output( double input ) {
    if ( input > 0 ) {
        return 1.0 ;
    } else {
        return -1.0 ;
    }
}

void cThreshold::output( double * inp, double * outp, int n) {
    for ( int i = 0 ; i < n ; i++ ) {
        if ( inp[i] > 0 ) {
            outp[i] = 1.0 ;
        } else {
            outp[i] = -1.0 ;
        }
    }
}

double cThreshold::derivative( double input, double output ) {
    return 1.0 ;
}


#endif //CTHRESHOLD
