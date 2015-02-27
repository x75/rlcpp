#ifndef CTHRESHOLD_H
#define CTHRESHOLD_H

#include "cFunction.h"
using namespace std;

class cThreshold : public cFunction {
    public:
        cThreshold() ;
        ~cThreshold() ;
        int printName() ;
        double output( double input );
        void output( double * input, double * output, int n );
        double derivative( double input, double output );
};

#endif //CTHRESHOLD_H
