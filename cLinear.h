#ifndef CLINEAR_H
#define CLINEAR_H

#include "cFunction.h"
using namespace std;

class cLinear : public cFunction {
    public:
        cLinear() ;
        ~cLinear() ;
        int printName() ;
        double output( double input );
        void output( double * input, double * output, int n );
        double derivative( double input, double output );
        void derivative( double * input, double * output, double * derivative, int n );
};

#endif //CLINEAR_H
