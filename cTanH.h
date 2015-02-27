#ifndef CTANH_H
#define CTANH_H

#include "cFunction.h"
using namespace std;
class cTanH : public cFunction {
    public:
        cTanH() ;
        ~cTanH() ;
        int printName() ;
        double output( double input );
        void output( double * input, double * output, int n );
        double derivative( double input, double output );
        void derivative( double * input, double * output, double * derivative, int n );
};

#endif //CTANH_H
