#ifndef CFUNCTION_H
#define CFUNCTION_H

#include <iostream>
using namespace std;


class cFunction {
    public:
        cFunction() {}
        virtual ~cFunction() {}
        virtual int printName() =0;
        virtual double output( double input ) =0;
        virtual void output( double * input, double * output, int n ) =0;
        virtual double derivative( double input, double output ) =0;
};

#endif //CFUNCTION_H
