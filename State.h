#ifndef STATE
#define STATE

struct State {
    bool continuous ;
    bool discrete ;

    int stateDimension ;
    int numberOfStates ;

    int discreteState ;
    double * continuousState ;
};

#endif //STATE
