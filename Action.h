#ifndef ACTION
#define ACTION

struct Action {
    bool continuous ;
    bool discrete ;

    int actionDimension ;
    int numberOfActions ;

    int discreteAction ;
    double * continuousAction ;
};

#endif //Action
