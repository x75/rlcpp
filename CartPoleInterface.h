# ifndef CARTPOLEINTERFACE_H
# define CARTPOLEINTERFACE_H

# include "Experiment.h"
# include "CartPole.h"

class CartPoleInterface : public Experiment {
    public:
        CartPoleInterface( const char * parameterFile ) ;
        ~CartPoleInterface() {};

};

# endif //CARTPOLEINTERFACE_H
