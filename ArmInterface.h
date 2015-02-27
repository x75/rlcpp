# ifndef ARMINTERFACE_H
# define ARMINTERFACE_H

# include "Experiment.h"
# include "Arm.h"

class ArmInterface : public Experiment {
    public:
        ArmInterface( const char * parameterFile ) ;
        ~ArmInterface() {};

};

# endif //ARMINTERFACE_H
