# ifndef CARTPOLEINTERFACE
# define CARTPOLEINTERFACE

# include "CartPoleInterface.h"
# include "CartPole.h"

CartPoleInterface::CartPoleInterface( const char * pFile ) {

    parameterFile = pFile ;

    parameterPath.assign( parameterFile ) ;

    readParameterFile() ;

}

int main( int argn, char *argv[] ) {

    if ( argn != 2 ) {

        cout << "You need to specify exactly one configuration (parameter) file." << endl ;
        return 0 ;

    }

    CartPoleInterface * interface = new CartPoleInterface( argv[1] ) ;

    CartPole * cartpole = new CartPole() ;

    interface->getParametersAndRunExperiments( cartpole ) ;

    delete cartpole ;
    delete interface ;
}

# endif //CARTPOLEINTERFACE



