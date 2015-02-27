# ifndef SMALLINTERFACE
# define SMALLINTERFACE

# include "SmallInterface.h"
# include "SmallMaze.h"

SmallInterface::SmallInterface( const char * pFile ) {

    parameterFile = pFile ;

    parameterPath.assign( parameterFile ) ;

    readParameterFile() ;

}

int main( int argn, char *argv[] ) {

     if ( argn != 2 ) {

        cout << "You need to specify exactly one configuration (parameter) file." << endl ;
        return 0 ;

    }

    SmallInterface * interface = new SmallInterface( argv[1] ) ;

    SmallMaze * maze = new SmallMaze() ;

    interface->getParametersAndRunExperiments( maze ) ;

    delete interface ;

    delete maze ;

}

# endif //SMALLINTERFACE



