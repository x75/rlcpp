# ifndef ARMINTERFACE
# define ARMINTERFACE

# include "ArmInterface.h"
# include "Arm.h"

#include <ros/ros.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

ArmInterface::ArmInterface( const char * pFile ) {

  parameterFile = pFile ;

  parameterPath.assign( parameterFile ) ;

  readParameterFile() ;

}


// void my_handler1(sig_t s){
//   printf("Caught signal %d\n",s);
//   ros::shutdown();
//   exit(1); 
// }

void my_handler(int s){
  printf("Caught signal %d\n",s);
  ros::shutdown();
  exit(1); 
}

int main( int argn, char *argv[] ) {

  if ( argn != 2 ) {

    cout << "You need to specify exactly one configuration (parameter) file." << endl ;
    return 0 ;

  }

  // struct sigaction sigIntHandler;

  // sigIntHandler.sa_handler = my_handler;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;

  // sigaction(SIGINT, &sigIntHandler, NULL);
  signal (SIGINT,my_handler);

  ArmInterface * interface = new ArmInterface( argv[1] ) ;

  Arm * arm = new Arm() ;

  // while(ros::ok()) {
  interface->getParametersAndRunExperiments( arm ) ;
  // }

  delete arm ;
  delete interface ;
  pause();
}

# endif //ARMINTERFACE



