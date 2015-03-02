CFLAGS=-Wall -O4

# World.h
UTILFILES = StateActionUtils.cpp cNeuralNetwork.cpp Algorithm.cpp StateActionAlgorithm.cpp Experiment.cpp
ALGORITHMS = Sarsa.cpp Qlearning.cpp QVlearning.cpp Acla.cpp Cacla.cpp
CPWORLD = CartPole.cpp
CPINTERFACE = CartPoleInterface.cpp
SMALLWORLD = SmallMaze.cpp
SMALLINTERFACE = SmallInterface.cpp
ARMWORLD = Arm.cpp
ARMINTERFACE = ArmInterface.cpp

LDFLAGS = -L/usr/lib -L/usr/lib/gcc/x86_64-linux-gnu/4.8 -lstdc++
# ROS_LDFLAGS = -L/opt/ros/indigo/lib -lroscpp -lrosconsole -lroscpp_serialization -lroslib -lrostime
ROS_LDFLAGS = -L/home/src/ros/hydro/catkin_ws/install_isolated/lib -lroscpp -lrosconsole -lroscpp_serialization -lroslib -lrostime


# objects := $(patsubst %.cpp,%.o,$(wildcard *.cpp))
objects := $(patsubst %.cpp,%.o,$(UTILFILES))
objects += $(patsubst %.cpp,%.o,$(ALGORITHMS))

arm_objs = $(patsubst %.cpp,%.o,$(ARMWORLD))
arm_objs += $(patsubst %.cpp,%.o,$(ARMINTERFACE))

arm_objs_inc = $(patsubst %.cpp,%.h,$(ARMWORLD))
arm_objs_inc += $(patsubst %.cpp,%.h,$(ARMINTERFACE))

all: cp small arm

#		cp
cp:
	c++ $(CFLAGS) $(UTILFILES) $(ALGORITHMS) $(CPWORLD) $(CPINTERFACE) -o CartPole
	@echo
	@echo Done compiling CartPole.
	@echo Run with:    ./CartPole \'cfg\'
	@echo where \'cfg\' is a configuration file.
	@echo

#		small
small:
	c++ $(CFLAGS) $(UTILFILES) $(ALGORITHMS) $(SMALLWORLD) $(SMALLINTERFACE) -o SmallMaze
	@echo
	@echo Done compiling SmallMaze.
	@echo Run with:    ./SmallMaze \'cfg\'
	@echo where \'cfg\' is a configuration file.
	@echo

arm_objs: arm_objs_inc

arm: $(objects) $(arm_objs)
# c++ $(CFLAGS) $(ARM_LDFLAGS) $(UTILFILES) $(ALGORITHMS) $(ARMWORLD) $(ARMINTERFACE) -o Arm
	g++ $(objects) $(arm_objs) -o Arm $(LDFLAGS) $(ROS_LDFLAGS) 
	@echo
	@echo Done compiling Arm.
	@echo Run with:    ./Arm \'cfg\'
	@echo where \'cfg\' is a configuration file.
	@echo

# foo : $(objects)
# 	ld -o foo $(ARM_LDFLAGS) $(objects)

clean:
	rm -rf *.o
	rm -f *.out
	@echo done removing

