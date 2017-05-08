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
# ROS_DEFS = -DUSE_ROS
ROS_LDFLAGS = -L/opt/ros/jade/lib -lroscpp -lrosconsole -lroscpp_serialization -lroslib -lrostime
# ROS_LDFLAGS = -L/home/src/ros/hydro/catkin_ws/install_isolated/lib -lroscpp -lrosconsole -lroscpp_serialization -lroslib -lrostime
# ROS_LDFLAGS = -L/opt/ros/jade/lib -lroscpp_serialization -lrostime -l:/usr/lib/x86_64-linux-gnu/libboost_date_time.so -l:/usr/lib/x86_64-linux-gnu/libboost_system.so -l:/usr/lib/x86_64-linux-gnu/libboost_thread.so -l:/usr/lib/x86_64-linux-gnu/libpthread.so -lcpp_common -l:/usr/lib/x86_64-linux-gnu/libboost_system.so -l:/usr/lib/x86_64-linux-gnu/libboost_thread.so -l:/usr/lib/x86_64-linux-gnu/libpthread.so -l:/usr/lib/x86_64-linux-gnu/libconsole_bridge.so 
# ROS_LDFLAGS = -L/opt/ros/jade/lib -lroscpp_serialization -lrostime -l:/usr/lib/x86_64-linux-gnu/libboost_date_time.so -l:/usr/lib/x86_64-linux-gnu/libboost_system.so -l:/usr/lib/x86_64-linux-gnu/libboost_thread.so -l:/usr/lib/x86_64-linux-gnu/libpthread.so -lcpp_common -l:/usr/lib/x86_64-linux-gnu/libboost_system.so -l:/usr/lib/x86_64-linux-gnu/libboost_thread.so -l:/usr/lib/x86_64-linux-gnu/libpthread.so -l:/usr/lib/x86_64-linux-gnu/libconsole_bridge.so 

# objects := $(patsubst %.cpp,%.o,$(wildcard *.cpp))
objects := $(patsubst %.cpp,%.o,$(UTILFILES))
objects += $(patsubst %.cpp,%.o,$(ALGORITHMS))

cp_objs = $(patsubst %.cpp,%.o,$(CPWORLD))
cp_objs += $(patsubst %.cpp,%.o,$(CPINTERFACE))

cp_objs_inc = $(patsubst %.cpp,%.h,$(CPWORLD))
cp_objs_inc += $(patsubst %.cpp,%.h,$(CPINTERFACE))

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

cp_objs: $(cp_objs_inc)
cp_ros: $(objects) $(cp_objs)
# c++ $(CFLAGS) $(UTILFILES) $(ALGORITHMS) $(CPWORLD) $(CPINTERFACE) -o CartPole
	g++ $(objects) $(cp_objs) -o CartPoleRos $(LDFLAGS) $(ROS_LDFLAGS)
	@echo
	@echo Done compiling CartPoleRos.
	@echo Run with:    ./CartPoleRos \'cfg\'
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

arm_objs: $(arm_objs_inc)

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
	rm -rf Arm CartPole CartPoleRos SmallMaze
	rm -rf *.o
	rm -f *.out
	@echo done removing

