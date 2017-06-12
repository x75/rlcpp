Import of Hado van Hasselt's Reinforcement Learning Code, c++ version
rlcpp.tar.gz <sup><a id="fnr.1" class="footref" href="#fn.1">1</a></sup>. 

To build it do the following, assuming you have ROS Jade installed in
/opt/ros/jade:

    mkdir build
    cmake ..
    make

Should create CartPoleRos and ArmRos executables in the build directory.

-   Added cmake file to build CartPoleRos and ArmRos
-   ROS interface to the CACLA implementation so the RL algorithms can
    be used on a variety of ROS enabled systems.


# Footnotes

<sup><a id="fn.1" href="#fnr.1">1</a></sup> <http://web.archive.org/web/20131213231018/http://homepages.cwi.nl/~hasselt/code.html>
