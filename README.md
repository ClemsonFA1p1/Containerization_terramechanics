# ProjectChrono_ARMLAB


Using the image below, one can run project chrono, pychrono deep reinforcement learning

Docker Image : https://hub.docker.com/repository/docker/sanskrj/projchrono

docker pull sanskrj/projchrono:v10


To run this repository following settings must be completed :
1. The build directory of chrono needs to include MATLAB and MATLAB SDK root :

 When you see the CMake window, 
 Set the ENABLE_MODULE_MATLAB as 'on', then press 'Configure' (to refresh the variable list)
 
 Set the CH_MATLAB_SDK to the path where you have your Matlab '/extern' subdirectory.
 
 This changes depending on where you installed Matlab. 
 
 For example, it could be C:/Program Files/MATLAB/R2015b/extern
 
 Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.
 
 2. Add the MATLAB environment paths in the terminal that you are working, 
 
 <matlabroot> is install directory of your matlab, it can be obtained by typing matlabroot in command window of application.
    
`export LD_LIBRARY_PATH=<matlabroot>/bin/glnxa64:<matlabroot>/sys/os/glnxa64:$LD_LIBRARY_PATH`

`export PATH=<matlabroot>/bin:$PATH`

3. Update the Chrono_matlab engine codes for -automation command (its specific to windows OS)
  
  
Place the demo_VEH_SteeringController.cpp and CMakelist in chrono/src/demos/vehicle directory 

