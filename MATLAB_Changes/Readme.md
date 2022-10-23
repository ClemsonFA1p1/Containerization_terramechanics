
Adding Network to Docker :

sudo docker network ls
sudo docker network create --driver bridge common2 >> o/p d91714e820d6458c09637f9750979741f48bfa13a36755f4c47a02f259c8ea71

sudo docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --network common2 sanskrj/ubuntu:projchrono_v8

cd /opt/matlab_2022b/bin 
$ ./activate_matlab.sh
Add license 

This contains matlab changes specific to linux os 

Add Export paths in given evironment for instantiating matlab in any script, and change CMAKEList.txt for that particular script

`export LD_LIBRARY_PATH=<matlabroot>/bin/glnxa64:<matlabroot>/sys/os/glnxa64:$LD_LIBRARY_PATH`

`export PATH=<matlabroot>/bin:$PATH`

export LD_LIBRARY_PATH=/opt/matlab_2022a/bin/glnxa64:/opt/matlab_2022a/sys/os/glnxa64:$LD_LIBRARY_PATH
export PATH=/opt/matlab_2022a/bin:$PATH



