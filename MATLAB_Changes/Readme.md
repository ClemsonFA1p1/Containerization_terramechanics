This contains matlab changes specific to linux os 

Add Export paths in given evironment for instantiating matlab in any script, and change CMAKEList.txt for that particular script

`export LD_LIBRARY_PATH=<matlabroot>/bin/glnxa64:<matlabroot>/sys/os/glnxa64:$LD_LIBRARY_PATH`

`export PATH=<matlabroot>/bin:$PATH`
