# MuJoCo_ARBML

### MuJoCo Template Repository using ARBML library
* Articulated Rigid-Body Motion Library ( ARBML )

### Instructions
```console
// clone repo and submodule recursively
$ git clone --recursive https://github.com/HumanoidRoboticsLab/template_arbml.git
// use it, if you want to update the submodule to the latest commit
$ git submodule update --remote

$ mkdir model # or 
$ git clone https://github.com/HumanoidRoboticsLab/model.git

// Follow instructions explained in libraries dir README.md first
$ mkdir build && cd build
$ cmake ..
$ make -j16

$ ./bin/simulate or ./bin/template
```
