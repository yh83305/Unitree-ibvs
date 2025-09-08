```
sudo apt install -y libboost-dev libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
sudo ln -s /usr/include/eigen3/unsupported /usr/local/include/unsupported
```

```
# Install pybind11
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build && cd build
cmake .. -DPYBIND11_TEST=OFF
make -j
sudo make install
```

```
cd ~/unitree_ws                                             
catkin_make                                                
echo "source ~/unitree_ws/devel/setup.bash">>~/.bashrc     
source ~/.bashrc                                           
roslaunch unitree_gazebo z1.launch UnitreeGripperYN:=false
```

```
cd z1_controller
mkdir build & cd build
cmake ..
make
./sim_ctrl
```

```
cd z1_sdk
mkdir build & cd build
cmake ..
make
cd ..
cd examples_py
python3 example_highcmd.py
```
