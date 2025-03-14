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
cd ~/unitree_ws                                             #打开该文件夹
catkin_make                                                 #初始化ROS工作空间
echo "source ~/unitree_ws/devel/setup.bash">>~/.bashrc     #将ros路径添加到环境变量，可由pwd命令获取当前路径替换该路径
source ~/.bashrc                                            #更新环境变量
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