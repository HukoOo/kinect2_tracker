# Kinect2 Skeleton tracker #
This is modified version of the https://github.com/mcgi5sr2/kinect2_tracker for V-rep mapping 

### kinect2_tracker_node
ROS wrapper for the skeleton tracker with OpenNI2 and NiTE2 

### vrep_publisher_node
Vrep wrapper for joint mapping with RosInterface



## Step 0: Install Updates
---------------
```
sudo apt-get install cmake git libusb-dev freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev libgtk2.0-dev python-dev python-numpy libavcodec-dev libavformat-dev libswscale-dev 
```

## Step 1: libFreeNect, the device driver for Kinect
---------------
- Clone libfreenect:
```
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
```

- Build and Install
```
mkdir build
cd build
cmake .. -DBUILD_OPENNI2_DRIVER=ON
make
sudo make install
```

- Add to library path:
```
sudo vi /etc/ld.so.conf.d/custom.conf
```
```
/usr/local/lib64/ 
/usr/local/lib64/OpenNI2-FreenectDriver/
```

## Step 2: Install OpenNI and NiTE
---------------
- Download OpenNI from:
http://openni.ru/openni-sdk/index.html

- Uncompress OpenNI and install:
```
unzip OpenNI-Linux-x64-2.2.0.33.tar.zip
tar xvf OpenNI-Linux-x64-2.2.tar.bz2
mv OpenNI-Linux-x64-2.2 ~/Developer/Work/
cd ~/Developer/Work/OpenNI-Linux-x64-2.2
sudo ./install.sh
```

- Download NiTE from:
https://sourceforge.net/projects/roboticslab/files/External/nite/NiTE-Linux-x64-2.2.tar.bz2

- Uncompress NiTE and install:
```
tar xvf NiTE-Linux-x64-2.2.tar.bz2
cd NiTE-Linux-x64-2.2
sudo ./install.sh
```

- Copy libFreenectDriver.so to OpenNI2 directory:
```
cp /usr/local/lib64/OpenNI2-FreenectDriver/libFreenectDriver.so \
OpenNI-Linux-x64-2.2/Redist/OpenNI2/Drivers/
```

- Copy all libraries to `/usr/local/lib` 
```
sudo cp -R OpenNI-Linux-x64-2.2/Redist/* /usr/local/lib
sudo cp -R NiTE-Linux-x64-2.2/Redist/* /usr/local/lib
```

- Add to library path:
```
sudo vi /etc/ld.so.conf.d/custom.conf
```
```
/usr/local/lib64/ 
/usr/local/lib64/OpenNI2-FreenectDriver/
# Added
/usr/local/lib
```
```
sudo ldconfig 
```

- Add OpenNI and NiTE to .bashrc:
```
cat OpenNI-Linux-x64-2.2/OpenNIDevEnvironment >> ~/.bashrc
cat NiTE-Linux-x64-2.2/NiTEDevEnvironment >> ~/.bashrc
```


## Step 3: Clone ROS package and Install
---------------
clone this repository and modify the path(NITE2_DIR and OPENNI2_DIR) in CMakeLists.txt.

then simply ```catkin_make ```


To run the program copy the NiTE to .ros path and launch the file
```
./setup_nite.bash
roslaunch kinect2_tracker tracker.launch
```
## Step 4: Run the V-rep simulation
---------------
run the V-rep and load test environment

the path of ```.ttt``` file is ```/scene/test.ttt```

![alt text](img/result.png?)