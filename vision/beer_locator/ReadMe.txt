1. roscore
2. rosrun kinect2_bridge kinect2_bridge 
3. rosrun beer_locator beer_locator image  --marker pivo.jpg

Notes: Build with OpenCV 2.4.10
ak nejde zbildit beer_locator treba prebuildit cele OpenCV:
cd ~/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_OPENCL=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make
sudo make install
