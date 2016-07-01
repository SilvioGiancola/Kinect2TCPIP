mkdir -p ~/Kinect2TCPIP/build-Server
cd ~/Kinect2TCPIP/build-Server
cmake ../src -DCMAKE_BUILD_TYPE=Release
make -j4 Kinect2Server
echo "COMPILATION DONE"
#sudo reboot

