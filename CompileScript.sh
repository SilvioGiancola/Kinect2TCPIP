mkdir -p ~/Kinect2TCPIP/build
cd ~/Kinect2TCPIP/build
cmake ../src -DCMAKE_BUILD_TYPE=Release -DSERVER=YES -DCLIENT=YES -DQT_VERSION=4
make -j4
echo "COMPILATION DONE"
sudo reboot

