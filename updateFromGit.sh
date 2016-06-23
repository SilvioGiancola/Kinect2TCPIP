cd ~/git/Kinect2TCPIP
git stash
git pull
cd build
cmake ..
make -j4
sudo reboot
