git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
mkdir build && cd build
cmake .. -DPAHO_WITH_SSL=ON
make
sudo make install
sudo ldconfig
# 安装paho.mqtt.cpp
cd ..
cd ..
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
