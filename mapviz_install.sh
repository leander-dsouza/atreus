 #!/usr/bin/env bash

sudo curl -sSL https://get.docker.com/ | sh 

sudo usermod -aG docker $USER

sudo apt-get update && sudo apt-get upgrade 

mkdir ~/mapproxy

sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

ATREUS_PATH=$(rospack find atreus)

cd $ATREUS_PATH && cd config/mapviz

sed -i "s|/home/leander/Gazebo/rhinoceROS/src/atreus|${ATREUS_PATH}|g" *.config