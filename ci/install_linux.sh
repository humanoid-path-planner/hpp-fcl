sudo apt-get -qq update

########################
# Mendatory dependencies
########################
sudo apt-get -qq --yes --force-yes install cmake
sudo apt-get -qq --yes --force-yes install libboost-all-dev
sudo apt-get -qq --yes --force-yes install libassimp-dev
sudo apt-get -qq --yes --force-yes install libeigen3-dev

# Octomap
git clone https://github.com/OctoMap/octomap
cd octomap
git checkout tags/v1.6.8
mkdir build
cd build
cmake ..
make
sudo make install

