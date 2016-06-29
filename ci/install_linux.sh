sudo add-apt-repository --yes ppa:libccd-debs/ppa
sudo apt-get -qq update

########################
# Mendatory dependencies
########################
sudo apt-get -qq --yes --force-yes install cmake
sudo apt-get -qq --yes --force-yes install libboost-all-dev
sudo apt-get -qq --yes --force-yes install libccd-dev

# Assimp
sudo echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub trusty robotpkg" >> /etc/apt/sources.list.d/robotpkg.list
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update
sudo apt-get install robotpkg-assimp

########################
# Optional dependencies
########################
sudo apt-get -qq --yes --force-yes install libflann-dev

# Octomap
git clone https://github.com/OctoMap/octomap
cd octomap
git checkout tags/v1.6.8
mkdir build
cd build
cmake ..
make
sudo make install

