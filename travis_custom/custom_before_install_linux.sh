# Add robotpkg
sudo sh -c "echo \"deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg\" >> /etc/apt/sources.list"
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update

# install eigenpy
sudo apt-get -qqy install robotpkg-py27-eigenpy

# set environment variables
export PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:/opt/openrobots/lib/pkgconfig"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/openrobots/lib"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/opt/openrobots"
