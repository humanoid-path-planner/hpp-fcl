brew update

# Add gepetto tap
brew tap gepetto/homebrew-gepetto

# install eigenpy
brew install boost assimp eigen octomap eigenpy

# set environment variables
export PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:/opt/openrobots/lib/pkgconfig"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/openrobots/lib"
