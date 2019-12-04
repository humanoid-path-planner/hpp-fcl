brew update

# Add gepetto tap
brew tap gepetto/homebrew-gepetto

# install eigenpy

brew install python@2
brew install boost assimp eigen octomap 

brew uninstall numpy
pip uninstall numpy -y
brew install eigenpy
brew link --overwrite numpy@1.16
