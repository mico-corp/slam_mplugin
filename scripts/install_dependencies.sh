#!/bin/sh

install_git_repo () {
	read -r -p "Do you want to install $1 [y/N] " response
	if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
	then
		if [ -d "./$1" ] 
		then
			echo "Library $1 already installed" 
		else
			git clone $2
			cd $1
			if [ -z "$3" ]   # Is parameter #1 zero length?
			then
				git checkout "$3"
			fi
			
			mkdir build ; cd build
			cmake ..
			make -j$(nproc)
			sudo make install 
			cd ../..
		fi
	fi
}


sudo apt-get install -y libpcl-dev libopencv-dev 
sudo apt-get install -y liblapack-dev libblas-dev libopenblas-dev libeigen3-dev
sudo apt-get install -y libomp-dev libsuitesparse-dev libcholmod3

mkdir ~/.mico/ -p
cd ~/.mico/
mkdir thirdparty_plugins/slam -p
cd thirdparty_plugins/slam

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2.git"
install_git_repo "DLoopDetector" "https://github.com/dorian3d/DLoopDetector.git"
install_git_repo "g2o" "https://github.com/RainerKuemmerle/g2o.git"

# Install g2o disabling native march
# read -r -p "Do you want to install latest version of G2O [y/N] " response
# if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
# then
#     if [ -d "g2o" ] 
# 	then
# 		echo "Library $1 already installed" 
# 	else
#         git clone "https://github.com/RainerKuemmerle/g2o.git"
#         cd g2o
#         mkdir build; cd build
#         cmake -DBUILD_WITH_MARCH_NATIVE=OFF .. 
# 		make -j$(nproc)
# 		sudo make install 
#         cd ../..
#     fi
# fi;