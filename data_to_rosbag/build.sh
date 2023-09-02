echo "Configuring and building data-to-rosbag ..."
mkdir build
cd build 
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4