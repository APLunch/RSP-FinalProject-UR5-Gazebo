STEP 1: Install Dependencies (Or you can run the install.sh) 

sudo apt update
sudo apt upgrade -y
sudo apt install -y python3
sudo apt install -y python3-pip
sudo pip install pysocks
sudo pip install requests
sudo pip install openai 


STEP 2: Build

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install


STEP 3 Source & Configure

You can change the model, api key and system prompt.


STEP 4: Run the Server & Client

ros2 run gpt_ros gpt_ros2_server
ros2 run gpt_ros gpt_ros2_client


STEP 5: Run the Bridge node

ros2 run gpt_bridge gpt_bridge_node


