user_name=$USER
host_name=$HOSTNAME

sed -i -e 's/jakob/$user_name/g' /home/$user_name/workspace/catkin/src/finder/launch/machines_test.launch
sed -i -e 's/jakob-think/$host_name/g' /home/$user_name/workspace/catkin/src/finder/launch/machines_test.launch

sed -i -e 's/jakob/$user_name/g' /home/$user_name/workspace/rosbuild/.rosinstall
sed -i -e 's/jakob-think/$host_name/g' /home/$user_name/workspace/rosbuild/.rosinstall

sed -i -e 's/jakob/$user_name/g' /home/$user_name/workspace/rosbuild/.setup.bash
sed -i -e 's/jakob-think/$host_name/g' /home/$user_name/workspace/rosbuild/.setup.bash

sed -i -e 's/jakob/$user_name/g' /home/$user_name/workspace/rosbuild/.setup.sh
sed -i -e 's/jakob-think/$host_name/g' /home/$user_name/workspace/rosbuild/.setup.sh


