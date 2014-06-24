user_name=$USER
host_name=$HOSTNAME
ouser_name='jakob'
ohost_name='jakob-think'

sed -i -e 's/$ohost_name/$host_name/g' /home/$user_name/workspace/catkin/src/finder/launch/machines_test.launch
sed -i -e 's/$ouser_name/$user_name/g' /home/$user_name/workspace/catkin/src/finder/launch/machines_test.launch

sed -i -e 's/$ohost_name/$host_name/g' /home/$user_name/workspace/rosbuild/.rosinstall
sed -i -e 's/$ouser_name/$user_name/g' /home/$user_name/workspace/rosbuild/.rosinstall

sed -i -e 's/$ohost_name/$host_name/g' /home/$user_name/workspace/rosbuild/setup.bash
sed -i -e 's/$ouser_name/$user_name/g' /home/$user_name/workspace/rosbuild/setup.bash

sed -i -e 's/$ohost_name/$host_name/g' /home/$user_name/workspace/rosbuild/setup.sh
sed -i -e 's/$ouser_name/$user_name/g' /home/$user_name/workspace/rosbuild/setup.sh


