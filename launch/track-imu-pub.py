# launch>
#  <arg name="razor_config_file" default="$(find razor_imu_9dof)/config/my_razor.yaml"/>
#  <node pkg="razor_imu_9dof" type="imu_node.py" name="imu_node" output="screen">
#    <rosparam file="$(arg razor_config_file)" command="load"/>
#  </node>
# </launch>
