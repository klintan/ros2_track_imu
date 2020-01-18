# <launch>
#  <arg name="razor_config_file" default="$(find razor_imu_9dof)/config/my_razor.yaml"/>
#  <node pkg="razor_imu_9dof" type="imu_node.py" name="imu_node" output="screen">
#    <rosparam file="$(arg razor_config_file)" command="load"/>
#  </node>
#  <node pkg="razor_imu_9dof" type="display_3D_visualization.py" name="display_3D_visualization_node" output="screen">
#  </node>
# </launch>
