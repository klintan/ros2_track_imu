# <launch>
#    <arg name="razor_config_file" default="$(find razor_imu_9dof)/config/my_razor.yaml"/>
#    <node pkg="razor_imu_9dof" type="imu_node.py" name="imu_node" output="screen">
#        <rosparam file="$(arg razor_config_file)" command="load"/>
#    </node>
#
#    <node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostic_aggregator"  clear_params="true">
#        <rosparam command="load" file="$(find razor_imu_9dof)/config/razor_diags.yaml" />
#    </node>
#
#    <node pkg="rqt_robot_monitor" type="rqt_robot_monitor" name="rqt_robot_monitor" />
# </launch>
