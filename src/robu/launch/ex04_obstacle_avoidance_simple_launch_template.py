import launch
import launch_ros.actions
from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():
    return launch.LaunchDescription([
        #Set some environment variables
        launch.actions.SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='0'),
        launch.actions.SetEnvironmentVariable(name='LDS_MODEL', value='LDS-02'),
        launch.actions.SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        #Start the remote node
        ExecuteProcess(
            name='bringup',
            cmd=['ssh -t pi@robpi "bash -i -c \'systemctl start --user remote_ros_robot_launch.service\'"'],
            output="screen",
            shell=True,
            emulate_tty=True
        ),
        #Start the remote control node
        launch_ros.actions.Node(
            package='robu',
            executable='remotectrl',
            name='remotectrl'
            )
  ])
