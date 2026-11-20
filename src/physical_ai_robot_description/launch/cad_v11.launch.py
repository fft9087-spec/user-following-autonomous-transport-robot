"""2WD + four castors CAD model with Gazebo + RViz; mode=view, slam, or nav."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('physical_ai_robot_description')
    defaults = dict(mode='view', gui='true', rviz='true', payload_mass='0.0',
        payload_x='0.0', payload_y='0.0', payload_height='0.20', computer_mass='0.0',
        caster_mu='0.6', wheel_mu='0.8', wheel_mu_lateral='0.8', wheel_torque='3.2', laser_z='0.3095', odometry_source='1',
        world=os.path.join(share,'worlds','corridor_speed_trial.world'),
        map=os.path.join(share,'maps','corridor_map.yaml'))
    def mode_is(value):
        return IfCondition(PythonExpression(["'",LaunchConfiguration('mode'),"' == '",value,"'"]))
    def inc(name,args,condition=None):
        kw={'condition':condition} if condition is not None else {}
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch',name)),
            launch_arguments=args.items(),**kw)
    sim_args={k:LaunchConfiguration(k) for k in defaults if k not in ['mode','map','rviz']}
    sim_args['use_sim_time']='true'
    # Explicit v11 model and separate provisional 2WD tuning.
    sim_args['model_xacro']=os.path.join(share,'urdf','cad_v11_robot.xacro')
    sim_args['params_file']=os.path.join(share,'config','nav2_mdh100.yaml')
    nodes=[DeclareLaunchArgument(k,default_value=v,**({'choices':['view','slam','nav']} if k=='mode' else {})) for k,v in defaults.items()]
    nodes += [inc('sim.launch.py',sim_args),
        inc('slam.launch.py',{'use_sim_time':'true'},mode_is('slam')),
        inc('nav.launch.py',{'map':LaunchConfiguration('map'),'use_sim_time':'true',
            'params_file':os.path.join(share,'config','nav2_mdh100.yaml')},mode_is('nav'))]
    for mode,config in [('view','cad_v11.rviz'),('slam','navigation.rviz'),('nav','navigation.rviz')]:
        condition=IfCondition(PythonExpression(["'",LaunchConfiguration('mode'),"' == '",mode,"' and '",LaunchConfiguration('rviz'),"' == 'true'"]))
        nodes.append(Node(package='rviz2',executable='rviz2',name='rviz2',arguments=['-d',os.path.join(share,'rviz',config)],
            parameters=[{'use_sim_time':True}],condition=condition,output='screen'))
    return LaunchDescription(nodes)
