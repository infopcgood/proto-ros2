"""Explicit Prootface pipeline; legacy launch.sh remains available."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults=dict(character='',profile='~/.local/share/prootface/calibration.json',
                  style='',side_width='256',side_height='144',presentation_fps='24',
                  backend='opencv',panel_map='',driver_strips='8',brightness='3',
                  web_host='127.0.0.1',web_port='8080',egl_platform='auto',
                  start_receiver='true',udp_host='127.0.0.1',udp_port='11573')
    arguments=[DeclareLaunchArgument(k,default_value=v,**({'choices':['opencv','ws','none']} if k=='backend' else {})) for k,v in defaults.items()]
    value=lambda name:LaunchConfiguration(name)
    integer=lambda name:ParameterValue(value(name),value_type=int)
    text=lambda name:ParameterValue(value(name),value_type=str)
    backend=lambda name:IfCondition(PythonExpression(["'",value('backend'),"' == '",name,"'"]))
    nodes=[
        Node(package='param_server_pkg',executable='param_server',parameters=[{
            'DISPLAY_WIDTH':ParameterValue(PythonExpression(['2 * ',value('side_width')]),value_type=int),
            'DISPLAY_HEIGHT':integer('side_height')}]),
        Node(package='screen_dispatcher_pkg',executable='screen_dispatcher'),
        Node(package='osf_receiver_pkg',executable='osf_receiver',condition=IfCondition(value('start_receiver')),
             parameters=[{'udp_host':text('udp_host'),'udp_port':integer('udp_port')}]),
        Node(package='prootface_pkg',executable='prootface',output='screen',parameters=[{
            'character_path':text('character'),'profile_path':text('profile'),'style_path':text('style'),
            'side_width':integer('side_width'),'side_height':integer('side_height'),
            'presentation_fps':integer('presentation_fps'),'egl_platform':text('egl_platform'),
            'web_host':text('web_host'),'web_port':integer('web_port')}]),
        Node(package='opencv2_display_pkg',executable='opencv2_display',condition=backend('opencv'),parameters=[{'scale':2}]),
        Node(package='proto_panel_display_pkg',executable='atlas_display',condition=backend('ws'),
             output='screen',parameters=[{'panel_map':text('panel_map'),'driver_strips':integer('driver_strips'),
                 'brightness':integer('brightness'),'expected_side_width':integer('side_width'),
                 'expected_side_height':integer('side_height')}]),
    ]
    return LaunchDescription(arguments+nodes)
