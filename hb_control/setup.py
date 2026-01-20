from setuptools import setup, find_packages
import os

package_name = 'hb_control'

setup(
    name=package_name,
    version='0.0.0',

    # Discover Python modules inside src/hb_control
    packages=find_packages(where='src'),
    package_dir={'': 'src'},

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Holonomic perception, control, MQTT, and camera nodes',
    license='TODO',

    # REQUIRED for ros2 run
    data_files=[
        # ament index marker
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),

        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'),
    	 ['config/camera_testing.yaml']),
 ],

    entry_points={
        'console_scripts': [
            'holonomic_perception = hb_control.holonomic_perception:main',
            'holonomic_controller = hb_control.holonomic_controller:main',
            'mqtt_node = hb_control.mqtt:main',
            'camera_testing = hb_control.camera_testing:main',
            'holonomic0_controller = hb_control.holonomic0_controller:main',
            'holonomic2_controller= hb_control.holonomic2_controller:main',
            'holonomic4_controller = hb_control.holonomic4_controller:main',
            'central_mqtt = hb_control.central_mqtt:main',
            'grid_planner = hb_control.grid_planner:main',
        ],
    },
)
