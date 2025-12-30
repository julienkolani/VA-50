from setuptools import setup, find_packages

package_name = 'turtlebot_game'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/safety_bridge.launch.py',
        ]),
        
        # Config files
        ('share/' + package_name + '/config', [
            'config/safety_config.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'websockets>=12.0',  
        'numpy',
    ],
    zip_safe=True,
    maintainer='Projet VA50 Promo 2025 Automne',
    maintainer_email='projetva50@utbm.edu',
    description='Modules de jeu TurtleBot + Safety Bridge WebSocket pour VA50 UTBM',
    license='MIT',
    
    entry_points={
        'console_scripts': [
            'safety_bridge = turtlebot_game.safety_bridge:main',
        ],
    },
)
