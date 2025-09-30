from setuptools import setup

package_name = 'emotion_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='EmotionBot with GUI, DeepFace, TurtleBot3 Gazebo Integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotion_bot_node = emotion_bot.emotion_bot_node:main',
        ],
    },
)
