from setuptools import setup

package_name = 'send_script'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics',
    maintainer_email='melosyhsieh44@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_script = send_script.send_script:main',
            'send_script_2 = send_script.send_script_2:main',
            'image_sub = send_script.image_sub:main',
            'tmfb = send_script.tmfb:main',
        ],
    },
)
