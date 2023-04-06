from setuptools import setup

package_name = 'pal_python'

setup(
    name=package_name,
    version='3.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=['Jordan Palacios'],
    maintainer_email=['jordan.palacios@pal-robotics.com'],
    description='PAL python utils',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
    },
)
