from setuptools import setup
import os

package_name = 'measure_process_ros2_pkg'

data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['resource/', 'launch/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiko',
    maintainer_email='fma527@ual.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'measure_process = measure_process_ros2_pkg.measure_process:main',
            'record_cpu_usage = measure_process_ros2_pkg.record_cpu_usage:main'
        ],
    },
)
