from setuptools import setup

package_name = "mission_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="do",
    maintainer_email="do@example.com",
    description="ROS2 OpenCV mission package for line tracing drone simulation",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_subscriber = mission_pkg.camera_subscriber:main",
            "line_follow = mission_pkg.line_follow:main",
            "bev_line_analyzer = mission_pkg.bev_line_analyzer:main",
            "line_state_node = mission_pkg.line_state_node:main",
            'mavros_keyboard_move = mission_pkg.mavros_keyboard_move:main',
        ],
    },
)