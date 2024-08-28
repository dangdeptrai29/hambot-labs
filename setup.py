from setuptools import setup, find_packages

setup(
    name="robot_systems",
    version="1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "opencv-python",
        "adafruit-circuitpython-bno055",
        "adafruit-circuitpython-rplidar",
        "buildhat",
    ],
    author="Chance J Hamilton",
    description="A library for controlling a HamBot robot with IMU, Lidar, Camera, and motor control using Build HAT.",
    long_description=open('README.md').read(),
    long_description_content_type="text/markdown",
    url="https://github.com/biorobaw/HamBot",  # Replace with your GitHub repo URL
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.10',
)
