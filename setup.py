import os
import subprocess
import sys
from setuptools import setup, find_packages


# Function to create a virtual environment
def create_virtualenv():
    venv_dir = "hambot_env"

    if not os.path.exists(venv_dir):
        subprocess.check_call([sys.executable, "-m", "venv", venv_dir])
        print(f"Virtual environment created at {venv_dir}.")
    else:
        print(f"Virtual environment already exists at {venv_dir}.")

    # Activate the virtual environment and install the package
    if os.name == "nt":  # Windows
        activate_script = os.path.join(venv_dir, "Scripts", "activate")
    else:  # Unix or MacOS
        activate_script = os.path.join(venv_dir, "bin", "activate")

    print(f"To activate the virtual environment, run 'source {activate_script}'.")
    print("After activation, run 'pip install .' to install the hambot package in the environment.")


# Run the function if the script is executed directly
if __name__ == "__main__":
    create_virtualenv()

setup(
    name="hambot",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "opencv-python",
        "picamera",
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
