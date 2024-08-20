import os
import sys
import subprocess
from setuptools import setup, find_packages


def create_and_install_in_virtualenv():
    venv_dir = "hambot_env"

    if not os.path.exists(venv_dir):
        print("Creating virtual environment...")
        subprocess.check_call([sys.executable, "-m", "venv", venv_dir])
        print(f"Virtual environment created at {venv_dir}.")
    else:
        print(f"Virtual environment already exists at {venv_dir}.")

    # Determine the path to the Python interpreter in the virtual environment
    if os.name == "nt":  # Windows
        python_bin = os.path.join(venv_dir, "Scripts", "python")
    else:  # Unix or MacOS
        python_bin = os.path.join(venv_dir, "bin", "python")

    # Upgrade pip and install the package in the virtual environment
    subprocess.check_call([python_bin, "-m", "pip", "install", "--upgrade", "pip"])
    subprocess.check_call([python_bin, "-m", "pip", "install", "."])

    print(
        f"To activate the virtual environment in the future, run 'source {os.path.join(venv_dir, 'bin', 'activate')}'.")


# Run the function if the script is executed directly
if __name__ == "__main__":
    create_and_install_in_virtualenv()

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
