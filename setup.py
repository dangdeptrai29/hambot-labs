import os
import sys
import subprocess
from setuptools import setup, find_packages


# Function to create a virtual environment
def create_and_activate_virtualenv():
    venv_dir = "hambot_env"

    if not os.path.exists(venv_dir):
        print("Creating virtual environment...")
        subprocess.check_call([sys.executable, "-m", "venv", venv_dir])
        print(f"Virtual environment created at {venv_dir}.")
    else:
        print(f"Virtual environment already exists at {venv_dir}.")

    # Activate the virtual environment and install the package
    if os.name == "nt":  # Windows
        activate_script = os.path.join(venv_dir, "Scripts", "activate.bat")
        subprocess.check_call(
            [activate_script, "&&", "pip", "install", "--upgrade", "pip", "&&", "pip", "install", "."])
    else:  # Unix or MacOS
        activate_script = os.path.join(venv_dir, "bin", "activate")
        subprocess.check_call(f"source {activate_script} && pip install --upgrade pip && pip install .", shell=True)

    print(f"To activate the virtual environment in the future, run 'source {activate_script}'.")


# If running directly, perform the virtual environment setup
if __name__ == "__main__":
    create_and_activate_virtualenv()

# Standard setup.py configuration
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
    author="Your Name",
    description="A library for controlling a HamBot robot with IMU, Lidar, Camera, and motor control using Build HAT.",
    long_description=open('README.md').read(),
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/hambot",  # Replace with your GitHub repo URL
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
