


# Build venv using same python3 ros2 was built against (e.g., /usr/bin/python3 on Ubuntu)
#python3 -m venv --system-site-packages ~/venvs/ros2

# Source venv just vreated above
source ~/venvs/ros2/bin/activate

# Source ros2
source /opt/ros/jazzy/setup.bash # e.g., humble, iron, jazzy

# Install needed python packages
#pip install PySide6 PySide6-Essentials PySide6-Addons pyqtgraph

# Install needed apt packages
#sudo apt install -y libxkbcommon-x11-0 libxcb-keysyms1 libxcb-icccm4 \
#                    libxcb-cursor0 libxcb-render-util0 libgl1
