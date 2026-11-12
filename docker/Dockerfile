# Development image for ghost_sim (the Ghost SDK on swappable simulators), for Gazebo: Gazebo
# Harmonic (the LTS that pairs with ROS 2 Jazzy, so ros_gz bridging to ROS 2 stays an apt install)
# on osrf/ros:jazzy-desktop, the packages ghost_sim builds with, and a Python 3.10 environment for
# mavgui and examples/ (the SDK's grmblinkpy is built for 3.10).
# docker/compose.yaml mounts the project and the SDK; usage: docker/README.md.
FROM osrf/ros:jazzy-desktop
ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic (gz-sim 8), from the OSRF repository.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo "$VERSION_CODENAME") main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Python 3.10 for mavgui and examples/, managed with uv: a standalone CPython
# in /opt/gsim/python and the virtual environment /opt/gsim/venv, with the versions pinned in
# requirements-mavgui.txt (the file the README's conda env installs too). The venv is not on PATH,
# so python3 stays the system 3.12 that ROS 2 needs; the scripts use $GSIM_PY (scripts/env.sh).
ENV UV_PYTHON_INSTALL_DIR=/opt/gsim/python
COPY requirements-mavgui.txt /tmp/requirements-mavgui.txt
RUN --mount=from=ghcr.io/astral-sh/uv:0.8.0,source=/uv,target=/usr/local/bin/uv \
    uv venv --python 3.10 /opt/gsim/venv && \
    uv pip install --python /opt/gsim/venv/bin/python -r /tmp/requirements-mavgui.txt && \
    rm -rf /root/.cache/uv /tmp/requirements-mavgui.txt
ENV GSIM_PY=/opt/gsim/venv/bin/python

# The packages ghost_sim builds with (the same list as install.md, "Without Docker"), cppzmq-dev which linking
# against gz-sim needs (find_package(gz-sim8) fails without it), and numpy for the Python checks
# that run with the system python3.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libegl-dev libgtest-dev \
    cppzmq-dev \
    python3-numpy \
  && rm -rf /var/lib/apt/lists/*

# The container runs as root, with its home (/root) mounted from the host by docker/compose.yaml.
CMD ["bash"]
