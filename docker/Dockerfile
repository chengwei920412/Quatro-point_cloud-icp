FROM ros:noetic-perception
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-catkin-tools \
        libomp-dev \
        cmake \
        git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && mkdir -p /ws/src && cd /ws && catkin init"

CMD ["/bin/bash"]
