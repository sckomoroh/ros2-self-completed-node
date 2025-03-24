FROM osrf/ros:iron-desktop

ARG USERNAME
ARG USER_UID
ARG USER_GID

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y
RUN apt install -y gdb

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


WORKDIR /3party
RUN git clone https://github.com/rogersce/cnpy.git

WORKDIR /3party/cnpy/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install
RUN ldconfig


USER $USERNAME

WORKDIR /ws

RUN echo '. /opt/ros/iron/setup.bash' >> /home/$USERNAME/.bashrc && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

CMD ["bash"]
