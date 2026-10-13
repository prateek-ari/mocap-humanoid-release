# ubuntu:24.04 (multi-arch index digest, verified 2026-10-12 with
# `docker buildx imagetools inspect ubuntu:24.04`). Keep identical to
# docker/gmr-live/Dockerfile. Bump deliberately.
FROM ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libdbus-1-3 \
        libdrm2 \
        libegl1 \
        libexpat1 \
        libfontconfig1 \
        libfreetype6 \
        libgl1 \
        libgl1-mesa-dri \
        libglvnd0 \
        libglx0 \
        libgpg-error0 \
        libharfbuzz0b \
        libice6 \
        libsm6 \
        libstdc++6 \
        libudev1 \
        libvulkan1 \
        libx11-6 \
        libx11-xcb1 \
        libxcb-glx0 \
        libxcb1 \
        libxext6 \
        libxi6 \
        libxkbcommon-x11-0 \
        libxrandr2 \
        libxrender1 \
        xdg-utils \
        zlib1g \
    && rm -rf /var/lib/apt/lists/*

ENV HOME=/home/xsens \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1

RUN mkdir -p /home/xsens && chown 1000:1000 /home/xsens

# Unprivileged by default; docker/xsens-humanoid/run.sh also passes --user.
USER 1000:1000

WORKDIR /home/xsens

ENTRYPOINT ["/opt/xsens/MVN-Studio.appimage", "--appimage-extract-and-run"]
