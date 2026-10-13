# syntax=docker/dockerfile:1
# Xsens MVN ROS 2 driver (Jazzy) — see docs/engineer-reference.md, "Docker images".
# ros:jazzy-ros-base (multi-arch index digest, resolved 2026-10-10). Bump deliberately.
FROM ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca

ARG XSENS_REPO=https://github.com/xsens/xsens_mvn_ros2
# Pinned upstream commit (master @ 2026-08-03). Override to bump:
#   docker build --build-arg XSENS_REF=<sha> ...
ARG XSENS_REF=d2a515b876e2b9f4f127f3fbde1d20c5189adc95

LABEL org.opencontainers.image.title="xsens-ros2" \
      org.opencontainers.image.description="Xsens MVN ROS 2 (Jazzy) driver, built from ${XSENS_REPO}" \
      org.opencontainers.image.source="${XSENS_REPO}"

ENV COLCON_WS=/colcon_ws
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${COLCON_WS}

# --- Fetch driver source -----------------------------------------------------
# The repo root is itself a colcon workspace (src/pkgs + src/deps, the latter
# carrying the vendored Xsens MVN/XME SDKs). Fetch only the pinned commit.
RUN git init -q . \
 && git remote add origin "${XSENS_REPO}" \
 && git fetch -q --depth 1 origin "${XSENS_REF}" \
 && git checkout -q FETCH_HEAD \
 && rm -rf .git

# --- Resolve and install dependencies ----------------------------------------
# NOT bit-reproducible: the base image and driver commit are pinned, but
# `rosdep update` fetches the current rosdistro index and `rosdep install` pulls
# unpinned apt packages from the live Ubuntu/ROS archives, so two builds on
# different days can differ. Record the resulting image digest (docker image
# inspect) when a build is validated; pinning every apt version here is not
# practical because the dependency set is derived from the upstream package.xml
# files at build time.
# rviz2 is only an exec_depend used for visualization; skipping it keeps the
# image lean and headless-friendly (launch with launch_rviz:=false).
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src -y \
      --rosdistro "${ROS_DISTRO}" \
      --skip-keys "rviz2" \
 && rm -rf /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache

# --- Build --------------------------------------------------------------------
# Explicit Release flags: the repo ships a colcon_defaults.yaml targeting
# Debug + coverage, but that file is only honoured when COLCON_DEFAULTS_FILE
# points at it (it doesn't here). BUILD_TESTING stays ON and build/ is kept
# so `colcon test` can run inside the container.
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build \
      --base-paths src/pkgs src/deps \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --no-warn-unused-cli \
 && rm -rf log

# --- Entrypoint sourcing the overlay ------------------------------------------
COPY <<'EOF' /entrypoint.sh
#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${COLCON_WS}/install/setup.bash" ]; then
    source "${COLCON_WS}/install/setup.bash"
fi
exec "$@"
EOF
RUN chmod +x /entrypoint.sh

# MVN Studio streams MVNX network frames to this UDP port (stream-mode default,
# parameter `udp_port` in xsens_stream_node.yaml).
EXPOSE 9763/udp

# Run unprivileged (uid 1000 is the stock "ubuntu" user in the Jazzy base image).
# The workspace under /colcon_ws is read-only for this user; mount a writable
# volume for any bag/log output (tools/checks/ros2_synthetic_smoke.sh does).
USER 1000:1000

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
