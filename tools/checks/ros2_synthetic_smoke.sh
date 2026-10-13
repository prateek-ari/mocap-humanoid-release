#!/usr/bin/env bash
set -euo pipefail

# End-to-end synthetic MXTP02 -> Xsens ROS 2 stream-node -> topics -> rosbag test.
# Requires the repository's xsens-ros2:jazzy image; no suit or licence is used.

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
image_name=${XSENS_ROS_IMAGE:-xsens-ros2:jazzy}
duration_s=${SMOKE_DURATION_S:-20}
container_name="mocap-g1-xsens-smoke-$$"
artifact_dir=$(mktemp -d -t mocap-g1-ros2-smoke-XXXXXX)
sender_pid=""

cleanup() {
  if [[ -n "${sender_pid}" ]]; then
    kill "${sender_pid}" 2>/dev/null || true
    wait "${sender_pid}" 2>/dev/null || true
  fi
  docker rm -f "${container_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  echo "ERROR: Docker image ${image_name} is absent." >&2
  echo "Build it with: docker build -t ${image_name} ${repo_root}/docker/xsens-ros2" >&2
  exit 2
fi

docker run -d \
  --name "${container_name}" \
  --network=host \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/ros-log \
  -v "${artifact_dir}:/artifacts" \
  "${image_name}" \
  bash -c 'ros2 launch xsens_mvn_ros2 xsens_stream.launch.py launch_rviz:=false' \
  >"${artifact_dir}/container.id"

python3 "${repo_root}/tools/mxtp02_replayer.py" \
  --synthetic --host 127.0.0.1 --port 9763 --rate 60 \
  --duration "${duration_s}" --with-joint-angles --quiet \
  >"${artifact_dir}/sender.log" 2>&1 &
sender_pid=$!

topics=""
for _ in $(seq 1 30); do
  topics=$(docker exec "${container_name}" /entrypoint.sh ros2 topic list 2>/dev/null || true)
  if [[ -n "${topics}" ]]; then
    break
  fi
  sleep 1
done
printf '%s\n' "${topics}" >"${artifact_dir}/topics.txt"

if [[ -z "${topics}" ]]; then
  docker logs "${container_name}" >"${artifact_dir}/driver.log" 2>&1 || true
  echo "ERROR: stream container exposed no ROS 2 topics; artifacts: ${artifact_dir}" >&2
  exit 1
fi

# These are semantic patterns rather than hard-coded namespaces because the
# upstream launch file can prefix actor topics. Override as a space-separated
# list through EXPECTED_TOPIC_PATTERNS when testing a custom launch config.
read -r -a expected_patterns <<<"${EXPECTED_TOPIC_PATTERNS:-diagnostics tf joint link}"
for pattern in "${expected_patterns[@]}"; do
  if ! rg -qi "${pattern}" "${artifact_dir}/topics.txt"; then
    docker logs "${container_name}" >"${artifact_dir}/driver.log" 2>&1 || true
    echo "ERROR: no ROS topic matched '${pattern}'; artifacts: ${artifact_dir}" >&2
    exit 1
  fi
done

# Topic advertisement alone is not proof that the paired datagrams were
# accepted. Require one decoded motion message before starting the bag.
if ! timeout 10 docker exec "${container_name}" \
  /entrypoint.sh ros2 topic echo --once /link_states \
  >"${artifact_dir}/link_states.txt" 2>&1; then
  docker logs "${container_name}" >"${artifact_dir}/driver.log" 2>&1 || true
  echo "ERROR: driver advertised /link_states but published no decoded frame; artifacts: ${artifact_dir}" >&2
  exit 1
fi

diagnostics_topic=$(rg -i 'diagnostics' "${artifact_dir}/topics.txt" | head -n 1)
if [[ -n "${diagnostics_topic}" ]]; then
  timeout 5 docker exec "${container_name}" \
    /entrypoint.sh ros2 topic echo --once "${diagnostics_topic}" \
    >"${artifact_dir}/diagnostics.txt"
fi

set +e
docker exec "${container_name}" /entrypoint.sh timeout --signal=INT 5 \
  ros2 bag record -a -o /artifacts/synthetic_bag \
  >"${artifact_dir}/rosbag.log" 2>&1
bag_rc=$?
set -e
if [[ ${bag_rc} -ne 0 && ${bag_rc} -ne 124 && ${bag_rc} -ne 130 ]]; then
  echo "ERROR: rosbag recording failed (${bag_rc}); artifacts: ${artifact_dir}" >&2
  exit 1
fi
if [[ ! -s "${artifact_dir}/synthetic_bag/metadata.yaml" ]]; then
  echo "ERROR: rosbag metadata was not produced; artifacts: ${artifact_dir}" >&2
  exit 1
fi

docker exec "${container_name}" /entrypoint.sh \
  ros2 bag info /artifacts/synthetic_bag \
  >"${artifact_dir}/bag-info.txt"
for topic in /joint_states /link_states /tf; do
  if ! rg -q "Topic: ${topic//\//\\/} .* Count: [1-9][0-9]*" "${artifact_dir}/bag-info.txt"; then
    docker logs "${container_name}" >"${artifact_dir}/driver.log" 2>&1 || true
    echo "ERROR: rosbag contains no messages for ${topic}; artifacts: ${artifact_dir}" >&2
    exit 1
  fi
done

wait "${sender_pid}"
sender_pid=""
docker logs "${container_name}" >"${artifact_dir}/driver.log" 2>&1 || true
echo "PASS: paired synthetic datagrams produced decoded ROS 2 motion topics and a rosbag."
echo "Artifacts: ${artifact_dir}"
