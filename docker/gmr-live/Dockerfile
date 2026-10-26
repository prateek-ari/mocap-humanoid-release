# ubuntu:24.04 (multi-arch index digest, verified 2026-10-12 with
# `docker buildx imagetools inspect ubuntu:24.04`). Bump deliberately.
FROM ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55

ARG DEBIAN_FRONTEND=noninteractive
ARG GMR_REF=bb1bbe40774794fceb2a7c579a3464a28e68c844

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        libegl1 \
        libgl1 \
        libglfw3 \
        libglvnd0 \
        libglx0 \
        libx11-6 \
        libxcursor1 \
        libxext6 \
        libxi6 \
        libxinerama1 \
        libxrandr2 \
        libxxf86vm1 \
        python3 \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python -m pip install --no-cache-dir --upgrade pip==26.2.1 setuptools==84.0.0 wheel==0.48.0

RUN git init -q /opt/GMR \
    && git -C /opt/GMR remote add origin https://github.com/YanjieZe/GMR.git \
    && git -C /opt/GMR fetch --depth 1 origin "${GMR_REF}" \
    && git -C /opt/GMR checkout -q FETCH_HEAD \
    && test "$(git -C /opt/GMR rev-parse HEAD)" = "${GMR_REF}"

COPY retarget_configs/gmr-issue-180.patch /tmp/gmr-issue-180.patch
COPY retarget_configs/gmr-ground-offset-config.patch /tmp/gmr-ground-offset-config.patch
COPY retarget_configs/gmr-optional-torch.patch /tmp/gmr-optional-torch.patch
COPY retarget_configs/bvh_lafan1_to_g1.json /tmp/bvh_lafan1_to_g1.json

RUN git -C /opt/GMR apply /tmp/gmr-issue-180.patch \
    && git -C /opt/GMR apply /tmp/gmr-ground-offset-config.patch \
    && git -C /opt/GMR apply /tmp/gmr-optional-torch.patch \
    && cp /tmp/bvh_lafan1_to_g1.json \
        /opt/GMR/general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json

RUN /opt/venv/bin/python -m pip install --no-cache-dir \
        'imageio[ffmpeg]>=2.37,<3' \
        'loop_rate_limiters>=1.2,<2' \
        'mink==1.2.0' \
        'mujoco==3.11.0' \
        'numpy>=2.2,<2.3' \
        'rich>=14,<16' \
        'scipy>=1.15,<1.16' \
    && /opt/venv/bin/python -m pip install --no-cache-dir -e /opt/GMR --no-deps

WORKDIR /opt/mocap_humanoid
COPY pyproject.toml ./
COPY mocap_g1 ./mocap_g1
# GMR ships no live MVN configs for the Booster K1 and T1; the live tool installs them.
COPY retarget_configs/xsens_mvn_to_booster_k1.json ./retarget_configs/xsens_mvn_to_booster_k1.json
COPY retarget_configs/xsens_mvn_to_booster_t1_29dof.json ./retarget_configs/xsens_mvn_to_booster_t1_29dof.json
COPY tools/xsens/live_g1_sim.py ./tools/xsens/live_g1_sim.py
# run-offline.sh executes this script from the image (not a host bind mount), so
# the image tag identifies exactly which loop code ran. Rebuild after editing it.
COPY tools/xsens/loop_bvh_g1.py ./tools/xsens/loop_bvh_g1.py

RUN /opt/venv/bin/python -m pip install --no-cache-dir -e . --no-deps \
    && /opt/venv/bin/python -c \
        'import general_motion_retargeting, mujoco, mink, daqp, mocap_g1; print(mujoco.__version__)' \
    && chmod -R a+rX /opt/mocap_humanoid

# Run unprivileged by default (uid 1000 is the stock "ubuntu" user in 24.04).
# docker/gmr-live/run*.sh override this with --user "$(id -u):$(id -g)" and a
# writable HOME tmpfs; everything under /opt is read-only for that user, which
# is all the entrypoint needs.
USER 1000:1000

# COPY preserves restrictive host modes. Verify the actual unprivileged runtime
# can read the entrypoint and its imports, not just the root build user.
RUN /opt/venv/bin/python tools/xsens/live_g1_sim.py --help >/dev/null \
    && /opt/venv/bin/python tools/xsens/loop_bvh_g1.py --help >/dev/null

ENV PATH=/opt/venv/bin:${PATH} \
    PYTHONUNBUFFERED=1 \
    MUJOCO_GL=glfw

ENTRYPOINT ["python", "/opt/mocap_humanoid/tools/xsens/live_g1_sim.py"]
CMD ["--port", "5555"]
