# Persist conda envs/pkgs across container restarts
docker run --name isaacsim_go2 --entrypoint bash -it --gpus all \
--network=host \
--runtime=nvidia \
-e X11_FORWARDING_ENABLED=1 \
-e PRIVACY_CONSENT=Y \
-e ACCEPT_EULA=Y \
-e DISPLAY=$DISPLAY \
-e "ACCEPT_EULA=Y" \
-e XAUTHORITY=/root/.Xauthority \
-v $(pwd):/workspace/repo-go2:rw \
-w /workspace/repo-go2 \
-v isaacsim_go2_conda_envs:/root/anaconda3/envs:rw \
-v isaacsim_go2_conda_pkgs:/root/anaconda3/pkgs:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/.Xauthority:/root/.Xauthority:rw \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v ./IsaacSimAssests:/root/IsaacSimAssests:rw \
-v ./scenes:/root/scenes:rw \
isaacsim_go2_5.1.0:latest
