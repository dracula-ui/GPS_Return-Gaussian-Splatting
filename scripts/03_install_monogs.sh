#!/usr/bin/env bash
# MonoGS (3DGS SLAM) in its own conda env, patched with the ROS/ZMQ live dataset.
# Needs an NVIDIA GPU + driver. CUDA 11.8 toolkit is installed inside the conda env (system CUDA not required).
source "$(dirname "$0")/common.sh"
MONOGS_COMMIT="6c9254c319d8bff5caeef65259e6bb0941a9b9f6"
command -v nvidia-smi >/dev/null || die "No NVIDIA driver found (nvidia-smi). MonoGS needs a CUDA GPU."
CAP="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -1)"; say "GPU compute capability $CAP"

# 1. conda
if ! command -v conda >/dev/null && [ ! -d "$HOME/miniconda3" ]; then
  say "Installing Miniconda"
  curl -sSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh
  bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
fi
if command -v conda >/dev/null; then source "$(conda info --base)/etc/profile.d/conda.sh"; else source "$HOME/miniconda3/etc/profile.d/conda.sh"; fi

# 2. MonoGS source (pinned) + patch
if [ ! -d "$MONOGS_DIR/.git" ]; then
  git clone --recursive https://github.com/muskie82/MonoGS.git "$MONOGS_DIR"
fi
( cd "$MONOGS_DIR" && git checkout -q "$MONOGS_COMMIT" && git submodule update --init --recursive )
if ! [ -f "$MONOGS_DIR/utils/ros_dataset.py" ]; then
  ( cd "$MONOGS_DIR" && git apply "$ROOT/slam/patches/monogs_ros.patch" ) || die "patch did not apply (MonoGS not at $MONOGS_COMMIT?)"
fi
cp "$ROOT/slam/monogs_files/ros_dataset.py" "$MONOGS_DIR/utils/ros_dataset.py"
cp "$ROOT/slam/monogs_files/rtlf_protocol.py" "$MONOGS_DIR/utils/rtlf_protocol.py"

# 3. conda env (python 3.10 + torch 2.1.2/cu118 so Ampere and Ada GPUs work; upstream's py3.7/cu116 env is too old for Ada)
if ! conda env list | grep -q "^$CONDA_ENV "; then
  conda create -y -n "$CONDA_ENV" python=3.10
fi
conda activate "$CONDA_ENV"
python -m pip install --upgrade pip
python -m pip install torch==2.1.2 torchvision==0.16.2 --index-url https://download.pytorch.org/whl/cu118
if ! command -v nvcc >/dev/null || ! nvcc --version | grep -q "release 11.8"; then
  say "Installing CUDA 11.8 toolkit into the conda env (large download)"
  conda install -y -c "nvidia/label/cuda-11.8.0" cuda-toolkit
fi
export CUDA_HOME="${CONDA_PREFIX}"
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST="${CAP}"
python -m pip install "numpy<2" plyfile==0.8.1 tqdm opencv-python==4.8.1.78 munch trimesh evo==1.11.0 open3d==0.17.0 \
  torchmetrics imgviz PyOpenGL glfw PyGLM wandb lpips rich pyyaml pillow pyzmq
( cd "$MONOGS_DIR" && python -m pip install --no-build-isolation submodules/simple-knn submodules/diff-gaussian-rasterization )

# 4. configs with the simulated camera intrinsics
python "$ROOT/slam/make_configs.py" --monogs "$MONOGS_DIR"
python - <<PY
import torch; print("torch", torch.__version__, "cuda available:", torch.cuda.is_available())
import diff_gaussian_rasterization, simple_knn; print("CUDA extensions import OK")
PY
say "MonoGS installed at $MONOGS_DIR (conda env '$CONDA_ENV')"
