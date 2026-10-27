# Post-flight reconstruction. UNVERIFIED skeleton: pin every ARG during M5.
ARG CUDA_BASE=nvidia/cuda:12.4.1-devel-ubuntu22.04   # TODO M5: match workstation driver
FROM ${CUDA_BASE}

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

# TODO M5: build pinned COLMAP (CUDA enabled) and OpenMVS
# TODO M5: pip install -e offline/x650_reconstruction offline/x650_model_export

WORKDIR /work
