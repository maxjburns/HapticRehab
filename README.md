# HapticRehab
ME327 Final Project - Mattias Cooper, Andrew Zerbe, Andrew Zaman, Max Burns.


## OpenPose Setup
The following steps are intended for Ubuntu 22.04 and use this tutorial: https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_installation_0_index.html

First step is to install OpenPose:
```
cd gits
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
cd openpose/
git submodule update --init --recursive --remote
```

Then we need to build OpenPose:
```
cd openpose
mkdir build/
cd build/
cmake-gui ..
```

In the GUI that opens, press configure. Then, if using Ubuntu select "UNIX MAKEFILES", and press finish.

Then, check "BUILD_PYTHON" and if GPU-less, set "GPU_MODE" to "CPU_ONLY". Press configure again, then press finish. Now, press "Generate"

This will take a few minutes. Once it's done, exit cmake gui and enter the following:
```
make -j`nproc`
```

This will take a while. Once done, you should be able to run test_tracking.py from the root directory, or the demo outlined here: https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_01_demo.html

## Environment Setup

Install conda if you haven't already (normal venv works too)

```
conda create -n HapticRehabEnv
conda activate HapticRehabEnv
pip install opencv-python numpy
conda install -c conda-forge libstdcxx-ng
```

## Intended Use

All python scripts should be run from HapticRehab/ or the paths will get confused. Do not run from the pose_tracking folder directly.