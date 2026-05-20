# ============================================================
# Robot Aircraft Mid-Level Project Dockerfile
# 로봇항공기대회 중급부문 개발용 Dockerfile
#
# Stack:
#   - ROS2 Humble
#   - Ubuntu 22.04 Jammy
#   - Gazebo Classic
#   - OpenCV / NumPy / scikit-image
#
# Purpose:
#   Week 1   : OpenCV line mask, debug overlay, image/mask saving
#   Week 2   : BEV transform, skeleton/centerline, error calculation
#   Week 3   : FOLLOW_LINE control, LINE_LOST handling, logging
#   Week 4   : intersection/branch detection, state machine extension
#   Week 5~6 : data collection, OpenCV pseudo-label generation
#   Week 7~8 : AI segmentation training, offline inference comparison
#   Week 9   : Jetson inference, AI mask generator integration
#   Week 10  : AI main + OpenCV fallback, final tuning
## ============================================================
# Robot Aircraft Mid-Level Project Dockerfile
# 로봇항공기대회 중급부문 개발용 Dockerfile
#
# Stack:
#   - ROS2 Humble
#   - Ubuntu 22.04 Jammy
#   - Gazebo Classic
#   - OpenCV / NumPy / scikit-image
#
# Purpose:
#   Week 1   : OpenCV line mask, debug overlay, image/mask saving
#   Week 2   : BEV transform, skeleton/centerline, error calculation
#   Week 3   : FOLLOW_LINE control, LINE_LOST handling, logging
#   Week 4   : intersection/branch detection, state machine extension
#   Week 5~6 : data collection, OpenCV pseudo-label generation
#   Week 7~8 : AI segmentation training, offline inference comparison
#   Week 9   : Jetson inference, AI mask generator integration
#   Week 10  : AI main + OpenCV fallback, final tuning
#
# Note:
#   - This v1 image includes basic AI packages: PyTorch, Ultralytics, ONNX.
#   - TensorRT is intentionally excluded because Jetson deployment should be handled separately.
#   - This environment is separated from HRI_Drone_project.ble.
# ============================================================


# ------------------------------------------------------------
# (고정) ROS2 Humble Desktop 이미지를 기본 베이스로 사용합니다.
# (Fixed) Use ROS2 Humble Desktop as the base image.
#
# 이유:
#   - ROS2 Humble은 Ubuntu 22.04 Jammy 기반입니다.
#   - desktop 이미지는 RViz, GUI 관련 기반 패키지가 포함되어 있습니다.
#   - Gazebo/RViz/OpenCV GUI 디버깅에 유리합니다.
#
# Reason:
#   - ROS2 Humble is based on Ubuntu 22.04 Jammy.
#   - The desktop image includes GUI-related packages.
#   - It is useful for Gazebo/RViz/OpenCV GUI debugging.
# ------------------------------------------------------------
FROM osrf/ros:humble-desktop


# ------------------------------------------------------------
# (고정) 비대화형 설치 모드와 기본 로케일을 설정합니다.
# (Fixed) Set non-interactive apt mode and default locale.
# ------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV PYTHONUNBUFFERED=1



# ------------------------------------------------------------
# (고정) Docker 빌드 중 source 명령어를 쓰기 위해 bash를 기본 셸로 사용합니다.
# (Fixed) Use bash as the default shell so that source commands work during build.
# ------------------------------------------------------------
SHELL ["/bin/bash", "-c"]


# ------------------------------------------------------------
# (핵심) Gazebo 저장소 등록에 필요한 기본 도구를 설치합니다.
# (Core) Install basic tools required to register the Gazebo repository.
#
# curl:
#   Gazebo 저장소 키를 다운로드할 때 사용합니다.
#
# gnupg:
#   저장소 GPG 키 처리를 위해 필요합니다.
#
# lsb-release:
#   Ubuntu 코드명(jammy)을 자동으로 확인할 때 사용합니다.
#
# ca-certificates:
#   HTTPS 인증서 검증에 필요합니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    && curl https://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# (필수) 기본 개발 도구를 설치합니다.
# (Required) Install core development tools.
#
# build-essential:
#   C/C++ 컴파일러, make 등 기본 빌드 도구 묶음입니다.
#
# cmake / ninja-build:
#   C++/ROS/Gazebo 관련 패키지 빌드에 필요합니다.
#
# git:
#   소스코드 버전 관리 및 외부 패키지 다운로드에 필요합니다.
#
# vim / nano:
#   컨테이너 내부에서 빠르게 파일을 수정할 때 사용합니다.
#
# wget / curl:
#   파일 다운로드와 설치 스크립트 실행에 사용합니다.
#
# less / tree:
#   로그 확인 및 폴더 구조 확인용 유틸리티입니다.
#
# htop:
#   CPU/RAM 사용량을 컨테이너 내부에서 확인할 때 유용합니다.
#
# tmux:
#   하나의 터미널 안에서 여러 세션을 관리할 때 사용합니다.
#
# xterm:
#   ROS teleop이나 디버깅용 별도 터미널 실행에 사용될 수 있습니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    vim \
    nano \
    wget \
    curl \
    less \
    tree \
    htop \
    tmux \
    xterm \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# (필수) USB, 카메라, GUI, OpenGL 관련 도구를 설치합니다.
# (Required) Install USB, camera, GUI, and OpenGL related tools.
#
# usbutils:
#   lsusb 명령어를 제공하여 USB 카메라/보드 연결 확인에 사용합니다.
#
# v4l-utils:
#   v4l2-ctl 명령어를 제공하여 /dev/video0 카메라 포맷과 FPS를 확인합니다.
#
# x11-apps:
#   xeyes 등 X11 GUI 테스트 프로그램을 제공합니다.
#
# mesa-utils:
#   glxinfo, glxgears 등 OpenGL 상태 확인에 사용합니다.
#
# libgl1-mesa-glx / libgl1-mesa-dri:
#   OpenCV imshow, Gazebo GUI, RViz 등 그래픽 출력에 필요한 OpenGL 계열 라이브러리입니다.
#
# libglib2.0-0:
#   OpenCV 및 여러 GUI/영상 처리 라이브러리에서 필요합니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    usbutils \
    v4l-utils \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# (필수) Python과 OpenCV 기반 영상처리 패키지를 설치합니다.
# (Required) Install Python and OpenCV-based image-processing packages.
#
# python3:
#   Python 실행 환경입니다.
#
# python3-pip:
#   Python 패키지 설치 관리자입니다.
#
# python3-venv:
#   필요 시 가상환경을 만들 수 있게 합니다.
#
# python3-opencv:
#   OpenCV Python 바인딩입니다.
#   line mask, camera capture, debug overlay에 사용합니다.
#
# python3-numpy:
#   이미지 배열, 좌표, lateral error, heading error 계산에 사용합니다.
#
# python3-matplotlib:
#   로그, mask, error 그래프를 시각화할 때 사용합니다.
#
# python3-skimage:
#   skeletonize, morphology, connected component 분석에 사용합니다.
#
# python3-yaml:
#   threshold, camera, BEV 파라미터를 yaml 설정파일로 관리할 때 사용합니다.
#
# python3-tqdm:
#   데이터셋 처리, pseudo-label 생성 진행률 표시용입니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-opencv \
    python3-numpy \
    python3-matplotlib \
    python3-skimage \
    python3-yaml \
    python3-tqdm \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# (필수) ROS2 빌드 및 패키지 개발 도구를 설치합니다.
# (Required) Install ROS2 build and package development tools.
#
# python3-colcon-common-extensions:
#   ROS2 workspace를 colcon build로 빌드하기 위한 기본 확장 묶음입니다.
#
# python3-rosdep:
#   ROS 패키지 의존성 설치를 관리하는 도구입니다.
#
# python3-vcstool:
#   여러 ROS/Git 저장소를 .repos 파일로 관리할 때 사용합니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
#apt/distutils로 설치된 sympy와 PyTorch 의존성 충돌을 피합니다.
#Avoid conflict between apt/distutils-installed sympy and PyTorch dependency.
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    sympy==1.13.1 \
    mpmath==1.3.0


# ------------------------------------------------------------
# (AI) PyTorch GPU 버전을 설치합니다.
# (AI) Install PyTorch with CUDA GPU support.
#
# torch:
#   딥러닝 모델 학습/추론의 핵심 프레임워크입니다.
#
# torchvision:
#   이미지 변환, dataset utility, vision model 관련 기능을 제공합니다.
#
# torchaudio:
#   지금 당장은 필수는 아니지만 PyTorch 공식 설치 조합에 포함되는 경우가 많아 같이 둡니다.
#
# --index-url https://download.pytorch.org/whl/cu121:
#   CUDA 12.1용 PyTorch wheel을 사용합니다.
#   호스트 드라이버가 더 최신이면 보통 이전 CUDA runtime 기반 wheel을 실행할 수 있습니다.
#
# torch:
#   Core deep learning framework for training and inference.
#
# torchvision:
#   Provides image transforms, dataset utilities, and vision model support.
#
# torchaudio:
#   Not essential for this project yet, but commonly installed with the official PyTorch stack.
#
# --index-url https://download.pytorch.org/whl/cu121:
#   Uses PyTorch wheels built for CUDA 12.1.
#   A newer host NVIDIA driver can generally run applications built with an older CUDA runtime.
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
    torch \
    torchvision \
    torchaudio \
    --index-url https://download.pytorch.org/whl/cu121

   
# ------------------------------------------------------------
# (AI) 영상 segmentation / dataset / augmentation 패키지를 설치합니다.
# (AI) Install segmentation, dataset, and augmentation packages.
#
# ultralytics:
#   YOLO 계열 detection/segmentation 모델 학습 및 추론에 사용합니다.
#   나중에 line mask generator를 AI segmentation 모델로 교체할 때 후보로 사용합니다.
#
# albumentations:
#   이미지 segmentation 학습용 augmentation에 강합니다.
#   밝기, blur, perspective, noise, crop 등을 데이터 증강에 사용할 수 있습니다.
#
# segmentation-models-pytorch:
#   U-Net, FPN, DeepLabV3+ 등 segmentation baseline을 빠르게 실험할 수 있습니다.
#
# timm:
#   segmentation backbone이나 vision model 실험에 자주 쓰이는 모델 라이브러리입니다.
#
# pillow:
#   이미지 파일 입출력 보조 라이브러리입니다.
#
# pandas:
#   dataset index, label table, 실험 결과 csv 관리에 사용합니다.
#
# scipy:
#   수치 계산, morphology, 보간, 후처리 등에 사용할 수 있습니다.
#
# tensorboard:
#   학습 loss, metric, image logging을 확인할 때 사용합니다.
#
# ultralytics:
#   Used for YOLO-based detection/segmentation training and inference.
#
# albumentations:
#   Strong image augmentation library for segmentation datasets.
#
# segmentation-models-pytorch:
#   Provides quick baselines such as U-Net, FPN, and DeepLabV3+.
#
# timm:
#   Common model library for vision backbones.
#
# pillow:
#   Helper library for image file I/O.
#
# pandas:
#   Used for dataset index, label tables, and experiment result CSV files.
#
# scipy:
#   Useful for numerical processing, morphology, interpolation, and post-processing.
#
# tensorboard:
#   Used for visualizing training loss, metrics, and image logs.
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
    ultralytics \
    albumentations \
    segmentation-models-pytorch \
    timm \
    pillow \
    pandas \
    scipy \
    tensorboard


# ------------------------------------------------------------
# (AI Export) ONNX / ONNX Runtime GPU 패키지를 설치합니다.
# (AI Export) Install ONNX and ONNX Runtime GPU packages.
#
# onnx:
#   PyTorch 모델을 ONNX 형식으로 export할 때 사용합니다.
#
# onnxsim:
#   ONNX 그래프를 단순화해서 추론 최적화 전처리에 사용합니다.
#
# onnxruntime-gpu:
#   PC 환경에서 ONNX 모델의 GPU 추론을 테스트할 때 사용합니다.
#
# 주의:
#   Jetson TensorRT 배포는 JetPack/TensorRT 버전에 묶이므로 여기서 고정하지 않습니다.
#   Jetson 단계에서는 별도 Dockerfile 또는 Jetson용 branch로 분리하는 것이 맞습니다.
#
# onnx:
#   Used to export PyTorch models into ONNX format.
#
# onnxsim:
#   Simplifies ONNX graphs before inference optimization.
#
# onnxruntime-gpu:
#   Used to test GPU inference for ONNX models on the PC.
#
# Note:
#   Jetson TensorRT deployment depends on JetPack/TensorRT versions,
#   so it should not be fixed in this x86 desktop Dockerfile.
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
    onnx \
    onnxsim \
    onnxruntime-gpu


# ------------------------------------------------------------
# (PX4 SITL) Python build dependencies
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
    kconfiglib \
    empy==3.3.4 \
    pyros-genmsg \
    jinja2 \
    jsonschema \
    packaging \
    toml \
    pyyaml


# ------------------------------------------------------------
# (필수) ROS2 vision/image 관련 패키지를 설치합니다.
# (Required) Install ROS2 vision/image related packages.
#
# ros-humble-cv-bridge:
#   OpenCV image와 ROS sensor_msgs/Image를 변환합니다.
#
# ros-humble-image-transport:
#   ROS 이미지 토픽 전송을 위한 표준 계층입니다.
#
# ros-humble-sensor-msgs:
#   Camera/Image/IMU 등 센서 메시지 타입을 제공합니다.
#
# ros-humble-camera-info-manager:
#   카메라 intrinsic/calibration 정보를 관리할 때 사용합니다.
#
# ros-humble-image-pipeline:
#   image_proc 등 ROS 이미지 처리 파이프라인 패키지 묶음입니다.
#
# ros-humble-rqt-image-view:
#   ROS 이미지 토픽을 GUI로 확인할 때 사용합니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-camera-info-manager \
    ros-humble-image-pipeline \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# (권장) ROS2 상태머신/제어/로그 분석에 유용한 패키지를 설치합니다.
# (Recommended) Install useful ROS2 packages for state machines, control, and logging.
#
# ros-humble-std-msgs:
#   기본 메시지 타입을 명시적으로 확보합니다.
#
# ros-humble-geometry-msgs:
#   Twist, Vector3, Pose 등 제어/상태 표현 메시지에 사용합니다.
#
# ros-humble-nav-msgs:
#   Odometry, Path 등 주행/경로 관련 메시지에 사용될 수 있습니다.
#
# ros-humble-tf2-ros:
#   좌표계 변환이 필요할 때 사용합니다.
#
# ros-humble-plotjuggler-ros:
#   error, control output, state 로그를 실시간으로 플롯할 때 사용합니다.
#
# ros-humble-rosbag2-storage-mcap:
#   rosbag2 데이터를 MCAP 포맷으로 저장할 때 사용합니다.
#
# ros-humble-foxglove-bridge:
#   Foxglove Studio와 ROS2 데이터를 연결할 때 사용합니다.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-plotjuggler-ros \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# (권장) Git safe.directory 설정을 추가합니다.
# (Recommended) Add Git safe.directory configuration.
#
# 호스트와 컨테이너의 사용자/그룹 ID가 다를 때 Git이 ownership 오류를 낼 수 있습니다.
# 개발 컨테이너에서는 모든 디렉터리를 safe directory로 허용해서 마찰을 줄입니다.
# ------------------------------------------------------------
RUN git config --global --add safe.directory '*'


# ------------------------------------------------------------
# (필수) ROS2 workspace와 데이터 폴더를 미리 만듭니다.
# (Required) Create ROS2 workspace and data directories.
#
# /ros_ws/src:
#   ROS2 패키지 소스가 들어갈 위치입니다.
#
# /data/raw:
#   원본 카메라 프레임 저장 위치입니다.
#
# /data/masks:
#   OpenCV line mask 저장 위치입니다.
#
# /data/overlay:
#   debug overlay 이미지 저장 위치입니다.
#
# /config:
#   threshold, BEV, camera 설정 yaml 저장 위치입니다.
#
# /logs:
#   error, state, control 로그 저장 위치입니다.
# ------------------------------------------------------------
RUN mkdir -p \
    /project/ros_ws/src \
    /project/data/raw \
    /project/data/masks \
    /project/data/overlay \
    /project/data/dataset \
    /project/config \
    /project/logs \
    /project/models/checkpoints \
    /project/models/exported


# ------------------------------------------------------------
# Gazebo Classic is used for PX4 SITL mission_sim.
# (PX4 SITL) Gazebo Classic and ROS-Gazebo Classic packages
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    tini \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --force-reinstall "numpy==1.26.4"

# ------------------------------------------------------------
# (필수) 기본 작업 디렉터리를 ROS2 workspace로 설정합니다.
# (Required) Set the default working directory to the ROS2 workspace.
#
# 컨테이너에 들어갔을 때 바로 /ros_ws에서 시작하게 합니다.
# ------------------------------------------------------------
WORKDIR /project

# ------------------------------------------------------------
# (필수) 터미널 시작 시 ROS2/Gazebo 환경을 자동으로 로드합니다.
# (Required) Automatically source ROS2/Gazebo environments on shell startup.
#
# source /opt/ros/humble/setup.bash:
#   ROS2 Humble 기본 환경을 로드합니다.
#
# if [ -f /ros_ws/install/setup.bash ]; then source ...:
#   사용자가 colcon build를 한 뒤 생성되는 workspace 환경을 자동 로드합니다.
#
#
# cd /ros_ws:
#   터미널 시작 위치를 ROS workspace로 맞춥니다.
# ------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /project/ros_ws/install/setup.bash ]; then source /project/ros_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "export PROJECT_ROOT=/project" >> /root/.bashrc && \
    echo "export ROS_WS=/project/ros_ws" >> /root/.bashrc && \
    echo "export PX4_ROOT=/project/firmware/PX4-Autopilot" >> /root/.bashrc && \
    echo "cd /project" >> /root/.bashrc

RUN echo "export PX4_ROOT=/project/firmware/PX4-Autopilot" >> /root/.bashrc && \
    echo "export PX4_BUILD_DIR=\$PX4_ROOT/build/px4_sitl_default" >> /root/.bashrc && \
    echo "export PX4_GAZEBO_CLASSIC_DIR=\$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> /root/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=\$PX4_BUILD_DIR/build_gazebo-classic:\$GAZEBO_PLUGIN_PATH" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$PX4_GAZEBO_CLASSIC_DIR/models:\$GAZEBO_MODEL_PATH" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:\$PX4_BUILD_DIR/build_gazebo-classic:\$LD_LIBRARY_PATH" >> /root/.bashrc


# ------------------------------------------------------------
# (고정) 컨테이너 시작 시 기본 명령어를 bash로 설정합니다.
# (Fixed) Set bash as the default container command.
#
# 실제 compose에서는 tail -f /dev/null로 컨테이너를 유지할 예정입니다.
# 그래도 Dockerfile 자체의 기본 진입점은 bash로 둡니다.
# ------------------------------------------------------------
ENTRYPOINT ["tini", "--", "/ros_entrypoint.sh"]
CMD ["bash"]