#!/bin/bash

# 1. 화면 권한 허용
# OpenCV imshow, Gazebo, RViz 같은 GUI 창을 컨테이너에서 띄우기 위함
xhost +local:docker

# 2. 컨테이너 이름 정의
# docker-compose.yaml의 container_name과 반드시 같아야 함
CONTAINER_NAME="asec_mid_container"

# 4. 컨테이너 실행
# 없으면 만들고, 꺼져 있으면 켬
echo "🚀 Starting ASEC Mid Drone Container..."
docker compose up -d

# 5. 컨테이너 접속
echo "🔌 Entering Workspace..."
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    docker exec -it $CONTAINER_NAME bash
else
    echo "❌ Error: Container is not running!"
    echo "Check logs: docker logs $CONTAINER_NAME"
fi