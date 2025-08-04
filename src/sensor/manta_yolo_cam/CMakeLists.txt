cmake_minimum_required(VERSION 3.8)
project(manta_yolo_cam)

# ────────────── 공통 설정 ──────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)          # onnxruntime 권장

# ────────────── ROS 2 / OpenCV ─────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ────────────── 경로 변수 정리 ──────────
# ① Vimba X SDK
set(VIMBA_ROOT   "/home/acca/VimbaX_2025-2")
set(VIMBA_INC    "${VIMBA_ROOT}/api/include")          # VmbC/, VmbCPP/ 포함
set(VIMBA_LIB    "${VIMBA_ROOT}/api/lib")              # libVmbCPP.so, libVmbC.so

# ② YOLOs-CPP + 내장 ONNX Runtime 1.20.1
set(YOLOS_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/../YOLOs-CPP")
set(ORT_INC      "${YOLOS_DIR}/onnxruntime-linux-x64-1.20.1/include")
set(ORT_LIB      "${YOLOS_DIR}/onnxruntime-linux-x64-1.20.1/lib")

# ────────────── 실행 파일 ───────────────
add_executable(${PROJECT_NAME}
  src/camera_inference_node.cpp        # 노드 cpp (위치 그대로라면 OK)
)

# ────────────── 헤더 / 라이브러리 경로 연결 ──────────
target_include_directories(${PROJECT_NAME} PUBLIC
  ${VIMBA_INC}
  ${VIMBA_INC}/VmbCPP
  ${YOLOS_DIR}/include
  ${ORT_INC}
)

target_link_directories(${PROJECT_NAME} PUBLIC
  ${VIMBA_LIB}
  ${ORT_LIB}
)

# ────────────── 의존성 & 링크 ──────────
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  onnxruntime            # ${ORT_LIB}/libonnxruntime.so
  VmbCPP
  VmbC
)

# ────────────── 설치 / 패키지 export ──
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
