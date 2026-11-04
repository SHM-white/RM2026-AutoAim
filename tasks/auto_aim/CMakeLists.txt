cmake_minimum_required(VERSION 3.16)

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

if(ENABLE_LEGACY_CBOARD)
  target_sources(auto_aim PRIVATE multithread/commandgener.cpp)
endif()

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)