add_library(slamrgbd SHARED
        camera.cpp
        frame.cpp
        mappoint.cpp
        map.cpp
        config.cpp
        visual_odometry.cpp)

target_link_libraries(slamrgbd ${THIRD_PARTY_LIBS})