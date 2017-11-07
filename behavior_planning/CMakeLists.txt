project(Behavior_Planning)

set(SOURCE_FILES
        src/main.cpp
        src/vehicle.cpp
        src/road.cpp
        src/snapshot.cpp
        src/costfunctions.cpp)
add_executable(Behavior_Planning ${SOURCE_FILES})
