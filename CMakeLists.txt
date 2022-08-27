cmake_minimum_required(VERSION 3.0.2)
project(Project0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++17 -pthread")

include_directories(
  inc
  inc/exported 
  inc/realize
  inc/realize/src# 添加vroom
  inc/exported/behaviac_generated/types/internal
  src
)

link_directories(
  libs
)

# 依赖库为OpenCV
find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
# include_directories(/usr/include/eigen3)
include_directories(OpenCV_INCLUDE_DIRS)

# 添加json支持
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_LIBRARIES})

aux_source_directory(src/.  SRC_LIST1)
aux_source_directory(inc/exported/behaviac_generated/types/internal  SRC_LIST2)

add_executable(project0 ${SRC_LIST1} ${SRC_LIST2})
target_link_libraries(project0  pthread ${OpenCV_LIBS} ${JSONCPP_LIBRARIES}
${CMAKE_CURRENT_SOURCE_DIR}/libs/libbehaviac_gcc_release.a)
target_link_libraries(project0 
        vroom -ldl
        ssl -ldl
        crypto -ldl
        Threads::Threads
        
        )

