cmake_minimum_required(VERSION 3.15)
project(MovementTracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 디버깅: 경로 출력
message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "VCPKG_TARGET_TRIPLET: ${VCPKG_TARGET_TRIPLET}")

# 수동으로 vcpkg 경로 설정
set(CMAKE_PREFIX_PATH "C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows" ${CMAKE_PREFIX_PATH})

# MongoDB 드라이버 찾기 (mongo-cxx-driver용)
find_package(mongocxx CONFIG REQUIRED)
find_package(bsoncxx CONFIG REQUIRED)

# 기존 실행파일 (읽기 전용)
add_executable(movement_tracker movement_tracker.cpp)
target_link_libraries(movement_tracker PRIVATE 
    mongo::mongocxx_shared
    mongo::bsoncxx_shared
)

# Windows에서 HTTP 요청을 위한 WinINet 라이브러리 추가
if(WIN32)
    target_link_libraries(movement_tracker PRIVATE wininet)
endif()
