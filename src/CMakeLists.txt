set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(PCL REQUIRED)
find_package(Eigen3)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(
        ${PROJECT_NAME} SHARED
        ${SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME} PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME} PUBLIC
        ${PCL_LIBRARIES}
        pthread
)

add_executable(${PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${SRC_FILES})

target_link_libraries(
        ${PROJECT_NAME}_prog
        PRIVATE
        ${PROJECT_NAME}
)
