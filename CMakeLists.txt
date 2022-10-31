cmake_minimum_required(VERSION 3.10)

project(slam_scene_viewer VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# for install

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Export package for use from the build tree
set(SLAM_SCENE_VIEWER_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/slam_scene_viewer/cmake)

set_target_properties(slam_scene_viewer PROPERTIES EXPORT_NAME SlamSceneViewer)

install(
        TARGETS slam_scene_viewer
        EXPORT SlamSceneViewerTargets
)
install(
        EXPORT SlamSceneViewerTargets
        NAMESPACE SlamSceneViewer::
        DESTINATION ${SLAM_SCENE_VIEWER_CMAKE_EXPORT_DIR}
)

export(
        TARGETS slam_scene_viewer
        NAMESPACE SlamSceneViewer::
        FILE SlamSceneViewerTargets.cmake
)
export(
        PACKAGE SlamSceneViewer
)

configure_package_config_file(
        SlamSceneViewerConfig.cmake.in
        ${CMAKE_BINARY_DIR}/SlamSceneViewerConfig.cmake
        INSTALL_DESTINATION ${SLAM_SCENE_VIEWER_CMAKE_EXPORT_DIR}
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Write version to file
write_basic_package_version_file(
        SlamSceneViewerConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
)

# Install cmake targets
install(
        FILES ${CMAKE_BINARY_DIR}/SlamSceneViewerConfig.cmake
        ${CMAKE_BINARY_DIR}/SlamSceneViewerConfigVersion.cmake
        DESTINATION ${SLAM_SCENE_VIEWER_CMAKE_EXPORT_DIR}
)

# Install header files

install(
        DIRECTORY ${CMAKE_SOURCE_DIR}/src/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/slam_scene_viewer
)