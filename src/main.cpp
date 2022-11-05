//
// Created by csl on 10/16/22.
//
#include "slam-scene-viewer/scene_viewer.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        SceneViewer viewer("../scene-shot");

        CubePlane plane(0.0f, 0.0f, -30.0f, 1.0f, 2.0f, 1.0f);
        auto planeIdName = viewer.AddCubePlane(plane, true, 0.5);
        auto featureIdName = viewer.AddFeatures(plane.GenerateFeatures(10, CubePlane::ALL), 6.0, 0.6);
        auto poseIdName = viewer.AddPose(plane.LtoW);

        Eigen::AngleAxisf r1(-M_PI_4, Eigen::Vector3f(0, 0, 1));
        Eigen::AngleAxisf r2(+M_PI_2, Eigen::Vector3f(1, 0, 0));
        auto rot = (r2 * r1).toRotationMatrix();
        auto CtoW = Posef(rot.inverse(), Eigen::Vector3f(2, 1, 1));
        auto cameraIdName = viewer.AddCamera(CtoW, Colour::Green());

        auto lineIdName = viewer.AddLine({2, 1, 1}, {0, 0, 0});

        viewer.RunMultiThread();
        LOG_VAR("Hello, world!")

        std::cin.get();
        viewer.RemoveEntities(planeIdName);
        std::cin.get();
        viewer.RemoveEntities(featureIdName);
        std::cin.get();
        viewer.RemoveEntities(poseIdName);
        std::cin.get();
        viewer.RemoveEntities(cameraIdName);
        std::cin.get();
        viewer.RemoveEntities(lineIdName);

    } catch (const std::exception &e) {
    }
    return 0;
}