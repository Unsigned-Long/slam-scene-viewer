//
// Created by csl on 10/16/22.
//
#include "slam-scene-viewer/scene_viewer.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        SceneViewer viewer("../scene-shot");
        {
            CubePlane plane(0.0f, 0.0f, -30.0f, 1.0f, 2.0f, 1.0f);
            viewer.AddCubePlane(plane, true, 0.5);
            viewer.AddFeatures(plane.GenerateFeatures(10, CubePlane::ALL), 6.0, 0.6);
            viewer.AddPose(plane.LtoW);

            Eigen::AngleAxisf r1(-M_PI_4, Eigen::Vector3f(0, 0, 1));
            Eigen::AngleAxisf r2(+M_PI_2, Eigen::Vector3f(1, 0, 0));
            auto rot = (r2 * r1).toRotationMatrix();
            auto CtoW = Posef(rot.inverse(), Eigen::Vector3f(2, 1, 1));
            viewer.AddCamera(CtoW, Colour::Black());
        }

        // viewer.RunSingleThread();
        viewer.RunMultiThread();
        LOG_VAR("Hello, world!")

    } catch (const std::exception &e) {
    }
    return 0;
}