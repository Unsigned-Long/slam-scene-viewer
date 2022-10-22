//
// Created by csl on 10/16/22.
//
#include "pcl-viewer/scene_viewer.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        SceneViewer viewer;
        {
            CubePlane plane(0.0f, 0.0f, 45.0f, 2.0f, 3.0f, 0.0f);
            viewer.AddCubePlane("CubePlane", plane, true, 0.5);
            viewer.AddFeatures("Features", plane.GenerateFeatures(10, CubePlane::ALL), 6.0, 0.6);
            viewer.AddPose("Pose", plane.curToW);
        }

        viewer.RunSingleThread();

    } catch (const std::exception &e) {
    }
    return 0;
}