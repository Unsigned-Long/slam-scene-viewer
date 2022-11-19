//
// Created by csl on 10/16/22.
//
#include "slam-scene-viewer/scene_viewer.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        SceneViewer viewer("../scene-shot");

        CubePlane plane(0.0f, 0.0f, -30.0f, 1.0f, 2.0f, 1.0f);
        viewer.AddCubePlane(plane, true);
        viewer.AddFeatures(plane.GenerateFeatures(10, CubePlane::ALL, 1.0f), 6.0f);
        viewer.AddPose(plane.LtoW);

        Eigen::AngleAxisf r1(-M_PI_4, Eigen::Vector3f(0, 0, 1));
        Eigen::AngleAxisf r2(+M_PI_2, Eigen::Vector3f(1, 0, 0));
        auto rot = (r2 * r1).toRotationMatrix();
        auto CtoW = Posef(rot.inverse(), Eigen::Vector3f(2, 1, 1));
        auto LtoW = Posef(rot.inverse(), Eigen::Vector3f(2, 2, 1));
        auto ItoW = Posef(rot.inverse(), Eigen::Vector3f(2, 2, 2));

        viewer.AddCamera(CtoW, Colour::Green());
        viewer.AddLiDAR(LtoW, ns_viewer::Colour::Red());
        viewer.AddIMU(ItoW, ns_viewer::Colour::Blue());

        viewer.AddLine({1, 2, 1}, {0, 0, 0});
        viewer.AddArrow(
                {CtoW.translation(0), CtoW.translation(1), CtoW.translation(2)},
                {ItoW.translation(0), ItoW.translation(1), ItoW.translation(2)},
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.5f)
        );
        viewer.AddArrow(
                {LtoW.translation(0), LtoW.translation(1), LtoW.translation(2)},
                {ItoW.translation(0), ItoW.translation(1), ItoW.translation(2)},
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.5f)
        );

        viewer.RunMultiThread();
        std::cout << "hello, world!" << std::endl;

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}