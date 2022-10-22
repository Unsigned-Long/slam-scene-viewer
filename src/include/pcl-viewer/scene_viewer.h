//
// Created by csl on 10/22/22.
//

#ifndef PCL_VIEWER_SCENE_VIEWER_H
#define PCL_VIEWER_SCENE_VIEWER_H

#include "colour.hpp"
#include "pose.hpp"
#include "cube_plane.h"
#include "pcl/visualization/pcl_visualizer.h"

namespace ns_viewer {
    class SceneViewer {
    private:
        pcl::visualization::PCLVisualizer::Ptr _viewer;
    public:

        explicit SceneViewer(const Colour &background = Colour::White(), bool addOriginCoord = true)
                : _viewer(new pcl::visualization::PCLVisualizer("SceneViewer")) {
            _viewer->setBackgroundColor(background.r, background.g, background.b);
            // coordinates
            if (addOriginCoord) {
                _viewer->addCoordinateSystem(1.0, "Origin");
            }
            // shot
            using std::placeholders::_1;
            _viewer->registerKeyboardCallback(
                    [this](auto &&PH1) { KeyBoardCallBack(std::forward<decltype(PH1)>(PH1)); }
            );
        }

        void RunSingleThread(int time = 100);

        void AddCubePlane(const std::string &name, const CubePlane &plane, bool lineMode = false, float opacity = 1.0f);

        void AddFeatures(const std::string &name, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features,
                         float size = 6.0f, float opacity = 1.0f);

        void AddPose(const std::string &name, const Posef &pose, float size = 0.3);

    protected:

        void KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev);
    };

}

#endif //PCL_VIEWER_SCENE_VIEWER_H
