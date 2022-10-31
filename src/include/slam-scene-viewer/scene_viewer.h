//
// Created by csl on 10/22/22.
//

#ifndef SLAM_SCENE_VIEWER_SCENE_VIEWER_H
#define SLAM_SCENE_VIEWER_SCENE_VIEWER_H

#include <utility>

#include "slam-scene-viewer/colour.hpp"
#include "slam-scene-viewer/pose.hpp"
#include "slam-scene-viewer/cube_plane.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "thread"
#include "artwork/logger/logger.h"
#include "filesystem"

namespace ns_viewer {
    class SceneViewer {
    protected:
        pcl::visualization::PCLVisualizer::Ptr _viewer;
        std::shared_ptr<std::thread> _thread;
        const std::string _saveDir;

        static ColourWheel COLOUR_WHEEL;
        static std::size_t CUBE_PLANE_COUNT;
        static std::size_t FEATURE_COUNT;
        static std::size_t POSE_COUNT;
        static std::size_t CAMERA_COUNT;

    public:

        explicit SceneViewer(std::string sceneShotSaveDir = "", const Colour &background = Colour::White(),
                             bool addOriginCoord = true)
                : _viewer(new pcl::visualization::PCLVisualizer("SceneViewer")),
                  _thread(nullptr), _saveDir(std::move(sceneShotSaveDir)) {
            _viewer->setBackgroundColor(background.r, background.g, background.b);
            // coordinates
            if (addOriginCoord) {
                _viewer->addCoordinateSystem(1.0, "Origin");
            }
            if (std::filesystem::exists(_saveDir)) {
                // shot
                using std::placeholders::_1;
                _viewer->registerKeyboardCallback(
                        [this](auto &&PH1) { KeyBoardCallBack(std::forward<decltype(PH1)>(PH1)); }
                );
            }
        }

        virtual ~SceneViewer();

        void RunSingleThread(int time = 100);

        void RunMultiThread(int time = 100);

        void AddCubePlane(const CubePlane &plane, bool lineMode = false, float opacity = 1.0f);

        void AddFeatures(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features,
                         float size = 6.0f, float opacity = 1.0f);

        void AddPose(const Posef &LtoW, float size = 0.3);

        void AddCamera(const Posef &CtoW,
                       const Colour &color = COLOUR_WHEEL.GetUniqueColour(), float size = 0.3f);

    protected:

        void KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev);
    };

}

#endif //SLAM_SCENE_VIEWER_SCENE_VIEWER_H
