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

        const Colour _bgc;
        const bool _addOriginCoord;

    public:

        explicit SceneViewer(std::string sceneShotSaveDir = "", const Colour &background = Colour::White(),
                             bool addOriginCoord = true)
                : _viewer(new pcl::visualization::PCLVisualizer("SceneViewer")),
                  _thread(nullptr), _saveDir(std::move(sceneShotSaveDir)),
                  _bgc(background), _addOriginCoord(addOriginCoord) {
            InitSceneViewer();
        }

        virtual ~SceneViewer();

        [[nodiscard]] const auto &GetViewer() const;

        static void SetColourWheel(const ColourWheel &colourWheel);

        static ColourWheel &GetColourWheel();

        void SetWindowName(const std::string &name);

        SceneViewer &operator()(const std::string &name);

    public:

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

        void RemoveAllEntities();

        void InitSceneViewer();
    };

}

#endif //SLAM_SCENE_VIEWER_SCENE_VIEWER_H
