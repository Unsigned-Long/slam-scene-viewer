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
        const std::string _saveDir;
        const Colour _bgc;
        static ColourWheel COLOUR_WHEEL;
        const bool _addOriginCoord;
        std::mutex _mt;

    private:
        std::shared_ptr<std::thread> _thread;

        static std::size_t CUBE_PLANE_COUNT;
        static std::size_t FEATURE_COUNT;
        static std::size_t POSE_COUNT;
        static std::size_t CAMERA_COUNT;

        static std::size_t LINE_COUNT;
        const static std::string SHAPE_PREFIX;
        const static std::string POINT_CLOUD_PREFIX;
        const static std::string COORD_PREFIX;

    public:

        explicit SceneViewer(std::string sceneShotSaveDir = "", const Colour &background = Colour::White(),
                             bool addOriginCoord = true)
                : _viewer(new pcl::visualization::PCLVisualizer("SceneViewer")),
                  _thread(nullptr), _saveDir(std::move(sceneShotSaveDir)),
                  _bgc(background), _addOriginCoord(addOriginCoord) {
            InitSceneViewer();
        }

        virtual ~SceneViewer();

        pcl::visualization::PCLVisualizer::Ptr GetViewer();

        static void SetColourWheel(const ColourWheel &colourWheel);

        static ColourWheel &GetColourWheel();

        void SetWindowName(const std::string &name);

        SceneViewer &operator()(const std::string &name);

        std::mutex &GetMutex();

        inline void Lock();

        inline void UnLock();

    public:

        void RunSingleThread(int time = 100);

        void RunMultiThread(int time = 100);

        std::vector<std::string> AddCubePlane(const CubePlane &plane, bool lineMode = false, float opacity = 1.0f);

        std::vector<std::string>
        AddFeatures(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features, float size = 6.0f);

        std::vector<std::string> AddPose(const Posef &LtoW, float size = 0.3);

        std::vector<std::string> AddCamera(const Posef &CtoW,
                                           const Colour &color = COLOUR_WHEEL.GetUniqueColour(), float size = 0.3f);

        std::vector<std::string> AddLine(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2,
                                         const Colour &color = COLOUR_WHEEL.GetUniqueColour(), float size = 2.0f);

        void RemoveEntities(const std::string &name) {
            if (name.empty()) {
                return;
            } else {
                if (name.front() == SHAPE_PREFIX.front()) {
                    _viewer->removeShape(name);
                } else if (name.front() == COORD_PREFIX.front()) {
                    _viewer->removeCoordinateSystem(name);
                } else if (name.front() == POINT_CLOUD_PREFIX.front()) {
                    _viewer->removePointCloud(name);
                }
            }
        }

        void RemoveEntities(const std::vector<std::string> &names) {
            for (const auto &item: names) {
                RemoveEntities(item);
            }
        }

        void RemoveEntities(const std::vector<std::vector<std::string>> &names) {
            for (const auto &items: names) {
                RemoveEntities(items);
            }
        }

        void RemoveEntities();

    protected:

        void KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev);

        void InitSceneViewer();

        static void AppendNames(std::vector<std::string> &names, const std::string &name);

        static void AppendNames(std::vector<std::string> &names, const std::vector<std::string> &newNames);

        static inline std::string GetShapeName(const std::string &desc);

        static inline std::string GetPointCloudName(const std::string &desc);

        static inline std::string GetCoordName(const std::string &desc);
    };

}

#endif //SLAM_SCENE_VIEWER_SCENE_VIEWER_H
