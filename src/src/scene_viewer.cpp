//
// Created by csl on 10/22/22.
//

#include "slam-scene-viewer/scene_viewer.h"

namespace ns_viewer {
    ColourWheel SceneViewer::COLOUR_WHEEL = ColourWheel(1.0f);
    std::size_t SceneViewer::CUBE_PLANE_COUNT = 0;
    std::size_t SceneViewer::FEATURE_COUNT = 0;
    std::size_t SceneViewer::POSE_COUNT = 0;
    std::size_t SceneViewer::CAMERA_COUNT = 0;

    void SceneViewer::KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev) {
        if (ev.isAltPressed()) {
            std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
            const std::string filename = _saveDir + "/" + std::to_string(curTimeStamp) + ".png";
            _viewer->saveScreenshot(filename);
            LOG_INFO("the scene shot is saved to path: '", filename, "'.");
        }
    }

    void SceneViewer::RunSingleThread(int time) {
        LOG_INFO("adjust the camera and press 'Alter' key to save the current scene.")
        while (!_viewer->wasStopped()) {
            // ms
            _viewer->spinOnce(time);
        }
    }

    void SceneViewer::RunMultiThread(int time) {
        LOG_INFO("adjust the camera and press 'Alter' key to save the current scene.")
        this->_thread = std::make_shared<std::thread>([this, time]() {
            while (!_viewer->wasStopped()) {
                // ms
                _viewer->spinOnce(time);
            }
        });
    }

    void SceneViewer::AddCubePlane(const CubePlane &plane, bool lineMode, float opacity) {
        const auto name = "CUBE-PLANE-" + std::to_string(CUBE_PLANE_COUNT++);
        const auto &localToWorld = plane.LtoW;
        const auto &color = plane.color;

        Eigen::Vector3f localInWorld = localToWorld.translation;
        Eigen::Quaternionf Q_localToWorld(localToWorld.quaternion());

        _viewer->addCube(localInWorld, Q_localToWorld, plane.xSpan, plane.ySpan, plane.zSpan, name);

        if (lineMode) {
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2.0, name
            );
        } else {
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, opacity, name
            );
        }

        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,
                color.r, color.g, color.b, name
        );
    }

    void SceneViewer::AddFeatures(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features,
                                  float size, float opacity) {
        const auto name = "FEATURE-" + std::to_string(FEATURE_COUNT++);

        for (auto &p: features->points) {
            p.a = static_cast<std::uint8_t>(opacity * 255.0f);
        }
        _viewer->addPointCloud(features, name);
        _viewer->setPointCloudRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, size, name
        );
    }

    void SceneViewer::AddPose(const Posef &LtoW, float size) {
        const auto name = "POSE-" + std::to_string(POSE_COUNT++);

        Eigen::Isometry3f localToWorld(LtoW.quaternion());
        localToWorld.pretranslate(LtoW.translation);

        _viewer->addCoordinateSystem(
                size, Eigen::Affine3f(localToWorld.affine()), name
        );
    }

    SceneViewer::~SceneViewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    void SceneViewer::AddCamera(const Posef &CtoW, const Colour &color, float size) {
        const auto name = "CAMERA-" + std::to_string(CAMERA_COUNT++);

        AddPose(CtoW, size);

        float left = -0.8f * size, right = 0.8f * size, top = -0.6f * size, bottom = 0.6f * size, front = 0.8f * size;

        pcl::PointXYZ leftBottom = {left, bottom, front};
        pcl::PointXYZ leftTop = {left, top, front};
        pcl::PointXYZ rightTop = {right, top, front};
        pcl::PointXYZ rightBottom = {right, bottom, front};
        pcl::PointXYZ center = {0.0, 0.0, 0.0};

        std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> data{
                {leftBottom,  rightBottom},
                {leftTop,     rightTop},
                {leftTop,     leftBottom},
                {rightTop,    rightBottom},
                {leftTop,     center},
                {leftBottom,  center},
                {rightTop,    center},
                {rightBottom, center},
        };
        for (int i = 0; i < data.size(); ++i) {
            auto &[p1, p2] = data.at(i);
            const std::string curName = name + "-" + std::to_string(i);
            {
                auto r1 = CtoW.trans(Eigen::Vector3f(p1.x, p1.y, p1.z));
                p1.x = r1(0), p1.y = r1(1), p1.z = r1(2);
            }
            {
                auto r2 = CtoW.trans(Eigen::Vector3f(p2.x, p2.y, p2.z));
                p2.x = r2(0), p2.y = r2(1), p2.z = r2(2);
            }
            _viewer->addLine(p1, p2, color.r, color.g, color.b, curName);
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, size * 4.0, curName
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, curName
            );
        }
    }

}

