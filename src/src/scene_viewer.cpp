//
// Created by csl on 10/22/22.
//

#include "pcl-viewer/scene_viewer.h"

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon1(new pcl::PointCloud<pcl::PointXYZ>);
        polygon1->push_back(leftBottom);
        polygon1->push_back(leftTop);
        polygon1->push_back(rightTop);
        polygon1->push_back(rightBottom);

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon2(new pcl::PointCloud<pcl::PointXYZ>);
        polygon2->push_back(center);
        polygon2->push_back(leftBottom);
        polygon2->push_back(leftTop);

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon3(new pcl::PointCloud<pcl::PointXYZ>);
        polygon3->push_back(center);
        polygon3->push_back(rightBottom);
        polygon3->push_back(rightTop);

        std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> data{
                {"FRONT", polygon1},
                {"LEFT",  polygon2},
                {"RIGHT", polygon3}
        };

        for (const auto &item: data) {
            for (auto &p: (*item.second)) {
                auto result = CtoW.trans(Eigen::Vector3f(p.x, p.y, p.z));
                p.x = result(0), p.y = result(1), p.z = result(2);
            }
            _viewer->addPolygon<pcl::PointXYZ>(item.second, color.r, color.g, color.b, name + "-" + item.first);
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, size * 4.0,
                    name + "-" + item.first
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY,
                    color.a, name + "-" + item.first
            );
        }
    }

}

