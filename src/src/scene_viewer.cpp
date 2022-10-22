//
// Created by csl on 10/22/22.
//

#include "pcl-viewer/scene_viewer.h"

namespace ns_viewer {
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

    void SceneViewer::AddCubePlane(const std::string &name, const CubePlane &plane, bool lineMode, float opacity) {
        const auto &pose = plane.curToW;
        const auto &color = plane.color;

        Eigen::Vector3f position = pose.translation;
        Eigen::Quaternionf quat(pose.quaternion());

        _viewer->addCube(position, quat, plane.xSpan, plane.ySpan, plane.zSpan, name);

        if (lineMode) {
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 4.0, name
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

    void SceneViewer::AddFeatures(const std::string &name,
                                  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features,
                                  float size, float opacity) {
        for (auto &p: features->points) {
            p.a = static_cast<std::uint8_t>(opacity * 255.0f);
        }
        _viewer->addPointCloud(features, name);
        _viewer->setPointCloudRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, size, name
        );
    }

    void SceneViewer::AddPose(const std::string &name, const Posef &pose, float size) {
        Eigen::Isometry3f curToWorld(pose.quaternion());
        curToWorld.pretranslate(pose.translation);

        _viewer->addCoordinateSystem(
                size, Eigen::Affine3f(curToWorld.affine()), name
        );
    }

    SceneViewer::~SceneViewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

}

