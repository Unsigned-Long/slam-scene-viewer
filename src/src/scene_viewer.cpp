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
    std::size_t SceneViewer::LINE_COUNT = 0;
    const std::string SceneViewer::SHAPE_PREFIX = "S";
    const std::string SceneViewer::POINT_CLOUD_PREFIX = "P";
    const std::string SceneViewer::COORD_PREFIX = "C";

    void SceneViewer::KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev) {
        if (ev.isAltPressed()) {
            std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
            const std::string filename = _saveDir + "/" + std::to_string(curTimeStamp) + ".png";
            _viewer->saveScreenshot(filename);
            LOG_INFO("the scene shot is saved to path: '", filename, "'.")
        }
    }

    void SceneViewer::RunSingleThread(int time) {
        if (std::filesystem::exists(_saveDir)) {
            LOG_INFO("adjust the camera and press 'Alter' key to save the current scene.")
        }
        while (!_viewer->wasStopped()) {
            // ms
            _viewer->spinOnce(time);
        }
        _viewer.reset(new pcl::visualization::PCLVisualizer("SceneViewer"));
        InitSceneViewer();
    }

    void SceneViewer::RunMultiThread(int time) {
        if (std::filesystem::exists(_saveDir)) {
            LOG_INFO("adjust the camera and press 'Alter' key to save the current scene.")
        }
        this->_thread = std::make_shared<std::thread>([this, time]() {
            while (!_viewer->wasStopped()) {
                Lock();
                _viewer->spinOnce(1);
                UnLock();

                // ms
                std::this_thread::sleep_for(std::chrono::milliseconds(time));
            }
        });
    }

    std::vector<std::string> SceneViewer::AddCubePlane(const CubePlane &plane, bool lineMode, float opacity) {
        std::vector<std::string> names;

        const auto name = GetShapeName("CUBE-PLANE-" + std::to_string(CUBE_PLANE_COUNT++));
        AppendNames(names, name);
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
        return names;
    }

    std::vector<std::string>
    SceneViewer::AddFeatures(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features, float size) {
        std::vector<std::string> names;

        const auto name = GetPointCloudName("FEATURE-" + std::to_string(FEATURE_COUNT++));
        AppendNames(names, name);

        _viewer->addPointCloud(features, name);
        _viewer->setPointCloudRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, size, name
        );
        return names;
    }

    std::vector<std::string> SceneViewer::AddPose(const Posef &LtoW, float size) {
        std::vector<std::string> names;

        const auto name = GetCoordName("POSE-" + std::to_string(POSE_COUNT++));
        AppendNames(names, name);


        Eigen::Isometry3f localToWorld(LtoW.quaternion());
        localToWorld.pretranslate(LtoW.translation);

        _viewer->addCoordinateSystem(
                size, Eigen::Affine3f(localToWorld.affine()), name
        );

        return names;
    }

    SceneViewer::~SceneViewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    std::vector<std::string> SceneViewer::AddCamera(const Posef &CtoW, const Colour &color, float size) {
        std::vector<std::string> names;

        const auto name = "CAMERA-" + std::to_string(CAMERA_COUNT++);
        AppendNames(names, AddPose(CtoW, size));

        float left = -0.8f * size, right = 0.8f * size, top = -0.6f * size, bottom = 0.6f * size, front = 0.8f * size;

        pcl::PointXYZ leftBottom = {left, bottom, front};
        pcl::PointXYZ leftTop = {left, top, front};
        pcl::PointXYZ rightTop = {right, top, front};
        pcl::PointXYZ rightBottom = {right, bottom, front};
        pcl::PointXYZ center = {0.0,
                                0.0,
                                0.0};

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
        for (auto &[p1, p2]: data) {

            auto r1 = CtoW.trans(Eigen::Vector3f(p1.x, p1.y, p1.z));
            p1.x = r1(0), p1.y = r1(1), p1.z = r1(2);

            auto r2 = CtoW.trans(Eigen::Vector3f(p2.x, p2.y, p2.z));
            p2.x = r2(0), p2.y = r2(1), p2.z = r2(2);

            AppendNames(names, AddLine(p1, p2, color, size * 4.0f));
        }

        return names;
    }

    pcl::visualization::PCLVisualizer::Ptr SceneViewer::GetViewer() {
        return _viewer;
    }

    void SceneViewer::SetColourWheel(const ColourWheel &colourWheel) {
        SceneViewer::COLOUR_WHEEL = colourWheel;
    }

    void SceneViewer::SetWindowName(const std::string &name) {
        _viewer->setWindowName(name);
    }

    SceneViewer &SceneViewer::operator()(const std::string &name) {
        this->SetWindowName(name);
        return *this;
    }

    void SceneViewer::RemoveEntities() {
        _viewer->removeAllShapes();
        _viewer->removeAllPointClouds();
        _viewer->removeAllCoordinateSystems();
    }

    void SceneViewer::InitSceneViewer() {
        _viewer->setBackgroundColor(_bgc.r, _bgc.g, _bgc.b);
        // coordinates
        if (_addOriginCoord) {
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

    ColourWheel &SceneViewer::GetColourWheel() {
        return COLOUR_WHEEL;
    }

    std::vector<std::string>
    SceneViewer::AddLine(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const Colour &color, float size) {
        std::vector<std::string> names;

        const auto name = GetShapeName("LINE-" + std::to_string(LINE_COUNT++));
        AppendNames(names, name);

        _viewer->addLine(p1, p2, color.r, color.g, color.b, name);
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, size, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );
        return names;
    }

    void SceneViewer::AppendNames(std::vector<std::string> &names, const std::string &name) {
        names.push_back(name);
    }

    void SceneViewer::AppendNames(std::vector<std::string> &names, const std::vector<std::string> &newNames) {
        names.resize(names.size() + newNames.size());
        std::copy_n(newNames.cbegin(), newNames.size(), names.end() - static_cast<int>(newNames.size()));
    }

    std::string SceneViewer::GetShapeName(const std::string &desc) {
        return SHAPE_PREFIX + '-' + desc;
    }

    std::string SceneViewer::GetPointCloudName(const std::string &desc) {
        return POINT_CLOUD_PREFIX + '-' + desc;
    }

    std::string SceneViewer::GetCoordName(const std::string &desc) {
        return COORD_PREFIX + '-' + desc;
    }

    std::mutex &SceneViewer::GetMutex() {
        return _mt;
    }

    void SceneViewer::Lock() {
        _mt.lock();
    }

    void SceneViewer::UnLock() {
        _mt.unlock();
    }

}

