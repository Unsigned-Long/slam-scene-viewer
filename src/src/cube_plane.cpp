//
// Created by csl on 10/22/22.
//

#include "pcl-viewer/cube_plane.h"

namespace ns_viewer {
    ColourWheel CubePlane::COLOUR_WHEEL = ColourWheel(1.0f);
    std::default_random_engine CubePlane::RANDOM_ENGINE = {};

    CubePlane::CubePlane(float roll, float pitch, float yaw, float dx, float dy, float dz, float width,
                         float height, float thickness, const ns_viewer::Colour &color)
            : xSpan(width), ySpan(thickness), zSpan(height), color(color) {

        // compute ths plane pose
        auto y = Eigen::AngleAxisf(DEG_TO_RAD * yaw, Vector3f(0.0, 0.0, 1.0));
        auto p = Eigen::AngleAxisf(DEG_TO_RAD * pitch, Vector3f(1.0, 0.0, 0.0));
        auto r = Eigen::AngleAxisf(DEG_TO_RAD * roll, Vector3f(0.0, 1.0, 0.0));
        auto angleAxis = r * p * y;
        curToW = Posef(angleAxis.matrix(), Vector3f(dx, dy, dz));
    }

    bool CubePlane::IsFaceWithALL(int obj) {
        return (Face::ALL == (Face::ALL & obj));
    }

    bool CubePlane::IsFaceWithFRONT(int obj) {
        return (Face::FRONT == (Face::FRONT & obj));
    }

    bool CubePlane::IsFaceWithBACK(int obj) {
        return (Face::BACK == (Face::BACK & obj));
    }

    bool CubePlane::IsFaceWithLEFT(int obj) {
        return (Face::LEFT == (Face::LEFT & obj));
    }

    bool CubePlane::IsFaceWithRIGHT(int obj) {
        return (Face::RIGHT == (Face::RIGHT & obj));
    }

    bool CubePlane::IsFaceWithTOP(int obj) {
        return (Face::TOP == (Face::TOP & obj));
    }

    bool CubePlane::IsFaceWithBOTTOM(int obj) {
        return (Face::BOTTOM == (Face::BOTTOM & obj));
    }

    pcl::PointCloud<pcl::PointXY>::Ptr
    CubePlane::GenerateFeatures(int size, float xMin, float xMax, float yMin, float yMax) {
        // generate random feature
        RANDOM_ENGINE.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> ux(xMin, xMax);
        std::uniform_real_distribution<float> uy(yMin, yMax);

        pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
        cloud->resize(size);

        for (int i = 0; i < size; ++i) {
            auto &pt = cloud->at(i);
            pt.x = ux(RANDOM_ENGINE);
            pt.y = uy(RANDOM_ENGINE);
        }
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    CubePlane::GenerateFeatures(int size, int faceOption) const {
        if (faceOption == Face::ALL) {
            faceOption = Face::FRONT | Face::BACK | Face::LEFT | Face::RIGHT | Face::TOP | Face::BOTTOM;
        }
        // feature cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr feature = GenerateFeatures(size, Colour::Black(), faceOption);
        for (auto &p: feature->points) {
            const auto &color = COLOUR_WHEEL.GetUniqueColour();
            p.r = static_cast<std::uint8_t>(color.r * 255.0f);
            p.g = static_cast<std::uint8_t>(color.g * 255.0f);
            p.b = static_cast<std::uint8_t>(color.b * 255.0f);
        }

        return feature;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    CubePlane::GenerateFeatures(int size, const Colour &color, int faceOption) const {
        if (faceOption == Face::ALL) {
            faceOption = Face::FRONT | Face::BACK | Face::LEFT | Face::RIGHT | Face::TOP | Face::BOTTOM;
        }
        // feature cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr feature(new pcl::PointCloud<pcl::PointXYZRGBA>);

        auto trans = [this](pcl::PointXYZRGBA &p) {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            Eigen::Vector3f result = curToW.trans(pt);
            p.x = result(0), p.y = result(1), p.z = result(2);
        };

        if (IsFaceWithFRONT(faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = 0.5f * ySpan;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(color.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(color.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(color.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(color.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWithBACK(faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = -0.5f * ySpan;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(color.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(color.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(color.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(color.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWithLEFT(faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * ySpan, 0.5f * ySpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = -0.5f * xSpan;
                pt3d.y = pt2d.x;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(color.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(color.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(color.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(color.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWithRIGHT(faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * ySpan, 0.5f * ySpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = 0.5f * xSpan;
                pt3d.y = pt2d.x;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(color.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(color.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(color.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(color.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWithTOP(faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * ySpan, 0.5f * ySpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = pt2d.y;
                pt3d.z = 0.5f * zSpan;
                pt3d.r = static_cast<std::uint8_t>(color.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(color.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(color.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(color.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWithBOTTOM(faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * ySpan, 0.5f * ySpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = pt2d.y;
                pt3d.z = -0.5f * zSpan;
                pt3d.r = static_cast<std::uint8_t>(color.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(color.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(color.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(color.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        return feature;
    }

    CubePlane::CubePlane(float roll, float pitch, float yaw,
                         float dx, float dy, float dz,
                         float width, const Colour &color)
            : CubePlane(roll, pitch, yaw, dx, dy, dz, width, 2.0f, 0.1f, color) {}
}
