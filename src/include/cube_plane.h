//
// Created by csl on 10/22/22.
//

#ifndef PCL_VIEWER_CUBE_PLANE_H
#define PCL_VIEWER_CUBE_PLANE_H

#include "colour.hpp"
#include "pose.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "random"
#include "chrono"

namespace ns_viewer {
    struct CubePlane {
    public:
        enum Face : int {
            /**
             * @brief options
             */
            ALL = 1 << 0,
            FRONT = 1 << 1,
            BACK = 1 << 2,
            LEFT = 1 << 3,
            RIGHT = 1 << 4,
            TOP = 1 << 5,
            BOTTOM = 1 << 6
        };

        static bool IsFaceWithALL(int obj);

        static bool IsFaceWithFRONT(int obj);

        static bool IsFaceWithBACK(int obj);

        static bool IsFaceWithLEFT(int obj);

        static bool IsFaceWithRIGHT(int obj);

        static bool IsFaceWithTOP(int obj);

        static bool IsFaceWithBOTTOM(int obj);

    protected:
        static ColourWheel COLOUR_WHEEL;
        static std::default_random_engine RANDOM_ENGINE;

    public:
        Posef WtoL;
        Posef LtoW;
        float xSpan;
        float ySpan;
        float zSpan;
        Colour color;

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz,
                  float width = 1.0f, float height = 2.0f, float thickness = 0.1f,
                  const Colour &color = CubePlane::COLOUR_WHEEL.GetUniqueColour());

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz, float width,
                  const Colour &color);

    public:
        [[nodiscard]] pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        GenerateFeatures(int size, const Colour &colour, int faceOption = Face::ALL) const;

        [[nodiscard]] pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        GenerateFeatures(int size, int faceOption = Face::ALL) const;

    protected:
        [[nodiscard]] static pcl::PointCloud<pcl::PointXY>::Ptr
        GenerateFeatures(int size, float xMin, float xMax, float yMin, float yMax);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //PCL_VIEWER_CUBE_PLANE_H
