#pragma once

#include <eigen3/Eigen/Core>

#include "pointcloud.hpp"

/* Collection of static methods in order to create point clouds for visualization purposes */

namespace PointCloudUtils
{
    /* Add a frustum to an existing cloud.
    * The initial frustum is centered at (0,0,0), orientated along the Z axis.
    * The position of top left and bottom right corners are the 2d coordinates of the rectangle where the frustum intersects the plane Z = 1
    * It is then rotated using the quaternion,
    * and finally moved to a new center.
    */

    void addFrustum(PointCloud& cloud,
                    const Eigen::Vector3d& center,
                    const Eigen::Quaterniond& q,
                    const Eigen::Vector2d& leftTopCorner,
                    const Eigen::Vector2d& rightBottomCorner,
                    const Eigen::Vector3i& color,
                    float lineLength = 1.0f,
                    int nPointsPerLine = 1000
                   );

    // Add a sphere to an existing cloud. Points are randomly sampled on the sphere.
    void addSphere(PointCloud& cloud,
                   const Eigen::Vector3d& center,
                   const Eigen::Vector3i& color,
                   float sphereRadius,
                   int nPoints
                   );

    bool computeCentroid(const PointCloud& cloud, Eigen::Vector3d& centroid);

}
