#pragma once

#include <vector>

#include "point.hpp"
#include "camera.hpp"
#include "nvmreader.hpp"
#include "detectionsreader.hpp"
#include "pointcloud.hpp"


class BBoxPointsExtractor
{
public:
    BBoxPointsExtractor(const PointCloud& points,
                        const std::vector<Camera>& cameras,
                        const Intrinsics& intrinsics,
                        const Detections& detections
                        );

    void exportFrustumAssociatedToFrame(int frameIdx) const;
    PointCloud createPointCloudWithCameraCenters() const;
    PointCloud createPointCloudWithCameraDirections() const;
    PointCloud createPointCloudWithFrustum() const;

    PointCloud getPointsAssociatedWithDetectionOnFrame(int i, Detection& detection) const;

private:
    Eigen::Vector2d convertCoordSys(const Eigen::Vector2d& v) const;
    // Order: top left, top right, bottom right, bottom left
    std::array<Eigen::Vector2d, 4> convertBBX(const Detection& detection) const;


private:
    const PointCloud& m_Cloud;
    const std::vector<Camera>& m_Cameras;
    const Intrinsics& m_Intrinsics;
    const Detections& m_Detections;

};
