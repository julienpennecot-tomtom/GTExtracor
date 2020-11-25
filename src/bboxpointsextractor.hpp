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
                        const Detections& detections,
                        const std::vector<int>& pointIndexForFrame
                        );

//    void exportFrustumAssociatedToFrame(int frameIdx) const;

    std::vector<PointCloud> extractDetections() const;

    PointCloud getPointsAssociatedWithDetectionOnFrame(int i, const Detection& detection) const;
//    PointCloud createPointCloudCenterAxisFrustum(int cameraIdx) const;

//    void addFrustum(PointCloud& cloud,
//                    const Eigen::Vector3d& center,
//                    const Eigen::Quaterniond& q,
//                    float left,
//                    float right,
//                    float bottom,
//                    float top,
//                    const Eigen::Vector3i& color
//                   ) const;

//    void addSphere(PointCloud& cloud,
//                   const Eigen::Vector3d& center,
//                   const Eigen::Vector3i& color,
//                   float sphereRadius,
//                   int nPoints
//                   ) const;

private:
    Eigen::Vector2d convertCoordSys(const Eigen::Vector2d& v) const;
    // Order: top left, top right, bottom right, bottom left
    Eigen::Vector3d project2dPointIn3dSpace(const Eigen::Vector2d& p) const;
    inline int getFirstPointForFrame(int i) const {return m_PointIndexForFrame[i];}
    int getLastPointForFrame(int i) const;


private:
    const PointCloud& m_Cloud;
    const std::vector<Camera>& m_Cameras;
    const Intrinsics& m_Intrinsics;
    const Detections& m_Detections;
    const std::vector<int>& m_PointIndexForFrame;

};
