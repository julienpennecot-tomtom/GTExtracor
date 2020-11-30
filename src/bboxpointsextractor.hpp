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

    std::vector<PointCloud> extractDetections() const;
    std::vector<PointCloud> extractDetections(int first, int last) const;
    PointCloud getPointsAssociatedWithDetectionOnFrame(int i, const Detection& detection) const;

private:
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
