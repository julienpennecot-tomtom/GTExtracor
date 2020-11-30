#include "bboxpointsextractor.hpp"

#include <eigen3/Eigen/Core>
#include <random>
#include <fstream>
#include <iostream>

BBoxPointsExtractor::BBoxPointsExtractor(const PointCloud& cloud,
                                         const std::vector<Camera>& cameras,
                                         const Intrinsics& intrinsics,
                                         const Detections& detections,
                                         const std::vector<int>& pointIndexForFrame
                                         )
    : m_Cloud(cloud),
      m_Cameras(cameras),
      m_Intrinsics(intrinsics),
      m_Detections(detections),
      m_PointIndexForFrame(pointIndexForFrame)
{

}

inline int BBoxPointsExtractor::getLastPointForFrame(int i) const
{
    if (i == m_Cameras.size()) {
        std::cout << "Trying to access the last frame -> last point" << std::endl;
        return m_Cloud.size();
    } else {
        return m_PointIndexForFrame[i+1];
    }
}

// This method intersects a ray going through the camera center and a point in the sensor (in pixel coordinates) with a plane
// perpendicular to the principal axis at distance f of the center.
Eigen::Vector3d BBoxPointsExtractor::project2dPointIn3dSpace(const Eigen::Vector2d& p) const
{
    float f = m_Intrinsics.fx;
    float cx = m_Intrinsics.cx;
    float cy = m_Intrinsics.cy;
    return Eigen::Vector3d(p[0] - cx, p[1] - cy, f);
}

std::vector<PointCloud> BBoxPointsExtractor::extractDetections() const{
    return extractDetections(0, m_Detections.size());
}

std::vector<PointCloud> BBoxPointsExtractor::extractDetections(int first, int last) const
{
    std::vector<PointCloud> signs;
    for (int i = first; i < last; i++) {
        std::cout << "Starting working on frame " << i << std::endl;
        Camera camera = m_Cameras[i];
        std::string frameKey = camera.key();
        // When trying to access an unordered map with the [] operator, if the key doesn't exist if creates a default object.
        // So because m_Detections is const, we cannot access it with []

        const auto& frameDetectionsIt = m_Detections.find(frameKey);
        // Skip this frame key if it could not be found
        if (frameDetectionsIt == m_Detections.end()) {
            std::cout << "WARNING: Could not find detection corresponding to key: " << frameKey << std::endl;
            continue;
        }
        // Loop over the detections in a frame.
        for (const auto& detection : frameDetectionsIt->second) {
            int signID = detection.classid;

            // Create new empty point clouds for each index up to signID so that we can access it with the [] operator
            while (signs.size() <= signID) {
                PointCloud cloud;
                signs.push_back(cloud);
            }

            const PointCloud& detectedSignPoints = getPointsAssociatedWithDetectionOnFrame(i, detection);
            signs[signID].add(detectedSignPoints);
        }
    }
    return signs;
}


PointCloud BBoxPointsExtractor::getPointsAssociatedWithDetectionOnFrame(int i , const Detection& detection) const
{
    float f = m_Intrinsics.fx;
    float cx = m_Intrinsics.cx;
    float cy = m_Intrinsics.cy;
    const Camera& camera = m_Cameras[i];
    Eigen::Vector3d center(camera.center());
    Eigen::Quaterniond q = camera.quaternion().inverse();

    Eigen::Vector3d tl_camera = project2dPointIn3dSpace(Eigen::Vector2d(detection.x, detection.y));
    Eigen::Vector3d tr_camera = project2dPointIn3dSpace(Eigen::Vector2d(detection.x + detection.w, detection.y));
    Eigen::Vector3d br_camera = project2dPointIn3dSpace(Eigen::Vector2d(detection.x + detection.w, detection.y + detection.h));
    Eigen::Vector3d bl_camera = project2dPointIn3dSpace(Eigen::Vector2d(detection.x, detection.y + detection.h));

    Eigen::Vector3d tl = center + (q * tl_camera).normalized();
    Eigen::Vector3d tr = center + (q * tr_camera).normalized();
    Eigen::Vector3d br = center + (q * br_camera).normalized();
    Eigen::Vector3d bl = center + (q * bl_camera).normalized();

    Eigen::Vector3d leftPlaneNormal = ((tl - center).cross(bl - center)).normalized();
    float dLeftPlane = leftPlaneNormal.dot(center);

    Eigen::Vector3d topPlaneNormal = ((tr - center).cross(tl - center)).normalized();
    float dTopPlane = topPlaneNormal.dot(center);

    Eigen::Vector3d rightPlaneNormal = ((br - center).cross(tr - center)).normalized();
    float dRightPlane = rightPlaneNormal.dot(center);

    Eigen::Vector3d bottomPlaneNormal = ((bl - center).cross(br - center)).normalized();
    float dBottomPlane = bottomPlaneNormal.dot(center);

    // Now that we have the equation of the left plane of the bounding box, we filter points to get only
    // the points on the right of this plane.
    PointCloud cloud;

    for (int idx = getFirstPointForFrame(i); idx < getLastPointForFrame(i); idx++)
    {
        Point point = m_Cloud.points()[idx];
//        std::cout << "point[" << idx << "]: " << point << std::endl;
        assert(point.imIndex() == i );

        if ((point.pos()).dot(leftPlaneNormal) < dLeftPlane
            && (point.pos()).dot(topPlaneNormal) < dTopPlane
            && (point.pos()).dot(rightPlaneNormal) < dRightPlane
            && (point.pos()).dot(bottomPlaneNormal) < dBottomPlane
           )
        {
            cloud.push_back(point);
        }
    }
    return cloud;
}
