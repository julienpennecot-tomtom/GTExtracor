#include "bboxpointsextractor.hpp"

#include <eigen3/Eigen/Core>
#include <random>
#include <fstream>
#include <iostream>

BBoxPointsExtractor::BBoxPointsExtractor(const PointCloud& cloud,
                                         const std::vector<Camera>& cameras,
                                         const Intrinsics& intrinsics,
                                         const Detections& detections)
    : m_Cloud(cloud),
      m_Cameras(cameras),
      m_Intrinsics(intrinsics),
      m_Detections(detections)
{

}

Eigen::Vector2d BBoxPointsExtractor::convertCoordSys(const Eigen::Vector2d& v) const
{
    // TODO get the image size automatically.
    const float W = 2048.f;
    const float H = 1536.f;

    float cx = m_Intrinsics.cx;
    float cy = m_Intrinsics.cy;

    float fx = m_Intrinsics.fx;
    float fy = m_Intrinsics.fy;

//    Eigen::Vector2d converted(v[0]*2*cx/W - cx, v[1]*2*cy/H - cy);
//    Eigen::Vector2d converted((v[0] - cx)/fx, (v[1] - cy)/fy);
//    std::cout << v.transpose() << " -> " << converted.transpose() << std::endl;

    return Eigen::Vector2d(v[0]*2*cx/W -cx, -v[1]*2*cy/H +cy);
//    return Eigen::Vector2d(v[0] - cx, v[1] - cy);
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

PointCloud BBoxPointsExtractor::getPointsAssociatedWithDetectionOnFrame(int i, Detection& detection) const
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
    std::cout << "Top left:     " << Eigen::Vector2d(detection.x, detection.y).transpose() << " -> " << tl_camera.transpose() << std::endl;
    std::cout << "bottom right: " << Eigen::Vector2d(detection.x, detection.y).transpose() << " -> " << br_camera.transpose() << std::endl;

//    Eigen::Vector3d tl = center + (q * tl_camera);
//    Eigen::Vector3d tr = center + (q * tr_camera);
//    Eigen::Vector3d br = center + (q * br_camera);
//    Eigen::Vector3d bl = center + (q * bl_camera);

    Eigen::Vector3d tl = center + (q * tl_camera).normalized()*20;
    Eigen::Vector3d tr = center + (q * tr_camera).normalized()*20;
    Eigen::Vector3d br = center + (q * br_camera).normalized()*20;
    Eigen::Vector3d bl = center + (q * bl_camera).normalized()*20;

    Eigen::Vector3d leftPlaneNormal = ((tl - center).cross(bl - center)).normalized();
    float dLeftPlane = leftPlaneNormal.dot(center);

    Eigen::Vector3d topPlaneNormal = ((tr - center).cross(tl - center)).normalized();
    float dTopPlane = topPlaneNormal.dot(center);

    Eigen::Vector3d rightPlaneNormal = ((br - center).cross(tr - center)).normalized();
    float dRightPlane = rightPlaneNormal.dot(center);

    Eigen::Vector3d bottomPlaneNormal = ((bl - center).cross(br - center)).normalized();
    float dBottomPlane = bottomPlaneNormal.dot(center);

    std::cout << "leftPlaneNormal: " << leftPlaneNormal.transpose() << std::endl;
    std::cout << "rightPlaneNormal: " << rightPlaneNormal.transpose() << std::endl;
    std::cout << "topPlaneNormal: " << topPlaneNormal.transpose() << std::endl;
    std::cout << "bottomPlaneNormal: " << bottomPlaneNormal.transpose() << std::endl;

//    PointCloud detectionCorners;
//    addSphere(detectionCorners, tl, Eigen::Vector3i(0,0,255), 0.2, 100);
//    addSphere(detectionCorners, tr, Eigen::Vector3i(0,0,255), 0.2, 100);
//    addSphere(detectionCorners, br, Eigen::Vector3i(0,0,255), 0.2, 100);
//    addSphere(detectionCorners, bl, Eigen::Vector3i(0,0,255), 0.2, 100);
//    detectionCorners.exportPLY("DetectionCorners"+std::to_string(i)+".ply" );

    // Now that we have the equation of the left plane of the bounding box, we filter points to get only
    // the points on the right of this plane.
    PointCloud cloud;
    for (auto& point : m_Cloud.points())
    {
        if (point.imIndex() == i
            && (point.pos()).dot(leftPlaneNormal) < dLeftPlane
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

PointCloud BBoxPointsExtractor::createPointCloudCenterAxisFrustum(int cameraIdx) const
{
    const int nPointsPerLine = 100;
    const int nPointsPerSphere = 200;

    const float lineLength = 1.0;
    const float sphereRadius = 0.2;
    Eigen::Vector3i colorSphere(255, 0, 255);
    Eigen::Vector3i colorXAxis(255, 0, 0);
    Eigen::Vector3i colorYAxis(0, 255, 0);
    Eigen::Vector3i colorZAxis(0, 0, 255);
    Eigen::Vector3i colorFrustum(255, 255, 255);


    int nPoints = 7 * nPointsPerLine + nPointsPerSphere;

    PointCloud cloud;
    cloud.reserve(nPoints);

    Camera camera = m_Cameras[cameraIdx];
    Eigen::Vector3d center(camera.center());
    Eigen::Quaterniond q = camera.quaternion().inverse();

    // First the axis
    Eigen::Vector3d dirX = q * Eigen::Vector3d(1.0, 0.0, 0.0);
    dirX.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + dirX * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, colorXAxis));
    }

    Eigen::Vector3d dirY = q * Eigen::Vector3d(0.0, 1.0, 0.0);
    dirY.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + dirY * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, colorYAxis));
    }

    Eigen::Vector3d dirZ = q * Eigen::Vector3d(0.0, 0.0, 1.0);
    dirZ.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + dirZ * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, colorZAxis));
    }

    // Then the sphere:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1.0,1.0);

    for (int i = 0; i < nPointsPerSphere; i++)
    {
        double x = distribution(generator);
        double y = distribution(generator);
        double z = distribution(generator);
        Eigen::Vector3d p(x, y, z);
        Eigen::Vector3d pos = center + sphereRadius * p.normalized();
        Point point(pos, colorSphere);
        cloud.push_back(point);
    }

    float cx = m_Intrinsics.cx;
    float cy = m_Intrinsics.cy;
    addFrustum(cloud, center, q, 0, 2*cx, 0, 2*cy, colorFrustum);

    return cloud;
}

void BBoxPointsExtractor::addFrustum(PointCloud& cloud,
                                     const Eigen::Vector3d& center,
                                     const Eigen::Quaterniond& q,
                                     float left,
                                     float right,
                                     float bottom,
                                     float top,
                                     const Eigen::Vector3i& color
                                     ) const
{
    const int nPointsPerLine = 1000;
    const float lineLength = 20.0;

    // topLeft
    Eigen::Vector3d topLeft = q * project2dPointIn3dSpace(Eigen::Vector2d(left, top));
    topLeft.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + topLeft * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, color));
    }

    // topRight
    Eigen::Vector3d topRight = q * project2dPointIn3dSpace(Eigen::Vector2d(right, top));
    topRight.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + topRight * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, color));
    }

    // bottomRight
    Eigen::Vector3d bottomRight = q * project2dPointIn3dSpace(Eigen::Vector2d(right, bottom));
    bottomRight.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + bottomRight * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, color));
    }

    // bottomLeft
    Eigen::Vector3d bottomLeft = q * project2dPointIn3dSpace(Eigen::Vector2d(left, bottom));
    bottomLeft.normalize();
    for (int i = 0; i < nPointsPerLine; i++)
    {
        Eigen::Vector3d p = center + bottomLeft * float(i) * lineLength / nPointsPerLine;
        cloud.push_back(Point(p, color));
    }
}

void BBoxPointsExtractor::addSphere(PointCloud& cloud,
                                    const Eigen::Vector3d& center,
                                    const Eigen::Vector3i& color,
                                    float sphereRadius,
                                    int nPoints
                                    ) const
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1.0,1.0);

    cloud.reserve(cloud.size() + nPoints);

    for (int i = 0; i < nPoints; i++)
    {
        double x = distribution(generator);
        double y = distribution(generator);
        double z = distribution(generator);
        Eigen::Vector3d p(x, y, z);
        Eigen::Vector3d pos = center + sphereRadius * p.normalized();
        Point point(pos, color);
        cloud.push_back(point);
    }
}
