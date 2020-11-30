#include <random>

#include "pointcloudutils.hpp"


namespace PointCloudUtils {

    void addSphere(PointCloud& cloud,
                   const Eigen::Vector3d& center,
                   const Eigen::Vector3i& color,
                   float sphereRadius,
                   int nPoints
                  )
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


    void addFrustum(PointCloud& cloud,
                    const Eigen::Vector3d& center,
                    const Eigen::Quaterniond& q,
                    const Eigen::Vector2d& leftTopCorner,
                    const Eigen::Vector2d& rightBottomCorner,
                    const Eigen::Vector3i& color,
                    float lineLength,
                    int nPointsPerLine
                   )
    {
        cloud.reserve(cloud.size() + 4 * nPointsPerLine);

        // topLeft
        Eigen::Vector3d topLeft = q * Eigen::Vector3d(leftTopCorner[0], leftTopCorner[1], 1.0f);
        topLeft.normalize();
        for (int i = 0; i < nPointsPerLine; i++)
        {
            Eigen::Vector3d p = center + topLeft * float(i) * lineLength / nPointsPerLine;
            cloud.push_back(Point(p, color));
        }

        // topRight
        Eigen::Vector3d topRight = q * Eigen::Vector3d(rightBottomCorner[0], leftTopCorner[1], 1.0f);
        topRight.normalize();
        for (int i = 0; i < nPointsPerLine; i++)
        {
            Eigen::Vector3d p = center + topRight * float(i) * lineLength / nPointsPerLine;
            cloud.push_back(Point(p, color));
        }

        // bottomRight
        Eigen::Vector3d bottomRight = q * Eigen::Vector3d(rightBottomCorner[0], rightBottomCorner[1], 1.0f);
        bottomRight.normalize();
        for (int i = 0; i < nPointsPerLine; i++)
        {
            Eigen::Vector3d p = center + bottomRight * float(i) * lineLength / nPointsPerLine;
            cloud.push_back(Point(p, color));
        }

        // bottomLeft
        Eigen::Vector3d bottomLeft = q * Eigen::Vector3d(leftTopCorner[0], rightBottomCorner[1], 1.0f);
        bottomLeft.normalize();
        for (int i = 0; i < nPointsPerLine; i++)
        {
            Eigen::Vector3d p = center + bottomLeft * float(i) * lineLength / nPointsPerLine;
            cloud.push_back(Point(p, color));
        }
    }

    bool computeCentroid(const PointCloud& cloud, Eigen::Vector3d& centroid)
    {
        if (cloud.size() == 0) {
            return false;
        }
        centroid = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
        for (const auto& point : cloud.points())
        {
            centroid += point.pos();
        }
        centroid /= cloud.size();
        return true;
    }

} // namespace PointCloudUtils

