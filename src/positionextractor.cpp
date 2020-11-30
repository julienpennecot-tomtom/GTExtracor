#include <fstream>

#include "positionextractor.hpp"
#include "pointcloudutils.hpp"

PositionExtractor::PositionExtractor(const std::vector<PointCloud>& signsClouds)
    : m_SignsClouds(signsClouds)
{
    computePositions();
}

void PositionExtractor::computePositions()
{
    m_SignPositions.clear();
    for (const auto& cluster : m_SignsClouds)
    {
        Eigen::Vector3d centroid;
        bool centroidExists = PointCloudUtils::computeCentroid(cluster, centroid);
        m_SignPositions.push_back(std::pair(centroid, centroidExists));
    }
}

void PositionExtractor::exportPositionsCSV(const std::string& filename) const
{
    std::ofstream outfile;
    outfile.open(filename);
    outfile << "x, y, z, signId" << std::endl;
    for (int i = 0; i < m_SignPositions.size(); i++)
    {
        // We just write a sign positiion out if the position exist
        // We might have no points for a sign for instance if a sign is too far.
        if (m_SignPositions[i].second)
        {
            outfile << m_SignPositions[i].first[0] << ","
                    << m_SignPositions[i].first[1] << ","
                    << m_SignPositions[i].first[2] << ","
                    << i << std::endl;
        }
    }
}


void PositionExtractor::exportPositionsPLY(const std::string& filename) const
{
    PointCloud cloud;
    for (int i = 0; i < m_SignPositions.size(); i++)
    {
        // We just write a sign positiion out if the position exist
        // We might have no points for a sign for instance if a sign is too far.
        if (m_SignPositions[i].second)
        {
            PointCloudUtils::addSphere(cloud, m_SignPositions[i].first, Eigen::Vector3i(255, 0, 0), 0.2, 200);
        }
    }
    cloud.exportPLY(filename);

}

