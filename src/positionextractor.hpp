#pragma once

#include <vector>

#include "pointcloud.hpp"

class PositionExtractor
{
public:
    PositionExtractor(const std::vector<PointCloud>& signsClouds);
    void computePositions();
    void exportPositionsCSV(const std::string& filename) const;
    // Convenience method in order to visualize the positions by exporting a red sphere around it.
    void exportPositionsPLY(const std::string& filename) const;


private:
    const std::vector<PointCloud>& m_SignsClouds;
    std::vector<std::pair<Eigen::Vector3d, bool>> m_SignPositions;
};
