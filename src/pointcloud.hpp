#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

#include "point.hpp"

class PointCloud
{
public:
    PointCloud();

    void reserve(int n);
    void push_back(const Point& p);
    inline uint size() const {return m_Points.size();}

    void exportPCD(const std::string& filename) const;
    void exportPLY(const std::string& filename) const;
    const std::vector<Point>& points() const {return m_Points;}

    // This works, but it is really slow as we have to go through all points to find just the points belonging to frame i
    PointCloud filterForFrame(int i) const;

    void add(const PointCloud& cloud);

private:
    static uint32_t rgb2uint(uint8_t r, uint8_t g, uint8_t b)
    {
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        return rgb;
    }

private:
    std::vector<Point> m_Points;
};
