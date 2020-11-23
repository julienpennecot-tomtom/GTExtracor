#include "pointcloud.hpp"

#include <fstream>


PointCloud::PointCloud()
{

}

void PointCloud::reserve(int n) {
    m_Points.reserve(n);
}

void PointCloud::push_back(const Point& p)
{
    m_Points.push_back(p);
}

void PointCloud::exportPCD(const std::string& filename) const
{
    std::ofstream outfile;
    outfile.open(filename);
    outfile << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    outfile << "VERSION .7" << std::endl;
    outfile << "FIELDS x y z rgb" << std::endl;
    outfile << "SIZE 4 4 4 4" << std::endl;
    outfile << "TYPE F F F U" << std::endl;
    outfile << "COUNT 1 1 1 1" << std::endl;
    outfile << "WIDTH " << m_Points.size() << std::endl;
    outfile << "HEIGHT 1" << std::endl;
    outfile << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    outfile << "POINTS " << m_Points.size() << std::endl;
    outfile << "DATA ascii" << std::endl;

    for (const auto& point : m_Points)
    {
        uint32_t color = rgb2uint(point.color()[0], point.color()[1], point.color()[2]);
        outfile << point.pos()[0] << " " << point.pos()[1] << " " << point.pos()[2] << " " << color << std::endl;
    }
}

void PointCloud::exportPLY(const std::string& filename) const
{

    std::ofstream outfile;
    outfile.open(filename);
    outfile << "ply" << std::endl;
    outfile << "format ascii 1.0" << std::endl;
    outfile << "element vertex " <<  m_Points.size() << std::endl;
    outfile << "property float32 x" << std::endl;
    outfile << "property float32 y" << std::endl;
    outfile << "property float32 z" << std::endl;
    outfile << "property uchar red" << std::endl;
    outfile << "property uchar green" << std::endl;
    outfile << "property uchar blue" << std::endl;
    outfile << "end_header" << std::endl;
    for (const auto& point : m_Points)
    {
        outfile << point.pos()[0] << " "
                << point.pos()[1] << " "
                << point.pos()[2] << " "
                << point.color()[0] << " "
                << point.color()[1] << " "
                << point.color()[2]<< std::endl;
    }
}

PointCloud PointCloud::filterForFrame(int i) const
{
    PointCloud cloud;
    for (const auto& point : m_Points)
    {
        if (point.imIndex() == i) {
            cloud.push_back(point);
        }
    }
    return cloud;
}

void PointCloud::add(const PointCloud& cloud)
{
    for (const auto& point : cloud.points())
    {
        this->push_back(point);
    }
}
