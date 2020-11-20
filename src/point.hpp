#pragma once

#include <string>
#include <sstream>
#include <eigen3/Eigen/Eigen>

class Point{
public:
    Point(const std::string& line)
    {
        std::stringstream ss(line);
        float x,y,z;
        ss >> x;
        ss >> y;
        ss >> z;
        m_Pos = Eigen::Vector3d(x, y,z);
        float r, g, b;
        ss >> r;
        ss >> g;
        ss >> b;
        m_Color = Eigen::Vector3i(r, g, b);
        // The next field is the number of measurements, but as the nvm file was generated, it is always 1, so we do not care about this value
        int dummy;
        ss >> dummy;
        ss >> m_ImageIndex;
    }

    Point(const Eigen::Vector3d& pos, const Eigen::Vector3i& color) : m_Pos(pos), m_Color(color), m_ImageIndex(-1) {}

    ~Point() {}

    inline const Eigen::Vector3d& pos() const {return m_Pos;}
    inline const Eigen::Vector3i& color() const {return m_Color;}
    inline const int imIndex() const {return m_ImageIndex;}


private:
    Eigen::Vector3d m_Pos;
    Eigen::Vector3i m_Color;
    int m_ImageIndex;
};
