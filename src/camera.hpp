#pragma once

#include <string>
#include <sstream>
#include <eigen3/Eigen/Eigen>

class Camera
{
public:
    Camera(std::string line)
    {
        std::stringstream ss(line);
        ss >> m_Filename;
        ss >> m_FocalLength;
        float x,y,z,w;
        ss >> w;
        ss >> x;
        ss >> y;
        ss >> z;
        m_Quaternion = Eigen::Quaterniond(w, x, y,z);
        float xPos, yPos;
        ss >> xPos;
        ss >> yPos;
        m_Center = Eigen::Vector2d(xPos, yPos);
    }

    ~Camera() {}

    // Getters
    inline const std::string& filename() const {return m_Filename;}
    inline const Eigen::Quaterniond& quaternion() const {return m_Quaternion;}
    inline const Eigen::Vector2d& center() const {return m_Center;}
    inline const Eigen::Vector2d& position() const {return m_Center;}

private:
    std::string m_Filename;
    double m_FocalLength;
    Eigen::Quaterniond m_Quaternion;
    Eigen::Vector2d m_Center;
    double m_RadialDistortion;

};

