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
        float xPos, yPos, zPos;
        ss >> xPos;
        ss >> yPos;
        ss >> zPos;

        m_Center = Eigen::Vector3d(xPos, yPos, zPos);
    }

    ~Camera() {}

    // Getters
    inline const std::string& filename() const {return m_Filename;}
    inline const Eigen::Quaterniond& quaternion() const {return m_Quaternion;}
    inline const Eigen::Vector3d& center() const {return m_Center;}
    inline const Eigen::Vector3d& position() const {return m_Center;}
    inline std::string key() const {return m_Filename.substr(6,22);}

private:
    std::string m_Filename;
    double m_FocalLength;
    Eigen::Quaterniond m_Quaternion;
    Eigen::Vector3d m_Center;
    double m_RadialDistortion;

};

