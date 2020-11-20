#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "nvmreader.hpp"

NVMReader::NVMReader(const std::string& filepath)
{
    std::string line;
    std::ifstream myfile (filepath);
    if (myfile.is_open())
    {
        // Read the file version header
        std::getline(myfile,line);
        std::stringstream ss(line);
        std::string dummy;
        ss >> dummy;
        ss >> dummy;
        ss >> m_Intrinsics.fx;
        ss >> m_Intrinsics.cx;
        ss >> m_Intrinsics.fy;
        ss >> m_Intrinsics.cy;

        // Read empty line
        std::getline(myfile,line);
        // Read the number of cameras:
        std::getline(myfile,line);
        int nCameras = std::stoi(line);
        std::cout << "# of cameras: " << nCameras << std::endl;
        m_Cameras.reserve(nCameras);

        // Read and store all the cameras
        for (uint i = 0; i < nCameras; i++)
        {
            std::getline(myfile,line);
            Camera camera(line);
            m_Cameras.push_back(camera);
        }

        // Read the number of points:
        std::getline(myfile,line);  // First read an empty line
        std::getline(myfile,line);
        int nPoints = std::stoi(line);
        std::cout << "# of points: " << nPoints << std::endl;
        m_Cloud.reserve(nPoints);

        for (uint i = 0; i < nPoints; i++)
        {
            std::getline(myfile,line);
            Point point(line);
            m_Cloud.push_back(point);
        }
        std::cout << "Done reading: " << nPoints << " points" << std::endl;
        myfile.close();
    } else {
        std::cerr << "Error: could not find file: " << filepath << std::endl;
    }
    std::cout << "Fx: " << m_Intrinsics.fx << " Cx: " << m_Intrinsics.cx << " Fy: " << m_Intrinsics.fy << " Cy: " << m_Intrinsics.cy << std::endl;
}

NVMReader::~NVMReader()
{

}


