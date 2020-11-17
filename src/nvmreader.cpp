#include <fstream>
#include <iostream>
#include <string>

#include "nvmreader.hpp"

NVMReader::NVMReader(const std::string& filepath)
{
    std::string line;
    std::ifstream myfile (filepath);
    if (myfile.is_open())
    {
        // Read the file version header
        std::getline(myfile,line);
        // Read empty line
        std::getline(myfile,line);
        // Read the number of cameras:
        std::getline(myfile,line);
        m_nCameras = std::stoi(line);
        std::cout << "# of cameras: " << m_nCameras << std::endl;
        m_Cameras.reserve(m_nCameras);

        // Read and store all the cameras
        for (uint i = 0; i < m_nCameras; i++)
        {
            std::getline(myfile,line);
            Camera camera(line);
            m_Cameras.push_back(camera);
        }

        // Read the number of points:
        std::getline(myfile,line);  // First read an empty line
        std::getline(myfile,line);
        m_nPoints = std::stoi(line);
        std::cout << "# of points: " << m_nPoints << std::endl;
        m_Points.reserve(m_nPoints);

        for (uint i = 0; i < m_nPoints; i++)
        {
            std::getline(myfile,line);
            Point point(line);
            m_Points.push_back(point);
        }
        std::cout << "Done reading: " << m_nPoints << " points" << std::endl;
        myfile.close();
    } else {
        std::cerr << "Error: could not find file: " << filepath << std::endl;
    }
}

NVMReader::~NVMReader()
{

}
