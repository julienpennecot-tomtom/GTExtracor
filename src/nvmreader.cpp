#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "nvmreader.hpp"
#include "pointcloudutils.hpp"


NVMReader::NVMReader(const std::string& filepath)
{
    int frameIdx = -1;
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

            // We need to store the indices of the points where the frame changes (for quick access)
            if (point.imIndex() != frameIdx) {
                if (frameIdx > point.imIndex()) {
                    std::cout << "Probably an ERROR because point image index is less than frame id" << std::endl;
                }
                while (frameIdx < point.imIndex()) {
                    frameIdx++;
                    m_PointIndexForFrame.push_back(i);
                }
            }

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

PointCloud NVMReader::createPointCloudWithCameraCenters(float sphereRadius, const Eigen::Vector3i& color, int nPointsPerSphere) const
{
    int nPoints = m_Cameras.size() * nPointsPerSphere;

    PointCloud cloud;
    cloud.reserve(nPoints);

    for (const auto& camera : m_Cameras)
    {
        Eigen::Vector3d center(camera.center());
        PointCloudUtils::addSphere(cloud, center, color, sphereRadius, nPointsPerSphere);
    }
    return cloud;
}

PointCloud NVMReader::createPointCloudWithCameraDirections(int lineLength, const Eigen::Vector3i& color, int nPointsPerLine) const
{
    int nPoints = m_Cameras.size() * nPointsPerLine;

    PointCloud cloud;
    cloud.reserve(nPoints);

    for (const auto& camera : m_Cameras)
    {
        Eigen::Vector3d center(camera.center());
        Eigen::Quaterniond q = camera.quaternion().inverse();
        Eigen::Vector3d dir = q * Eigen::Vector3d(0.0, 0.0, 1.0);
        dir.normalize();
        for (int i = 0; i < nPointsPerLine; i++)
        {
            Eigen::Vector3d p = center + dir * float(i) * lineLength / nPointsPerLine;
            cloud.push_back(Point(p, color));
        }
    }
    return cloud;
}

PointCloud NVMReader::createPointCloudFromFrameRange(int firstFrame, int lastFrame) const
{
    assert(lastFrame >= firstFrame);
    PointCloud cloud;
    // We estimate the number of points to about 1000 points per frame. It is not so important if it is not exact,
    cloud.reserve(1000 * (lastFrame - firstFrame + 1));

    int firstPointIdx = m_PointIndexForFrame[firstFrame];
    // We do some kind of bound checking for the last point index.
    int lastPointIdx;
    if (lastFrame == m_Cameras.size()) {
        lastPointIdx = m_Cloud.size();
    } else {
        lastPointIdx = m_PointIndexForFrame[lastFrame+1];
    }
    std::cout << "firstPointIdx: " << firstPointIdx << std::endl;
    std::cout << "lastPointIdx: " << lastPointIdx << std::endl;

    for (int pointIdx = firstPointIdx; pointIdx < lastPointIdx; pointIdx++) {
        cloud.push_back(m_Cloud.points()[pointIdx]);
    }

    return cloud;
}



