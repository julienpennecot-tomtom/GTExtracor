#pragma once

#include <string>

#include "camera.hpp"
#include "point.hpp"


class NVMReader
{
    /* ******************************************************************************************************************
     * This class and the associated classes (Camera and Point) are here to parse a nvm file.
     * We are just interested in the image filenames, orientations and positions
     * For the points we are only interested in the point positions, colors and to which image they are associated with
     *
     * This is by no means a mature class for reading nvm files, there are no error checkings etc...
     *
     * The nvm format is described here: http://ccwu.me/vsfm/doc.html
     ******************************************************************************************************************/
public:
    NVMReader(const std::string& filepath);
    ~NVMReader();

private:
    unsigned int m_nCameras;
    unsigned int m_nPoints;
    std::vector<Camera> m_Cameras;
    std::vector<Point> m_Points;

};

