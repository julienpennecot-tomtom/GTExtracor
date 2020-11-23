#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include "camera.hpp"

struct Detection {
    unsigned int x;
    unsigned int y;
    unsigned int w;
    unsigned int h;
    unsigned int classid;
    float score;
    unsigned int keyframe;
};

// TOOD It would be good to create a namespace here in order not to have this typedefs available globally if we include this file.
typedef std::vector<Detection> FrameDetections;
typedef std::unordered_map<std::string, FrameDetections> Detections;

class DetectionsReader
{
public:
    DetectionsReader(const std::string& dirpath);

    ~DetectionsReader() {}

    inline const std::unordered_map<std::string, FrameDetections>& detections() const {return m_Detections;}

private:
    std::unordered_map<std::string, FrameDetections>  m_Detections;
//    std::vector<FrameDetections> m_Detections;
};

