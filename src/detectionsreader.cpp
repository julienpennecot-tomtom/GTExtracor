#include "detectionsreader.hpp"

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <functional>


DetectionsReader::DetectionsReader(const std::string& dirpath)
{
    // Get the list of detection files in the passed directory
    std::vector<boost::filesystem::path> paths;
    std::string ext = ".csv";
    if (boost::filesystem::exists(dirpath) && boost::filesystem::is_directory(dirpath))
    {
        for (auto const & entry : boost::filesystem::recursive_directory_iterator(dirpath))
        {
            if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ext)
                paths.emplace_back(entry.path().filename());
        }
    }

    // We need to sort them as we want to make sure that it is in order.
    std::sort(paths.begin(), paths.end(), std::less<boost::filesystem::path>());

    // We can now start reading the files.
    for (int i = 0; i < paths.size(); i++)
    {
        const auto& path = paths[i];
        std::vector<Detection> frameDetections;
        std::string key = path.string().substr(0, 22);
        std::string line;
        boost::filesystem::path filepath = boost::filesystem::path(dirpath) / boost::filesystem::path(path);
        std::ifstream myfile (filepath);
        if (myfile.is_open())
        {
            // Read the file version header
            std::getline(myfile,line);
            while (std::getline(myfile, line))
            {
                std::stringstream ss(line);
                unsigned int x, y, h, w, classid, keyframe;
                float score;
                ss >> x;
                // We need to skip the comas,
                if(ss.peek() == ',') ss.ignore();
                ss >> y;
                if(ss.peek() == ',') ss.ignore();
                ss >> h;
                if(ss.peek() == ',') ss.ignore();
                ss >> w;
                if(ss.peek() == ',') ss.ignore();
                ss >> classid;
                if(ss.peek() == ',') ss.ignore();
                ss >> score;
                if(ss.peek() == ',') ss.ignore();
                ss >> keyframe;
                if(ss.peek() == ',') ss.ignore();
                frameDetections.push_back({x, y, h, w, classid, score, keyframe});
            }
            myfile.close();
        } else {
            std::cout << "Could not open file: " << filepath << std::endl;
        }
        m_Detections[key] = frameDetections;
    }
}
