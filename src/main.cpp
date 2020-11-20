#include <iostream>

#include "nvmreader.hpp"
#include "detectionsreader.hpp"
#include "bboxpointsextractor.hpp"
#include "pointcloud.hpp"

int main() {
    DetectionsReader detectionsReader("/home/julien/projects/scout/data/result_CVAT_for_signfinder");
    auto detections = detectionsReader.detections();
    std::cout << "# detections " << detections.size() << std::endl;

//    for (int i = 0; i < detections.size(); i++)
//    {
//        std::cout << "# signs in frame[" << i << "]: " <<  detections[i].size() << std::endl;
//    }

//    auto frameDetection = detections[50];
//    for (int i = 0; i < frameDetection.size(); i++)
//    {
//        std::cout << "detection: " << frameDetection[i].x << ", "
//                  << frameDetection[i].y << std::endl;
//    }

    NVMReader nvmreader("/home/julien/projects/scout/data/stereo_pointcloud/scene.nvm");
    PointCloud cloud = nvmreader.getPoints();
//    cloud.exportPCD("scene.pcd");
//    cloud.exportPLY("scene.ply");

    BBoxPointsExtractor bboxPointsExtractor(nvmreader.getPoints(), nvmreader.getCameras(), nvmreader.getIntrinsices(), detections);
    PointCloud centersCloud = bboxPointsExtractor.createPointCloudWithCameraCenters();
    centersCloud.exportPLY("centers.ply");
    PointCloud camDirsCloud = bboxPointsExtractor.createPointCloudWithCameraDirections();
    camDirsCloud.exportPLY("cam_dirs.ply");
    PointCloud frustumsCloud = bboxPointsExtractor.createPointCloudWithFrustum();
    frustumsCloud.exportPLY("frustums.ply");
    return 0;
}
