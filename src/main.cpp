#include <iostream>

#include "nvmreader.hpp"
#include "detectionsreader.hpp"
#include "bboxpointsextractor.hpp"
#include "pointcloud.hpp"

int main() {

    // Read detections
    DetectionsReader detectionsReader("/home/julien/projects/scout/data/result_CVAT_for_signfinder");
    auto detections = detectionsReader.detections();
    std::cout << "# detections " << detections.size() << std::endl;

//    for (const auto &detectionFrame : detections) {
//            std::cout << detectionFrame.first << ": " << detectionFrame.second.size() << '\n';
//        }

    // Read NVM
    NVMReader nvmreader("/home/julien/projects/scout/data/stereo_pointcloud/scene.nvm");
    PointCloud cloud = nvmreader.getPoints();

    std::string frameKey = nvmreader.getCameras()[90].key();
    std::cout << "\nFrame 90 key: " << frameKey << std::endl;
    std::cout << "detections[" << frameKey << "]: " << detections[frameKey].size() << std::endl;


    // Export some point clouds for visulization
//    cloud.exportPCD("scene.pcd");
//    cloud.exportPLY("scene.ply");
//    PointCloud centersCloud = bboxPointsExtractor.createPointCloudWithCameraCenters();
//    centersCloud.exportPLY("centers.ply");
//    PointCloud camDirsCloud = bboxPointsExtractor.createPointCloudWithCameraDirections();
//    camDirsCloud.exportPLY("cam_dirs.ply");
//    PointCloud frustumsCloud = bboxPointsExtractor.createPointCloudWithFrustum();
//    frustumsCloud.exportPLY("frustums.ply");

//    // Create and export a frustum for the whole FOV
//    bboxPointsExtractor.createPointCloudCenterAxisFrustum(90).exportPLY("CenterAxisFrustum.ply");

    BBoxPointsExtractor bboxPointsExtractor(nvmreader.getPoints(), nvmreader.getCameras(), nvmreader.getIntrinsices(), detections);
    // Let's first test with frame 90 as it has a big sign.
    PointCloud accumulatedFrames;
    for (int i = 80; i < 110; i++)
    {
        accumulatedFrames.add(cloud.filterForFrame(i));
        PointCloud frame;
        frame.add(cloud.filterForFrame(i));
        frame.exportPLY("frame" + std::to_string(i)+".ply");
    }
    accumulatedFrames.exportPLY("accumulatedFrames.ply");


    // Create and export a frustum for a detection
    for (int i = 85; i < 96; i++)
    {
        PointCloud detectionFrustum;
        Camera camera = nvmreader.getCameras()[i];
        Detection detection = detections[frameKey][0];
        bboxPointsExtractor.addFrustum(detectionFrustum,
                                        camera.center(),
                                        camera.quaternion().inverse(),
                                        detection.x,
                                        detection.x + detection.w,
                                        detection.y,
                                        detection.y + detection.h,
                                        Eigen::Vector3i(255, 255, 255)
                                       );
        detectionFrustum.exportPLY("DetectionFrustum" + std::to_string(i) + ".ply");
        }

    PointCloud accumulatedSign;
    for(int i = 80; i < 100; i++)
    {
        std::string frameKey = nvmreader.getCameras()[i].key();
        std::cout << "\nFrame " << i << " key: " << frameKey << std::endl;
        if (detections[frameKey].size() > 0)
        {
            PointCloud sign;
            Detection detection = detections[frameKey][0];
            sign.add(bboxPointsExtractor.getPointsAssociatedWithDetectionOnFrame(i, detection));
            sign.exportPLY("sign"+std::to_string(i)+".ply");

            accumulatedSign.add(sign);
        }
    }
    accumulatedSign.exportPLY("accumulatedSign.ply");

    return 0;
}
