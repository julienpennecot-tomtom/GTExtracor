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

    // Export some point clouds for visualization/debug purpose
    //std::cout << "Exporting full scene. It might take a while" << std::endl;
    //cloud.exportPLY("scene.ply");
    PointCloud centersCloud = nvmreader.createPointCloudWithCameraCenters(0.2, Eigen::Vector3i(255, 0, 0), 100);
    centersCloud.exportPLY("centers.ply");
    PointCloud camDirsCloud = nvmreader.createPointCloudWithCameraDirections(1., Eigen::Vector3i(255, 255, 255), 100);
    camDirsCloud.exportPLY("cam_dirs.ply");

    // Let's first test with a big sign around frame 90
    PointCloud accumulatedFrames = nvmreader.createPointCloudFromFrameRange(80, 100);
    accumulatedFrames.exportPLY("accumulatedFrames.ply");

    // Now we can start extracting signs.
    BBoxPointsExtractor bboxPointsExtractor(nvmreader.getPoints(), nvmreader.getCameras(), nvmreader.getIntrinsices(), detections);


    std::string frameKey = nvmreader.getCameras()[90].key();
    std::cout << "\nFrame 90 key: " << frameKey << std::endl;
    std::cout << "detections[" << frameKey << "]: " << detections[frameKey].size() << std::endl;

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
