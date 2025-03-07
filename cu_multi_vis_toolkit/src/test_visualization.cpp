#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iomanip>

// #include <vtkSmartPointer.h>
// #include <vtkImageData.h>
// #include <vtkImageActor.h>
// #include <vtkRenderer.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkImageMapper3D.h>
#include <pcl/pcl_config.h> // Include for version macros
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>

#include <cu_multi_vis_toolkit/pointcloud.h>
#include <cu_multi_vis_toolkit/pose.h>


PointCloud pointCloudHandler;  // Instance of PointCloud class

int initFileIndex = 1;                  // Start with the first point cloud file
int currentFileIndex = initFileIndex;   // Set current file index to current pc file index
int prevFileIndex = initFileIndex;
const int FileInc = 50;

const std::string camera_config_path = CAMERA_CONFIG_PATH;
const std::string dataset_path = DATASET_PATH;

// std::string calib_dir = dataset_path + "calib/";
// std::string transforms_dir = calib_dir + "transforms/";

const std::string mesh_dir = MESH_DIR_PATH;
std::string objFilePath = mesh_dir + "hunter.obj";
pcl::PointCloud<pcl::PointXYZ> meshCloud;
pcl::PolygonMesh radarRigMesh;
// pcl::TextureMesh radarRigMesh;

int robot = 0;
std::string env = "kittredge_loop";
std::string env_dir = dataset_path; // + env + "/" + env + "_run" +  std::to_string(robot) + "/";

std::string groundtruth_dir = env_dir + "poses/";

// LIDAR
std::string lidar_dir = env_dir + "lidar/";
std::string lidar_timestamp_path = lidar_dir + "timestamps.txt";
std::string lidar_labels_dir = lidar_dir + "labels/gt_labels/";
std::string lidar_osm_labels_dir = lidar_dir + "labels/gt_osm_labels/";
std::map<double, int> cloudIndexTimestampMap;                           // Lidar timestamps
double cloudTimestamp;                                                  // Current lidar PC timestamp

std::map<double, Pose> poseMap;  // Ground truth poses

struct semanticPoint {
    PCL_ADD_POINT4D;        // Adds x, y, z fields
    float intensity;
    PCL_ADD_RGB;            // Adds r, g, b fields
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensures new allocators are aligned
} EIGEN_ALIGN16;

// Register the point struct with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(semanticPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, rgb, rgb) // Note that PCL packs the r, g, and b into a single float
)

static std::map<int, std::vector<int>> color_map = {
    {0, {0, 0, 0}},               // "unlabeled"
    {1, {0, 0, 0}},               // "outlier"
    {10, {0, 0, 142}},            // "car"
    {11, {119, 11, 32}},          // "bicycle"
    {13, {250, 80, 100}},         // "bus"
    {15, {0, 0, 230}},            // "motorcycle"
    {16, {255, 0, 0}},            // "on-rails"
    {18, {0, 0, 70}},             // "truck"
    {20, {51, 0, 51}},            // "other-vehicle"
    {30, {220, 20, 60}},          // "person"
    {31, {200, 40, 255}},         // "bicyclist"
    {32, {90, 30, 150}},          // "motorcyclist"
    {40, {128, 64, 128}},         // "road"
    {44, {250, 170, 160}},        // "parking"
    {45, {0, 0, 255}},            // "OSM BUILDING"
    {46, {255, 0, 0}},            // "OSM ROAD"
    {48, {244, 35, 232}},         // "sidewalk"
    {49, {81, 0, 81}},            // "other-ground"
    {50, {0, 100, 0}},            // "building"
    {51, {190, 153, 153}},        // "fence"
    {52, {0, 150, 255}},          // "other-structure"
    {60, {170, 255, 150}},        // "lane-marking"
    {70, {107, 142, 35}},         // "vegetation"
    {71, {0, 60, 135}},           // "trunk"
    {72, {152, 251, 152}},        // "terrain"
    {80, {153, 153, 153}},        // "pole"
    {81, {0, 0, 255}},            // "traffic-sign"
    {99, {255, 255, 50}},         // "other-object"
    // Commented-out entries from labels:
    // {252, {245, 150, 100}},   // "moving-car"
    // {253, {}},                // "moving-bicyclist"
    // {254, {30, 30, 25}},      // "moving-person"
    // {255, {90, 30, 150}},     // "moving-motorcyclist"
    // {256, {}},                // "moving-on-rails"
    // {257, {}},                // "moving-bus"
    // {258, {}},                // "moving-truck"
    // {259, {}},                // "moving-other-vehicle"
};

// Initialize PCL Visualizer
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("CU-Multi Visualization Toolkit"));

// Lidar PC
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloudRGB(new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr accumCloud(new pcl::PointCloud<pcl::PointXYZI>);

// Images
// vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
// vtkSmartPointer<vtkImageActor> imageActor = vtkSmartPointer<vtkImageActor>::New();  // Create an Image Actor to display the image

float packRGB(std::vector<uint8_t> rgb) {
    assert(rgb.size() == 3);
    uint8_t r = rgb[0];
    uint8_t g = rgb[1];
    uint8_t b = rgb[2];

    std::cout << "Input RGB: r = " << static_cast<int>(r) 
              << ", g = " << static_cast<int>(g) 
              << ", b = " << static_cast<int>(b) << std::endl;

    uint32_t packed = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    
    // uint32_t packed = ((r << 16) & 0x00FF0000) | ((g << 8) & 0x0000FF00) | (b & 0x000000FF);

    uint32_t rgb_unpacked = *reinterpret_cast<int*>(&packed);
    uint8_t ru = (rgb_unpacked >> 16) & 0x0000ff;
    uint8_t gu = (rgb_unpacked >> 8)  & 0x0000ff;
    uint8_t bu = (rgb_unpacked)       & 0x0000ff;

    std::cout << "Input RGB: ru = " << static_cast<int>(ru) 
              << ", gu = " << static_cast<int>(gu) 
              << ", bu = " << static_cast<int>(bu) << std::endl;

    return *reinterpret_cast<float*>(&packed);
}

// int labelIntToLabelRGB(int label_int) {
//     auto it = color_map.find(label_int);
//     if (it != color_map.end()) {
//         std::vector<int> rgbu = it->second;
//         int rgb = ((int)rgbu[0]) << 16 | ((int)rgbu[1]) << 8 | ((int)rgbu[2]);
//         // std::cout << "\n label ID: " << label_int << ", color: (" << rgbu[0] << ", " << rgbu[1] << ", " << rgbu[2] << ").\n";
//         return rgb;
//     } else {
//         std::cerr << "Label INT not found, returning default RGB.\n";
//         return packRGB({0, 0, 0});  // Default to black if the label is not found
//     }
// }

float labelIntToLabelRGB(int label_int) {
    auto it = color_map.find(label_int);
    if (it != color_map.end()) {
        std::vector<int> rgbu = it->second;
        // Pack RGB values into a float
        uint32_t rgb = ((rgbu[0] & 0xFF) << 16) | ((rgbu[1] & 0xFF) << 8) | (rgbu[2] & 0xFF);
        float packed_rgb;
        std::memcpy(&packed_rgb, &rgb, sizeof(float));  // Convert uint32_t to float
        // std::cout << "\nLabel ID: " << label_int 
                //   << ", Color: (" << rgbu[0] << ", " << rgbu[1] << ", " << rgbu[2] << ").\n";
        return packed_rgb;
    } else {
        std::cerr << "Label INT not found, returning default RGB.\n";
        return 0.0f;  // Default to black (packed as float)
    }
}

bool setSemanticCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRGB, const std::string &labelFilename) {
    // Check if the file exists
    std::ifstream label_file(labelFilename, std::ios::binary);
    if (!label_file.is_open()) {
        std::cout << "Could not find .label file: " << labelFilename.c_str() << std::endl;
        return false;
    }

    std::vector<int32_t> labels;
    int32_t label;
    while (label_file.read(reinterpret_cast<char*>(&label), sizeof(int32_t))) {
        labels.push_back(label);
    }

    pcl::PointXYZRGBA labeledPoint;
    pcl::PointCloud<pcl::PointXYZRGBA> labeledCloud;
    // Assuming cloud is populated and has the same number of points as labels
    for (size_t i = 0; i < cloud->points.size() && i < labels.size(); ++i) {
        labeledPoint.x = cloud->points[i].x;
        labeledPoint.y = cloud->points[i].y;
        labeledPoint.z = cloud->points[i].z;
        labeledPoint.rgb = labelIntToLabelRGB(labels[i]);

        labeledCloud.push_back(labeledPoint);
    }

    *cloudRGB = labeledCloud;

    return true;
}
                      
void updatePointCloudDisplay(int currentFileIndex) {
    // viewer->removePointCloud("current_scan");
    viewer->removePointCloud("current_scan");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> intensity_distribution(cloudRGB);
    viewer->addPointCloud<pcl::PointXYZRGBA>(cloudRGB, intensity_distribution, "current_scan");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current_scan");

    std::string accum_cloud_prev_name = "accumulated_ds_cloud_" + std::to_string(prevFileIndex);
    std::string accum_cloud_name = "accumulated_ds_cloud_" + std::to_string(currentFileIndex);
    if (currentFileIndex < prevFileIndex) {
        viewer->removePointCloud(accum_cloud_name);
        viewer->removePointCloud(accum_cloud_prev_name);
    }
    else {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> accumulated_ds_intensity_distribution(downsampledCloud, "z");
        viewer->addPointCloud<pcl::PointXYZI>(downsampledCloud, accumulated_ds_intensity_distribution, accum_cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.2, accum_cloud_name);      
    }
    prevFileIndex = currentFileIndex;
}

void updateOBJMeshInViewer(pcl::visualization::PCLVisualizer::Ptr viewer, const Eigen::Affine3f& transform) {
  // Remove the existing mesh if it exists
  viewer->removePolygonMesh("mesh");

  // Apply the transformation
  pcl::PointCloud<pcl::PointXYZ> transformedCloud;
  pcl::transformPointCloud(meshCloud, transformedCloud, transform);

  // Convert back to PolygonMesh
  pcl::toPCLPointCloud2(transformedCloud, radarRigMesh.cloud);

  // Add the updated mesh to the PCLVisualizer
  viewer->addPolygonMesh(radarRigMesh, "mesh");
  // viewer->addTextureMesh(radarRigMesh, "mesh", 0);

  // Optionally set properties for the mesh
  viewer->setRepresentationToSurfaceForAllActors(); // Render as surface
}

void addOBJMeshToViewer(pcl::visualization::PCLVisualizer::Ptr viewer) {
    // Load the OBJ file
    if (pcl::io::loadOBJFile(objFilePath, radarRigMesh) == -1) {
        std::cerr << "Error: Could not load OBJ file: " << objFilePath << std::endl;
        return;
    }

    // Transform the mesh
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(radarRigMesh.cloud, cloud); // Convert to PointCloud

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float angle = M_PI / 2; // 90 degrees in radians
    transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitY()));

    // // Apply the transformation
    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud(cloud, transformedCloud, transform);

    // Convert back to PolygonMesh
    pcl::toPCLPointCloud2(transformedCloud, radarRigMesh.cloud);

    // set global mesh cloud
    meshCloud = transformedCloud;

    // Add the mesh to the PCLVisualizer
    viewer->addPolygonMesh(radarRigMesh, "mesh");
    // viewer->addTextureMesh(radarRigMesh, "mesh", 0);

    // Optionally set properties for the mesh
    viewer->setRepresentationToSurfaceForAllActors(); // Render as surface
}

void updateDisplay () {
    std::string pointcloudPath = lidar_dir + "pointclouds/lidar_pointcloud_" + std::to_string(currentFileIndex) + ".bin";
    std::string labelPath = lidar_labels_dir + "lidar_pointcloud_" + std::to_string(currentFileIndex) + ".bin";
    if (pointCloudHandler.setCurrentCloud(pointcloudPath, cloud) &&
        pointCloudHandler.setCurrentTimestamp(currentFileIndex, cloudIndexTimestampMap, cloudTimestamp) //&&
        // setImage(currentFileIndex)
        ) {
        Pose interpolatedPose = PoseHandler::interpolatePose(cloudTimestamp, poseMap);
        pointCloudHandler.transform(cloud, interpolatedPose);
        pointCloudHandler.downsample(cloud, downsampledCloud, 3.0f);

        // 
        setSemanticCloud(cloud, cloudRGB, labelPath);
        // pointCloudHandler.downsample(cloud, downsampledCloud, 3.0f);

        Eigen::Affine3f transform = PoseHandler::poseToAffine3f(interpolatedPose);
        viewer->removeCoordinateSystem("base_frame");
        viewer->addCoordinateSystem(1.0, transform, "base_frame");

        updatePointCloudDisplay(currentFileIndex);

        updateOBJMeshInViewer(viewer, transform);

        std::cout << "Loaded file: lidar_pointcloud_" << currentFileIndex << ".bin with timestamp: " << cloudTimestamp << std::endl;
    }
}

// Handles keyboard events for navigation
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    std::cout.precision(20);

    if (event.keyDown()) {
        if (event.getKeySym() == "Up") {
            currentFileIndex += FileInc;
            updateDisplay();
        } 
        else if (event.getKeySym() == "Down") {
            if (currentFileIndex > initFileIndex) {
                currentFileIndex -= FileInc;
                updateDisplay();
            }
        }

        if (event.getKeySym() == "s") {
            viewer->saveCameraParameters(camera_config_path); 
            std::cout << "Camera parameters saved." << std::endl;
        } else if (event.getKeySym() == "l") {
            viewer->loadCameraParameters(camera_config_path);
            std::cout << "Camera parameters loaded." << std::endl;
        }
    }
}

int main() {
    viewer->getRenderWindow()->GlobalWarningDisplayOff(); // Add This Line

    if (PoseHandler::loadPoses(groundtruth_dir, poseMap)) {
        std::cout << "Successfully loaded " << poseMap.size() << " poses." << std::endl;
    } else {
        std::cerr << "Failed to load poses." << std::endl;
        return 1;
    }

    if (pointCloudHandler.loadTimestamps(lidar_timestamp_path, cloudIndexTimestampMap)) {
        std::cout << "Successfully loaded " << cloudIndexTimestampMap.size() << " LIDAR timestamps." << std::endl;
    } else {
        std::cerr << "Failed to load LIDAR timestamps." << std::endl;
        return 1;
    }

    std::string pointcloudPath = lidar_dir + "pointclouds/lidar_pointcloud_" + std::to_string(currentFileIndex) + ".bin";
    if (!(pointCloudHandler.setCurrentCloud(pointcloudPath, cloud) &&
          pointCloudHandler.setCurrentTimestamp(currentFileIndex, cloudIndexTimestampMap, cloudTimestamp))) {
        std::cerr << "Failed to load initial point cloud." << std::endl;
        return 1;
    }

    // if (!setImage(currentFileIndex)) {
    //     return 1;
    // }

    pointCloudHandler.downsample(cloud, downsampledCloud, 3.0f);

    updatePointCloudDisplay(currentFileIndex);

    viewer->setBackgroundColor(1.0, 1.0, 1.0);
    viewer->addCoordinateSystem(2.0);
    viewer->initCameraParameters();

    // Add the OBJ mesh
    addOBJMeshToViewer(viewer);

    // Register keyboard callback
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    // // For image data
    // imageActor->GetMapper()->SetInputData(vtkImage);
    // vtkSmartPointer<vtkRenderer> imageRenderer = vtkSmartPointer<vtkRenderer>::New();   // Create a Renderer for the Image
    // vtkSmartPointer<vtkRenderWindow> renderWindow = viewer->getRenderWindow(); // Get the VTK Render Window from the PCL Visualizer
    // imageRenderer->AddActor(imageActor);
    // renderWindow->AddRenderer(imageRenderer); // Add the image renderer to the render window

    // // Adjust viewport for PCL and image
    // viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->SetViewport(0.5, 0.0, 1.0, 1.0); // PCL on the right
    // imageRenderer->SetViewport(0.0, 0.0, 0.5, 1.0); // Image on the left


    while (!viewer->wasStopped()) {
        viewer->spin();
    }

    return 0;
}
