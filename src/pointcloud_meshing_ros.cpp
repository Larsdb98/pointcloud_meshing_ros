#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h> // for kdTree
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class PointCloudMeshingNode
{
private:
    ros::Subscriber sub;
    std::string pointcloud_topic, file_out_directory;

public:
    PointCloudMeshingNode(ros::NodeHandle &nodeHandle)
    {
        // std::string pointcloud_topic, file_out_directory;
        nodeHandle.param<std::string>("pointcloud_topic", pointcloud_topic, "/wrist_camera/depth/points_xyzrgb_world_frame");
        nodeHandle.param<std::string>("file_out_directory", file_out_directory, "~/Desktop/output_test_mesh.obj");

        std::cout << "pointcloud topic: " << pointcloud_topic << std::endl;
        std::cout << "file out directory: " << file_out_directory << std::endl;

        sub = nodeHandle.subscribe(pointcloud_topic, 1, &PointCloudMeshingNode::pointcloudCallback, this);
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        ROS_INFO("Received pointcloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Greedy surface triangulation requires normals to work:
        // Normal estimation*

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);

        std::cout << "Normals computed by PCL:" << std::endl;
        std::cout << *normals << std::endl;

        //* normals should not contain the point normals + surface curvatures 
    

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        // Greedy surface triangulation (actual meshing process)
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        pcl::PolygonMesh triangles;
        tree2->setInputCloud (cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

        gp3.setSearchRadius(0.025);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(1000);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
        gp3.setNormalConsistency(true);

        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        // Convert pcl::PCLPointCloud2 to pcl::PolygonMesh
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        // pcl::fromROSMsg(triangles.cloud, pcl_cloud);
        // pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

        // Export mesh as .obj
        
        pcl::io::saveOBJFile("/home/lars/Master_Thesis_workspaces/VIS4ROB_Vulkan_Glasses/catkin_ws/output/test.obj", triangles);
        ROS_INFO("Mesh exported");
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_meshing_ros_node");
    ros::NodeHandle nh;

    PointCloudMeshingNode node(nh);

    ros::spin();

    return 0;
}