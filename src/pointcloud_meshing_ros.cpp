#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/filter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class PointCloudMeshingNode
{
private:
    ros::Subscriber sub;
    ros::Publisher success_pub;
    std::string pointcloud_topic, file_out_directory;
    

public:
    PointCloudMeshingNode(ros::NodeHandle &nodeHandle)
    {
        nodeHandle.param<std::string>("/pointcloud_to_mesh_topic", pointcloud_topic, "/wrist_camera/depth/points_xyzrgb_world_frame");
        nodeHandle.param<std::string>("/mesh_file_export_directory", file_out_directory, "~/Desktop/output_test_mesh.obj");

        std::cout << "Pointcloud topic: " << pointcloud_topic << std::endl;
        std::cout << "File out directory: " << file_out_directory << std::endl;

        sub = nodeHandle.subscribe(pointcloud_topic, 1, &PointCloudMeshingNode::pointcloudCallback, this);

        // When a new mesh has been created, set bool message to true and publish boolean message.
        success_pub = nodeHandle.advertise<std_msgs::Bool>("meshing_success", 1000);
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        ROS_INFO("Received pointcloud. Processing...");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // The pointclouds may contain NaN values for coordinates & other stuff. We need to specify that the clouds are NOT dense.
        // PCL is_dense should be true only if the point cloud doesn't contain any NaNs. We can't assume this.

        cloud->is_dense = false;
        std::vector<int> index1;

        pcl::removeNaNFromPointCloud(*cloud, *cloud_clean, index1);

        cloud = cloud_clean;

        // Greedy surface triangulation requires normals to work:
        // Normal estimation*

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);

        n.setViewPoint(0.0, 0.0, 1.0); // higher viewpoint, by default it's the origin. This fixes the normal orientation issue.
        // All normals now point upwards. This can be assumed given that we work with sand piles. 

        n.compute(*normals);

        //* normals should not contain the point normals + surface curvatures 
    

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        // Greedy surface triangulation (actual meshing process)
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        pcl::PolygonMesh triangles;
        tree2->setInputCloud (cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

        gp3.setSearchRadius(0.1); // was 0.025 initially
        /* 
        Setting the setSearchRadius parameter to 0.1 meshed the maze.ply pointcloud nicely but some patches of points have an incorrect normal orientation
        This causes the mesh of these patches to be disconnected from the rest of the mesh. Maybe there are other variabels to try out?
        I might think that this happens due to the way we estimate the normals.
         */
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

        // Export mesh as .obj & publish success message
        std_msgs::Bool success_msg;
        
        try {
            // pcl::io::saveOBJFile("/home/lars/Master_Thesis_workspaces/VIS4ROB_Vulkan_Glasses/catkin_ws/output/test.obj", triangles);
            pcl::io::saveOBJFile(file_out_directory, triangles);
            ROS_INFO("Mesh exported !");

            success_msg.data = true;
            success_pub.publish(success_msg);
        }

        catch (...){
            success_msg.data = false;
            success_pub.publish(success_msg);
            ROS_ERROR("Unable to save pcl::PolygonMesh as OBJ file !");
        }

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