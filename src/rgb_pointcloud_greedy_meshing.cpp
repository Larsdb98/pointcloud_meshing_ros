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
public:
    PointCloudMeshingNode()
    {
        ros::NodeHandle nh;
        std::string pointcloud_topic, file_out_directory;
        nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/camera/depth/colored_points_masked");
        nh.param<std::string>("file_out_directory", file_out_directory, "~/output_directory");


        std::cout << "pointcloud topic: " << pointcloud_topic << std::endl;
        std::cout << "file out directory: " << file_out_directory << std::endl;

        sub = nh.subscribe("/camera/depth/colored_points_masked", 1, &PointCloudMeshingNode::pointcloudCallback, this);
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        ROS_INFO("Received pointcloud");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        // Greedy surface triangulation requires normals to work:
        // Normal estimation*

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);

        //* normals should not contain the point normals + surface curvatures


        // Concatenate the XYZ and normal fields*

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

        // Manually copy RGB information
        for (size_t i = 0; i < cloud_with_normals->points.size(); ++i)
        {
            cloud_with_normals->points[i].r = cloud->points[i].r;
            cloud_with_normals->points[i].g = cloud->points[i].g;
            cloud_with_normals->points[i].b = cloud->points[i].b;
        }

        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        // Greedy surface triangulation (actual meshing process)
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        pcl::PolygonMesh triangles;
        tree2->setInputCloud (cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

        gp3.setSearchRadius(0.025);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
        gp3.setNormalConsistency(false);

        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        // Convert pcl::PCLPointCloud2 to pcl::PolygonMesh
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        // pcl::fromROSMsg(triangles.cloud, pcl_cloud);
        // pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

        // Export mesh as .obj
        pcl::io::saveOBJFile("output_mesh.obj", triangles);

        exportTexture(*cloud, triangles.polygons, "output_texture.jpg");



        ROS_INFO("Mesh exported");
    }

    void exportTexture(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                       const std::vector<pcl::Vertices>& polygons,
                       const std::string& filename)
    {
        cv::Mat texture(cloud.height, cloud.width, CV_8UC3, cv::Scalar(0, 0, 0));

        for (const auto& polygon : polygons)
        {
            for (const auto& index : polygon.vertices)
            {
                texture.at<cv::Vec3b>(index) = cv::Vec3b(cloud.points[index].b, cloud.points[index].g, cloud.points[index].r);
            }
        }

        cv::imwrite(filename, texture);
    }

private:
    
    ros::Subscriber sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_meshing_node");

    PointCloudMeshingNode node;

    ros::spin();

    return 0;
}