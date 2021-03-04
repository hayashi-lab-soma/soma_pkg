#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char *argv[])
{
    if(argc < 2){
        std::cout << "Please give me input PCD file path" << std::endl;
        return -1;
    }
    if(argc < 3){
        std::cout << "Please give me save PCD file path" << std::endl;
        return -1;
    }



    std::string pcdfile = argv[1];
    std::string savepath = argv[2];

    //入力点群
    pcl::PointCloud<PointT>::Ptr in_cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (pcdfile, *in_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
        PCL_ERROR("path:%s", pcdfile.c_str());
        return -1;
    }

    PCL_INFO("Load: %s", pcdfile.c_str());

    std::cout << "Cloud size : " << in_cloud->points.size() << std::endl;
    std::cout << "Converting ..." << std::endl;

    pcl::PointCloud<PointT>::Ptr out_cloud (new pcl::PointCloud<PointT>);
    out_cloud->width = in_cloud->width;
    out_cloud->height = in_cloud->height;
    for (size_t i = 0; i < in_cloud->points.size (); ++i)
    {
        PointT tmp;
        tmp.r = in_cloud->points[i].r;
        tmp.g = in_cloud->points[i].g;
        tmp.b = in_cloud->points[i].b;

        tmp.z = in_cloud->points[i].x;
        tmp.y = in_cloud->points[i].y;
        tmp.x = -in_cloud->points[i].z;

        out_cloud->points.push_back(tmp);
    }

    int ret = pcl::io::savePCDFile(savepath, *out_cloud);

    std::cout << "Copied cloud size : " << out_cloud->points.size() << std::endl;
    std::cout << "Save:" << savepath << "(" << ret << ")" << std::endl;
    return 0;
}