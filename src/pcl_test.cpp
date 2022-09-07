#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "boundary_estimation.hpp"

int main()
{
	std::string filename = "0.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader reader;
	reader.read(filename, *cloud_ptr);
	
	pcl::PointCloud<pcl::Boundary>::Ptr output(new pcl::PointCloud<pcl::Boundary>);
	AC ac;
	ac.edge_detection(cloud_ptr, output);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloudRGB->resize(cloud_ptr->size());
	for (size_t i = 0; i < output->size(); i++)
	{
		cloudRGB->points[i].x = cloud_ptr->points[i].x;
		cloudRGB->points[i].y = cloud_ptr->points[i].y;
		cloudRGB->points[i].z = cloud_ptr->points[i].z;
		if (output->points[i].boundary_point)
		{
			cloudRGB->points[i].r = 0;
			cloudRGB->points[i].g = 255;
			cloudRGB->points[i].b = 0;
		}
		else
		{
			cloudRGB->points[i].r = 150;
			cloudRGB->points[i].g = 150;
			cloudRGB->points[i].b = 150;
		}
	}
	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloudRGB);
	system("PAUSE");
}

// #include <pcl/visualization/cloud_viewer.h>
// #include <iostream>
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include "boundary_estimation.hpp"

// int user_data;

// void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
// {
//     viewer.setBackgroundColor (1.0, 0.5, 1.0);
//     pcl::PointXYZ o;
//     o.x = 1.0;
//     o.y = 0;
//     o.z = 0;
//     viewer.addSphere (o, 0.25, "sphere", 0);
//     std::cout << "i only run once" << std::endl;

// }

// void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
// {
//     static unsigned count = 0;
//     std::stringstream ss;
//     ss << "Once per viewer loop: " << count++;
//     viewer.removeShape ("text", 0);
//     viewer.addText (ss.str(), 200, 300, "text", 0);

//     //FIXME: possible race condition here:
//     user_data++;
// }

// int main ()
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile ("cat.pcd", *cloud);

//     pcl::visualization::CloudViewer viewer("Cloud Viewer");

//     //blocks until the cloud is actually rendered
//     viewer.showCloud(cloud);

//     //use the following functions to get access to the underlying more advanced/powerful
//     //PCLVisualizer

//     //This will only get called once
//     viewer.runOnVisualizationThreadOnce (viewerOneOff);

//     //This will get called once per visualization iteration
//     viewer.runOnVisualizationThread (viewerPsycho);
//     while (!viewer.wasStopped ())
//     {
//     //you can also do cool processing here
//     //FIXME: Note that this is running in a separate thread from viewerPsycho
//     //and you should guard against race conditions yourself...
//     user_data++;
//     } 
//     return 0;
// }
