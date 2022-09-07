#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Eigen>

class AC
{
public:
	void edge_detection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, pcl::PointCloud<pcl::Boundary>::Ptr &output)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
		normal_ptr = this->compute_normals(cloud_ptr);
		std::vector<int> neighbor_idx;
		std::vector<float> neighbor_dist;
		pcl::search::KdTree<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud_ptr);

		Eigen::Vector4f u = Eigen::Vector4f::Zero(), v = Eigen::Vector4f::Zero();
		output->resize(cloud_ptr->size());

		for (size_t i = 0; i < cloud_ptr->size(); i++)
		{
			kdtree.nearestKSearch(cloud_ptr->points[i], 40, neighbor_idx, neighbor_dist);
			this->getCoordinateSystemOnPlane(normal_ptr->points[i], u, v);
			output->points[i].boundary_point = this->isBoundaryPoint(cloud_ptr, cloud_ptr->points[i], neighbor_idx, u, v, 0.9);
		}
		
	}
private:
	pcl::PointCloud<pcl::Normal>::Ptr compute_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud_ptr);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		ne.setSearchMethod(kdtree);
		ne.setKSearch(30);
		ne.compute(*normal_ptr);
		return normal_ptr;
	}

private:
	void getCoordinateSystemOnPlane(const pcl::Normal &p_coeff, Eigen::Vector4f &u, Eigen::Vector4f &v)
	{
		pcl::Vector4fMapConst p_coeff_v = p_coeff.getNormalVector4fMap();
		v = p_coeff_v.unitOrthogonal();
		u = p_coeff_v.cross3(v);
	}

	bool isBoundaryPoint(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &q_point,
		const std::vector<int> &indices,
		const Eigen::Vector4f &u, const Eigen::Vector4f &v,
		const float angle_threshold)
	{
		if (indices.size() < 3) return false;
		if (!std::isfinite(q_point.x) || !std::isfinite(q_point.y) || !std::isfinite(q_point.z)) return false;
		std::vector<float> angles(indices.size());
		float max_dif = FLT_MIN, dif;
		int cp = 0;
		for (const auto &index : indices)
		{
			if (!std::isfinite(cloud->points[index].x) ||
				!std::isfinite(cloud->points[index].y) ||
				!std::isfinite(cloud->points[index].z)) continue;
			Eigen::Vector4f delta = cloud->points[index].getVector4fMap() - q_point.getVector4fMap();
			if (delta == Eigen::Vector4f::Zero()) continue;
			angles[cp++] = std::atan2(v.dot(delta), u.dot(delta));
		}
		if (cp == 0) return false;
		angles.resize(cp);
		std::sort(angles.begin(), angles.end());
		for (size_t i = 0; i < angles.size(); i++)
		{
			dif = angles[i + 1] - angles[i];
			max_dif = std::max(max_dif, dif);
		}
		dif = 2 * static_cast<float> (M_PI) - angles.back() + angles[0];
		max_dif = std::max(max_dif, dif);
		return (max_dif > angle_threshold);
	}
};
