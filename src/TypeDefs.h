#ifndef UR_TYPEDEFS_H
#define UR_TYPEDEFS_H
#include <Eigen/Dense>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3d, double, cs::cartesian, x(), y(), z())

namespace ur
{
	// typedef boost::geometry::model::box<Eigen::Vector3d> Voxel;
	typedef boost::geometry::model::linestring<Eigen::Vector3d, std::vector, Eigen::aligned_allocator> LineString;
	typedef boost::geometry::model::polygon<Eigen::Vector3d, true, true, std::vector, std::vector,
	                                        Eigen::aligned_allocator, std::allocator> Polygon;
	typedef boost::geometry::model::multi_polygon<Polygon> MultiPolygon;

	class ExtrudedPolygon
	{
	public:
		ExtrudedPolygon(Polygon footprint, const double floor, const double ceiling):
			footprint(std::move(footprint)),
			floor(floor), ceiling(ceiling)
		{
		}

		const Polygon footprint;
		const double floor, ceiling;
	};
}
#endif // TYPEDEFS_H
