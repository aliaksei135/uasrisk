#ifndef TYPEDEFS_H
#define TYPEDEFS_H
#include <Eigen/Dense>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3d, double, cs::cartesian, x(), y(), z())

namespace ur
{
	typedef boost::geometry::model::box<Eigen::Vector3d> Voxel;
	typedef boost::geometry::model::linestring<Eigen::Vector3d> LineString;
	typedef boost::geometry::model::polygon<Eigen::Vector3d> Polygon;
	typedef boost::geometry::model::multi_polygon<Polygon> MultiPolygon;
}
#endif // TYPEDEFS_H
