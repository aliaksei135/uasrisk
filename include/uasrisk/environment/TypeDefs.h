#ifndef UR_TYPEDEFS_H
#define UR_TYPEDEFS_H

#include <unsupported/Eigen/CXX11/Tensor>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3f, double, cs::cartesian, x(), y(), z())

namespace ur
{
	/* Data Typedefs */

	// The Eigen Matrix type to use across all grid operations
	typedef Eigen::Tensor<float, 3> Matrix;
	// The Scalar data type of the Matrix
	typedef Matrix::Scalar FPScalar;

	// A 3D geospatial position in world coordinates in lon lat alt or xyz order
	typedef Eigen::Vector<FPScalar, 3> Position;
	typedef Eigen::Vector<FPScalar, 3> Vector;

	// An index directly into gridmap matrices in local indices in xyz order
	typedef Eigen::Array3i Index;

	// The xy size
	typedef Eigen::Array3i Size;
	typedef Eigen::Array<FPScalar, 3, 1> Length;
	typedef uint64_t Time;

	/* /Data Typedefs */


	/* Geometry Typedefs */
	// typedef boost::geometry::model::box<Eigen::Vector3d> Voxel;
	typedef boost::geometry::model::linestring<Eigen::Vector3f, std::vector, Eigen::aligned_allocator> LineString;
	typedef boost::geometry::model::polygon<Eigen::Vector3f, true, true, std::vector, std::vector,
	                                        Eigen::aligned_allocator, std::allocator> Polygon;
	typedef boost::geometry::model::multi_polygon<Polygon> MultiPolygon;

	class ExtrudedPolygon
	{
	public:
		ExtrudedPolygon(Polygon footprint, const FPScalar floor, const FPScalar ceiling):
			footprint(std::move(footprint)),
			floor(floor), ceiling(ceiling)
		{
		}

		const Polygon footprint;
		const FPScalar floor, ceiling;
	};

	/* /Geometry Typedefs */
}
#endif // TYPEDEFS_H
