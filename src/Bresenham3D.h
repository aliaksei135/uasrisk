#ifndef BRESENHAM3D_H
#define BRESENHAM3D_H
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace ur
{
	namespace util
	{
		/**
	 * @brief A 3D Implementation of the integer Bresenham algorithm
	*/
		class Bresenham3D
		{
		public:
			template <typename T = int_fast32_t>
			static std::vector<ur::Index, Eigen::aligned_allocator<ur::Index>> line3d(
				const ur::Index& start, const ur::Index& end)
			{
				// Collect results here
				std::vector<ur::Index, Eigen::aligned_allocator<ur::Index>> out;

				// Split out values for readability
				T x0 = start[0];
				T y0 = start[1];
				T z0 = start[2];
				T x1 = end[0];
				T y1 = end[1];
				T z1 = end[2];

				// global error terms
				T dx = x1 - x0;
				T dy = y1 - y0;
				T dz = z1 - z0;

				// global cell counts
				T l = abs(dx);
				T m = abs(dy);
				T n = abs(dz);

				// doubled global cell counts
				T l2 = 2 * l;
				T m2 = 2 * m;
				T n2 = 2 * n;

				// propagation signs
				int_fast8_t xs = dx > 0 ? 1 : -1;
				int_fast8_t ys = dy > 0 ? 1 : -1;
				int_fast8_t zs = dz > 0 ? 1 : -1;

				// current point coords
				T px = x0;
				T py = y0;
				T pz = z0;

				// reused local error terms
				T err1, err2;

				// reused iterator var
				uint_fast32_t i;

				if ((l >= m) && (l >= n))
				{
					// x driving
					err1 = m2 - l;
					err2 = n2 - l;
					for (i = 0; i < l; ++i)
					{
						out.emplace_back(px, py, pz);
						if (err1 > 0)
						{
							py += ys;
							err1 -= l2;
						}
						if (err2 > 0)
						{
							pz += zs;
							err2 -= l2;
						}
						err1 += m2;
						err2 += n2;
						px += xs;
					}
				}
				else if ((m >= l) && (m >= n))
				{
					// y driving
					err1 = l2 - m;
					err2 = n2 - m;
					for (i = 0; i < m; ++i)
					{
						out.emplace_back(px, py, pz);
						if (err1 > 0)
						{
							px += xs;
							err1 -= m2;
						}
						if (err2 > 0)
						{
							pz += zs;
							err2 -= m2;
						}
						err1 += l2;
						err2 += n2;
						px += ys;
					}
				}
				else
				{
					//z driving
					err1 = m2 - n;
					err2 = l2 - n;
					for (i = 0; i < n; ++i)
					{
						out.emplace_back(px, py, pz);
						if (err1 > 0)
						{
							py += ys;
							err1 -= n2;
						}
						if (err2 > 0)
						{
							px += xs;
							err2 -= n2;
						}
						err1 += m2;
						err2 += l2;
						px += zs;
					}
				}
				out.emplace_back(px, py, pz);
				return out;
			}
		};
	}
}
#endif // BRESENHAM3D_H
