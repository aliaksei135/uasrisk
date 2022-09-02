/* eigen_tensor_ext.h
 *
 * Functions extending functionality of the Eigen tensor library.
 * Functionality is focused on efficient slicing/chipping of tensors to Eigen
 * matrix representations.
 */

//MIT License
//
//Copyright (c) 2020 MattiasFredriksson
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//                                                          copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#pragma once
#include <Eigen/Geometry>
#include <unsupported/Eigen/CXX11/Tensor>
#include "Eigen/Dense"
#include "tensor_traits.h"

#pragma region Eigen alias
namespace tensorial {

	using EigenStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
	using InnerTStride = Eigen::Stride<1, Eigen::Dynamic>;

	// Tensor alias
	template <typename FP, int rank>
	using Tensor = Eigen::Tensor<FP, rank, Eigen::RowMajor>;

	template <typename FP, int rank>
	using TensorMap = Eigen::TensorMap<Eigen::Tensor<FP, rank, Eigen::RowMajor>>;
	template <typename FP, int rank>
	using TensorMapC =
		Eigen::TensorMap<const Eigen::Tensor<FP, rank, Eigen::RowMajor>>;

	template <typename FP, int rank>
	using TensorRef = Eigen::TensorRef<Eigen::Tensor<FP, rank, Eigen::RowMajor>>;
	template <typename FP, int rank>
	using TensorRefC =
		Eigen::TensorRef<const Eigen::Tensor<FP, rank, Eigen::RowMajor>>;

	template <int rank>
	using Tensord = Tensor<double, rank>;
	template <int rank>
	using Tensorf = Tensor<float, rank>;

	template <typename FP = double, int dim = Eigen::Dynamic>
	using Vector = Eigen::Matrix<FP, dim, 1, Eigen::ColMajor>;
	template <typename FP = double>
	using Vector2 = Eigen::Matrix<FP, 2, 1, Eigen::ColMajor>;
	template <typename FP = double>
	using Vector3 = Eigen::Matrix<FP, 3, 1, Eigen::ColMajor>;
	template <typename FP = double>
	using Vector4 = Eigen::Matrix<FP, 4, 1, Eigen::ColMajor>;

	template <typename FP = double, int dim = Eigen::Dynamic>
	using RowVector = Eigen::Matrix<FP, 1, dim, Eigen::RowMajor>;
	template <typename FP = double>
	using RowVector2 = Eigen::Matrix<FP, 1, 2, Eigen::RowMajor>;
	template <typename FP = double>
	using RowVector3 = Eigen::Matrix<FP, 1, 3, Eigen::RowMajor>;
	template <typename FP = double>
	using RowVector4 = Eigen::Matrix<FP, 1, 4, Eigen::RowMajor>;

	template <typename FP = double>
	using Matrix3 = Eigen::Matrix<FP, 3, 3, Eigen::RowMajor>;
	template <typename FP = double>
	using Matrix4 = Eigen::Matrix<FP, 4, 4, Eigen::RowMajor>;

	template <typename FP = double>
	using MatrixN2 = Eigen::Matrix<FP, Eigen::Dynamic, 2, Eigen::RowMajor>;
	template <typename FP = double>
	using MatrixN3 = Eigen::Matrix<FP, Eigen::Dynamic, 3, Eigen::RowMajor>;
	template <typename FP = double>
	using MatrixN4 = Eigen::Matrix<FP, Eigen::Dynamic, 4, Eigen::RowMajor>;
	template <typename FP = double,
		int rows = Eigen::Dynamic,
		int cols = Eigen::Dynamic>
		using MatrixNN = Eigen::Matrix<FP, rows, cols, Eigen::RowMajor>;
	template <typename FP = double, int width = Eigen::Dynamic>
	using Matrix = MatrixNN<FP, width, width>;

	template <typename FP = double>
	using MatrixMapN2 = Eigen::Map<MatrixN2<FP>>;
	template <typename FP = double>
	using MatrixMapN3 = Eigen::Map<MatrixN3<FP>>;
	template <typename FP = double>
	using MatrixMapN4 = Eigen::Map<MatrixN4<FP>>;
	template <typename FP = double>
	using MatrixMapNN = Eigen::Map<MatrixNN<FP>>;
	template <typename FP = double>
	using MatrixMapNNC = Eigen::Map<const MatrixNN<FP>, Eigen::RowMajor>;

	template <typename FP = double>
	using MatrixRefN2 = Eigen::Ref<MatrixN2<FP>>;
	template <typename FP = double>
	using MatrixRefN3 = Eigen::Ref<MatrixN3<FP>>;
	template <typename FP = double>
	using MatrixRefN4 = Eigen::Ref<MatrixN4<FP>>;
	template <typename FP = double>
	using MatrixRefNN = Eigen::Ref<MatrixNN<FP>>;

	template <typename FP = double>
	using MapN2 = Eigen::Map<Eigen::Matrix<FP, Eigen::Dynamic, 2, Eigen::RowMajor>>;
	template <typename FP = double>
	using MapNNC = Eigen::Map<
		const Eigen::Matrix<FP, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

	template <typename FP = double>
	using RefN2 = Eigen::Ref<Eigen::Matrix<FP, Eigen::Dynamic, 2, Eigen::RowMajor>>;
	template <typename FP = double>
	using RefNNC = Eigen::Ref<
		const Eigen::Matrix<FP, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

	/* Note:
	Eigen quaternions are ordered [x,y,z,w] in memory with the scalar last, this is
	the same order as Scipy.Rotatin module.
	*/
	template <typename FP = double>
	using Quat = Eigen::Quaternion<FP>;
#pragma endregion

#pragma region slice

	/**
	 * <summary>Convert dense tensor to matrix.</summary>
	 */
	template <typename Scalar, int rank, typename sizeType>
	auto tensor2matrix(const Eigen::Tensor<Scalar, rank>& tensor) {
		return Eigen::Map<const MatrixType<Scalar>>(
			tensor.data(), tensor.dimension(0), tensor.dimension(1));
	}
	/**
	 * <summary>Convert matrix map to tensor.</summary>
	 */
	template <typename Scalar, int Rows, int Cols, int Major>
	auto matrix2tensor(
		const Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Major>>& matrix) {
		return Eigen::TensorMap<Eigen::Tensor<Scalar, 2, Major>>(
			matrix.data(), matrix.rows(), matrix.cols());
	}
	/**
	 * <summary>Convert dense matrix to tensor.</summary>
	 */
	template <typename Scalar, int Rows, int Cols, int Major>
	auto matrix2tensor(Eigen::Matrix<Scalar, Rows, Cols, Major>& matrix) {
		return Eigen::TensorMap<Eigen::Tensor<Scalar, 2, Major>>(
			matrix.data(), matrix.rows(), matrix.cols());
	}
	/**
	 * <summary>Convert vector map to tensor.</summary>
	 */
	template <typename Scalar, int Rows, int Cols, int Major>
	auto vector2tensor(
		const Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Major>>& vector) {
		return Eigen::TensorMap<Eigen::Tensor<Scalar, 1, Eigen::RowMajor>>(
			vector.data(), vector.size());
	}
	/**
	 * <summary>Convert dense vector to tensor.</summary>
	 */
	template <typename Scalar, int Rows, int Cols, int Major>
	auto vector2tensor(Eigen::Matrix<Scalar, Rows, Cols, Major>& vector) {
		return Eigen::TensorMap<Eigen::Tensor<Scalar, 1, Eigen::RowMajor>>(
			vector.data(), vector.size());
	}

#pragma region Helpers

	/*	Unpacks variadic arg into a std::array of type T and size N.
	 */
	template <class T, size_t N, class... Values>
	void assign_values(std::array<T, N>& arr, Values... vals) {
		static_assert(N ==
			sizeof...(vals));  // assert variadic count match array size N
		int j = 0;
		for (auto i : std::initializer_list<std::common_type_t<Values...>>{ vals... })
			arr[j++] = i;
	}

	/*	Index offset for the N first dimensions for row/column tensors. (Column
	 * untested).
	 */
	template <typename TensorType, typename... Ix>
	std::int64_t tensor_offset(const TensorType& t, Ix... index) {
		constexpr bool isrow = is_eigen_row_major_tensor<TensorType>::value;
		constexpr std::int64_t N = std::int64_t{ sizeof...(Ix) };
		constexpr std::int64_t incr = isrow ? -1 : 1;
		constexpr std::int64_t beg = isrow ? TensorType::NumIndices - 1 : 0;
		constexpr std::int64_t end = isrow ? N - 1 : 0;
		constexpr std::int64_t last = isrow ? -1 : N;
		static_assert(
			N <= TensorType::NumIndices,
			"Invalid number of indices");  // , expected at most" +
										   // std::to_string(TensorType::NumIndices));
		std::int64_t cum_stride = 1;
		for (std::int64_t i = beg; i != end;
			i += incr)  // Only iterated for row tensors.
			cum_stride *= t.dimension(i);

		// Make variadic arg -> array
		std::array<std::int64_t, N> inds;
		assign_values(inds, index...);

		std::int64_t off = 0;
		for (std::int64_t i = end; i != last; i += incr) {
			off += inds[i] * cum_stride;
			cum_stride *= t.dimension(i);
		}
		return off;
	}
	template <typename TensorType>
	std::int64_t tensor_offset(const TensorType& t) {
		/* Tensor of rank <= 2
		*/
		return 0;
	}

#pragma endregion

#pragma region slice row major column vector

	/**
	 * <summary>Get a column vector slice (subtensor) from a row major tensor.</summary>
	 * <param name="tensor">Row major tensor of rank N.</param>
	 * <param name="slice_offsets"> Offset indices for the subtensor within the first N-1 rank dimensions.</param>
	 * <returns>A mapped matrix view of the subtensor slice.</returns>
	 */
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		std::negation<is_eigen_mutable_tensor<TensorType>>>::value,
		int> = 0>
		Eigen::Map<
		const Eigen::
		Matrix<typename TensorType::Scalar, Eigen::Dynamic, 1, Eigen::ColMajor>>
		slice_vector(
			TensorType& tensor,
			typename Ix... slice_offset) {  // slice_offsets[TensorType::Dimensions - 2]
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(
			std::size_t{ sizeof...(Ix) } == TensorType::NumIndices - 1,
			"Incorrect number of indices passed to slice function.");  //, expected "
																	   //+
																	   // std::to_string(TensorType::NumIndices
																	   //- 1));
		Eigen::Index d1 = tensor.dimension(TensorType::NumIndices - 1);
		Eigen::Stride<0, 0> stride;
		// Calc. offset
		Eigen::Index offset = tensor_offset(tensor, slice_offset...);

		return Eigen::Map<const Eigen::Matrix<TensorType::Scalar, Eigen::Dynamic, 1,
			Eigen::ColMajor>>(
				tensor.data() + offset, d1, 1, stride);
	}
	/**
	 * <summary>Get a column vector slice (subtensor) from a row major
	 * tensor.</summary> <param name="tensor"> Row major tensor of rank N.</param>
	 * <param name="slice_offsets"> Offset indices for the subtensor within the
	 * first N-2 rank dimensions.</param> <returns>A mapped matrix view of the
	 * subtensor slice.</returns>
	 */
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<is_eigen_row_major_tensor<TensorType>,
		is_eigen_mutable_tensor<TensorType>>::value,
		int> = 0>
		Eigen::Map<
		Eigen::
		Matrix<typename TensorType::Scalar, Eigen::Dynamic, 1, Eigen::ColMajor>>
		slice_vector(TensorType& tensor, typename Ix... slice_offset) {
		// Return const. version
		return *reinterpret_cast<Eigen::Map<Eigen::Matrix<
			typename TensorType::Scalar, Eigen::Dynamic, 1, Eigen::ColMajor>>*>(
				&slice_vector(tensor, slice_offset...));
	}

#pragma endregion

#pragma region slice matrix

#pragma region slice row major matrix

	namespace intern {

		template <typename TensorType>
		inline EigenStride tensor_stride_slice_rowm_rowt(const TensorType& tensor, Eigen::Index& d1, Eigen::Index& d2) {
			d1 = tensor.dimension(TensorType::NumIndices - 2);
			d2 = tensor.dimension(TensorType::NumIndices - 1);
			return EigenStride(d2, 1);
		}

		template <typename TensorType>
		inline EigenStride tensor_stride_slice_colm_rowt(const TensorType& tensor, Eigen::Index& d1, Eigen::Index& d2) {
			Eigen::Index d1 = tensor.dimension(TensorType::NumIndices - 2);
			Eigen::Index d2 = tensor.dimension(TensorType::NumIndices - 1);
			return EigenStride(1, d1);
		}

		/* Slice return types
		*/

		// RR:Row-Row return types
		template <typename TensorType>
		using SliceRowRowMRType =
			Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
		template <typename TensorType>
		using SliceMatrixRRCReturnType =
			const Eigen::Map<const SliceRowRowMRType<TensorType>, Eigen::RowMajor, EigenStride>;
		template <typename TensorType>
		using SliceMatrixRRReturnType =
			Eigen::Map<SliceRowRowMRType<TensorType>, Eigen::RowMajor, EigenStride>;

	}

#pragma region slice row matrix from row tensor

#pragma region const
	/**
	 * <summary>Get a row major matrix slice (or subtensor) from a row major tensor.</summary>
	 * <param name="tensor"> Row major tensor of rank N.</param>
	 * <param name="slice_offsets"> Offset indices for the subtensor within the first N-2 rank dimensions.</param>
	 * <returns>A mapped matrix view of the subtensor slice.</returns>
	 */
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_const_tensor<TensorType>
		>::value,
		int> = 0>
		intern::SliceMatrixRRCReturnType<TensorType>
		slice_matrix(TensorType& tensor, typename Ix... slice_offset) {
		// Asserts
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(
			std::size_t{ sizeof...(Ix) } == TensorType::NumIndices - 2,
			"Incorrect number of indices passed to slice function, expected Rank - 2.");
		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = intern::tensor_stride_slice_rowm_rowt(tensor, d1, d2);
		// Offset
		Eigen::Index offset = tensor_offset(tensor, slice_offset...);
		// Make const
		return intern::SliceMatrixRRCReturnType<TensorType>(tensor.data() + offset, d1, d2, stride);
	}

	/* <summary>Get a row major matrix view from a tensor with 2 rank dimensions.< / summary>
	*/
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_const_tensor<TensorType>
		>::value,
		int> = 0 >
		intern::SliceMatrixRRCReturnType<TensorType>
		slice_matrix(TensorType& tensor) {
		// Asserts
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(TensorType::NumIndices == 2,
			"Expected tensor of rank dimension 2.");
		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = intern::tensor_stride_slice_rowm_rowt(tensor, d1, d2);
		// Make const
		return intern::SliceMatrixRRCReturnType<TensorType>(tensor.data(), d1, d2, stride);
	}

#pragma endregion

#pragma region non-const

	/**
	 * <summary>Get a row major matrix slice (subtensor) from a row major tensor.</summary>
	 * <param name="tensor"> Row major tensor of rank N.</param>
	 * <param name="slice_offsets"> Offset indices for the subtensor within the first N-2 rank dimensions.</param>
	 * <returns>A mapped matrix view of the subtensor slice.</returns>
	 */
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_mutable_tensor<TensorType>>::value,
		int> = 0>
		intern::SliceMatrixRRReturnType<TensorType>
		slice_matrix(TensorType& tensor, typename Ix... slice_offset) {
		// Asserts
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(
			std::size_t{ sizeof...(Ix) } == TensorType::NumIndices - 2,
			"Incorrect number of indices passed to slice function, expected Rank - 2.");
		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = intern::tensor_stride_slice_rowm_rowt(tensor, d1, d2);
		// Offset
		Eigen::Index offset = tensor_offset(tensor, slice_offset...);
		// Make non-const
		return intern::SliceMatrixRRReturnType<TensorType>(tensor.data() + offset, d1, d2, stride);
	}

	/* <summary>Get a row major matrix view from a tensor with 2 rank dimensions.< / summary>
	*/
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_mutable_tensor<TensorType>>::value,
		int> = 0>
		intern::SliceMatrixRRReturnType<TensorType>
		slice_matrix(TensorType& tensor) {
		// Asserts
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(TensorType::NumIndices == 2,
			"Number of indices should match the tensor rank dimension - 2.");

		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = intern::tensor_stride_slice_rowm_rowt(tensor, d1, d2);
		// Make non-const
		return intern::SliceMatrixRRReturnType<TensorType>(tensor.data(), d1, d2, stride);
	}

#pragma endregion

#pragma endregion

#pragma region slice column matrix from row tensor

	/**
	 * <summary>Get a column major matrix slice (subtensor) from a row major tensor.</summary>
	 * <param name="tensor"> Row major tensor of rank N.</param>
	 * <param name="slice_offsets"> Offset indices for the subtensor within the first N-2 rank dimensions.</param>
	 * <returns>A mapped matrix view of the subtensor slice.</returns>
	 */
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		std::negation<is_eigen_mutable_tensor<TensorType>>>::value,
		int> = 0>
		const Eigen::Map<const Eigen::Matrix<typename TensorType::Scalar,
		Eigen::Dynamic,
		Eigen::Dynamic,
		Eigen::ColMajor>,
		Eigen::ColMajor,
		EigenStride>
		slice_matrix_col(TensorType& tensor, typename Ix... slice_offset) {
		// Asserts
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(
			std::size_t{ sizeof...(Ix) } == TensorType::NumIndices - 2,
			"Invalid number of indices");

		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = tensor_stride_slice_colm_rowt(tensor, d1, d2);
		// Offset
		Eigen::Index offset = tensor_offset(tensor, slice_offset...);
		// Make const
		using DenseRowMajorMatrix = Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
		using MappedConstDenseType = Eigen::Map<DenseRowMajorMatrix, Eigen::ColMajor, EigenStride>;
		return MappedConstDenseType(tensor.data() + offset, d1, d2, stride);
	}
	/**
	 * <summary>Get a column major matrix slice (subtensor) from a row major tensor.</summary>
	 * <param name="tensor"> Row major tensor of rank N.</param>
	 * <param name="slice_offsets"> Offset indices for the subtensor within the first N-2 rank dimensions.</param>
	 * <returns>A mapped matrix view of the subtensor slice.</returns>
	 */
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_mutable_tensor<TensorType>>::value,
		int> = 0>
		Eigen::Map<Eigen::Matrix<typename TensorType::Scalar,
		Eigen::Dynamic,
		Eigen::Dynamic,
		Eigen::ColMajor>,
		Eigen::ColMajor,
		EigenStride>
		slice_matrix_col(TensorType& tensor, typename Ix... slice_offset) {

		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = tensor_stride_slice_colm_rowt(tensor, d1, d2);
		// Offset
		Eigen::Index offset = tensor_offset(tensor, slice_offset...);
		// Make non-const
		using DenseRowMajorMatrix = Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
		using MappedDenseType = Eigen::Map<DenseRowMajorMatrix, Eigen::ColMajor, EigenStride>;
		return MappedDenseType(tensor.data() + offset, d1, d2, stride);
	}


	/* <summary>Get a column major matrix view from a row major tensor with 2 rank dimensions.< / summary>
	*/
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<
		is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		std::negation<is_eigen_mutable_tensor<TensorType>>>::value,
		int> = 0>
		const Eigen::Map<const Eigen::Matrix<typename TensorType::Scalar,
		Eigen::Dynamic,
		Eigen::Dynamic,
		Eigen::ColMajor>,
		Eigen::ColMajor,
		EigenStride>
		slice_matrix_col(TensorType& tensor) {
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(TensorType::NumIndices == 2,
			"Number of indices should match the tensor rank dimension - 2.");

		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = tensor_stride_slice_colm_rowt(tensor, d1, d2);
		// Make const
		using DenseRowMajorMatrix = Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
		using MappedConstDenseType = Eigen::Map<DenseRowMajorMatrix, Eigen::ColMajor, EigenStride>;
		return MappedConstDenseType(tensor.data(), d1, d2, stride);
	}

	/* <summary>Get a column major matrix view from a row major tensor with 2 rank dimensions.< / summary>
	*/
	template <typename TensorType,
		typename... Ix,
		std::enable_if_t<
		std::conjunction<is_eigen_row_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_mutable_tensor<TensorType>>::value,
		int> = 0>
		Eigen::Map<Eigen::Matrix<typename TensorType::Scalar,
		Eigen::Dynamic,
		Eigen::Dynamic,
		Eigen::ColMajor>,
		Eigen::ColMajor,
		EigenStride>
		slice_matrix_col(TensorType& tensor) {
		static_assert(TensorType::Layout == Eigen::RowMajor,
			"Invalid tensor layout type, expected tensor to be row major.");
		static_assert(TensorType::NumIndices == 2,
			"Number of indices should match the tensor rank dimension - 2.");

		// Stride
		Eigen::Index d1, d2;
		EigenStride stride = tensor_stride_slice_colm_rowt(tensor, d1, d2);
		// Make non-const
		using DenseRowMajorMatrix = Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
		using MappedDenseType = Eigen::Map<DenseRowMajorMatrix, Eigen::ColMajor, EigenStride>;
		return MappedDenseType(tensor.data(), d1, d2, stride);
	}

#pragma endregion

#pragma endregion

#pragma region slice column major matrix

	/**
	 * const *
	 * <summary>Get a matrix slice from a column major tensor of rank 3.</summary>
	 * <param name="tensor"> Column major tensor of rank 3.</param>
	 * <param name="index"> Index of the (matrix) sub-tensor within the tensor.</param>
	 * <returns>A matrix map of the sub-tensor slice.</returns>
	 */
	template <typename TensorType,
		std::enable_if_t<
		std::conjunction<
		is_eigen_col_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		std::negation<is_eigen_mutable_tensor<TensorType>>>::value,
		int> = 0>
		Eigen::Map<const Eigen::Matrix<typename TensorType::Scalar,
		Eigen::Dynamic,
		Eigen::Dynamic>,
		Eigen::ColMajor,
		Eigen::InnerStride<>>
		slice_matrix_col(TensorType tensor, Eigen::Index index) {
		static_assert(is_eigen_mappable_tensor<TensorType>::value); // Only slice dense types.
		static_assert(TensorType::NumIndices == 3,
			"This function currently only support 3-dim tensors.");
		static_assert(
			TensorType::Layout == Eigen::ColMajor,
			"Invalid tensor layout type, expected tensor to be column major.");
		auto d1 = tensor.dimension(TensorType::NumIndices - 2);
		auto d2 = tensor.dimension(TensorType::NumIndices - 1);
		Eigen::InnerStride<> stride(tensor.dimension(0));
		return Eigen::Map<
			const Eigen::Matrix<TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic>,
			Eigen::ColMajor, Eigen::InnerStride<>>(tensor.data() + index, d1, d2,
				stride);
	}

	/**
	 * <summary>Get a matrix slice from a column major tensor of rank 3.</summary>
	 * <param name="tensor"> Column major tensor of rank 3.</param>
	 * <param name="index"> Index of the (matrix) sub-tensor within the
	 * tensor.</param> <returns>A matrix map of the sub-tensor slice.</returns>
	 */
	template <typename TensorType,
		std::enable_if_t<
		std::conjunction<
		is_eigen_col_major_tensor<TensorType>,
		is_eigen_mappable_tensor<TensorType>,
		is_eigen_mutable_tensor<TensorType>>::value,
		int> = 0>
		Eigen::Map<
		Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic>,
		Eigen::ColMajor,
		Eigen::InnerStride<>>
		slice_matrix_col(TensorType tensor, Eigen::Index index) {

		// Non-const nested type
		auto value{ slice_matrix_col(const_cast<const TensorType&>(tensor), index) };
		Eigen::InnerStride<> stride(tensor.dimension(0));

		using DenseRowMajorMatrix = Eigen::Matrix<typename TensorType::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
		using MappedDenseType = Eigen::Map<DenseRowMajorMatrix, Eigen::ColMajor, Eigen::InnerStride<>>;
		return MappedDenseType(const_cast<typename TensorType::Scalar*>(value.data()), value.rows(), value.cols(), stride);
	}

#pragma endregion

#pragma endregion
#pragma endregion

#pragma region buffer

	/* Native formats, uses the same character set as python's struct lib, see:
	* https://docs.python.org/3/library/struct.html#format-characters
	*
	* Default entry 'E' defines unsupported types!
	*/

	template<typename NT> struct NativeTypeChar { static const char fmat = 'E'; };
	template<> struct NativeTypeChar<std::int8_t> { static const  char fmat = 'b'; };
	template<> struct NativeTypeChar<std::uint8_t> { static const char fmat = 'B'; };
	template<> struct NativeTypeChar<std::int16_t> { static const char fmat = 'h'; };
	template<> struct NativeTypeChar<std::uint16_t> { static const char fmat = 'H'; };
	template<> struct NativeTypeChar<std::int32_t> { static const char fmat = 'i'; };
	template<> struct NativeTypeChar<std::uint32_t> { static const char fmat = 'I'; };
	template<> struct NativeTypeChar<std::int64_t> { static const char fmat = 'q'; };
	template<> struct NativeTypeChar<std::uint64_t> { static const char fmat = 'Q'; };
	template<> struct NativeTypeChar<float> { static const char fmat = 'f'; };
	template<> struct NativeTypeChar<double> { static const char fmat = 'd'; };

	/* Wrapper for Eigen::Tensor with shared ownership in similarity to std::shared_pointer<...>.
	 * Intended use is for Tensors with undefined lifetime such as when tied to both Python and CPP side.
	 * Implementation provides a lightweight wrapper for natively typed tensors with python bindings provided
	 * in <this_file>_pyb.h. While providing similar behavior as py::array_t or other python/numpy types the
	 * implementation is CPP friendly and interchangable with an eigen tensor.
	 *
	 * Currently uses std::shared_ptr to implement handle object lifetime but this may change in the future.
	*/
	template<typename FP, int rank> class TensorWrapper {
	public:

		using Type = TensorWrapper<FP, rank>;
		using TensorType = Tensor<FP, rank>;
		using ViewType = TensorMap<FP, rank>;
		using ViewTypeC = TensorMapC<FP, rank>;

		/* Empty invalid buffer.
		*/
		TensorWrapper() :
			buffer_() { }

		TensorWrapper(std::array<std::int64_t, rank> shape) :
			buffer_(new TensorType(shape))
		{
			for (int i = 0; i < rank; i++)
			{
				assert(shape[i] != 0);
			}
		}

		TensorWrapper(const TensorWrapper& copy) :
			buffer_(copy.buffer_) {}

		TensorWrapper(TensorWrapper&& move) = delete;

		~TensorWrapper() = default;

		TensorWrapper& operator=(const TensorWrapper& cpy) {
			if (this == &cpy) {
				return *this;
			}
			assert(cpy.buffer_);
			buffer_ = cpy.buffer_;
			return *this;
		}

		/* Access a 2-dimensional subtensor given offsets.
		*/
		template<typename... Ix>
		auto matrix(typename Ix... slice_offset) {
			assert(buffer_);
			return slice_matrix(*buffer_, slice_offset...);
		}

		/* Access a 2-dimensional subtensor given offsets.
		*/
		template<typename... Ix>
		auto matrix(typename Ix... slice_offset) const {
			assert(buffer_);
			return slice_matrix(*buffer_, slice_offset...);
		}

		/* Access the underlying tensor.
		 */
		TensorType& data() {
			assert(buffer_);
			return *buffer_;
		}

		/* Access the underlying tensor.
		 */
		const TensorType& data() const {
			assert(buffer_);
			return *buffer_;
		}

		/* Access the underlying tensor.
		 */
		TensorType& operator*() {
			return data();
		}

		/* Access the underlying tensor.
		 */
		const TensorType& operator*() const {
			return data();
		}

		/* Access a sharable pointer to the underlying tensor.
		 */
		std::shared_ptr<TensorType> shared() {
			assert(buffer_);
			return buffer_;
		}

		/* Get a view over the underlying tensor.
		*/
		ViewType view() {
			assert(buffer_);
			return ViewType(buffer_->data(), buffer_->dimensions());
		}

		/* Get a readonly view over the underlying tensor.
		*/
		ViewTypeC view() const {
			assert(buffer_);
			return ViewTypeC(buffer_->data(), buffer_->dimensions());
		}

		/* Shape of the specific rank dimension.
		*/
		Eigen::Index dimension(std::size_t rank_dim) const {
			assert(rank_dim < rank);
			return buffer_->dimension(rank_dim);
		}

		/* Total number of elements. Cumultative product of shape dimensions.
		*/
		Eigen::Index size() const {
			return buffer_->size();
		}

		/* Tensor shape
		*/
		auto dimensions() const {
			return buffer_->dimensions();
		}

		FP* raw() {
			return buffer_->data();
		}

	private:
		std::shared_ptr<TensorType> buffer_;
	};


	template<typename FP> class MatrixBuffer {
	public:
		using Type = MatrixBuffer<FP>;
		using MatrixType = MatrixNN<FP>;
		using ViewType = MatrixMapNN<FP>;

		MatrixBuffer(std::int64_t rows, std::int64_t cols) :
			buffer_(new MatrixType(rows, cols)) {}

		MatrixBuffer(const MatrixBuffer& copy) :
			buffer_(copy.buffer_) {}

		MatrixBuffer(MatrixBuffer&& move) = delete;

		~MatrixBuffer() = default;

		/* Access the underlying matrix.
		 */
		ViewType buffer() {
			return ViewType(*buffer_);
		}

		std::shared_ptr<MatrixType> shared() {
			return buffer_;
		}

	private:
		std::shared_ptr<MatrixType> buffer_;
	};

#pragma endregion

#pragma region tensoriterator

	/**
	 * <summary>Iterator providing access to subtensors along the first rank dimension of a dense tensor type.
	 * </summary>
	 */
	template <typename TensorType>
	struct tensoriterator {
		/* Assert dense type */
		static_assert(is_eigen_mappable_tensor<TensorType>::value);
	public:
		using Scalar = typename TensorType::Scalar;
		static const int Rank = TensorType::NumIndices;

		static const bool is_const_tensor =
			std::is_const<typename eigen_nested_type<TensorType>::type>::value;

		// Dense type
		using DenseTensorType = typename std::conditional<
			std::is_const<typename eigen_nested_type<TensorType>::type>::value,
			typename const Tensor<Scalar, Rank>,
			typename Tensor<Scalar, Rank>>::type;

		// Mapped tensor type
		using MapTensorT = typename std::conditional<
			std::is_const<typename eigen_nested_type<TensorType>::type>::value,
			typename TensorMapC<Scalar, Rank>,
			typename TensorMap<Scalar, Rank>>::type;

		// Subtensor type
		using SubTensorT = typename std::conditional<
			std::is_const<typename eigen_nested_type<TensorType>::type>::value,
			typename TensorMapC<Scalar, Rank - 1>,
			typename TensorMap<Scalar, Rank - 1>>::type;


		// Parent type
		using ParentTensorT = typename std::conditional<
			std::is_const<typename eigen_nested_type<TensorType>::type>::value,
			typename TensorMapC<Scalar, Rank + 1>,
			typename TensorMap<Scalar, Rank + 1>>::type;


	private:
		/* Mapped tensor. */
		MapTensorT m_tensor;
		Eigen::Index m_stride;
		Eigen::array<Eigen::DenseIndex, Rank - 1> m_shape;

		void init() {
			assert(this->m_stride == 1);
			// Store subtensor shape
			for (size_t i = 1; i < Rank; i++) {
				Eigen::Index d = m_tensor.dimension(i);
				this->m_shape[i - 1] = d;
				this->m_stride *= d;
			}
		}

	public:

		/*
		 * <summary>RH constructor == non-reference, non-dense constructor. </summary>
		 */
		tensoriterator(TensorType&& tensor)
			: m_tensor(tensor.data(), tensor.dimensions()), m_stride(1) {
			init();
		}

		/*
		 * <summary>LH constructor == reference constructors (dense and non-dense).</summary>
		 */
		tensoriterator(TensorType& tensor)
			: m_tensor(tensor.data(), tensor.dimensions()), m_stride(1) {
			init();
		}

		/*
		 * <summary>Get the size of the iterated dimension. </summary>
		 */
		size_t count() { return m_tensor.dimension(0); }
		/*
		 * <summary> Deprecated Get the size of the iterated dimension.</summary>
		 */
		size_t size() {
			static_assert(false);
			return count();
		}



		const Eigen::array<Eigen::DenseIndex, Rank - 1>& shape() const {
			return this->m_shape;
		}

		Eigen::Index stride() const { return this->m_stride; }

		/*	Wrapper for TensorMap where the struct is replaced on assignment
			instead of overwriting the data buffer. Meaning that the default
			behavior for assignment is to replace the subtensor instance.

			Replace subtensor view variable:

				new(&this->tensor) Tensor<FP, rank - 1>(other.tensor);

			----

			To overwrite data in the underlying tensor use the iterator:

				tensoriterator iter(..);
				iter(i) = iter(j);

			or assign directly to the tensor variable:

				subtensor subt;
				subt = some_matching_tensor;
				subt = other_subt.tensor();

		*/
		struct subtensor {

		private:
			/* View over data in the reference tensor. */
			SubTensorT m_tensor;

		public:
			subtensor(SubTensorT tensor) : m_tensor(tensor) {}

			/* Implicit tensor conversion
			*/
			operator SubTensorT& () { return this->m_tensor; }
			operator const SubTensorT& () const { return this->m_tensor; }

			/* Access tensor reference.
			*/
			SubTensorT& ref() { return this->m_tensor; }
			const SubTensorT& ref() const { return this->m_tensor; }
			SubTensorT& tensor() { return this->m_tensor; }
			const SubTensorT& tensor() const { return this->m_tensor; }

#pragma region iterator compabatible functions

			subtensor operator++() {
				new (&this->m_tensor) SubTensorT(m_tensor.data() + m_tensor.size(), m_tensor.dimensions());
				return *this;
			}

			bool operator==(const subtensor& other) {
				return m_tensor.data() == other.m_tensor.data();
			}

			bool operator!=(const subtensor& other) {
				return m_tensor.data() != other.m_tensor.data();
			}

			subtensor operator*() {
				return m_tensor;
			}

#pragma endregion

#pragma region forward Eigen::Tensor accessors

			Eigen::Index size() {
				return m_tensor.size();
			}

			Scalar* data() {
				return m_tensor.data();
			}

#pragma endregion

			/* assign the tensor to the underlying buffer.
			*/
			subtensor& operator=(const SubTensorT& assignee) noexcept {
				if (assignee.data() == this->data())
					return *this;
				this->m_tensor = assignee;
				return *this;
			}

			tensoriterator<SubTensorT> iter() {
				return tensoriterator<SubTensorT>(this->tensor());
			}

#pragma region subtensor : copy & assign

			subtensor(const subtensor& o) : m_tensor(o.m_tensor) { }
			subtensor(subtensor&& o) : m_tensor(std::move(o.m_tensor)) { }

			subtensor& operator=(subtensor& other) = delete;
			subtensor& operator=(const subtensor& other) noexcept {
				if (&other == this)
					return *this;
				new (&this->m_tensor) SubTensorT(other.m_tensor);
				return *this;
			}
			subtensor& operator=(subtensor&& other) noexcept {
				if (&other == this)
					return *this;
				new (&this->m_tensor) SubTensorT(other.m_tensor);
				return *this;
			}

#pragma endregion
		};


		/**
		 * <summary>Get a replaceable view of the indexed subtensor in the first rank
		 *			dimension of the reference.</summary>
		 * <param name="index"> Index of the subtensor within the first rank dimension.</param>
		 * <returns>Replaceable view of the subtensor. A view allow assignment of asubtensor variable
		 *			without overwriting the underlying tensor.
		 * </returns>
		 */
		tensoriterator::subtensor operator[](const Eigen::Index index) const {
			return tensoriterator::subtensor(this->operator()(index));
		}
		/**
		 * <summary>Get a view of the subtensor in the first rank dimension of the reference tensor.</summary>
		 * <param name="index">Index of the subtensor within the first rank dimension.</param>
		 * <returns>View over indexed subtensor.</returns>
		 */
		SubTensorT operator()(const Eigen::Index index) const {
			return SubTensorT(m_tensor.data() + index * this->m_stride, this->m_shape);
		}

		tensoriterator::subtensor begin() {
			return tensoriterator::subtensor(this->operator()(0));
		}

		tensoriterator::subtensor end() {
			return tensoriterator::subtensor(this->operator()(m_tensor.dimension(0)));
		}

#pragma region copy & assign

		tensoriterator(const tensoriterator& o)
			: m_tensor(o.tensor), m_stride(o.m_stride), m_shape(o.m_shape) {}

		tensoriterator(tensoriterator&& o)
			: m_tensor(std::move(o.m_tensor)),
			m_stride(o.m_stride),
			m_shape(std::move(o.m_shape)) {}

		tensoriterator& operator=(tensoriterator& other) = delete;

		tensoriterator& operator=(const tensoriterator& other) noexcept {
			if (&other == this)
				return *this;
			new (&this->m_tensor) SubTensorT(other.m_tensor);
			this->m_stride = other.m_stride;
			this->m_shape = other.m_shape;
			return *this;
		}

		tensoriterator& operator=(tensoriterator&& other) noexcept {
			if (&other == this)
				return *this;
			this->m_tensor = std::move(other.m_tensor);
			this->m_stride = other.m_stride;
			this->m_shape = std::move(other.m_shape);
			return *this;
		}

#pragma endregion
	};

#pragma endregion

#pragma region helper functions

	/*
	 *	Verify the number of dimensions in given rank dimension match.
	 */
	template <typename T0>
	void assert_rank_dimension(const T0& t, int rank, int dimension) {
		if (t.dimension(rank) != dimension) {
			std::ostringstream buf;
			buf << "Expected tensor argument to be of dimension " << dimension
				<< " in rank dimension " << rank << ", was " << t.dimension(rank);
			buf << " for tensor argument with shape (" << t.dimensions() << ").";
			throw std::invalid_argument(buf.str());
		}
	}

	/*
	 *	Verify the number of dimensions within the given rank of the tensor
	 *arguments match.
	 */
	template <typename T0, typename T1>
	void assert_rank_dimension_match(const T0& t0, const T1& t1, int rank) {
		if (t0.dimension(rank) != t1.dimension(rank)) {
			std::ostringstream buf;
			buf << "Mismatch in dimensions within rank " << rank
				<< " of the tensor arguments. First argument was of shape: (";
			buf << t0.dimensions() << "), second argument of shape (" << t1.dimensions()
				<< ").";
			throw std::invalid_argument(buf.str());
		}
	}
	/*
	 *	Verify the number of dimensions within the given rank of the tensor
	 *arguments match.
	 */
	template <typename T0, typename T1>
	void assert_rank_dimension_match(const T0& t0,
		const T1& t1,
		int rank0,
		int rank1) {
		if (t0.dimension(rank0) != t1.dimension(rank1)) {
			std::ostringstream buf;
			buf << "Mismatch in rank dimensions for rank " << rank0
				<< " in the first tensor argument and rank ";
			buf << rank1 << " in the second. First argument was of shape: (";
			buf << t0.dimensions() << "), second argument of shape (" << t1.dimensions()
				<< ").";
			throw std::invalid_argument(buf.str());
		}
	}

	/*
	 *	Eigen::array operators
	 */

	template <typename TIndex, int rank>
	std::ostream& operator<<(std::ostream& stream,
		const Eigen::array<TIndex, rank>& arr) {
		int i = 0;
		for (; i < rank - 1; i++)
			stream << arr[i] << ", ";
		stream << arr[i];
		return stream;
	}
	// Divide Eigen::array
	template <typename Index, int rank>
	Eigen::array<Index, rank> operator/(const Eigen::array<Index, rank>& numerator,
		const Eigen::array<Index, rank>& denom) {
		Eigen::array<Index, rank> res;
		for (int i = 0; i < rank; i++)
			res[i] = numerator[i] / denom[i];
		return res;
	}
	// Mult Eigen::array
	template <typename Index, int rank>
	Eigen::array<Index, rank> operator*(const Eigen::array<Index, rank>& numerator,
		const Eigen::array<Index, rank>& denom) {
		Eigen::array<Index, rank> res;
		for (int i = 0; i < rank; i++)
			res[i] = numerator[i] * denom[i];
		return res;
	}

#pragma endregion

}  // namespace tensorial
