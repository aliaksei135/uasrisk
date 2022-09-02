/*  Tensor traits
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
#include <type_traits>
#include <unsupported/Eigen/CXX11/Tensor>

namespace tensorial {

	template<typename T>
	struct void_ { typedef void type; };

	// Determine nested PlainObjectType from an EigenTensor type 
	template <typename Type> struct eigen_nested_type { using type = Type; };
	template <typename PlainObjectType>
	struct eigen_nested_type<Eigen::TensorMap<PlainObjectType>> { using type = PlainObjectType; };
	template <typename PlainObjectType>
	struct eigen_nested_type<Eigen::TensorRef<PlainObjectType>> { using type = PlainObjectType; };

	// Expression verifying if T is a valid Eigen::Tensor... by checking if T::Base is a valid expression, 
	//  will generate compiler garbage if the expression is valid but not a valid tensor type.
	template <typename T>
	using EigenTensorIdentifier = typename void_<typename std::remove_const_t<T>::Base>::type;

	// Check if type T inherits from TensorBase<T>
	template <typename T, typename = void>
	struct check_if_tensor_base { static constexpr bool value = false; };
	template <typename T> // Specialization verifying if the EigenTensorIdentifier expression is valid
	struct check_if_tensor_base<T, EigenTensorIdentifier<T>> { static constexpr bool value = std::is_base_of<Eigen::TensorBase<std::remove_const_t<T>>, std::remove_const_t<T>>::value; };

	// Check if type T is an Eigen::Tensor<>
	template <typename T, typename = void>
	struct check_if_eigen_tensor { static constexpr bool value = false; };
	template <typename T>   // Specialization verifying if the EigenTensorIdentifier expression is valid
	struct check_if_eigen_tensor<T, EigenTensorIdentifier<T>> {
		static constexpr bool value =
			std::is_same<typename eigen_nested_type<T>::type, T>::value ||
			std::is_same<const typename eigen_nested_type<T>::type, T>::value;
	};

	// True if type T inherits from TensorBase (accepts any T e.g. int/float/Tensor..)
	template <typename T>
	constexpr bool is_any_eigen_tensor = check_if_tensor_base<T>::value;
	// True if type T is a dense tensor of type Eigen::Tensor (accepts any T e.g. int/float/Tensor..)
	template <typename T>
	constexpr bool is_eigen_tensor = is_any_eigen_tensor<T> && check_if_eigen_tensor<T>::value;


	template <typename T> using is_eigen_nested_mutable = std::negation<std::is_const<typename eigen_nested_type<T>::type>>;
	template <typename T> using is_eigen_mutable_tensor = std::conjunction<
		std::negation<std::is_const<T>>,
		is_eigen_nested_mutable<T>>;
	/* Non-mutable */
	template <typename T> using is_eigen_const_tensor = std::negation<is_eigen_mutable_tensor<T>>;

	template <typename T> using is_eigen_row_major_tensor = std::bool_constant<T::Layout == Eigen::RowMajor>;
	template <typename T> using is_eigen_col_major_tensor = std::bool_constant<T::Layout == Eigen::ColMajor>;


	// Determine if T == TensorRef<Tensor>
	template <typename T> using is_eigen_tensor_ref = std::disjunction<
		std::is_same<Eigen::TensorRef<Eigen::Tensor<typename T::Scalar, T::NumIndices, T::Layout, typename T::Index>>, typename std::remove_const<typename std::remove_reference<T>::type>::type>,
		std::is_same<Eigen::TensorRef<const Eigen::Tensor<typename T::Scalar, T::NumIndices, T::Layout, typename T::Index>>, typename std::remove_const<typename std::remove_reference<T>::type>::type>>;
	// Determine if T == TensorMap<Tensor> (Does not handle MakePointer)
	template <typename T> using is_eigen_tensor_map = std::disjunction<
		std::is_same<Eigen::TensorMap<Eigen::Tensor<typename T::Scalar, T::NumIndices, T::Layout, typename T::Index>, T::Options>, typename std::remove_const<typename std::remove_reference<T>::type>::type>,
		std::is_same<Eigen::TensorMap<const Eigen::Tensor<typename T::Scalar, T::NumIndices, T::Layout, typename T::Index>, T::Options>, typename std::remove_const<typename std::remove_reference<T>::type>::type>>;

	// Determine if tensor is a mappable dense type (Tensor or TensorMap)
	template <typename T>
	using is_eigen_mappable_tensor = std::bool_constant < is_eigen_tensor<T> || is_eigen_tensor_map<T>::value>;

}