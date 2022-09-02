#pragma once
/* Header providing python bindings for certain objects in the associated header file.
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

#include "tensor_traits.h"
#include "eigen_tensor_ext.h"
#include "pybind11_eigen_tensor.h"

#include <pybind11/pybind11.h>
//#include <pybind11/eigen.h>


namespace tensorial {

	namespace py = pybind11;

	template<typename NT, int rank>
	void pyb_define_typed_tensor(py::module_& m) {
		using TensorType = TensorWrapper<NT, rank>;

		const char arr[]{ 'T', 'e', 'n', 's', 'o', 'r',  NativeTypeChar<NT>::fmat, 48 + rank, '\0' };

		py::class_<TensorType>(m, &arr[0])
			.def(py::init<std::array<std::int64_t, rank>>())
			.def("data", py::overload_cast<>(&TensorType::data), py::return_value_policy::reference_internal)
			.def("view", py::overload_cast<>(&TensorType::view, py::const_), py::return_value_policy::reference_internal);
	}

	template<int rank>
	void pyb_define_make_tensor_np_dtype(py::module_& m) {
		/* Generic make_tensor constructor.
		*/
		m.def("make_tensor", [](std::array<std::int64_t, rank> shape, const py::object& dtype) {

			py::dtype dtype_arg = py::dtype::from_args(dtype);

			py::object obj;
			if (dtype_arg.is(py::dtype::of<std::int8_t>())) {
				obj = py::cast(TensorWrapper<std::int8_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::uint8_t>())) {
				obj = py::cast(TensorWrapper<std::uint8_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::int16_t>())) {
				obj = py::cast(TensorWrapper<std::int16_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::uint16_t>())) {
				obj = py::cast(TensorWrapper<std::uint16_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::int32_t>())) {
				obj = py::cast(TensorWrapper<std::int32_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::uint32_t>())) {
				obj = py::cast(TensorWrapper<std::uint32_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::int64_t>())) {
				obj = py::cast(TensorWrapper<std::int64_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<std::uint64_t>())) {
				obj = py::cast(TensorWrapper<std::uint64_t, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<float>())) {
				obj = py::cast(TensorWrapper<float, rank>(shape));
			}
			else if (dtype_arg.is(py::dtype::of<double>())) {
				obj = py::cast(TensorWrapper<double, rank>(shape));
			}
			else {
				throw std::runtime_error(
					"Invalid or unsupported native type associated with format character: " +
					(std::string)py::str(dtype_arg));

			}
			return obj;
		}, py::arg("shape"), py::arg("dtype") = py::dtype::of<double>());
	}

	template<int rank>
	void pyb_define_tensors_rank(py::module_& m) {

		pyb_define_typed_tensor<std::int8_t, rank>(m);
		pyb_define_typed_tensor<std::uint8_t, rank>(m);
		pyb_define_typed_tensor<std::int16_t, rank>(m);
		pyb_define_typed_tensor<std::uint16_t, rank>(m);
		pyb_define_typed_tensor<std::int32_t, rank>(m);
		pyb_define_typed_tensor<std::uint32_t, rank>(m);
		pyb_define_typed_tensor<std::int64_t, rank>(m);
		pyb_define_typed_tensor<std::uint64_t, rank>(m);
		pyb_define_typed_tensor<float, rank>(m);
		pyb_define_typed_tensor<double, rank>(m);

		pyb_define_make_tensor_np_dtype<rank>(m);
	}

}