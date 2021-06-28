
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "matrix.h"

//namespace py = pybind11;

// type caster: Matrix <-> NumPy-array
namespace pybind11 { namespace detail {
  template <typename T> struct type_caster<Matrix<T>>
  {
    public:

      PYBIND11_TYPE_CASTER(Matrix<T>, _("Matrix<T>"));

      // Conversion part 1 (Python -> C++)
      bool load(handle src, bool convert)
      {
        if (!convert && !array_t<T>::check_(src))
          return false;

        auto buf = array_t<T, array::c_style | array::forcecast>::ensure(src);
        if (!buf)
          return false;

        auto dims = buf.ndim();
        if (dims < 1 || dims > 3)
          return false;

        std::vector<size_t> shape(buf.ndim());

        for ( int i=0 ; i<buf.ndim() ; i++ )
          shape[i] = buf.shape()[i];

        value = Matrix<T>(shape,buf.data());

        return true;
      }

      //Conversion part 2 (C++ -> Python)
      static handle cast(const Matrix<T>& src,
        return_value_policy policy, handle parent)
      {
        array a(std::move(src.shape()), std::move(src.strides(true)), src.data() );

        return a.release();
      }
  };
}}
