//
// Copyright (c) 2023 INRIA
//

#ifndef COAL_PYTHON_UTILS_STD_PAIR_H
#define COAL_PYTHON_UTILS_STD_PAIR_H

#include <boost/python.hpp>
#include <utility>

template <typename pair_type>
struct StdPairConverter {
  typedef typename pair_type::first_type T1;
  typedef typename pair_type::second_type T2;

  static PyObject* convert(const pair_type& pair) {
    return boost::python::incref(
        boost::python::make_tuple(pair.first, pair.second).ptr());
  }

  static void* convertible(PyObject* obj) {
    if (!PyTuple_CheckExact(obj)) return 0;
    if (PyTuple_Size(obj) != 2) return 0;
    {
      boost::python::tuple tuple(boost::python::borrowed(obj));
      boost::python::extract<T1> elt1(tuple[0]);
      if (!elt1.check()) return 0;
      boost::python::extract<T2> elt2(tuple[1]);
      if (!elt2.check()) return 0;
    }
    return obj;
  }

  static void construct(
      PyObject* obj,
      boost::python::converter::rvalue_from_python_stage1_data* memory) {
    boost::python::tuple tuple(boost::python::borrowed(obj));
    void* storage =
        reinterpret_cast<
            boost::python::converter::rvalue_from_python_storage<pair_type>*>(
            reinterpret_cast<void*>(memory))
            ->storage.bytes;
    new (storage) pair_type(boost::python::extract<T1>(tuple[0]),
                            boost::python::extract<T2>(tuple[1]));
    memory->convertible = storage;
  }

  static PyTypeObject const* get_pytype() {
    PyTypeObject const* py_type = &PyTuple_Type;
    return py_type;
  }

  static void registration() {
    boost::python::converter::registry::push_back(
        reinterpret_cast<void* (*)(_object*)>(&convertible), &construct,
        boost::python::type_id<pair_type>()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
            ,
        get_pytype
#endif
    );
  }
};

#endif  // ifndef COAL_PYTHON_UTILS_STD_PAIR_H
