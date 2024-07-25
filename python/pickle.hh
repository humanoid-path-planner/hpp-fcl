//
// Copyright (c) 2022 INRIA
//

#ifndef COAL_PYTHON_PICKLE_H
#define COAL_PYTHON_PICKLE_H

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <sstream>

using namespace boost::python;
using namespace coal;
//
template <typename T>
struct PickleObject : boost::python::pickle_suite {
  static boost::python::tuple getinitargs(const T&) {
    return boost::python::make_tuple();
  }

  static boost::python::tuple getstate(const T& obj) {
    std::stringstream ss;
    boost::archive::text_oarchive oa(ss);
    oa & obj;

    return boost::python::make_tuple(boost::python::str(ss.str()));
  }

  static void setstate(T& obj, boost::python::tuple tup) {
    if (boost::python::len(tup) == 0 || boost::python::len(tup) > 1) {
      throw eigenpy::Exception(
          "Pickle was not able to reconstruct the object from the loaded "
          "data.\n"
          "The pickle data structure contains too many elements.");
    }

    boost::python::object py_obj = tup[0];
    boost::python::extract<std::string> obj_as_string(py_obj.ptr());
    if (obj_as_string.check()) {
      const std::string str = obj_as_string;
      std::istringstream is(str);
      boost::archive::text_iarchive ia(is, boost::archive::no_codecvt);
      ia >> obj;
    } else {
      throw eigenpy::Exception(
          "Pickle was not able to reconstruct the model from the loaded data.\n"
          "The entry is not a string.");
    }
  }

  static bool getstate_manages_dict() { return false; }
};

#endif  // ifndef COAL_PYTHON_PICKLE_H
