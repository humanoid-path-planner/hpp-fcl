//
// Copyright (c) 2024 INRIA
//

#ifndef HPP_FCL_PYTHON_SERIALIZE_H
#define HPP_FCL_PYTHON_SERIALIZE_H
#include <fstream>

#define CLASS_WRAPPER_SAVE_FUNC(classname)                           \
  static void save(const classname& self, const std::string& path) { \
    std::ofstream ofs(path.c_str());                                 \
    if (!ofs.is_open())                                              \
      throw std::runtime_error("Could not open file " + path);       \
    boost::archive::text_oarchive oa(ofs);                           \
    oa << self;                                                      \
  }

#define CLASS_WRAPPER_LOAD_FUNC(classname)                     \
  static classname* load(const std::string& path) {            \
    std::ifstream ifs(path.c_str());                           \
    if (!ifs.is_open())                                        \
      throw std::runtime_error("Could not open file " + path); \
    boost::archive::text_iarchive ia(ifs);                     \
    classname* t = new classname();                            \
    ia >> *t;                                                  \
    return t;                                                  \
  }

#define DEF_SAVE_CLASS(python_class_name, classwrapper_name)  \
  def("save", &classwrapper_name::save, args("self", "path"), \
      "Save a hppfcl." python_class_name)

#define DEF_LOAD_CLASS(python_class_name, classwrapper_name) \
  def("load", &classwrapper_name::load, args("path"),        \
      "Load a hppfcl." python_class_name,                    \
      return_value_policy<manage_new_object>())

#endif  // ifndef HPP_FCL_SERIALIZE_H
