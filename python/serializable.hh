//
// Copyright (c) 2017-2024 CNRS INRIA
// This file was borrowed from the Pinocchio library:
// https://github.com/stack-of-tasks/pinocchio
//

#ifndef COAL_PYTHON_SERIALIZABLE_H
#define COAL_PYTHON_SERIALIZABLE_H

#include <boost/python.hpp>

#include "coal/serialization/archive.h"
#include "coal/serialization/serializer.h"

namespace coal {
namespace python {

using Serializer = ::coal::serialization::Serializer;

namespace bp = boost::python;

template <typename Derived>
struct SerializableVisitor
    : public bp::def_visitor<SerializableVisitor<Derived>> {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def("saveToText", &Serializer::saveToText<Derived>, bp::arg("filename"),
           "Saves *this inside a text file.")
        .def("loadFromText", &Serializer::loadFromText<Derived>,
             bp::arg("filename"), "Loads *this from a text file.")

        .def("saveToString", &Serializer::saveToString<Derived>,
             bp::arg("self"), "Parses the current object to a string.")
        .def("loadFromString", &Serializer::loadFromString<Derived>,
             bp::args("self", "string"),
             "Parses from the input string the content of the current object.")

        .def("saveToXML", &Serializer::saveToXML<Derived>,
             bp::args("filename", "tag_name"), "Saves *this inside a XML file.")
        .def("loadFromXML", &Serializer::loadFromXML<Derived>,
             bp::args("self", "filename", "tag_name"),
             "Loads *this from a XML file.")

        .def("saveToBinary", &Serializer::saveToBinary<Derived>,
             bp::args("self", "filename"), "Saves *this inside a binary file.")
        .def("loadFromBinary", &Serializer::loadFromBinary<Derived>,
             bp::args("self", "filename"), "Loads *this from a binary file.")

        .def("saveToBuffer", &Serializer::saveToBuffer<Derived>,
             bp::args("self", "buffer"), "Saves *this inside a binary buffer.")
        .def("loadFromBuffer", &Serializer::loadFromBuffer<Derived>,
             bp::args("self", "buffer"), "Loads *this from a binary buffer.");
  }
};
}  // namespace python
}  // namespace coal

#endif  // ifndef COAL_PYTHON_SERIALIZABLE_H
