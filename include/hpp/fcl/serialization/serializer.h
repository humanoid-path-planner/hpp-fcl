//
// Copyright (c) 2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_SERIALIZER_H
#define HPP_FCL_SERIALIZATION_SERIALIZER_H

#include "hpp/fcl/serialization/archive.h"

namespace hpp {
namespace fcl {
namespace serialization {

struct Serializer {
  /// \brief Loads an object from a text file.
  template <typename T>
  static void loadFromText(T& object, const std::string& filename) {
    ::hpp::fcl::serialization::loadFromText(object, filename);
  }

  /// \brief Saves an object as a text file.
  template <typename T>
  static void saveToText(const T& object, const std::string& filename) {
    ::hpp::fcl::serialization::saveToText(object, filename);
  }

  /// \brief Loads an object from a stream string.
  template <typename T>
  static void loadFromStringStream(T& object, std::istringstream& is) {
    ::hpp::fcl::serialization::loadFromStringStream(object, is);
  }

  /// \brief Saves an object to a string stream.
  template <typename T>
  static void saveToStringStream(const T& object, std::stringstream& ss) {
    ::hpp::fcl::serialization::saveToStringStream(object, ss);
  }

  /// \brief Loads an object from a string.
  template <typename T>
  static void loadFromString(T& object, const std::string& str) {
    ::hpp::fcl::serialization::loadFromString(object, str);
  }

  /// \brief Saves a Derived object to a string.
  template <typename T>
  static std::string saveToString(const T& object) {
    return ::hpp::fcl::serialization::saveToString(object);
  }

  /// \brief Loads an object from an XML file.
  template <typename T>
  static void loadFromXML(T& object, const std::string& filename,
                          const std::string& tag_name) {
    ::hpp::fcl::serialization::loadFromXML(object, filename, tag_name);
  }

  /// \brief Saves an object as an XML file.
  template <typename T>
  static void saveToXML(const T& object, const std::string& filename,
                        const std::string& tag_name) {
    ::hpp::fcl::serialization::saveToXML(object, filename, tag_name);
  }

  /// \brief Loads a Derived object from an binary file.
  template <typename T>
  static void loadFromBinary(T& object, const std::string& filename) {
    ::hpp::fcl::serialization::loadFromBinary(object, filename);
  }

  /// \brief Saves a Derived object as an binary file.
  template <typename T>
  static void saveToBinary(const T& object, const std::string& filename) {
    ::hpp::fcl::serialization::saveToBinary(object, filename);
  }

  /// \brief Loads an object from a binary buffer.
  template <typename T>
  static void loadFromBuffer(T& object, boost::asio::streambuf& container) {
    ::hpp::fcl::serialization::loadFromBuffer(object, container);
  }

  /// \brief Saves an object as a binary buffer.
  template <typename T>
  static void saveToBuffer(const T& object, boost::asio::streambuf& container) {
    ::hpp::fcl::serialization::saveToBuffer(object, container);
  }
};

}  // namespace serialization
}  // namespace fcl
}  // namespace hpp

#endif  // ifndef HPP_FCL_SERIALIZATION_SERIALIZER_H
