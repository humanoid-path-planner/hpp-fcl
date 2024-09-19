//
// Copyright (c) 2024 INRIA
//

#ifndef COAL_SERIALIZATION_SERIALIZER_H
#define COAL_SERIALIZATION_SERIALIZER_H

#include "coal/serialization/archive.h"

namespace coal {
namespace serialization {

struct Serializer {
  /// \brief Loads an object from a text file.
  template <typename T>
  static void loadFromText(T& object, const std::string& filename) {
    ::coal::serialization::loadFromText(object, filename);
  }

  /// \brief Saves an object as a text file.
  template <typename T>
  static void saveToText(const T& object, const std::string& filename) {
    ::coal::serialization::saveToText(object, filename);
  }

  /// \brief Loads an object from a stream string.
  template <typename T>
  static void loadFromStringStream(T& object, std::istringstream& is) {
    ::coal::serialization::loadFromStringStream(object, is);
  }

  /// \brief Saves an object to a string stream.
  template <typename T>
  static void saveToStringStream(const T& object, std::stringstream& ss) {
    ::coal::serialization::saveToStringStream(object, ss);
  }

  /// \brief Loads an object from a string.
  template <typename T>
  static void loadFromString(T& object, const std::string& str) {
    ::coal::serialization::loadFromString(object, str);
  }

  /// \brief Saves a Derived object to a string.
  template <typename T>
  static std::string saveToString(const T& object) {
    return ::coal::serialization::saveToString(object);
  }

  /// \brief Loads an object from an XML file.
  template <typename T>
  static void loadFromXML(T& object, const std::string& filename,
                          const std::string& tag_name) {
    ::coal::serialization::loadFromXML(object, filename, tag_name);
  }

  /// \brief Saves an object as an XML file.
  template <typename T>
  static void saveToXML(const T& object, const std::string& filename,
                        const std::string& tag_name) {
    ::coal::serialization::saveToXML(object, filename, tag_name);
  }

  /// \brief Loads a Derived object from an binary file.
  template <typename T>
  static void loadFromBinary(T& object, const std::string& filename) {
    ::coal::serialization::loadFromBinary(object, filename);
  }

  /// \brief Saves a Derived object as an binary file.
  template <typename T>
  static void saveToBinary(const T& object, const std::string& filename) {
    ::coal::serialization::saveToBinary(object, filename);
  }

  /// \brief Loads an object from a binary buffer.
  template <typename T>
  static void loadFromBuffer(T& object, boost::asio::streambuf& container) {
    ::coal::serialization::loadFromBuffer(object, container);
  }

  /// \brief Saves an object as a binary buffer.
  template <typename T>
  static void saveToBuffer(const T& object, boost::asio::streambuf& container) {
    ::coal::serialization::saveToBuffer(object, container);
  }
};

}  // namespace serialization
}  // namespace coal

#endif  // ifndef COAL_SERIALIZATION_SERIALIZER_H
