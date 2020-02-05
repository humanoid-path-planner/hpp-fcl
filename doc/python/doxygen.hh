#ifndef DOXYGEN_DOC_HH
#define DOXYGEN_DOC_HH

#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>

#ifndef DOXYGEN_DOC_MAX_NUMBER_OF_ARGUMENTS_IN_CONSTRUCTOR
#define DOXYGEN_DOC_MAX_NUMBER_OF_ARGUMENTS_IN_CONSTRUCTOR 10
#endif

namespace doxygen
{
template <typename _class>
struct class_doc_impl
{
static inline const char* run ()
{
  return "";
}
};

template <typename _class>
inline const char* class_doc ()
{
  return class_doc_impl<_class>::run();
}

template <typename FuncPtr>
inline const char* member_func_doc (FuncPtr)
{
  return "";
}

#define DOXYGEN_DOC_DECLARE_CONSTRUCTOR(z,nargs,unused)                        \
template <                                                                     \
  typename Class                                                               \
  BOOST_PP_COMMA_IF(nargs)                                                     \
  BOOST_PP_ENUM_PARAMS(nargs, class Arg)>                                      \
struct constructor_doc_##nargs##_impl {                                        \
static inline const char* run ()                                               \
{                                                                              \
  return "";                                                                   \
}                                                                              \
};                                                                             \
                                                                               \
template <                                                                     \
  typename Class                                                               \
  BOOST_PP_COMMA_IF(nargs)                                                     \
  BOOST_PP_ENUM_PARAMS(nargs, class Arg)>                                      \
inline const char* constructor_doc ()                                          \
{                                                                              \
  return constructor_doc_##nargs##_impl<                                       \
    Class                                                                      \
    BOOST_PP_COMMA_IF(nargs)                                                   \
    BOOST_PP_ENUM_PARAMS(nargs, Arg)>::run();                                  \
}

BOOST_PP_REPEAT(DOXYGEN_DOC_MAX_NUMBER_OF_ARGUMENTS_IN_CONSTRUCTOR, DOXYGEN_DOC_DECLARE_CONSTRUCTOR, ~)

/*
template <typename Class>
inline const char* constructor_doc ()
{
  return "";
}
*/

template <typename Class>
struct destructor_doc_impl
{
static inline const char* run ()
{
  return "";
}
};

template <typename Class>
inline const char* destructor_doc ()
{
  return destructor_doc_impl<Class>::run();
}

/* TODO class attribute can be handled by

template <typename Class, typename AttributeType>
const char* attribute_doc (AttributeType Class::* ptr)
{
  // Body looks like
  // if (ptr == &Class::attributeName)
  //   return "attrib documentation";
  return "undocumented";
}
*/

} // namespace doxygen

#endif // DOXYGEN_DOC_HH
