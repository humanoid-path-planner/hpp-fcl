#ifndef DOXYGEN_BOOST_DOC_HH
#define DOXYGEN_BOOST_DOC_HH

#ifndef DOXYGEN_DOC_HH
# error "You should have included doxygen.hh first."
#endif // DOXYGEN_DOC_HH

#include <boost/python.hpp>

namespace doxygen
{

namespace visitor
{

template <typename function_type>
struct member_func_impl : boost::python::def_visitor<member_func_impl<function_type> >
{
  member_func_impl(const char* n, function_type f) : name(n), function(f) {}
  template <class classT>
  inline void visit(classT& c) const
  {
    c.def(name, function, doxygen::member_func_doc(function));
  }

  const char* name;
  function_type function;
};

// TODO surprisingly, this does not work when defined here but it works when
// defined after the generated files are included.
template <typename function_type>
inline member_func_impl<function_type> member_func (const char* name, function_type function)
{
  return member_func_impl<function_type>(name, function);
}

} // namespace visitor

} // namespace doxygen

#endif // DOXYGEN_BOOST_DOC_HH
