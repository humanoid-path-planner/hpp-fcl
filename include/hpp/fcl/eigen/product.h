namespace internal {

template<typename Derived, typename OtherDerived, bool coefwise> struct deduce_fcl_type {};

template<typename Derived, typename OtherDerived>
struct deduce_fcl_type<Derived, OtherDerived, false> {
  typedef typename ProductReturnType<Derived, OtherDerived>::Type Type;
};
template<typename Derived, typename OtherDerived>
struct deduce_fcl_type<Derived, OtherDerived, true> {
  typedef CwiseBinaryOp<scalar_multiple_op<typename Derived::Scalar>, const Derived, const OtherDerived> Type;
};

}

template<typename Derived, typename OtherDerived>
struct FclProduct
{
  enum {
    COEFWISE = Derived::ColsAtCompileTime == 1 && OtherDerived::ColsAtCompileTime == 1
  };

  typedef typename internal::remove_fcl<Derived>::type EDerived;
  typedef typename internal::remove_fcl<OtherDerived>::type EOtherDerived;

  typedef FclOp<typename internal::deduce_fcl_type<EDerived, EOtherDerived, COEFWISE>::Type> ProductType;
  typedef FclOp<typename ProductReturnType<Transpose<Derived>, EOtherDerived >::Type> TransposeTimesType;
  typedef FclOp<typename ProductReturnType<EDerived, Transpose<EOtherDerived> >::Type> TimesTransposeType;
  typedef FclOp<typename ProductReturnType<ProductType, Transpose<EDerived>  >::Type> TensorTransformType;
  static EIGEN_STRONG_INLINE ProductType run (const Derived& l, const OtherDerived& r) { return ProductType (l, r); }
};

#define FCL_EIGEN_MAKE_PRODUCT_OPERATOR() \
      template <typename OtherDerived> \
      EIGEN_STRONG_INLINE const typename FclProduct<const typename FCL_EIGEN_CURRENT_CLASS::Base, const OtherDerived>::ProductType \
      operator*(const MatrixBase<OtherDerived>& other) const \
      { \
        return FclProduct<const typename FCL_EIGEN_CURRENT_CLASS::Base, const OtherDerived>::run (*this, other.derived()); \
      }
