template <typename Derived>
IVector3& operator=(const Eigen::MatrixBase<Derived>& other)
{
  const Vec3f& tmp (other);
  setValue (tmp);
  return *this;
}
