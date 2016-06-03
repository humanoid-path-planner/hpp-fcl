template <typename Derived>
IVector3& operator=(const FclType<Derived>& other)
{
  const Vec3f& tmp = other.fcl();
  setValue (tmp);
  return *this;
}
template <typename Derived>
IVector3& operator=(const Eigen::MatrixBase<Derived>& other)
{
  const Vec3f& tmp (other);
  setValue (tmp);
  return *this;
}
