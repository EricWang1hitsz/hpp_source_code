//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_motion_base_hpp__
#define __pinocchio_motion_base_hpp__

namespace pinocchio
{
  
  template<class Derived>
  class MotionBase
  {
  public:
    MOTION_TYPEDEF_TPL(Derived);
    
    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived & derived() const { return *static_cast<const Derived*>(this); }
    
    ConstAngularType angular() const { return derived().angular_impl(); }
    ConstLinearType linear() const { return derived().linear_impl(); }
    AngularType angular() { return derived().angular_impl(); }
    LinearType linear() { return derived().linear_impl(); }
    
    template<typename V3Like>
    void angular(const Eigen::MatrixBase<V3Like> & w)
    { derived().angular_impl(w.derived()); }
    
    template<typename V3Like>
    void linear(const Eigen::MatrixBase<V3Like> & v)
    { derived().linear_impl(v.derived()); }
    
    ToVectorConstReturnType toVector() const { return derived().toVector_impl(); }
    ToVectorReturnType toVector() { return derived().toVector_impl(); }
    operator Vector6() const { return toVector(); }
    
    ActionMatrixType toActionMatrix() const { return derived().toActionMatrix_impl(); }
    ActionMatrixType toDualActionMatrix() const { return derived().toDualActionMatrix_impl(); }
    operator Matrix6() const { return toActionMatrix(); }
    
    void setZero() { derived().setZero(); }
    
    template<typename M2>
    bool operator==(const MotionBase<M2> & other) const
    { return derived().isEqual_impl(other.derived()); }
    
    template<typename M2>
    bool operator!=(const MotionBase<M2> & other) const
    { return !(derived() == other.derived()); }
    
    Derived operator-() const { return derived().__opposite__(); }
    Derived operator+(const MotionBase<Derived> & v) const { return derived().__plus__(v.derived()); }
    Derived operator-(const MotionBase<Derived> & v) const { return derived().__minus__(v.derived()); }
    Derived & operator+=(const MotionBase<Derived> & v) { return derived().__pequ__(v.derived()); }
    Derived & operator-=(const MotionBase<Derived> & v) { return derived().__mequ__(v.derived()); }
    
    template<typename OtherScalar>
    MotionPlain operator*(const OtherScalar & alpha) const
    { return derived().__mult__(alpha); }
    template<typename OtherScalar>
    MotionPlain operator/(const OtherScalar & alpha) const
    { return derived().__div__(alpha); }
    
    template<typename OtherSpatialType>
    typename internal::MotionAlgebraAction<OtherSpatialType,Derived>::ReturnType
    cross(const OtherSpatialType & d) const
    {
      return derived().cross_impl(d);
    }
    
    bool isApprox(const Derived & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    { return derived().isApprox_impl(other, prec);}
    
    template<typename S2, int O2>
    typename internal::SE3GroupAction<Derived>::ReturnType
    se3Action(const SE3Tpl<S2,O2> & m) const
    { return derived().se3Action_impl(m); }
    
    template<typename S2, int O2>
    typename internal::SE3GroupAction<Derived>::ReturnType
    se3ActionInverse(const SE3Tpl<S2,O2> & m) const
    { return derived().se3ActionInverse_impl(m); }
    
    template<typename ForceDerived>
    Scalar dot(const ForceDense<ForceDerived> & f) const { return derived().dot(f.derived()); }
    
    void disp(std::ostream & os) const { derived().disp_impl(os); }
    friend std::ostream & operator << (std::ostream & os, const MotionBase<Derived> & v)
    {
      v.disp(os);
      return os;
    }
    
  }; // class MotionBase
  
} // namespace pinocchio

#endif // ifndef __pinocchio_motion_base_hpp__
