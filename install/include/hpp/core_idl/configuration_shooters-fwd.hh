#ifndef hpp_core_idl__configuration__shooters_hpp__
#define hpp_core_idl__configuration__shooters_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/configuration_shooters.idl
//

#include <hpp/core_idl/configuration_shooters-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/configuration-shooter/gaussian.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::ConfigurationShooter
//
namespace core_impl {
template <typename _Base, typename _Storage>
class ConfigurationShooterServant: public ServantBase<hpp::core::ConfigurationShooter, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::ConfigurationShooter HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::ConfigurationShooter, HppBase);

public:
  // standard constructor
  ConfigurationShooterServant(Server* server, const _Storage& s);
  virtual ~ConfigurationShooterServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::floatSeq* shoot ();


};

typedef ConfigurationShooterServant<POA_hpp::core_idl::ConfigurationShooter,hpp::core::ConfigurationShooterPtr_t > ConfigurationShooter;
} // namespace core_impl

//
// Class implementing IDL interface hpp::core_idl::configuration_shooter::Gaussian
//
namespace core_impl {

namespace configuration_shooter {
template <typename _Base, typename _Storage>
class GaussianServant: public hpp::corbaServer::core_impl::ConfigurationShooterServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::ConfigurationShooterServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::configuration_shooter::Gaussian, HppBase);

public:
  // standard constructor
  GaussianServant(Server* server, const _Storage& s);
  virtual ~GaussianServant();

  // methods corresponding to defined IDL attributes and operations
  
  void setCenter (const hpp::floatSeq& c);


  hpp::floatSeq* getCenter ();


  hpp::floatSeq* getSigmas ();


  void setSigmas (const hpp::floatSeq& s);


};

typedef GaussianServant<POA_hpp::core_idl::configuration_shooter::Gaussian,hpp::core::configurationShooter::GaussianPtr_t > Gaussian;
} // namespace core_impl

} // namespace configuration_shooter



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__configuration__shooters_hpp__

