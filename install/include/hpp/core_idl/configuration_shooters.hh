#ifndef hpp_core_idl__configuration__shooters_hxx__
#define hpp_core_idl__configuration__shooters_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/configuration_shooters.idl
//

#include <hpp/core_idl/configuration_shooters-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::ConfigurationShooter
//
namespace core_impl {
template <typename _Base, typename _Storage>
ConfigurationShooterServant<_Base, _Storage>::ConfigurationShooterServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::ConfigurationShooter, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
ConfigurationShooterServant<_Base, _Storage>::~ConfigurationShooterServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::floatSeq* ConfigurationShooterServant<_Base, _Storage>::shoot ()
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/configuration_shooters.idl:21
    Configuration_t q;
    get()->shoot();
    return vectorToFloatSeq(q);

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl

//
// Implementational code for IDL interface hpp::core_idl::configuration_shooter::Gaussian
//
namespace core_impl {

namespace configuration_shooter {
template <typename _Base, typename _Storage>
GaussianServant<_Base, _Storage>::GaussianServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::ConfigurationShooterServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
GaussianServant<_Base, _Storage>::~GaussianServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
void GaussianServant<_Base, _Storage>::setCenter (const hpp::floatSeq& c)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _c = hpp::corbaServer::floatSeqToVector (c);
     (getT()->center (_c));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* GaussianServant<_Base, _Storage>::getCenter ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->center ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* GaussianServant<_Base, _Storage>::getSigmas ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->sigmas ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void GaussianServant<_Base, _Storage>::setSigmas (const hpp::floatSeq& s)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _s = hpp::corbaServer::floatSeqToVector (s);
     (getT()->sigmas (_s));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl

} // namespace configuration_shooter



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__configuration__shooters_hxx__

