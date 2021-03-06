// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __robot_hh__
#define __robot_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_robot
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_robot
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_robot
#endif



#ifndef __common_hh_EXTERNAL_GUARD__
#define __common_hh_EXTERNAL_GUARD__
#include <hpp/common-idl.hh>
#endif



#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





_CORBA_MODULE hpp

_CORBA_MODULE_BEG

  _CORBA_MODULE corbaserver

  _CORBA_MODULE_BEG

#ifndef __hpp_mcorbaserver_mRobot__
#define __hpp_mcorbaserver_mRobot__

    class Robot;
    class _objref_Robot;
    class _impl_Robot;
    
    typedef _objref_Robot* Robot_ptr;
    typedef Robot_ptr RobotRef;

    class Robot_Helper {
    public:
      typedef Robot_ptr _ptr_type;

      static _ptr_type _nil();
      static _CORBA_Boolean is_nil(_ptr_type);
      static void release(_ptr_type);
      static void duplicate(_ptr_type);
      static void marshalObjRef(_ptr_type, cdrStream&);
      static _ptr_type unmarshalObjRef(cdrStream&);
    };

    typedef _CORBA_ObjRef_Var<_objref_Robot, Robot_Helper> Robot_var;
    typedef _CORBA_ObjRef_OUT_arg<_objref_Robot,Robot_Helper > Robot_out;

#endif

    // interface Robot
    class Robot {
    public:
      // Declarations for this interface type.
      typedef Robot_ptr _ptr_type;
      typedef Robot_var _var_type;

      static _ptr_type _duplicate(_ptr_type);
      static _ptr_type _narrow(::CORBA::Object_ptr);
      static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
      
      static _ptr_type _nil();

      static inline void _marshalObjRef(_ptr_type, cdrStream&);

      static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
        omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
        if (o)
          return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
        else
          return _nil();
      }

      static _core_attr const char* _PD_repoId;

      // Other IDL defined within this scope.
      
    };

    class _objref_Robot :
      public virtual ::CORBA::Object,
      public virtual omniObjRef
    {
    public:
      void loadRobotModel(const char* robotName, const char* rootJointType, const char* packageName, const char* modelName, const char* urdfSuffix, const char* srdfSuffix);
      void loadHumanoidModel(const char* robotName, const char* rootJointType, const char* packageName, const char* modelName, const char* urdfSuffix, const char* srdfSuffix);
      void loadRobotModelFromString(const char* robotName, const char* rootJointType, const char* urdfString, const char* srdfString);
      void loadHumanoidModelFromString(const char* robotName, const char* rootJointType, const char* urdfString, const char* srdfString);
      ::CORBA::Long getConfigSize();
      ::CORBA::Long getNumberDof();
      Names_t* getJointNames();
      Names_t* getJointTypes();
      Names_t* getAllJointNames();
      char* getParentJointName(const char* jointName);
      floatSeq* getJointConfig(const char* jointName);
      void setJointConfig(const char* jointName, const ::hpp::floatSeq& config);
      char* getJointType(const char* jointName);
      floatSeq* jointIntegrate(const ::hpp::floatSeq& jointCfg, const char* jointName, const ::hpp::floatSeq& speed, ::CORBA::Boolean saturate);
      floatSeqSeq* getCurrentTransformation(const char* jointName);
      Transform__slice* getJointPosition(const char* jointName);
      TransformSeq* getJointsPosition(const ::hpp::floatSeq& q, const ::hpp::Names_t& jointNames);
      floatSeq* getJointVelocity(const char* jointName);
      floatSeq* getJointVelocityInLocalFrame(const char* jointName);
      Transform__slice* getJointPositionInParentFrame(const char* jointName);
      Transform__slice* getRootJointPosition();
      void setRootJointPosition(const ::hpp::Transform_ position);
      void setJointPositionInParentFrame(const char* jointName, const ::hpp::Transform_ position);
      ::CORBA::Long getJointNumberDof(const char* jointName);
      ::CORBA::Long getJointConfigSize(const char* jointName);
      void setJointBounds(const char* jointName, const ::hpp::floatSeq& inJointBound);
      floatSeq* getJointBounds(const char* jointName);
      Transform__slice* getLinkPosition(const char* linkName);
      TransformSeq* getLinksPosition(const ::hpp::floatSeq& q, const ::hpp::Names_t& linkName);
      Names_t* getLinkNames(const char* jointName);
      void setDimensionExtraConfigSpace(::CORBA::ULong dimension);
      ::CORBA::ULong getDimensionExtraConfigSpace();
      void setExtraConfigSpaceBounds(const ::hpp::floatSeq& bounds);
      floatSeq* getCurrentConfig();
      floatSeq* shootRandomConfig();
      void setCurrentConfig(const ::hpp::floatSeq& dofArray);
      floatSeq* getCurrentVelocity();
      void setCurrentVelocity(const ::hpp::floatSeq& qDot);
      Names_t* getJointInnerObjects(const char* jointName);
      Names_t* getJointOuterObjects(const char* jointName);
      void getObjectPosition(const char* objectName, ::hpp::Transform_ cfg);
      void isConfigValid(const ::hpp::floatSeq& dofArray, ::CORBA::Boolean& validity, ::CORBA::String_out report);
      void distancesToCollision(::hpp::floatSeq_out distances, ::hpp::Names_t_out innerObjects, ::hpp::Names_t_out outerObjects, ::hpp::floatSeqSeq_out innerPoints, ::hpp::floatSeqSeq_out outerPoints);
      void autocollisionCheck(::hpp::boolSeq_out collide);
      void autocollisionPairs(::hpp::Names_t_out innerObjects, ::hpp::Names_t_out outerObjects, ::hpp::boolSeq_out active);
      void setAutoCollision(const char* innerObject, const char* outerObject, ::CORBA::Boolean active);
      floatSeq* getRobotAABB();
      ::CORBA::Double getMass();
      floatSeq* getCenterOfMass();
      floatSeq* getCenterOfMassVelocity();
      floatSeqSeq* getJacobianCenterOfMass();
      void addPartialCom(const char* comName, const ::hpp::Names_t& jointNames);
      floatSeq* getPartialCom(const char* comName);
      floatSeqSeq* getJacobianPartialCom(const char* comName);
      floatSeq* getVelocityPartialCom(const char* comName);
      char* getRobotName();
      void createRobot(const char* robotName);
      void appendJoint(const char* parentJoint, const char* jointName, const char* jointType, const ::hpp::Transform_ pos);
      void createPolyhedron(const char* inPolyName);
      void createBox(const char* name, ::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z);
      void createSphere(const char* name, ::CORBA::Double radius);
      void createCylinder(const char* name, ::CORBA::Double radius, ::CORBA::Double length);
      ::CORBA::ULong addPoint(const char* inPolyName, ::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z);
      ::CORBA::ULong addTriangle(const char* inPolyName, ::CORBA::ULong pt1, ::CORBA::ULong pt2, ::CORBA::ULong pt3);
      void addObjectToJoint(const char* jointName, const char* objectName, const ::hpp::Transform_ pos);

      inline _objref_Robot()  { _PR_setobj(0); }  // nil
      _objref_Robot(omniIOR*, omniIdentity*);

    protected:
      virtual ~_objref_Robot();

      
    private:
      virtual void* _ptrToObjRef(const char*);

      _objref_Robot(const _objref_Robot&);
      _objref_Robot& operator = (const _objref_Robot&);
      // not implemented

      friend class Robot;
    };

    class _pof_Robot : public _OMNI_NS(proxyObjectFactory) {
    public:
      inline _pof_Robot() : _OMNI_NS(proxyObjectFactory)(Robot::_PD_repoId) {}
      virtual ~_pof_Robot();

      virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
      virtual _CORBA_Boolean is_a(const char*) const;
    };

    class _impl_Robot :
      public virtual omniServant
    {
    public:
      virtual ~_impl_Robot();

      virtual void loadRobotModel(const char* robotName, const char* rootJointType, const char* packageName, const char* modelName, const char* urdfSuffix, const char* srdfSuffix) = 0;
      virtual void loadHumanoidModel(const char* robotName, const char* rootJointType, const char* packageName, const char* modelName, const char* urdfSuffix, const char* srdfSuffix) = 0;
      virtual void loadRobotModelFromString(const char* robotName, const char* rootJointType, const char* urdfString, const char* srdfString) = 0;
      virtual void loadHumanoidModelFromString(const char* robotName, const char* rootJointType, const char* urdfString, const char* srdfString) = 0;
      virtual ::CORBA::Long getConfigSize() = 0;
      virtual ::CORBA::Long getNumberDof() = 0;
      virtual Names_t* getJointNames() = 0;
      virtual Names_t* getJointTypes() = 0;
      virtual Names_t* getAllJointNames() = 0;
      virtual char* getParentJointName(const char* jointName) = 0;
      virtual floatSeq* getJointConfig(const char* jointName) = 0;
      virtual void setJointConfig(const char* jointName, const ::hpp::floatSeq& config) = 0;
      virtual char* getJointType(const char* jointName) = 0;
      virtual floatSeq* jointIntegrate(const ::hpp::floatSeq& jointCfg, const char* jointName, const ::hpp::floatSeq& speed, ::CORBA::Boolean saturate) = 0;
      virtual floatSeqSeq* getCurrentTransformation(const char* jointName) = 0;
      virtual Transform__slice* getJointPosition(const char* jointName) = 0;
      virtual TransformSeq* getJointsPosition(const ::hpp::floatSeq& q, const ::hpp::Names_t& jointNames) = 0;
      virtual floatSeq* getJointVelocity(const char* jointName) = 0;
      virtual floatSeq* getJointVelocityInLocalFrame(const char* jointName) = 0;
      virtual Transform__slice* getJointPositionInParentFrame(const char* jointName) = 0;
      virtual Transform__slice* getRootJointPosition() = 0;
      virtual void setRootJointPosition(const ::hpp::Transform_ position) = 0;
      virtual void setJointPositionInParentFrame(const char* jointName, const ::hpp::Transform_ position) = 0;
      virtual ::CORBA::Long getJointNumberDof(const char* jointName) = 0;
      virtual ::CORBA::Long getJointConfigSize(const char* jointName) = 0;
      virtual void setJointBounds(const char* jointName, const ::hpp::floatSeq& inJointBound) = 0;
      virtual floatSeq* getJointBounds(const char* jointName) = 0;
      virtual Transform__slice* getLinkPosition(const char* linkName) = 0;
      virtual TransformSeq* getLinksPosition(const ::hpp::floatSeq& q, const ::hpp::Names_t& linkName) = 0;
      virtual Names_t* getLinkNames(const char* jointName) = 0;
      virtual void setDimensionExtraConfigSpace(::CORBA::ULong dimension) = 0;
      virtual ::CORBA::ULong getDimensionExtraConfigSpace() = 0;
      virtual void setExtraConfigSpaceBounds(const ::hpp::floatSeq& bounds) = 0;
      virtual floatSeq* getCurrentConfig() = 0;
      virtual floatSeq* shootRandomConfig() = 0;
      virtual void setCurrentConfig(const ::hpp::floatSeq& dofArray) = 0;
      virtual floatSeq* getCurrentVelocity() = 0;
      virtual void setCurrentVelocity(const ::hpp::floatSeq& qDot) = 0;
      virtual Names_t* getJointInnerObjects(const char* jointName) = 0;
      virtual Names_t* getJointOuterObjects(const char* jointName) = 0;
      virtual void getObjectPosition(const char* objectName, ::hpp::Transform_ cfg) = 0;
      virtual void isConfigValid(const ::hpp::floatSeq& dofArray, ::CORBA::Boolean& validity, ::CORBA::String_out report) = 0;
      virtual void distancesToCollision(::hpp::floatSeq_out distances, ::hpp::Names_t_out innerObjects, ::hpp::Names_t_out outerObjects, ::hpp::floatSeqSeq_out innerPoints, ::hpp::floatSeqSeq_out outerPoints) = 0;
      virtual void autocollisionCheck(::hpp::boolSeq_out collide) = 0;
      virtual void autocollisionPairs(::hpp::Names_t_out innerObjects, ::hpp::Names_t_out outerObjects, ::hpp::boolSeq_out active) = 0;
      virtual void setAutoCollision(const char* innerObject, const char* outerObject, ::CORBA::Boolean active) = 0;
      virtual floatSeq* getRobotAABB() = 0;
      virtual ::CORBA::Double getMass() = 0;
      virtual floatSeq* getCenterOfMass() = 0;
      virtual floatSeq* getCenterOfMassVelocity() = 0;
      virtual floatSeqSeq* getJacobianCenterOfMass() = 0;
      virtual void addPartialCom(const char* comName, const ::hpp::Names_t& jointNames) = 0;
      virtual floatSeq* getPartialCom(const char* comName) = 0;
      virtual floatSeqSeq* getJacobianPartialCom(const char* comName) = 0;
      virtual floatSeq* getVelocityPartialCom(const char* comName) = 0;
      virtual char* getRobotName() = 0;
      virtual void createRobot(const char* robotName) = 0;
      virtual void appendJoint(const char* parentJoint, const char* jointName, const char* jointType, const ::hpp::Transform_ pos) = 0;
      virtual void createPolyhedron(const char* inPolyName) = 0;
      virtual void createBox(const char* name, ::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z) = 0;
      virtual void createSphere(const char* name, ::CORBA::Double radius) = 0;
      virtual void createCylinder(const char* name, ::CORBA::Double radius, ::CORBA::Double length) = 0;
      virtual ::CORBA::ULong addPoint(const char* inPolyName, ::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z) = 0;
      virtual ::CORBA::ULong addTriangle(const char* inPolyName, ::CORBA::ULong pt1, ::CORBA::ULong pt2, ::CORBA::ULong pt3) = 0;
      virtual void addObjectToJoint(const char* jointName, const char* objectName, const ::hpp::Transform_ pos) = 0;
      
    public:  // Really protected, workaround for xlC
      virtual _CORBA_Boolean _dispatch(omniCallHandle&);

    private:
      virtual void* _ptrToInterface(const char*);
      virtual const char* _mostDerivedRepoId();
      
    };


  _CORBA_MODULE_END

_CORBA_MODULE_END



_CORBA_MODULE POA_hpp
_CORBA_MODULE_BEG

  _CORBA_MODULE corbaserver
  _CORBA_MODULE_BEG

    class Robot :
      public virtual hpp::corbaserver::_impl_Robot,
      public virtual ::PortableServer::ServantBase
    {
    public:
      virtual ~Robot();

      inline ::hpp::corbaserver::Robot_ptr _this() {
        return (::hpp::corbaserver::Robot_ptr) _do_this(::hpp::corbaserver::Robot::_PD_repoId);
      }
    };

  _CORBA_MODULE_END

_CORBA_MODULE_END



_CORBA_MODULE OBV_hpp
_CORBA_MODULE_BEG

  _CORBA_MODULE corbaserver
  _CORBA_MODULE_BEG

  _CORBA_MODULE_END

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr



inline void
hpp::corbaserver::Robot::_marshalObjRef(::hpp::corbaserver::Robot_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_robot
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_robot
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_robot
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_robot
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_robot
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_robot
#endif

#endif  // __robot_hh__

