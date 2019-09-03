// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __problem_hh__
#define __problem_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_problem
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_problem
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_problem
#endif



#ifndef __common_hh_EXTERNAL_GUARD__
#define __common_hh_EXTERNAL_GUARD__
#include <hpp/common-idl.hh>
#endif
#ifndef __constraints_hh_EXTERNAL_GUARD__
#define __constraints_hh_EXTERNAL_GUARD__
#include <hpp/constraints_idl/constraints-idl.hh>
#endif
#ifndef __distances_hh_EXTERNAL_GUARD__
#define __distances_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/distances-idl.hh>
#endif
#ifndef __paths_hh_EXTERNAL_GUARD__
#define __paths_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/paths-idl.hh>
#endif
#ifndef ____constraints_hh_EXTERNAL_GUARD__
#define ____constraints_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/_constraints-idl.hh>
#endif
#ifndef __steering__methods_hh_EXTERNAL_GUARD__
#define __steering__methods_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/steering_methods-idl.hh>
#endif
#ifndef __path__planners_hh_EXTERNAL_GUARD__
#define __path__planners_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/path_planners-idl.hh>
#endif
#ifndef __path__validations_hh_EXTERNAL_GUARD__
#define __path__validations_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/path_validations-idl.hh>
#endif
#ifndef ____problem_hh_EXTERNAL_GUARD__
#define ____problem_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/_problem-idl.hh>
#endif
#ifndef __configuration__shooters_hh_EXTERNAL_GUARD__
#define __configuration__shooters_hh_EXTERNAL_GUARD__
#include <hpp/core_idl/configuration_shooters-idl.hh>
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

#ifndef __hpp_mcorbaserver_mProblem__
#define __hpp_mcorbaserver_mProblem__

    class Problem;
    class _objref_Problem;
    class _impl_Problem;
    
    typedef _objref_Problem* Problem_ptr;
    typedef Problem_ptr ProblemRef;

    class Problem_Helper {
    public:
      typedef Problem_ptr _ptr_type;

      static _ptr_type _nil();
      static _CORBA_Boolean is_nil(_ptr_type);
      static void release(_ptr_type);
      static void duplicate(_ptr_type);
      static void marshalObjRef(_ptr_type, cdrStream&);
      static _ptr_type unmarshalObjRef(cdrStream&);
    };

    typedef _CORBA_ObjRef_Var<_objref_Problem, Problem_Helper> Problem_var;
    typedef _CORBA_ObjRef_OUT_arg<_objref_Problem,Problem_Helper > Problem_out;

#endif

    // interface Problem
    class Problem {
    public:
      // Declarations for this interface type.
      typedef Problem_ptr _ptr_type;
      typedef Problem_var _var_type;

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

    class _objref_Problem :
      public virtual ::CORBA::Object,
      public virtual omniObjRef
    {
    public:
      void setRandomSeed(::CORBA::Long seed);
      void setMaxNumThreads(::CORBA::UShort n);
      ::CORBA::UShort getMaxNumThreads();
      Names_t* getAvailable(const char* type);
      Names_t* getSelected(const char* type);
      void setParameter(const char* name, const ::CORBA::Any& value);
      ::CORBA::Any* getParameter(const char* name);
      char* getParameterDoc(const char* name);
      ::CORBA::Boolean selectProblem(const char* name);
      void resetProblem();
      ::CORBA::Boolean loadPlugin(const char* pluginName);
      void movePathToProblem(::CORBA::ULong pathId, const char* problemName, const ::hpp::Names_t& jointNames);
      void setInitialConfig(const ::hpp::floatSeq& dofArray);
      floatSeq* getInitialConfig();
      void addGoalConfig(const ::hpp::floatSeq& dofArray);
      floatSeqSeq* getGoalConfigs();
      void resetGoalConfigs();
      ::CORBA::Boolean applyConstraints(const ::hpp::floatSeq& input, ::hpp::floatSeq_out output, ::CORBA::Double& residualError);
      ::CORBA::Boolean optimize(const ::hpp::floatSeq& input, ::hpp::floatSeq_out output, ::hpp::floatSeq_out residualError);
      void computeValueAndJacobian(const ::hpp::floatSeq& config, ::hpp::floatSeq_out value, ::hpp::floatSeqSeq_out jacobian);
      ::CORBA::Boolean generateValidConfig(::CORBA::ULong maxIter, ::hpp::floatSeq_out output, ::CORBA::Double& residualError);
      void createOrientationConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Quaternion_ p, const ::hpp::boolSeq& mask);
      void createTransformationConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Transform_ ref, const ::hpp::boolSeq& mask);
      void createTransformationSE3Constraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Transform_ frame1, const ::hpp::Transform_ frame2, const ::hpp::boolSeq& mask);
      void createTransformationConstraint2(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Transform_ frame1, const ::hpp::Transform_ frame2, const ::hpp::boolSeq& mask);
      void createLockedJoint(const char* lockedJointName, const char* jointName, const ::hpp::floatSeq& value);
      void createLockedExtraDof(const char* lockedDofName, ::CORBA::ULong index, const ::hpp::floatSeq& value);
      void createManipulability(const char* name, const char* function);
      void createComBeetweenFeet(const char* constraintName, const char* comName, const char* jointLName, const char* jointRName, const ::hpp::floatSeq& pointL, const ::hpp::floatSeq& pointR, const char* jointRefName, const ::hpp::floatSeq& pointRef, const ::hpp::boolSeq& mask);
      void createRelativeComConstraint(const char* constraintName, const char* comName, const char* jointLName, const ::hpp::floatSeq& point, const ::hpp::boolSeq& mask);
      void createConvexShapeContactConstraint(const char* constraintName, const ::hpp::Names_t& floorJoints, const ::hpp::Names_t& objectJoints, const ::hpp::floatSeqSeq& pts, const ::hpp::intSeqSeq& objectTriangles, const ::hpp::intSeqSeq& floorTriangles);
      void createStaticStabilityConstraint(const char* constraintName, const ::hpp::Names_t& jointNames, const ::hpp::floatSeqSeq& points, const ::hpp::floatSeqSeq& normals, const char* comRootJointName);
      void createPositionConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::floatSeq& point1, const ::hpp::floatSeq& point2, const ::hpp::boolSeq& mask);
      void createConfigurationConstraint(const char* constraintName, const ::hpp::floatSeq& goal, const ::hpp::floatSeq& weights);
      void createDistanceBetweenJointConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, ::CORBA::Double distance);
      void createDistanceBetweenJointAndObjects(const char* constraintName, const char* joint1Name, const ::hpp::Names_t& objects, ::CORBA::Double distance);
      void createIdentityConstraint(const char* constraintName, const ::hpp::Names_t& inJoints, const ::hpp::Names_t& outJoints);
      void resetConstraints();
      void resetConstraintMap();
      void addPassiveDofs(const char* constraintName, const ::hpp::Names_t& jointNames);
      void getConstraintDimensions(const char* constraintName, ::CORBA::ULong& inputSize, ::CORBA::ULong& inputDerivativeSize, ::CORBA::ULong& outputSize, ::CORBA::ULong& outputDerivativeSize);
      void setConstantRightHandSide(const char* constraintName, ::CORBA::Boolean constant);
      ::CORBA::Boolean getConstantRightHandSide(const char* constraintName);
      floatSeq* getRightHandSide();
      void setRightHandSide(const ::hpp::floatSeq& rhs);
      void setRightHandSideFromConfig(const ::hpp::floatSeq& config);
      void setRightHandSideByName(const char* constraintName, const ::hpp::floatSeq& rhs);
      void setRightHandSideFromConfigByName(const char* constraintName, const ::hpp::floatSeq& config);
      void addNumericalConstraints(const char* configProjName, const ::hpp::Names_t& constraintNames, const ::hpp::intSeq& priorities);
      void setNumericalConstraintsLastPriorityOptional(::CORBA::Boolean optional);
      void addLockedJointConstraints(const char* configProjName, const ::hpp::Names_t& lockedJointNames);
      char* displayConstraints();
      ::CORBA::Double getErrorThreshold();
      void setErrorThreshold(::CORBA::Double threshold);
      void setDefaultLineSearchType(const char* type);
      ::CORBA::ULong getMaxIterProjection();
      void setMaxIterProjection(::CORBA::ULong iterations);
      ::CORBA::ULong getMaxIterPathPlanning();
      void setMaxIterPathPlanning(::CORBA::ULong iterations);
      void scCreateScalarMultiply(const char* outName, ::CORBA::Double scalar, const char* inName);
      ::CORBA::Double getTimeOutPathPlanning();
      void setTimeOutPathPlanning(::CORBA::Double timeOut);
      void filterCollisionPairs();
      void selectPathPlanner(const char* pathPlannerType);
      void selectConfigurationShooter(const char* configurationShooterType);
      void selectDistance(const char* distanceType);
      void selectSteeringMethod(const char* steeringMethodType);
      void addPathOptimizer(const char* pathOptimizerType);
      void clearPathOptimizers();
      void addConfigValidation(const char* configValidationType);
      void clearConfigValidations();
      void selectPathValidation(const char* pathValidationType, ::CORBA::Double tolerance);
      void selectPathProjector(const char* pathProjectorType, ::CORBA::Double tolerance);
      ::CORBA::Boolean prepareSolveStepByStep();
      ::CORBA::Boolean executeOneStep();
      void finishSolveStepByStep();
      intSeq* solve();
      ::CORBA::Boolean directPath(const ::hpp::floatSeq& startConfig, const ::hpp::floatSeq& endConfig, ::CORBA::Boolean validate, ::CORBA::ULong& pathId, ::CORBA::String_out report);
      ::CORBA::Boolean reversePath(::CORBA::ULong pathId, ::CORBA::ULong& reversedPathId);
      void addConfigToRoadmap(const ::hpp::floatSeq& config);
      void addEdgeToRoadmap(const ::hpp::floatSeq& config1, const ::hpp::floatSeq& config2, ::CORBA::ULong pathId, ::CORBA::Boolean bothEdges);
      void appendDirectPath(::CORBA::ULong pathId, const ::hpp::floatSeq& config, ::CORBA::Boolean validate);
      void concatenatePath(::CORBA::ULong startId, ::CORBA::ULong endId);
      void extractPath(::CORBA::ULong pathId, ::CORBA::Double start, ::CORBA::Double end);
      void erasePath(::CORBA::ULong pathId);
      ::CORBA::Boolean projectPath(::CORBA::ULong patId);
      ::CORBA::Long numberPaths();
      intSeq* optimizePath(::CORBA::ULong inPathId);
      ::CORBA::Double pathLength(::CORBA::ULong inPathId);
      floatSeq* configAtParam(::CORBA::ULong inPathId, ::CORBA::Double atDistance);
      floatSeq* derivativeAtParam(::CORBA::ULong inPathId, ::CORBA::ULong orderId, ::CORBA::Double atDistance);
      floatSeqSeq* getWaypoints(::CORBA::ULong pathId, ::hpp::floatSeq_out times);
      void interruptPathPlanning();
      floatSeqSeq* nodes();
      ::CORBA::Long numberNodes();
      floatSeq* node(::CORBA::ULong nodeId);
      ::CORBA::Long connectedComponentOfEdge(::CORBA::ULong edgeId);
      ::CORBA::Long connectedComponentOfNode(::CORBA::ULong nodeId);
      ::CORBA::Long numberEdges();
      void edge(::CORBA::ULong edgeId, ::hpp::floatSeq_out q1, ::hpp::floatSeq_out q2);
      ::CORBA::Long numberConnectedComponents();
      floatSeqSeq* nodesConnectedComponent(::CORBA::ULong connectedComponentId);
      floatSeq* getNearestConfig(const ::hpp::floatSeq& config, ::CORBA::Long connectedComponentId, ::CORBA::Double& distance);
      void clearRoadmap();
      void resetRoadmap();
      void saveRoadmap(const char* filename);
      void readRoadmap(const char* filename);
      core_idl::Distance_ptr getDistance();
      void setDistance(::hpp::core_idl::Distance_ptr distance);
      core_idl::Path_ptr getPath(::CORBA::ULong pathId);
      ::CORBA::ULong addPath(::hpp::core_idl::PathVector_ptr path);
      core_idl::SteeringMethod_ptr getSteeringMethod();
      core_idl::PathValidation_ptr getPathValidation();
      core_idl::PathPlanner_ptr getPathPlanner();
      core_idl::Problem_ptr getProblem();
      constraints_idl::Implicit_ptr getConstraint(const char* constraintName);

      inline _objref_Problem()  { _PR_setobj(0); }  // nil
      _objref_Problem(omniIOR*, omniIdentity*);

    protected:
      virtual ~_objref_Problem();

      
    private:
      virtual void* _ptrToObjRef(const char*);

      _objref_Problem(const _objref_Problem&);
      _objref_Problem& operator = (const _objref_Problem&);
      // not implemented

      friend class Problem;
    };

    class _pof_Problem : public _OMNI_NS(proxyObjectFactory) {
    public:
      inline _pof_Problem() : _OMNI_NS(proxyObjectFactory)(Problem::_PD_repoId) {}
      virtual ~_pof_Problem();

      virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
      virtual _CORBA_Boolean is_a(const char*) const;
    };

    class _impl_Problem :
      public virtual omniServant
    {
    public:
      virtual ~_impl_Problem();

      virtual void setRandomSeed(::CORBA::Long seed) = 0;
      virtual void setMaxNumThreads(::CORBA::UShort n) = 0;
      virtual ::CORBA::UShort getMaxNumThreads() = 0;
      virtual Names_t* getAvailable(const char* type) = 0;
      virtual Names_t* getSelected(const char* type) = 0;
      virtual void setParameter(const char* name, const ::CORBA::Any& value) = 0;
      virtual ::CORBA::Any* getParameter(const char* name) = 0;
      virtual char* getParameterDoc(const char* name) = 0;
      virtual ::CORBA::Boolean selectProblem(const char* name) = 0;
      virtual void resetProblem() = 0;
      virtual ::CORBA::Boolean loadPlugin(const char* pluginName) = 0;
      virtual void movePathToProblem(::CORBA::ULong pathId, const char* problemName, const ::hpp::Names_t& jointNames) = 0;
      virtual void setInitialConfig(const ::hpp::floatSeq& dofArray) = 0;
      virtual floatSeq* getInitialConfig() = 0;
      virtual void addGoalConfig(const ::hpp::floatSeq& dofArray) = 0;
      virtual floatSeqSeq* getGoalConfigs() = 0;
      virtual void resetGoalConfigs() = 0;
      virtual ::CORBA::Boolean applyConstraints(const ::hpp::floatSeq& input, ::hpp::floatSeq_out output, ::CORBA::Double& residualError) = 0;
      virtual ::CORBA::Boolean optimize(const ::hpp::floatSeq& input, ::hpp::floatSeq_out output, ::hpp::floatSeq_out residualError) = 0;
      virtual void computeValueAndJacobian(const ::hpp::floatSeq& config, ::hpp::floatSeq_out value, ::hpp::floatSeqSeq_out jacobian) = 0;
      virtual ::CORBA::Boolean generateValidConfig(::CORBA::ULong maxIter, ::hpp::floatSeq_out output, ::CORBA::Double& residualError) = 0;
      virtual void createOrientationConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Quaternion_ p, const ::hpp::boolSeq& mask) = 0;
      virtual void createTransformationConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Transform_ ref, const ::hpp::boolSeq& mask) = 0;
      virtual void createTransformationSE3Constraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Transform_ frame1, const ::hpp::Transform_ frame2, const ::hpp::boolSeq& mask) = 0;
      virtual void createTransformationConstraint2(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::Transform_ frame1, const ::hpp::Transform_ frame2, const ::hpp::boolSeq& mask) = 0;
      virtual void createLockedJoint(const char* lockedJointName, const char* jointName, const ::hpp::floatSeq& value) = 0;
      virtual void createLockedExtraDof(const char* lockedDofName, ::CORBA::ULong index, const ::hpp::floatSeq& value) = 0;
      virtual void createManipulability(const char* name, const char* function) = 0;
      virtual void createComBeetweenFeet(const char* constraintName, const char* comName, const char* jointLName, const char* jointRName, const ::hpp::floatSeq& pointL, const ::hpp::floatSeq& pointR, const char* jointRefName, const ::hpp::floatSeq& pointRef, const ::hpp::boolSeq& mask) = 0;
      virtual void createRelativeComConstraint(const char* constraintName, const char* comName, const char* jointLName, const ::hpp::floatSeq& point, const ::hpp::boolSeq& mask) = 0;
      virtual void createConvexShapeContactConstraint(const char* constraintName, const ::hpp::Names_t& floorJoints, const ::hpp::Names_t& objectJoints, const ::hpp::floatSeqSeq& pts, const ::hpp::intSeqSeq& objectTriangles, const ::hpp::intSeqSeq& floorTriangles) = 0;
      virtual void createStaticStabilityConstraint(const char* constraintName, const ::hpp::Names_t& jointNames, const ::hpp::floatSeqSeq& points, const ::hpp::floatSeqSeq& normals, const char* comRootJointName) = 0;
      virtual void createPositionConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, const ::hpp::floatSeq& point1, const ::hpp::floatSeq& point2, const ::hpp::boolSeq& mask) = 0;
      virtual void createConfigurationConstraint(const char* constraintName, const ::hpp::floatSeq& goal, const ::hpp::floatSeq& weights) = 0;
      virtual void createDistanceBetweenJointConstraint(const char* constraintName, const char* joint1Name, const char* joint2Name, ::CORBA::Double distance) = 0;
      virtual void createDistanceBetweenJointAndObjects(const char* constraintName, const char* joint1Name, const ::hpp::Names_t& objects, ::CORBA::Double distance) = 0;
      virtual void createIdentityConstraint(const char* constraintName, const ::hpp::Names_t& inJoints, const ::hpp::Names_t& outJoints) = 0;
      virtual void resetConstraints() = 0;
      virtual void resetConstraintMap() = 0;
      virtual void addPassiveDofs(const char* constraintName, const ::hpp::Names_t& jointNames) = 0;
      virtual void getConstraintDimensions(const char* constraintName, ::CORBA::ULong& inputSize, ::CORBA::ULong& inputDerivativeSize, ::CORBA::ULong& outputSize, ::CORBA::ULong& outputDerivativeSize) = 0;
      virtual void setConstantRightHandSide(const char* constraintName, ::CORBA::Boolean constant) = 0;
      virtual ::CORBA::Boolean getConstantRightHandSide(const char* constraintName) = 0;
      virtual floatSeq* getRightHandSide() = 0;
      virtual void setRightHandSide(const ::hpp::floatSeq& rhs) = 0;
      virtual void setRightHandSideFromConfig(const ::hpp::floatSeq& config) = 0;
      virtual void setRightHandSideByName(const char* constraintName, const ::hpp::floatSeq& rhs) = 0;
      virtual void setRightHandSideFromConfigByName(const char* constraintName, const ::hpp::floatSeq& config) = 0;
      virtual void addNumericalConstraints(const char* configProjName, const ::hpp::Names_t& constraintNames, const ::hpp::intSeq& priorities) = 0;
      virtual void setNumericalConstraintsLastPriorityOptional(::CORBA::Boolean optional) = 0;
      virtual void addLockedJointConstraints(const char* configProjName, const ::hpp::Names_t& lockedJointNames) = 0;
      virtual char* displayConstraints() = 0;
      virtual ::CORBA::Double getErrorThreshold() = 0;
      virtual void setErrorThreshold(::CORBA::Double threshold) = 0;
      virtual void setDefaultLineSearchType(const char* type) = 0;
      virtual ::CORBA::ULong getMaxIterProjection() = 0;
      virtual void setMaxIterProjection(::CORBA::ULong iterations) = 0;
      virtual ::CORBA::ULong getMaxIterPathPlanning() = 0;
      virtual void setMaxIterPathPlanning(::CORBA::ULong iterations) = 0;
      virtual void scCreateScalarMultiply(const char* outName, ::CORBA::Double scalar, const char* inName) = 0;
      virtual ::CORBA::Double getTimeOutPathPlanning() = 0;
      virtual void setTimeOutPathPlanning(::CORBA::Double timeOut) = 0;
      virtual void filterCollisionPairs() = 0;
      virtual void selectPathPlanner(const char* pathPlannerType) = 0;
      virtual void selectConfigurationShooter(const char* configurationShooterType) = 0;
      virtual void selectDistance(const char* distanceType) = 0;
      virtual void selectSteeringMethod(const char* steeringMethodType) = 0;
      virtual void addPathOptimizer(const char* pathOptimizerType) = 0;
      virtual void clearPathOptimizers() = 0;
      virtual void addConfigValidation(const char* configValidationType) = 0;
      virtual void clearConfigValidations() = 0;
      virtual void selectPathValidation(const char* pathValidationType, ::CORBA::Double tolerance) = 0;
      virtual void selectPathProjector(const char* pathProjectorType, ::CORBA::Double tolerance) = 0;
      virtual ::CORBA::Boolean prepareSolveStepByStep() = 0;
      virtual ::CORBA::Boolean executeOneStep() = 0;
      virtual void finishSolveStepByStep() = 0;
      virtual intSeq* solve() = 0;
      virtual ::CORBA::Boolean directPath(const ::hpp::floatSeq& startConfig, const ::hpp::floatSeq& endConfig, ::CORBA::Boolean validate, ::CORBA::ULong& pathId, ::CORBA::String_out report) = 0;
      virtual ::CORBA::Boolean reversePath(::CORBA::ULong pathId, ::CORBA::ULong& reversedPathId) = 0;
      virtual void addConfigToRoadmap(const ::hpp::floatSeq& config) = 0;
      virtual void addEdgeToRoadmap(const ::hpp::floatSeq& config1, const ::hpp::floatSeq& config2, ::CORBA::ULong pathId, ::CORBA::Boolean bothEdges) = 0;
      virtual void appendDirectPath(::CORBA::ULong pathId, const ::hpp::floatSeq& config, ::CORBA::Boolean validate) = 0;
      virtual void concatenatePath(::CORBA::ULong startId, ::CORBA::ULong endId) = 0;
      virtual void extractPath(::CORBA::ULong pathId, ::CORBA::Double start, ::CORBA::Double end) = 0;
      virtual void erasePath(::CORBA::ULong pathId) = 0;
      virtual ::CORBA::Boolean projectPath(::CORBA::ULong patId) = 0;
      virtual ::CORBA::Long numberPaths() = 0;
      virtual intSeq* optimizePath(::CORBA::ULong inPathId) = 0;
      virtual ::CORBA::Double pathLength(::CORBA::ULong inPathId) = 0;
      virtual floatSeq* configAtParam(::CORBA::ULong inPathId, ::CORBA::Double atDistance) = 0;
      virtual floatSeq* derivativeAtParam(::CORBA::ULong inPathId, ::CORBA::ULong orderId, ::CORBA::Double atDistance) = 0;
      virtual floatSeqSeq* getWaypoints(::CORBA::ULong pathId, ::hpp::floatSeq_out times) = 0;
      virtual void interruptPathPlanning() = 0;
      virtual floatSeqSeq* nodes() = 0;
      virtual ::CORBA::Long numberNodes() = 0;
      virtual floatSeq* node(::CORBA::ULong nodeId) = 0;
      virtual ::CORBA::Long connectedComponentOfEdge(::CORBA::ULong edgeId) = 0;
      virtual ::CORBA::Long connectedComponentOfNode(::CORBA::ULong nodeId) = 0;
      virtual ::CORBA::Long numberEdges() = 0;
      virtual void edge(::CORBA::ULong edgeId, ::hpp::floatSeq_out q1, ::hpp::floatSeq_out q2) = 0;
      virtual ::CORBA::Long numberConnectedComponents() = 0;
      virtual floatSeqSeq* nodesConnectedComponent(::CORBA::ULong connectedComponentId) = 0;
      virtual floatSeq* getNearestConfig(const ::hpp::floatSeq& config, ::CORBA::Long connectedComponentId, ::CORBA::Double& distance) = 0;
      virtual void clearRoadmap() = 0;
      virtual void resetRoadmap() = 0;
      virtual void saveRoadmap(const char* filename) = 0;
      virtual void readRoadmap(const char* filename) = 0;
      virtual core_idl::Distance_ptr getDistance() = 0;
      virtual void setDistance(::hpp::core_idl::Distance_ptr distance) = 0;
      virtual core_idl::Path_ptr getPath(::CORBA::ULong pathId) = 0;
      virtual ::CORBA::ULong addPath(::hpp::core_idl::PathVector_ptr path) = 0;
      virtual core_idl::SteeringMethod_ptr getSteeringMethod() = 0;
      virtual core_idl::PathValidation_ptr getPathValidation() = 0;
      virtual core_idl::PathPlanner_ptr getPathPlanner() = 0;
      virtual core_idl::Problem_ptr getProblem() = 0;
      virtual constraints_idl::Implicit_ptr getConstraint(const char* constraintName) = 0;
      
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

    class Problem :
      public virtual hpp::corbaserver::_impl_Problem,
      public virtual ::PortableServer::ServantBase
    {
    public:
      virtual ~Problem();

      inline ::hpp::corbaserver::Problem_ptr _this() {
        return (::hpp::corbaserver::Problem_ptr) _do_this(::hpp::corbaserver::Problem::_PD_repoId);
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
hpp::corbaserver::Problem::_marshalObjRef(::hpp::corbaserver::Problem_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_problem
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_problem
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_problem
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_problem
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_problem
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_problem
#endif

#endif  // __problem_hh__
