# distutils: language = c++

cimport mc_control.c_mc_control as c_mc_control

cimport eigen.eigen as eigen

cimport sva.sva as sva

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_solver.mc_solver as mc_solver

cimport mc_tasks.mc_tasks as mc_tasks

cimport mc_rtc.mc_rtc as mc_rtc
cimport mc_rtc.gui.gui as mc_rtc_gui

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class Gripper(object):
  def __cinit__(self):
    self.impl = c_mc_control.GripperPtr(NULL)
  cdef c_mc_control.Gripper * _impl(self):
    assert(self.impl.get() != NULL)
    return self.impl.get()
  property names:
    def __get__(self):
      return self._impl().names
  property q:
    def __get__(self):
      return self._impl()._q

cdef Gripper GripperFromShPtr(c_mc_control.shared_ptr[c_mc_control.Gripper] p):
  cdef Gripper ret = Gripper()
  ret.impl = p
  return ret

cdef class ControllerResetData(object):
  def __cinit__(self):
    pass
  property q:
    def __get__(self):
      return self.impl.q

cdef ControllerResetData ControllerResetDataFromPtr(c_mc_control.ControllerResetData * p):
    cdef ControllerResetData ret = ControllerResetData()
    ret.impl = p
    return ret

cdef class MCController(object):
  def __cinit__(self):
    pass
  def run(self):
    return self.base.run()
  def reset(self, ControllerResetData data):
    self.base.reset(deref(data.impl))
  def robot(self):
    return mc_rbdyn.RobotFromC(self.base.robot())
  def env(self):
    return mc_rbdyn.RobotFromC(self.base.env())
  def robots(self):
    return mc_rbdyn.RobotsFromRawPtr(&(self.base.robots()))
  def set_joint_pos(self, jname, pos):
    return self.base.set_joint_pos(jname, pos)
  def play_next_stance(self):
    return self.base.play_next_stance()
  def read_msg(self, msg):
    return self.base.read_msg(msg)
  def read_write_msg(self, msg, out):
    return self.base.read_write_msg(msg, out)
  def supported_robots(self):
    return self.base.supported_robots()
  def logger(self):
    return mc_rtc.LoggerFromRef(self.base.logger())
  def gui(self):
    return mc_rtc_gui.StateBuilderFromShPtr(self.base.gui())
  property timeStep:
    def __get__(self):
      return self.base.timeStep
  property contactConstraint:
    def __get__(self):
      return mc_solver.ContactConstraintFromPtr(&self.base.contactConstraint)
  property dynamicsConstraint:
    def __get__(self):
      return mc_solver.DynamicsConstraintFromPtr(&self.base.dynamicsConstraint)
  property kinematicsConstraint:
    def __get__(self):
      return mc_solver.KinematicsConstraintFromPtr(&self.base.kinematicsConstraint)
  property selfCollisionConstraint:
    def __get__(self):
      return mc_solver.CollisionsConstraintFromPtr(&self.base.selfCollisionConstraint)
  property postureTask:
    def __get__(self):
      return mc_tasks.PostureTaskFromPtr(self.base.postureTask.get())
  property qpsolver:
    def __get__(self):
      return mc_solver.QPSolverFromRef(self.base.solver())
  property grippers:
    def __get__(self):
      return {g.first: GripperFromShPtr(g.second) for g in self.base.grippers}

cdef MCController MCControllerFromPtr(c_mc_control.MCController * p):
    cdef MCController ret = MCController()
    ret.base = p
    return ret

cdef class PythonRWCallback(object):
  def __cinit__(self, succ, out):
    self.impl.success = succ
    self.impl.out = out
  property success:
    def __get__(self):
      return self.impl.success
    def __set__(self, value):
      self.impl.success = value
  property out:
    def __get__(self):
      return self.impl.out
    def __set__(self, value):
      self.impl.out = value

cdef cppbool python_to_run_callback(void * f) with gil:
  return (<object>f).run_callback()

cdef void python_to_reset_callback(const c_mc_control.ControllerResetData & crd, void * f) with gil:
  (<object>f).reset_callback(ControllerResetDataFromPtr(&(c_mc_control.const_cast_crd(crd))))

cdef cppbool python_to_read_msg_callback(string & msg, void * f) with gil:
  return (<object>f).read_msg_callback(msg)

cdef c_mc_control.PythonRWCallback python_to_read_write_msg_callback(string & msg, void * f) with gil:
  cdef PythonRWCallback ret = PythonRWCallback(*(<object>f).read_write_msg_callback(msg))
  return ret.impl

cdef class MCPythonController(MCController):
  def __dealloc__(self):
    del self.impl
    self.impl = NULL
  def __cinit__(self, robot_modules, double dt):
    cdef mc_rbdyn.RobotModuleVector rmv = mc_rbdyn.RobotModuleVector(robot_modules)
    self.impl = self.base = new c_mc_control.MCPythonController(rmv.v, dt)
    try:
      self.run_callback
      c_mc_control.set_run_callback(deref(self.impl), &python_to_run_callback, <void*>(self))
    except AttributeError:
      raise TypeError("You need to implement a run_callback method in your object")
    try:
      self.reset_callback
      c_mc_control.set_reset_callback(deref(self.impl), &python_to_reset_callback, <void*>(self))
    except AttributeError:
      pass
    try:
      self.read_msg_callback
      c_mc_control.set_read_msg_callback(deref(self.impl), &python_to_read_msg_callback, <void*>(self))
    except AttributeError:
      pass
    try:
      self.read_write_msg_callback
      c_mc_control.set_read_write_msg_callback(deref(self.impl), &python_to_read_write_msg_callback, <void*>(self))
    except AttributeError:
      pass
