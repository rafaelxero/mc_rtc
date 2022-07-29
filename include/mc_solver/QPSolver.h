/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_MCCONTROLQPSOLVER_H_
#define _H_MCCONTROLQPSOLVER_H_

#include <mc_control/api.h>
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/pragma.h>
#include <mc_solver/ConstraintSet.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/api.h>
#include <mc_solver/msg/QPResult.h>

#include <Tasks/QPSolver.h>
#include <RBDyn/TorqueFeedbackTerm.h>

#include <memory>

namespace mc_tasks
{

struct MetaTask;

} // namespace mc_tasks

namespace mc_rtc
{

struct Logger;

namespace gui
{

struct StateBuilder;

} // namespace gui

} // namespace mc_rtc

namespace mc_control
{

struct MCController;

} // namespace mc_control

namespace mc_solver
{

// Work around GCC bug see: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=43407
MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wattributes")

/** Describe the type of feedback used to control the robot */
enum class MC_SOLVER_DLLAPI FeedbackType
{
  /** No feedback, i.e. open-loop control */
  None,
  /** Synonyn for None */
  OpenLoop = None,
  /** Use encoder values for actuated joints */
  Joints,
  /** Joints + encoder velocity obtained from numerical differentiation */
  JointsWVelocity,
  /** Run in closed loop w.r.t realRobots using the observation pipeline and integrate over the control state of the
     system */
  ObservedRobots,
  /** Synonym for ObservedRobots */
  ClosedLoop = ObservedRobots,
  /** Run in closed loop w.r.t realRobots using the observation pipeline and integrate over the real state of the system
   */
  ClosedLoopIntegrateReal
};

typedef std::map<std::string, int> ElapsedTimeMap;
 
MC_RTC_diagnostic_pop

/** \class QPSolver
 *
 * Wraps a tasks::qp::QPSolver instance
 *
 * Always ensure that the solver is up-to-date
 */

struct QPSolver
{
public:
  /** This token is used to give mc_control::MCController access to some internals */
  struct MC_SOLVER_DLLAPI ControllerToken
  {
    friend struct mc_control::MCController;
    friend struct QPSolver;

  private:
    ControllerToken() = default;
  };

  /** Constructor
   * \param robots Set of robots managed by this solver
   * \param timeStep Timestep of the solver
   *
   * \note The real robots will be created by copying the provided robots
   */
  MC_SOLVER_DLLAPI QPSolver(mc_rbdyn::RobotsPtr robots, double timeStep);

  /** Constructor (the solver creates its own Robots instance)
   * \param timeStep Timestep of the solver
   */
  MC_SOLVER_DLLAPI QPSolver(double timeStep);

  /** Add a constraint set
   * \param cs Constraint set added to the solver
   */
  MC_SOLVER_DLLAPI void addConstraintSet(ConstraintSet & cs);

  /** Remove a constraint set
   * \param cs Constrain set removed from the solver
   */
  MC_SOLVER_DLLAPI void removeConstraintSet(ConstraintSet & cs);

  /** Add a task to the solver
   * \param task Pointer to the added task, QPSolver does not take ownership of this pointer and the caller should make sure the object remains valid until it is removed from the solver
   */
  MC_SOLVER_DLLAPI void addTask(tasks::qp::Task * task);

  /** Add a task to the solver
   *
   * Adding the same task multiple times has no effect.
   *
   * \param task Pointer to an mc_tasks::MetaTask, QPSolver does not take
   * ownership of this pointer. The MetaTask update function will be
   * automatically called before the optimization is solved.
   *
   */
  MC_SOLVER_DLLAPI void addTask(mc_tasks::MetaTask * task);

  /** Add a task to the solver
   *
   * Simple wrapper to add a shared_ptr
   *
   * \param task Shared-pointer to a task T that is derived from
   * mc_tasks::MetaTask
   *
   */
  template<typename T>
  inline void addTask(std::shared_ptr<T> task)
  {
    static_assert(std::is_base_of<mc_tasks::MetaTask, T>::value ||
                  std::is_base_of<tasks::qp::Task, T>::value,
                  "You are trying to add a task that is neither a tasks::qp::Task or an mc_tasks::MetaTask");
    if(task)
    {
      addTask(task.get());
      shPtrTasksStorage.emplace_back(task);
    }
  }

  /** Remove a task from the solver
   * \param task Pointer to the removed task. The task is not deleted after being removed
   */
  MC_SOLVER_DLLAPI void removeTask(tasks::qp::Task * task);

  /** Remove a task from the solver
   *
   * Removing a task that is not in the solver has no effect.
   *
   * \param task Pointer to an mc_tasks::MetaTask. The task will not be
   * updated anymore and memory should be released by the task's owner.
   *
   */
  MC_SOLVER_DLLAPI void removeTask(mc_tasks::MetaTask * task);

  /** Remove a task from the solver
   *
   * Simple wrapper to remove a shared_ptr
   *
   * \param task Shared-pointer to a task T that is derived from
   * mc_tasks::MetaTask
   *
   */
  template<typename T>
  inline void removeTask(std::shared_ptr<T> task)
  {
    static_assert(std::is_base_of<mc_tasks::MetaTask, T>::value ||
                  std::is_base_of<tasks::qp::Task, T>::value,
                  "You are trying to remove a task that is neither a tasks::qp::Task or an mc_tasks::MetaTask");
    if (task)
    {
      removeTask(task.get());
    }
  }

  /** Add a constraint function from the solver
   * \param constraint Pointer to the ConstraintFunction. QPSolver does not take ownserhip of this pointer and the caller should make sure the object remains valid until it is removed from the solver
   */
  template<typename ... Fun>
  void addConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->addToSolver(robots().mbs(), *solver);
    solver->updateConstrSize();
    solver->updateNrVars(robots().mbs());
  }

  /** Remove a constraint function from the solver
   * \param constraint Pointer to the constraint that will be removed. It is not destroyed afterwards
   */
  template<typename ... Fun>
  void removeConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->removeFromSolver(*solver);
    solver->updateConstrSize();
    solver->updateNrVars(robots().mbs());
  }

  bool hasConstraint(const tasks::qp::Constraint* constraint);

  /** Gives access to the tasks::qp::BilateralContact entity in the solver from a contact id
   * \param id The contact id of the contact
   * \return The tasks:qp::BilateralContact entity from the solver if id is valid, otherwise, the first element of the pair is -1 and the reference is invalid
   */
  MC_SOLVER_DLLAPI std::pair<int, const tasks::qp::BilateralContact &> contactById(
      const tasks::qp::ContactId & id) const;

  /** Gives access to a part to lambdaVec given a contact index
   * \param cIndex The index of the contact
   * \return The lambdaVec associated
   */
  MC_SOLVER_DLLAPI Eigen::VectorXd lambdaVec(int cIndex) const;

  /** Reset all contacts in the solver and use the new set of contacts provided
   * \item contact Set of mc_rbdyn::Contact
   */
  MC_CONTROL_DLLAPI void setContacts(const std::vector<mc_rbdyn::Contact> & contacts = {});

  /* Called by the owning controller to actually set the contacts or internally by QPSolver when it has no owning
   * controller */
  MC_SOLVER_DLLAPI void setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts);

  /** Returns the current set of contacts */
  MC_SOLVER_DLLAPI const std::vector<mc_rbdyn::Contact> & contacts() const;

  /** Returns the MetaTasks currently in the solver */
  MC_SOLVER_DLLAPI const std::vector<mc_tasks::MetaTask *> & tasks() const;

  /** Desired resultant of contact force in robot surface frame
   * \param contact Contact for which the force is desired.
   * This contact must be one of the active contacts in the solver.
   * \return Contact force in robot surface frame
   */
  MC_SOLVER_DLLAPI const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const;

  /** Run one iteration of the QP.
   *
   * If succesful, will update the robots' configurations
   *
   * \param fType Type of feedback used to close the loop on sensory information
   *
   * \return True if successful, false otherwise.
   */
  MC_SOLVER_DLLAPI bool run(FeedbackType fType = FeedbackType::None);

  /** Run one iteration of the QP.
   *
   * If successful, will update the robots' configurations
   * \return True if successful, false otherwise.
   */
  virtual bool run(bool dummy); // Rafa's version
  
  void updateCurrentState();
  virtual bool solve();

  /** Provides the result of run() for robots.robot()
   * \param curTime Unused
   */
  MC_SOLVER_DLLAPI const QPResultMsg & send(double curTime = 0);

  /** Non-const access to QPResultMsg
   *
   * You are unlikely to need this function, it is used by the framework to
   * change the gripper's states
   *
   */
  inline QPResultMsg & result()
  {
    return qpRes;
  }

  /** Gives access to the main robot in the solver */
  MC_SOLVER_DLLAPI const mc_rbdyn::Robot & robot() const;
  /** Gives access to the main robot in the solver */
  MC_SOLVER_DLLAPI mc_rbdyn::Robot & robot();

  /** Gives access to the robot with the given index in the solver */
  MC_SOLVER_DLLAPI mc_rbdyn::Robot & robot(unsigned int idx);
  /** Gives access to the robot with the given index in the solver */
  MC_SOLVER_DLLAPI const mc_rbdyn::Robot & robot(unsigned int idx) const;

  /** Gives access to the environment robot in the solver (see mc_rbdyn::Robots) */
  MC_SOLVER_DLLAPI const mc_rbdyn::Robot & env() const;
  /** Gives access to the environment robot in the solver (see mc_rbdyn::Robots) */
  MC_SOLVER_DLLAPI mc_rbdyn::Robot & env();

  /** Gives access to the robots controlled by this solver */
  MC_SOLVER_DLLAPI const mc_rbdyn::Robots & robots() const;
  /** Gives access to the robots controlled by this solver */
  MC_SOLVER_DLLAPI mc_rbdyn::Robots & robots();

  /** Values calculated by the QP Solver for all robots */
  const std::shared_ptr<std::vector<rbd::MultiBodyConfig>> mbcs_calc() const;
  
  /** Values calculated by the QP Solver for the main robot */
  const rbd::MultiBodyConfig & mbc_calc() const;

  const std::vector<tasks::qp::Task *> getTasks() const { return solver->getTasks(); }
  const std::vector<tasks::qp::Equality *> & getEqConstr() const { return solver->getEqConstr(); }
  const std::vector<tasks::qp::Inequality *> & getInEqConstr() const { return solver->getInEqConstr(); }
  const std::vector<tasks::qp::GenInequality *> & getGenInEqConstr() const { return solver->getGenInEqConstr(); }
  const std::vector<tasks::qp::Bound *> & getBoundConstr() const { return solver->getBoundConstr(); }
  
  /** Allows to set the real robots used by this solver
   * XXX could be dangerous / misleading if users set it but tasks have stored
   * it too
   */
  void realRobots(std::shared_ptr<mc_rbdyn::Robots> realRobots);
  
  /** Gives access to the real robots used by this solver */
  MC_SOLVER_DLLAPI const mc_rbdyn::Robots & realRobots() const;
  /** Gives access to the real robots used by this solver */
  MC_SOLVER_DLLAPI mc_rbdyn::Robots & realRobots();

  /** Update number of variables
   *
   * This should be called when/if you add new robots into the scene after the
   * solver initialization, this is a costly operation.
   */
  MC_SOLVER_DLLAPI void updateNrVars();

  /** Update constraints matrix sizes
   *
   * \note This is mainly provided to allow safe usage of raw constraint from
   * Tasks rather than those wrapped in this library, you probably do not need
   * to call this
   */
  MC_SOLVER_DLLAPI void updateConstrSize();

  /** Returns the timestep of the solver
   * \return The timestep of the solver
   */
  MC_SOLVER_DLLAPI double dt() const;

  /** Returns the internal QP solver data
   * \return The data of the solver
   */
  MC_SOLVER_DLLAPI tasks::qp::SolverData & data();

  /** Use the dynamics constraint to fill torque in the main robot */
  MC_SOLVER_DLLAPI void fillTorque(const mc_solver::DynamicsConstraint & dynamicsConstraint);
  MC_SOLVER_DLLAPI void fillTorque(tasks::qp::MotionConstr* motionConstr);

  MC_SOLVER_DLLAPI boost::timer::cpu_times solveTime();

  MC_SOLVER_DLLAPI boost::timer::cpu_times solveAndBuildTime();

  /** Return the solvers result vector.
   * \return The solvers result vector.
   */
  MC_SOLVER_DLLAPI const Eigen::VectorXd & result() const;

  /** Set the logger for this solver instance */
  MC_SOLVER_DLLAPI void logger(std::shared_ptr<mc_rtc::Logger> logger);
  /** Access to the logger instance */
  MC_SOLVER_DLLAPI std::shared_ptr<mc_rtc::Logger> logger() const;

  /** Set the GUI helper for this solver instance */
  MC_SOLVER_DLLAPI void gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);
  /** Access to the gui instance */
  MC_SOLVER_DLLAPI std::shared_ptr<mc_rtc::gui::StateBuilder> gui() const;

  /** Set the controller that is owning this QPSolver instance */
  inline void controller(mc_control::MCController * ctl) noexcept
  {
    controller_ = ctl;
  }
  /** Returns the controller owning this instance (if any) (const) */
  inline const mc_control::MCController * controller() const noexcept
  {
    return controller_;
  }
  /** Returns the controller owning this instance (if any) */
  inline mc_control::MCController * controller() noexcept
  {
    return controller_;
  }

 protected:  
  std::shared_ptr<mc_rbdyn::Robots> robots_p;
  std::shared_ptr<mc_rbdyn::Robots> realRobots_p;
  double timeStep;

  /** Holds mc_rbdyn::Contact in the solver */
  std::vector<mc_rbdyn::Contact> contacts_;
  /** Holds unilateral contacts in the solver */
  std::vector<tasks::qp::UnilateralContact> uniContacts;
  /** Holds bilateral contacts in the solver */
  std::vector<tasks::qp::BilateralContact> biContacts;

  /** Holds MetaTask currently in the solver */
  std::vector<mc_tasks::MetaTask*> metaTasks_;

  /** Holds dynamics constraint currently in the solver */
  std::vector<mc_solver::DynamicsConstraint *> dynamicsConstraints_;

protected:
  /** The actual solver instance */
  std::shared_ptr<tasks::qp::QPSolver> solver;
  /** Latest result */
  QPResultMsg qpRes;

  // Rafa's version
  bool first_run_;
  bool feedback_;
  bool feedback_old_;
  bool switch_trigger;

  // Rafa's version
  std::vector<double> encoder_prev_;
  std::shared_ptr<std::vector<rbd::MultiBodyConfig>> mbcs_calc_;

  // Rafa's version
  ElapsedTimeMap elapsed_;
  
  std::vector<std::shared_ptr<void>> shPtrTasksStorage;

  /** Update qpRes from the latest run() */
  void __fillResult();
  
  /** Update qpRes from the latest run() */
  void __fillResult(const rbd::MultiBodyConfig & mbc);

  /** Pointer to the Logger */
  std::shared_ptr<mc_rtc::Logger> logger_ = nullptr;

  /** Pointer to the GUI helper */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_ = nullptr;

  void addTaskToGUI(mc_tasks::MetaTask * task);

  /** Run without feedback (open-loop) */
  bool runOpenLoop();

  /** Run with encoders' feedback */
  bool runJointsFeedback(bool wVelocity);

  /**
   * WARNING EXPERIMENTAL
   *
   * Runs the QP on an estimated robot state.
   *
   * Uses the real robot state (mbc.q and mbc.alpha) from realRobots() instances.
   * It is the users responsibility to ensure that the real robot instance is properly estimated
   * and filled. Typically, this will be done through the Observers pipeline.
   * For example, the following pipeline provides a suitable state:
   *
   * \code{.yaml}
   * RunObservers: [Encoder, KinematicInertial]
   * UpdateObservers: [Encoder, KinematicInertial]
   * \endcode
   *
   * @param integrateControlState If true, integration is performed over the control state, otherwise over the observed
   * state
   *
   * @return True if successful, false otherwise
   */
  bool runClosedLoop(bool integrateControlState);

  /** Feedback data */
  std::vector<std::vector<double>> prev_encoders_{};
  std::vector<std::vector<double>> encoders_alpha_{};
  std::vector<std::vector<std::vector<double>>> control_q_{};
  std::vector<std::vector<std::vector<double>>> control_alpha_{};

  /** Can be nullptr if this not associated to any controller */
  mc_control::MCController * controller_ = nullptr;

public:
  /** \deprecated{Default constructor, not made for general usage} */
  MC_SOLVER_DLLAPI QPSolver() {}

  void enableFeedback(bool fb);

  ElapsedTimeMap & getElapsedTimes();
};

struct MC_SOLVER_DLLAPI IntglTerm_QPSolver : public QPSolver
{
 public:

  IntglTerm_QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep,
                     torque_control::IntegralTerm::IntegralTermType intTermType = torque_control::IntegralTerm::None,
                     torque_control::IntegralTerm::VelocityGainType velGainType = torque_control::IntegralTerm::Diagonal,
                     double lambda = 1, double phiSlow = 0, double phiFast = 0,
                     double fastFilterWeight = 0);

  /** Constructor (the solver creates its own Robots instance)
   * \param timeStep Timestep of the solver
   */
  IntglTerm_QPSolver(double timeStep,
                     torque_control::IntegralTerm::IntegralTermType intTermType = torque_control::IntegralTerm::None,
                     torque_control::IntegralTerm::VelocityGainType velGainType = torque_control::IntegralTerm::Diagonal,
                     double lambda = 1, double phiSlow = 0, double phiFast = 0,
                     double fastFilterWeight = 0);
  
  bool run(bool dummy) override;

  const std::shared_ptr<torque_control::IntegralTerm> fbTerm() const;

 protected:

  std::shared_ptr<torque_control::IntegralTerm> fbTerm_;
};

struct MC_SOLVER_DLLAPI IntglTermAntiWindup_QPSolver : public IntglTerm_QPSolver
{
 public:

  IntglTermAntiWindup_QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep,
			       torque_control::IntegralTerm::IntegralTermType intTermType,
			       torque_control::IntegralTerm::VelocityGainType velGainType,
			       double lambda, double perc,
			       const Eigen::Vector3d & maxLinAcc,
			       const Eigen::Vector3d & maxAngAcc,
			       const Eigen::VectorXd & torqueL,
			       const Eigen::VectorXd & torqueU,
             double phiSlow, double phiFast, 
             double fastFilterWeight);
  
  /** Constructor (the solver creates its own Robots instance)
   * \param timeStep Timestep of the solver
   */
  IntglTermAntiWindup_QPSolver(double timeStep,
			       torque_control::IntegralTerm::IntegralTermType intTermType,
			       torque_control::IntegralTerm::VelocityGainType velGainType,
			       double lambda, double perc,
			       const Eigen::Vector3d & maxLinAcc,
			       const Eigen::Vector3d & maxAngAcc,
			       const Eigen::VectorXd & torqueL,
			       const Eigen::VectorXd & torqueU,
             double phiSlow, double phiFast, 
             double fastFilterWeight);
};

struct MC_SOLVER_DLLAPI PassivityPIDTerm_QPSolver : public QPSolver
{
 public:

  PassivityPIDTerm_QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep,
                            double beta, double lambda, double mu, double sigma, double cis);

  /** Constructor (the solver creates its own Robots instance)
   * \param timeStep Timestep of the solver
   */
  PassivityPIDTerm_QPSolver(double timeStep,
                            double beta, double lambda, double mu, double sigma, double cis);

  bool run(bool dummy) override;
  bool solve() override;

  const std::shared_ptr<torque_control::PassivityPIDTerm> fbTerm() const;
  
 protected:
  
  std::shared_ptr<torque_control::PassivityPIDTerm> fbTerm_;
};
 
}

#endif
