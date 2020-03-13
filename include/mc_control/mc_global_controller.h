/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/ControllerServer.h>
#include <mc_control/GlobalPlugin_fwd.h>
#include <mc_control/MCController.h>
#include <mc_control/api.h>

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/loader.h>
#include <mc_rtc/log/Logger.h>

#include <array>
#include <fstream>
#include <sstream>
#include <thread>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCGlobalController
{
public:
  struct GlobalConfiguration;

private:
  /* MCGlobalController is non-copyable */
  MCGlobalController(const MCGlobalController &) = delete;
  MCGlobalController & operator=(const MCGlobalController &) = delete;

  /* MCGlobalController is non-movable */
  MCGlobalController(MCGlobalController &&) = delete;
  MCGlobalController & operator=(MCGlobalController &&) = delete;

public:
  /*! \brief Create the global controller
   *
   * The global controller is in charge of handling incoming data
   * (sensors, encoders...), passing it to the Controller instance and
   * finally hand out the result to the simulation/robot.
   *
   * An additional configuration file can be passed at construction but
   * multiple configuration files are read by default:
   * - a global configuration file read from
   *   INSTALL_PREFIX/etc/mc_rtc.conf
   * - a global configuration file read from
   *   $HOME/.config/mc_rtc/mc_rtc.conf
   *
   * The configuration provided overrides settings in the local
   * configuration which overrides settings in the global configuration
   *
   * Global settings include the controller timestep, the main robot,
   * publication and logging options.
   *
   * For each controller a specific setting file will be loaded from:
   * - $CONTROLLER_INSTALL_PREFIX/etc/$CONTROLLER_NAME.conf
   * - $HOME/.config/mc_rtc/controllers/$CONTROLLER_NAME.conf
   *
   * Global settings found in controller-specific configuration files
   * will be discarded.
   *
   * \note On Windows, user configuration files are retrieved from
   * $APPDATA/mc_rtc
   *
   * \param conf Additional configuration to load
   *
   */
  MCGlobalController(const std::string & conf = "", std::shared_ptr<mc_rbdyn::RobotModule> rm = nullptr);

  /*! \brief Create the global controller with an existing GlobalConfiguration instance */
  MCGlobalController(const GlobalConfiguration & conf);

  /*! \brief Destructor */
  ~MCGlobalController();

  /*! \brief Returns a list of enabled controllers */
  std::vector<std::string> enabled_controllers() const;

  /*! \brief Returns a list of all the loaded controllers, whether they
   * are enabled or not.
   */
  std::vector<std::string> loaded_controllers() const;

  /*! \brief Returns a list of all the loaded robots, whether they
   * are enabled or not.
   */
  std::vector<std::string> loaded_robots() const;

  /*! \brief Returns the main robot module */
  std::shared_ptr<mc_rbdyn::RobotModule> get_robot_module();

  /*! \brief Returns the name of the current controller */
  std::string current_controller() const;

  /*! \brief Initialize the default controller with the given configuration
   *
   * In this version, the robot's initial attitude is provided by
   * mc_rbdyn::RobotModule
   *
   * \param initq Initial joint configuration
   *
   */
  void init(const std::vector<double> & initq);

  /*! \brief Initialize the default controller with the given configuration and attitude
   *
   * \param initq Initial joint configuration
   *
   * \param initAttitude Initial attitude (qw, qx, qy, qz, tx, ty, tz)
   *
   */
  void init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude);

  /** @name Sensing
   *
   * These functions are used to communicate sensors' information to the
   * controller
   *
   * @{
   */

  /*! \brief Sets the main robot position sensor (control+real)
   *
   * \param pos Position given by a sensor
   */
  void setSensorPosition(const Eigen::Vector3d & pos);
  /*! \brief Set multiple body sensors' position for the main robot
   * (control+real)
   */
  void setSensorPositions(const std::map<std::string, Eigen::Vector3d> & poses);
  /*! \brief Set multiple body sensors' position for a given robot */
  void setSensorPositions(mc_rbdyn::Robot & robot, const std::map<std::string, Eigen::Vector3d> & poses);

  /*! \brief Sets the main robot orientation sensor (control + real)
   *
   * \param ori Orientation given by a sensor
   *
   * \note By convention, this rotation should be given from the inertial frame
   * (i.e. a fixed frame in the real world) to a body frame of the robot. For
   * instance, on HRP-4 the body sensor orientation goes from the inertial
   * frame to the "base_link" frame.
   *
   */
  void setSensorOrientation(const Eigen::Quaterniond & ori);
  /*! \brief Set multiple body sensors' orientation for the main robot
   * (control+real) */
  void setSensorOrientations(const std::map<std::string, Eigen::Quaterniond> & oris);
  /*! \brief Set multiple body sensors' orientation for a given robot */
  void setSensorOrientations(mc_rbdyn::Robot & robot, const std::map<std::string, Eigen::Quaterniond> & oris);

  /*! \brief Sets the main robot linear velocity sensor (control+real)
   *
   * \param vel Linear velocity given by a sensor
   */
  void setSensorLinearVelocity(const Eigen::Vector3d & vel);
  /*! \brief Set multiple body sensor's linear velocities for the main robot
   * (control+real) */
  void setSensorLinearVelocities(const std::map<std::string, Eigen::Vector3d> & linearVels);
  /*! \brief Set multiple body sensor's linear velocities for a given robot */
  void setSensorLinearVelocities(mc_rbdyn::Robot & robot, const std::map<std::string, Eigen::Vector3d> & linearVels);

  /*! \brief Sets the main robot angular velocity sensor (control+real)
   *
   * \param vel Angular velocity given by a sensor
   */
  void setSensorAngularVelocity(const Eigen::Vector3d & vel);
  /*! \brief Set multiple body sensor's angular velocities for the main robot
   * (control + real)*/
  void setSensorAngularVelocities(const std::map<std::string, Eigen::Vector3d> & angularVels);
  /*! \brief Set multiple body sensor's angular velocities for a given robot */
  void setSensorAngularVelocities(mc_rbdyn::Robot & robot, const std::map<std::string, Eigen::Vector3d> & angularVels);

  /*! \brief Sets the main robot acceleration (control+real)
   *
   * \param acc Acceleration given by a sensor
   */
  void setSensorAcceleration(const Eigen::Vector3d & acc);
  /*! \brief Set multiple body sensors' acceleration for a given robot for the
   * main robot (control+real) */
  void setSensorAccelerations(const std::map<std::string, Eigen::Vector3d> & accels);
  /*! \brief Set multiple body sensors' acceleration for a given robot */
  void setSensorAccelerations(mc_rbdyn::Robot & robot, const std::map<std::string, Eigen::Vector3d> & accels);

  /*! \brief Sets the main robot actual joints' values (control+real)
   *
   * \param eValues Actual joint values provided by encoders
   *
   * \note It is expected that these values follow the order given by
   * ref_joint_order
   */
  void setEncoderValues(const std::vector<double> & eValues);

  /*! \brief Sets the main robot's actual joint velocities (control+real)
   *
   * \param eVelocities Actual joint velocities
   *
   * \note It is expected that these values follow the order given by
   * ref_joint_order
   */
  void setEncoderVelocities(const std::vector<double> & eVelocities);

  /*! \brief Sets the main robot's flexible joint values (control+real)
   *
   * \param eValues Flexible joint values (provided by an estimator)
   *
   * \note It is expected that these values follow the order given by
   * robot.flexibility()
   */
  void setFlexibilityValues(const std::vector<double> & fValues);

  /*! \brief Sets the main robot's actual joint torques (control+real)
   *
   * \param tValues  Actual joint torques (provided by sensors)
   *
   * \note It is expected that these values follow the order given by
   * ref_joint_order
   */
  void setJointTorques(const std::vector<double> & tValues);

  /*! \brief Force sensors' readings provided by the sensors (sets control+real)*/
  void setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches);
  /*! \brief Force sensors' readings for another robot than the main robot */
  void setWrenches(unsigned int robotIndex, const std::map<std::string, sva::ForceVecd> & wrenches);

  /*! \brief Gripper active joints actual values */
  void setActualGripperQ(const std::map<std::string, std::vector<double>> & grippersQ);

  /** @} */

  /*! \brief Runs one step of the controller
   *
   * \returns True if the current controller succeeded, false otherwise
   */
  bool run();

  /*! \brief Access the result of the latest run
   *
   * \param t Unused
   *
   * \returns A reference to the latest result
   */
  const mc_solver::QPResultMsg & send(const double & t);

  /*! \brief Access the current controller */
  MCController & controller();

  /*! \brief Const access to current controller */
  const MCController & controller() const;

  /*! \brief Access the main robot */
  mc_rbdyn::Robot & robot();

  /*! \brief Const access to the main robot */
  const mc_rbdyn::Robot & robot() const;

  /*! \brief Get the controller timestep */
  double timestep() const;

  /*! \brief Access the reference joint order
   *
   * This is provided by mc_rbdyn::RobotModule and represents the joint's order
   * in the native control system of the robot.
   */
  const std::vector<std::string> & ref_joint_order();

  /*! \brief Access the global controller configuration */
  const GlobalConfiguration & configuration() const;

  /*! \brief Add the given directories to the controller search path
   *
   * Calling this function with the same arguments multiple times
   * effectively refresh the loaded controller list
   *
   * e.g. the following code will rescan all directories
   *
   * add_controller_module_paths(configuration().controller_module_paths));
   *
   */
  void add_controller_module_paths(const std::vector<std::string> & paths);

  /*! \brief Add a controller to the enabled list
   *
   * This call enables a controller that was not enabled but loaded. If
   * the provided name does not exist then it does nothing and returns
   * false. Otherwise, the controller is created, added to the enabled map
   * and the function returns true.
   *
   * \param name Name of the controller to add
   */
  bool AddController(const std::string & name);

  /*! \brief Add an already created controller to the enabled list
   *
   * This call adds a controller that was created through another mean
   * than MCGlobalController internal mechanism.
   *
   * The function returns false and does nothing if name is already among
   * the enabled controllers or if controller is a null pointer.
   *
   * \param name Name of the controller to add
   *
   * \param controller Controller to add
   *
   */
  bool AddController(const std::string & name, std::shared_ptr<mc_control::MCController> controller);

  /*! \brief Switch to the requested controller
   *
   * If the requested controller is not enabled, does not exist or is already
   * running then this call has no effect. Otherwise, it will trigger a
   * controller switch at the next run call.
   *
   * \param name Name of the new controller to load
   */
  bool EnableController(const std::string & name);

  /** @name Grippers
   *
   * These functions can be used to manipulate the grippers through the global
   * controller interface
   *
   * @{
   */

  /*! \brief Get the current (active) joints values for the grippers */
  std::map<std::string, std::vector<double>> gripperQ();
  /*! \brief Get the joints in the grippers */
  std::map<std::string, std::vector<std::string>> gripperJoints();
  /*! \brief Get the active joints in the grippers */
  std::map<std::string, std::vector<std::string>> gripperActiveJoints();
  /*! \brief Set the current values of (active) joints in the grippers */
  void setGripperCurrentQ(const std::map<std::string, std::vector<double>> & gripperQs);
  /*! \brief Set the opening target(s) for a given gripper
   * \param name Name of the gripper
   * \param q Active joints values
   */
  void setGripperTargetQ(const std::string & name, const std::vector<double> & q);
  /*! \brief Set the gripper opening percentage for all grippers
   * \param pOpen Opening percentage (0: closed, 1: open)
   */
  void setGripperOpenPercent(double pOpen);
  /*! \brief Set the gripper opening percentage for a given gripper
   * \param name Name of the gripper
   * \param pOpen Opening percentage (0: closed, 1: open)
   */
  void setGripperOpenPercent(const std::string & name, double pOpen);
  /** @} */

  /** @name Services
   *
   * These functions acts as a proxy between the caller and the current
   * controller.
   *
   * @{
   *
   */

  /*! \brief Returns to half-sit pose after an experiment
   *
   * This service enables the HalfSitPose controller which only drives the
   * robot back to half-sitting posture while avoiding auto-collisions
   *
   */
  bool GoToHalfSitPose_service();
  /*! \brief See mc_rtc::MCGlobalController::GoToHalfSitPose_service */
  bool GoToHalfSitPose();

  /** @} */

  /*! \brief Create a new log file for the current controller
   *
   * The purpose is to split the log file for long-running controller,
   * therefore this does not restart the timer of the controller so multiple
   * log files could be stitched together.
   *
   */
  void refreshLog();

  /*! \brief Get access to the real robots instance.
   *
   * This can be accessed by external tools to provide information about other
   * robots than the main robot.
   */
  mc_rbdyn::Robots & realRobots();

  /*! \brief Get access to the main real robot instance. */
  mc_rbdyn::Robot & realRobot();

public:
  /*! \brief Returns true if the controller is running
   *
   * To prevent any serious issues, the controller will stop when the
   * underlying MCController instance fails to run
   *
   */
  bool running;

public:
  /*! \brief Store the controller configuration */
  struct GlobalConfiguration
  {
    GlobalConfiguration(const std::string & conf, std::shared_ptr<mc_rbdyn::RobotModule> rm);

    inline bool enabled(const std::string & ctrl);

    bool use_sandbox = false;

    bool verbose_loader = true;

    std::vector<std::string> robot_module_paths = {};
    std::shared_ptr<mc_rbdyn::RobotModule> main_robot_module;

    std::vector<std::string> observer_module_paths = {};
    std::vector<std::string> enabled_observers = {};
    std::unordered_map<std::string, mc_rtc::Configuration> observer_configs;

    std::vector<std::string> global_plugin_paths = {};
    std::vector<std::string> global_plugins = {};
    std::unordered_map<std::string, mc_rtc::Configuration> global_plugin_configs;

    std::vector<std::string> controller_module_paths = {};
    std::vector<std::string> enabled_controllers = {};
    std::string initial_controller = "";
    std::unordered_map<std::string, mc_rtc::Configuration> controllers_configs;
    double timestep = 0.002;

    bool log_real = false;

    bool enable_log = true;
    mc_rtc::Logger::Policy log_policy = mc_rtc::Logger::Policy::NON_THREADED;
    std::string log_directory;
    std::string log_template = "mc-control";

    bool enable_gui_server = true;
    double gui_timestep = 0.05;
    std::vector<std::string> gui_server_pub_uris{};
    std::vector<std::string> gui_server_rep_uris{};

    Configuration config;

    void load_controllers_configs();

    void load_plugin_configs();
  };

private:
  using duration_ms = std::chrono::duration<double, std::milli>;
  GlobalConfiguration config;
  std::string current_ctrl = "";
  std::string next_ctrl = "";
  MCController * controller_ = nullptr;
  MCController * next_controller_ = nullptr;
  std::unique_ptr<mc_rtc::ObjectLoader<MCController>> controller_loader;
  std::map<std::string, std::shared_ptr<mc_control::MCController>> controllers;

  std::vector<mc_observers::ObserverPtr> observers_;
  std::map<std::string, mc_observers::ObserverPtr> observersByName_;
  std::shared_ptr<mc_rbdyn::Robots> real_robots = nullptr;

  std::unique_ptr<mc_control::ControllerServer> server_ = nullptr;

  std::unique_ptr<mc_rtc::ObjectLoader<GlobalPlugin>> plugin_loader;
  struct PluginHandle
  {
    PluginHandle(const std::string & name, GlobalPluginPtr plugin) : name(name), plugin(std::move(plugin)) {}
    PluginHandle(const PluginHandle &) = delete;
    PluginHandle & operator=(const PluginHandle &) = delete;
    PluginHandle(PluginHandle &&) = default;
    PluginHandle & operator=(PluginHandle &&) = default;
    ~PluginHandle();
    std::string name;
    GlobalPluginPtr plugin;
    duration_ms plugin_before_dt{0};
    duration_ms plugin_after_dt{0};
  };
  std::vector<PluginHandle> plugins_;

  void initGUI();

  void start_log();
  void setup_log();
  std::map<std::string, bool> setup_logger_ = {};

  /** Timers and performance measure */
  duration_ms global_run_dt{0};
  duration_ms controller_run_dt{0};
  duration_ms log_dt{0};
  duration_ms gui_dt{0};
  double solver_build_and_solve_t = 0;
  double solver_solve_t = 0;
  double framework_cost = 0;
};

} // namespace mc_control
