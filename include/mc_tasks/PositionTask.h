/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the position of a frame

 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI PositionTask : public TrajectoryTaskGeneric<tasks::qp::PositionTask>
{
public:
  friend struct EndEffectorTask;

  /*! \brief Constructor
   *
   * \param frame Control frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  PositionTask(const mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500.0);

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  PositionTask(const std::string & bodyName,
               const mc_rbdyn::Robots & robots,
               unsigned int robotIndex,
               double stiffness = 2.0,
               double weight = 500);

  virtual ~PositionTask() = default;

  /*! \brief Reset the task
   *
   * Set the task objective to the current body position
   */
  void reset() override;

  /*! \brief Get the position target */
  inline const Eigen::Vector3d & position() const noexcept
  {
    return errorT->position();
  }

  /*! \brief Set the position target
   *
   * \param pos Body position in world frame
   *
   */
  inline void position(const Eigen::Vector3d & pos) noexcept
  {
    errorT->position(pos);
  }

  /*! \brief Change the body position target by a given amount
   *
   * \param com Modification applied to the body position
   *
   */
  void move_position(const Eigen::Vector3d & pos);
  
  /*! \brief Get the body point being controlled
   */
  inline const Eigen::Vector3d & bodyPoint() const noexcept
  {
    return errorT->bodyPoint();
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

protected:
  mc_rbdyn::ConstRobotFramePtr frame_;
  void addToLogger(mc_rtc::Logger & logger) override;
};

} // namespace mc_tasks
