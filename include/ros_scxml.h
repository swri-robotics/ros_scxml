#ifndef ROS_SCXML_H
#define ROS_SCXML_H
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <ros_scxml/ActiveStates.h>
#include <ros_scxml/TriggerEvent.h>

#include <QScxmlStateMachine>
#include <QWidget>
#include <QHash>

#include "ros_scxml_state.h"

class RosScxml: public QWidget
{
  Q_OBJECT

public:
  RosScxml(QScxmlStateMachine *machine);

  void eventTrigger_Callback(const std_msgs::String::ConstPtr& msg);
  void eventConnect_Callback(const std_msgs::String::ConstPtr& msg);
  QStringList stateNames;

  ros_scxml::ActiveStates activeStates;

  bool eventTrigger(ros_scxml::TriggerEvent::Request& req, ros_scxml::TriggerEvent::Response& res);
  bool start_state_machine(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  bool stop_state_machine(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
private:
  QScxmlStateMachine *m_machine;
  QHash<QString, RosScxmlState*> m_stateList;
};

#endif // ROS_SCXML_H

