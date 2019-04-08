#ifndef ROS_SCXML_H
#define ROS_SCXML_H

#include <std_msgs/String.h>
#include <ros_scxml/ActiveStates.h>

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
private:
  QScxmlStateMachine *m_machine;
  QHash<QString, RosScxmlState*> m_stateList;
  //QHash<QString,PackmlState> m_eventList;
};

#endif // ROS_SCXML_H

