#include <ros/ros.h>
#include "ros_scxml.h"
#include "ros_scxml_state.h"
#include <iostream>

#include <QHash>
#include <QString>

#define COMPRESS_MODE false

RosScxml::RosScxml(QScxmlStateMachine *machine):
    m_machine(machine)
{
  ROS_INFO("Getting state names");

  //Create a hash table of state objects by name
  //TODO: add options for compressed/not compressed
  foreach(QString stateName, machine->stateNames(COMPRESS_MODE))
  {
    RosScxmlState* stateWidget = new RosScxmlState();

    m_stateList.insert(stateName, stateWidget);
    m_machine->connectToState(stateName, stateWidget, &RosScxmlState::stateChange);
    ROS_INFO("Found state [%s]" , stateName.toStdString().c_str());
  }

}

void RosScxml::eventTrigger_Callback(const std_msgs::String::ConstPtr& msg)
{
  m_machine->submitEvent(msg->data.c_str());
}

void RosScxml::eventConnect_Callback(const std_msgs::String::ConstPtr& msg)
{
  m_machine->submitEvent(msg->data.c_str());
}
