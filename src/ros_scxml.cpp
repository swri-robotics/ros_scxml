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
    RosScxmlState* stateWidget = new RosScxmlState(this, stateName);

    m_stateList.insert(stateName, stateWidget);
    m_machine->connectToState(stateName, stateWidget, &RosScxmlState::stateChange);
    ROS_INFO("Connected to state [%s]" , stateName.toStdString().c_str());
  }
}

bool RosScxml::eventTrigger(ros_scxml::TriggerEvent::Request& req, ros_scxml::TriggerEvent::Response& res)
{
  //TODO: Do we submitEvents to a SM that is stopped? Check if running first?
  //TODO: Might need to connect to the event first, and fail if wee cannot connect?
  m_machine->submitEvent(req.event.c_str());

  res.message = "TODO: get feedback from the SM to know if the event is valid and what the status of submitting it is/was";
  res.success = true;
}

bool RosScxml::start_state_machine(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  //TODO: Should we cancel event before starting a SM that is stopped?
  m_machine->start();

  if(m_machine->isRunning())
  {
    res.message = "State Machine Running";
    res.success = true;
  }
  else
  {
    res.message = "Could not start State Machine";
    res.success = false;
  }
}
bool RosScxml::stop_state_machine(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  m_machine->stop();

  if(!m_machine->isRunning())
  {
    res.message = "State Machine Stopped";
    res.success = true;
  }
  else
  {
    res.message = "Could not stop State Machine";
    res.success = false;
  }
}
