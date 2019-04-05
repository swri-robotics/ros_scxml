#include "packml.h"
#include "packml_state.h"
#include <iostream>

PackML::PackML(QScxmlStateMachine *machine):
    m_machine(machine)
{
  PackmlState *stateAborted = new PackmlState();

  stateNames = machine->stateNames(false);

  machine->connectToState(QStringLiteral("Aborted"), stateAborted, &PackmlState::stateChange);
}

void PackML::packmlEvent_Callback(const std_msgs::String::ConstPtr& msg)
{
  m_machine->submitEvent(msg->data.c_str());
}
