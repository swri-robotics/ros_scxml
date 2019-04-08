#include <iostream>

#include <QWidget>

#include "ros_scxml_state.h"

void RosScxmlState::stateChange(bool state)
{
  if (state != m_state)
    m_state = state;


  ROS_INFO("State [%s] is now [%d]",  m_name.toStdString().c_str(), m_state);

  emit stateChanged(m_state);
}

