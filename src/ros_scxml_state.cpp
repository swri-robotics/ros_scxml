#include "ros_scxml_state.h"
#include <iostream>
#include <QWidget>

RosScxmlState::RosScxmlState(QWidget *parent)
  : QWidget(parent)
{
}

void RosScxmlState::stateChange(bool state)
{
  if (state != m_state)
    m_state = state;


  std::cout <<  "State is now:" << m_state;

  emit stateChanged();
}

