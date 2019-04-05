#include "packml_state.h"
#include <iostream>

PackmlState::PackmlState(QWidget *parent)
  : QWidget(parent)
{
}

void PackmlState::stateChange(bool state)
{
  if (state != m_state)
    m_state = state;


  std::cout <<  "State is now:" << m_state;

  emit stateChanged();
}

