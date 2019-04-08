#ifndef PACKML_H
#define PACKML_H

#include <std_msgs/String.h>
#include <packml/PackMLActiveStates.h>

#include <QScxmlStateMachine>
#include <QWidget>
#include <QHash>

#include "packml_state.h"

class PackML: public QWidget
{
  Q_OBJECT

public:
  PackML(QScxmlStateMachine *machine);

  void packmlEventTrigger_Callback(const std_msgs::String::ConstPtr& msg);
  void packmlEventConnect_Callback(const std_msgs::String::ConstPtr& msg);
  QStringList stateNames;

  packml::PackMLActiveStates activeStates;
private:
  QScxmlStateMachine *m_machine;
  QHash<QString,PackmlState> m_stateList;
  QHash<QString,PackmlState> m_eventList;
};

#endif // PACKML_H

