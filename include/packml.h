#ifndef PACKML_H
#define PACKML_H

#include <std_msgs/String.h>
#include <packml/PackMLActiveStates.h>

#include <QScxmlStateMachine>
#include <QWidget>

class PackML: public QWidget
{
  Q_OBJECT

public:
  PackML(QScxmlStateMachine *machine);

  void packmlEvent_Callback(const std_msgs::String::ConstPtr& msg);
  QStringList stateNames;

  packml::PackMLActiveStates activeStates;
private:
  QScxmlStateMachine *m_machine;
};

#endif // PACKML_H

