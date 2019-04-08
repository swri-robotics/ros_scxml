#ifndef ROS_SCXML_STATE_H
#define ROS_SCXML_STATE_H
#include <ros/ros.h>

#include <QScxmlStateMachine>
#include <QWidget>

class RosScxmlState: public QWidget
{
  Q_OBJECT

public:
  RosScxmlState(QWidget *parent = nullptr, QString name = nullptr)
  {
    this->setParent(parent);
    m_name = name;
  }

  void stateChange(bool state);

Q_SIGNALS:
  void stateChanged(bool state);

private:
  bool m_state = false;
  QString m_name;
};

#endif // ROS_SCXML_STATE_H

