#ifndef ROS_SCXML_STATE_H
#define ROS_SCXML_STATE_H

#include <QScxmlStateMachine>
#include <QWidget>

class RosScxmlState: public QWidget
{
  Q_OBJECT

public:
  RosScxmlState(QWidget *parent = nullptr);

  void stateChange(bool state);

Q_SIGNALS:
  void stateChanged();

private:
  bool m_state = false;
};

#endif // ROS_SCXML_STATE_H

