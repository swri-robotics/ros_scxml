#ifndef PACKMLSTATE_H
#define PACKMLSTATE_H

#include <QScxmlStateMachine>
#include <QWidget>

class PackmlState: public QWidget
{
  Q_OBJECT

public:
  PackmlState(QWidget *parent = nullptr);

  void stateChange(bool state);

Q_SIGNALS:
  void stateChanged();

private:
  bool m_state = false;
};

#endif // PACKMLSTATE_H

