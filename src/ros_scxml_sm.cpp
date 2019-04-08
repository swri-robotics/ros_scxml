#include <ros/ros.h>
#include <ros_scxml/ActiveStates.h>

#include <iostream>

#include <QTextStream>
#include <QApplication>

#include "ros_scxml.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "packml");
  ros::NodeHandle nh("~");

  ros::Time::init();

  ros::Rate throttle(100);


  QApplication app(argc, argv);

  ROS_INFO("loading state machine");

  QScxmlStateMachine *machine = QScxmlStateMachine::fromFile(
              QStringLiteral(":packml.scxml"));
  if (!machine->parseErrors().isEmpty()) {
      QTextStream errs(stderr, QIODevice::WriteOnly);
      const auto errors = machine->parseErrors();
      for (const QScxmlError &error : errors) {
          errs << error.toString();
      }

      return -1;
  }

  RosScxml state_machine(machine);
  machine->setParent(&state_machine);
  machine->start();

  ros::Publisher acticve_states_pub = nh.advertise<ros_scxml::ActiveStates>("active_states", 1);
  ros::Subscriber event_sub = nh.subscribe("state_event", 1, &RosScxml::eventTrigger_Callback, &state_machine);

  ROS_INFO("starting state machine");


  ROS_INFO("starting app");
  //app.exec();

  while(ros::ok())
  {
     ros::spinOnce();

     state_machine.activeStates.state_names.clear();
     foreach(QString str, machine->activeStateNames())
        state_machine.activeStates.state_names.push_back(str.toStdString());

     ros_scxml::ActiveStates as;

     as.state_names = state_machine.activeStates.state_names;

     acticve_states_pub.publish(as);

     app.processEvents(QEventLoop::AllEvents);
     throttle.sleep();
  }

  app.exit();

}
