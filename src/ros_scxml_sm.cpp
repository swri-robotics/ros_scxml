
#include <iostream>

#include <QTextStream>
#include <QApplication>

#include "ros_scxml.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_scxml");
  ros::NodeHandle nh("~");

  ros::Time::init();

  ros::Rate throttle(100);

  QApplication app(argc, argv);

  // get sensor config from params
  std::string state_machine_file;
  nh.getParam("state_machine", state_machine_file);
  ROS_INFO("Loading state machine [%s]", state_machine_file.c_str());

  QScxmlStateMachine *machine = QScxmlStateMachine::fromFile(
        QString::fromStdString(state_machine_file));

  if (!machine->parseErrors().isEmpty()) {
      QTextStream errs(stderr, QIODevice::WriteOnly);
      const auto errors = machine->parseErrors();
      for (const QScxmlError &error : errors) {

        ROS_ERROR("SCXML Parser [%s]: [%s]",
                  state_machine_file.c_str(),
                  error.toString().toStdString().c_str());
      }

      ros::shutdown();
      return 0;
  }

  ROS_INFO("Starting state machine");
  RosScxml state_machine(machine);
  machine->setParent(&state_machine);
  machine->start();

  ros::Publisher acticve_states_pub = nh.advertise<ros_scxml::ActiveStates>("active_states", 1);
  ros::ServiceServer event_service = nh.advertiseService("eventTrigger", &RosScxml::eventTrigger, &state_machine);
  ros::ServiceServer start_state_machine = nh.advertiseService("start", &RosScxml::start_state_machine, &state_machine);
  ros::ServiceServer stop_state_machine = nh.advertiseService("stop", &RosScxml::stop_state_machine, &state_machine);

  //TODO: Figure out how to intercept machine->dataModelChanged and die gracefully
  while(ros::ok() && machine->isInitialized())
  {
    ros::spinOnce();

    state_machine.activeStates.state_names.clear();
    foreach(QString str, machine->activeStateNames(false))
    {
      state_machine.activeStates.state_names.push_back(str.toStdString());
    }

    ros_scxml::ActiveStates as;

    as.state_names = state_machine.activeStates.state_names;

    acticve_states_pub.publish(as);

    app.processEvents(QEventLoop::AllEvents);
    throttle.sleep();
  }

  app.exit();

}
