
#include <iostream>
#include "ros_scxml/state_machine.h"

#include <QTextStream>
#include <QApplication>

#include "ros_scxml.h"


static const std::string CURRENT_STATE_TOPIC = "current_state";
static const std::string EXECUTE_STATE_TOPIC = "execute_state";
static const std::string PRINT_ACTIONS_SERVICE = "print_actions";

using namespace ros_scxml;

class ROSInterface: protected QObject
{
public:
  ROSInterface(ros::NodeHandle nh,StateMachine* sm):
    sm_(sm),
    current_state_("none")
  {
    state_pub_ = nh.advertise<std_msgs::String>(CURRENT_STATE_TOPIC,1);

    // this connection allows receiving the active state through a qt signals emitted by the sm
    connect(sm,&StateMachine::state_entered,[this](std::string state_name){
      current_state_ = state_name;
    });

    // publishes the active state name
    pub_timer_ = nh.createTimer(ros::Duration(0.2),[this](const ros::TimerEvent&e ){
      std_msgs::String msg;
      msg.data = current_state_;
      state_pub_.publish(msg);
    });

    // prompts the sm to execute an action.
    execute_state_subs_ = nh.subscribe<std_msgs::String>(EXECUTE_STATE_TOPIC,1,
                                                         [this](const std_msgs::StringConstPtr& msg){
      if(sm_->isBusy())
      {
        ROS_ERROR("State Machine is busy");
        return;
      }

      Response res = sm_->execute(Action{.id = msg->data});
      if(res)
      {
        ROS_INFO("Action %s successfully executed",msg->data.c_str());
      }
    });

    // prints the available actions at the current state
    print_actions_server_ = nh.advertiseService<std_srvs::Trigger::Request,std_srvs::Trigger::Response>(
        PRINT_ACTIONS_SERVICE,[this](std_srvs::Trigger::Request& req,
        std_srvs::Trigger::Response& res){

      if(!sm_->isRunning())
      {
        res.message = "SM is not running";
        res.success = false;
        ROS_ERROR_STREAM(res.message);
        return true;
      }

      std::vector<std::string> actions = sm_->getAvailableActions();
      if(actions.empty())
      {
        res.message = "No actions available within the current state";
        res.success = false;

        ROS_ERROR_STREAM(res.message);
        return true;
      }

      std::cout<<"\nSM Actions: "<<std::endl;
      for(const std::string& s : actions)
      {
        std::cout<<"\t-"<<s<<std::endl;
      }
      res.success = true;
      return true;
    });

    pub_timer_.start();
  }


protected:

  std::string current_state_;
  ros::Timer pub_timer_;
  ros::Publisher state_pub_;
  ros::Subscriber execute_state_subs_;
  ros::ServiceServer print_actions_server_;
  StateMachine* sm_;
};


int main(int argc, char **argv)
{
  using namespace ros_scxml;

  ros::init(argc, argv, "demo_scxml_state_machine");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate throttle(100);

  // qt application
  QApplication app(argc, argv);

  // get params
  std::string state_machine_file;
  if(!ph.getParam("state_machine_file", state_machine_file))
  {
    ROS_ERROR("failed to load state machine file");
    return -1;
  }

  // create state machine
  StateMachine* sm = new StateMachine();
  if(!sm->loadFile(state_machine_file))
  {
    ROS_ERROR("Failed to load state machine file %s",state_machine_file.c_str());
    return -1;
  }
  ROS_INFO("Loaded file");

  if(!sm->start())
  {
    ROS_ERROR("Failed to start SM");
    return -1;
  }

  // create ROS interface
  ROSInterface ros_interface(nh,sm);

  // main loop
  while(ros::ok())
  {
    app.processEvents(QEventLoop::AllEvents);
    throttle.sleep();
  }

  app.exit();
  spinner.stop();

  return 0;

}
