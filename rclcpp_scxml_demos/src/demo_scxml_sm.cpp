/*
 * @author Jorge Nicho
 * @file demo_scxml_sm.cpp
 * @date Jan 10, 2020
 * @copyright Copyright (c) 2020, Southwest Research Institute
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <QApplication>
#include "scxml_core/state_machine.h"
#include <boost/format.hpp>
#include <chrono>

static const std::string CURRENT_STATE_TOPIC = "current_state";
static const std::string EXECUTE_ACTION_TOPIC = "execute_action";
static const std::string PRINT_ACTIONS_SERVICE = "print_actions";
static const std::string PROCESS_EXECUTION_MSG = "process_msg";

using namespace scxml_core;

class ROSInterface: public QObject
{
public:
  ROSInterface(StateMachine* sm, std::shared_ptr<rclcpp::Node> node):
    sm_(sm),
	node_(node),
    current_state_("none")
  {
	rclcpp::QoS qos(rclcpp::KeepLast(7));
    state_pub_ = node_->create_publisher<std_msgs::msg::String>(CURRENT_STATE_TOPIC, qos);

    // this connection allows receiving the active state through a qt signals emitted by the sm
    connect(sm,&StateMachine::state_entered,[this](std::string state_name){
      current_state_ = state_name;
    });

    // prompts the sm to execute an action.
    execute_state_subs_ = node_->create_subscription<std_msgs::msg::String>(EXECUTE_ACTION_TOPIC,1,
                                                         [this](const std_msgs::msg::String::SharedPtr msg){
      if(sm_->isBusy())
      {
        RCLCPP_ERROR(node_->get_logger(), "State Machine is busy");
        return;
      }

      rclcpp::Clock ros_clock;
      Response res = sm_->execute(Action{.id = msg->data,.data = ros_clock.now()});
      if(!res)
      {
        return;
      }

      RCLCPP_INFO(node_->get_logger(), "Action %s successfully executed",msg->data.c_str());

      // checking for returned data
      if(!res.data.empty())
      {
        try
        {
          RCLCPP_INFO(node_->get_logger(), std::string("Time value returned from state: ") +
        		  std::to_string(boost::any_cast<double>(res.data)) + std::string(" seconds"));
        }
        catch(boost::bad_any_cast &e)
        {
          RCLCPP_WARN(node_->get_logger(), e.what() + std::string(": ") + res.data.type().name());
        }
      }
    });

    // prints the available actions at the current state
    print_actions_server_ = node_->create_service<std_srvs::srv::Trigger>(
        PRINT_ACTIONS_SERVICE,[this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
				std::shared_ptr<std_srvs::srv::Trigger::Response > res) -> void{

      if(!sm_->isRunning())
      {
        res->message = "SM is not running";
        res->success = false;
        RCLCPP_ERROR(node_->get_logger(), res->message);
        return;
      }

      std::vector<std::string> actions = sm_->getAvailableActions();
      if(actions.empty())
      {
        res->message = "No actions available within the current state";
        res->success = false;

        RCLCPP_ERROR(node_->get_logger(), res->message);
        return;
      }

      std::cout<<"\nSM Actions: "<<std::endl;
      for(const std::string& s : actions)
      {
        std::cout<<"\t-"<<s<<std::endl;
      }
      res->success = true;
      return;
    });

    // publishes the active state name
    pub_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.2),[this](){
      std_msgs::msg::String msg;
      msg.data = current_state_;
      state_pub_->publish(msg);
    });

  }


protected:

  std::string current_state_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_state_subs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_actions_server_;
  StateMachine* sm_;
};

class MockApplication
{
public:
  MockApplication(std::shared_ptr<rclcpp::Node> node):
	  node_(node)
  {
    continue_process_ = false;
    ready_ = false;
    process_msg_pub_ = node_->create_publisher<std_msgs::msg::String>(PROCESS_EXECUTION_MSG,1);
  }

  ~MockApplication()
  {

  }

  void resetProcess()
  {
    ready_ = true;
    continue_process_ = true;
    counter_ = 0;
    RCLCPP_INFO(node_->get_logger(),"Reseted process variables");
  }

  int getCounter() const
  {
    return counter_;
  }

  /**
   * @brief this is a blocking function
   * @return True on success, false otherwise
   */
  bool executeProcess()
  {
    if(!ready_)
    {
      return false;
    }

    //rclcpp::Duration process_pause(2.0);
    std::chrono::duration<double> process_pause(2.0); // seconds
    continue_process_ = true;
    while(continue_process_ && rclcpp::ok())
    {
      std_msgs::msg::String msg;
      msg.data = boost::str(boost::format("Incremented counter to %i") % counter_);
      process_msg_pub_->publish(msg);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(process_pause));
      counter_++;
    }
    return true;
  }

  void pauseProcess()
  {
    continue_process_= false;
  }

  void haltProcess()
  {
    continue_process_ = false;
    ready_ = false;
    RCLCPP_INFO(node_->get_logger(),"Process halted");
    return;
  }


protected:

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr process_msg_pub_;
  std::shared_ptr<rclcpp::Node> node_;
  std::atomic<bool> continue_process_;
  std::atomic<bool> ready_;
  int counter_;

};


int main(int argc, char **argv)
{
  using namespace scxml_core;

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("demo_scxml_state_machine",
		  rclcpp::NodeOptions());

  rclcpp::Rate throttle(100);

  // qt application
  QApplication app(argc, argv);

  // get params
  std::string state_machine_file;
  const std::vector<rclcpp::ParameterValue> parameters = {node->declare_parameter("state_machine_file")};
  // checking parameter(s)
  if(parameters.front().get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    RCLCPP_ERROR(node->get_logger(),"Failed to read state machine file parameter");
    return -1;
  }

  // getting parameters now
  state_machine_file = parameters[0].get<std::string>();

  // create state machine
  StateMachine* sm = new StateMachine();
  if(!sm->loadFile(state_machine_file))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load state machine file %s",state_machine_file.c_str());
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded file");

  // adding application methods to SM
  MockApplication process_app(node);
  std::vector< std::function<bool ()> > functions = {

    // custom function invoked when the "st3Reseting" state is entered
    [&]() -> bool{
      return sm->addEntryCallback("st3Reseting",[&](const Action& action) -> Response{

        // checking passed user data first
        if(!action.data.empty())
        {
          try
          {
            double secs = boost::any_cast<double>(action.data);
            RCLCPP_INFO(node->get_logger(),"State received time value of %f seconds",secs);
          }
          catch(boost::bad_any_cast &e)
          {
            RCLCPP_WARN(node->get_logger(), e.what());
          }
        }

        process_app.resetProcess();
        rclcpp::sleep_for(std::chrono::milliseconds(3000));
        // queuing action, should exit the state
        sm->postAction(Action{.id="trIdle"});
        return true;
      },false); // false = runs sequentially, use for non-blocking functions
    },

    // custom function invoked when the "st3Execute" state is entered
    [&]() -> bool{
      return sm->addEntryCallback("st3Execute",[&process_app](const Action& action) -> Response{
        return process_app.executeProcess();
      },true); // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st3Execute" state is exited
    [&]() -> bool{
      return sm->addExitCallback("st3Execute",[&process_app, &node](){
        process_app.pauseProcess();
        RCLCPP_INFO(node->get_logger(), "Process counter at %i", process_app.getCounter());
      });
    },

    // custom function invoked prior to entering the "st3Completing" state
    [&]() -> bool{
      return sm->addPreconditionCallback("st3Completing",[&process_app](const Action& action) -> Response{
        Response res;
        static const int REQ_VAL = 16;
        if(process_app.getCounter() >= REQ_VAL)
        {
          return true;
        }
        res.msg = boost::str(boost::format("Counter less than %i") % REQ_VAL);
        res.success = false;

        return res;
      });
    },

    // custom function invoked when the "st3Suspending" state is entered
    [&]() -> bool{
      return sm->addEntryCallback("st3Suspending",[&](const Action& action) -> Response{
        RCLCPP_INFO(node->get_logger(), "Suspending process");
        process_app.haltProcess();
        rclcpp::sleep_for(std::chrono::milliseconds(3000));

        // queuing action, should exit the state
        sm->postAction(Action{.id="trSuspending"});
        Response res;
        res.success = true;
        return std::move(res);
      },true); // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st3Completing" state is entered
    [&]() -> bool{
      return sm->addEntryCallback("st3Completing",[&process_app](const Action& action) -> Response{
        Response res;
        res.success = true;
        res.data = rclcpp::Clock().now();
        return std::move(res);
      },false); // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st2Clearing" state is entered, it will exit after waiting for 3 seconds
    [&]() -> bool{
      return sm->addEntryCallback("st2Clearing",[&](const Action& action) -> Response{
        RCLCPP_INFO(node->get_logger(), "Clearing to enable process, please wait ...");
        rclcpp::sleep_for(std::chrono::milliseconds(3000));

        // queuing action, should exit the state
        sm->postAction(Action{.id="trStopped"});
        return true;
      }, true); // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st2Clearing" state is exited
    [&]() -> bool{
      return sm->addExitCallback("st2Clearing",[&process_app, &node](){
        RCLCPP_INFO(node->get_logger(), "Done Clearing, Process is now good to go ...");
      });
    },
  };

  // calling all functions and evaluating returned args
  if(!std::all_of(functions.begin(),functions.end(),[](auto& f){
    return f();
  }))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to setup application specific functions");
    return -1;
  }

  // create ROS interface
  ROSInterface ros_interface(sm, node);

  // start sm
  if(!sm->start())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to start SM");
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "State Machine Ready");

  // main loop
  while(rclcpp::ok())
  {
    app.processEvents(QEventLoop::AllEvents);
    throttle.sleep();
    rclcpp::spin_some(node);
  }

  app.exit();

  return 0;

}
