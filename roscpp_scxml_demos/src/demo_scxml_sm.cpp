/*
 * @author Jorge Nicho
 * @file demo_scxml_sm.cpp
 * @date Jan 13, 2020
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
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <QApplication>
#include <scxml_core/state_machine.h>
#include <boost/format.hpp>

static const std::string CURRENT_STATE_TOPIC = "current_state";
static const std::string EXECUTE_ACTION_TOPIC = "execute_action";
static const std::string PRINT_ACTIONS_SERVICE = "print_actions";
static const std::string PROCESS_EXECUTION_MSG = "process_msg";

using namespace scxml_core;

class ROSInterface : protected QObject
{
public:
  ROSInterface(ros::NodeHandle nh, StateMachine* sm) : sm_(sm), current_state_("none")
  {
    state_pub_ = nh.advertise<std_msgs::String>(CURRENT_STATE_TOPIC, 1);

    // this connection allows receiving the active state through a qt signals emitted by the sm
    connect(sm, &StateMachine::state_entered, [this](std::string state_name) { current_state_ = state_name; });

    // publishes the active state name
    pub_timer_ = nh.createTimer(ros::Duration(0.2), [this](const ros::TimerEvent& e) {
      std_msgs::String msg;
      msg.data = current_state_;
      state_pub_.publish(msg);
    });

    // prompts the sm to execute an action.
    execute_state_subs_ =
        nh.subscribe<std_msgs::String>(EXECUTE_ACTION_TOPIC, 1, [this](const std_msgs::StringConstPtr& msg) {
          if (sm_->isBusy())
          {
            ROS_ERROR("State Machine is busy");
            return;
          }

          std::shared_future<Response> res_fut =
              sm_->execute(Action{ .id = msg->data, .data = ros::Time::now().toSec() });
          if (res_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          {
            ROS_INFO("Took too long to get response");
            return;
          }

          Response res = res_fut.get();
          if (!res)
          {
            return;
          }

          ROS_INFO("Action %s successfully executed", msg->data.c_str());

          // checking for returned data
          if (!res.data.empty())
          {
            try
            {
              ROS_INFO_STREAM("Time value returned from state: " << boost::any_cast<double>(res.data) << " seconds");
            }
            catch (boost::bad_any_cast& e)
            {
              ROS_WARN_STREAM(e.what() << ": " << res.data.type().name());
            }
          }
        });

    // prints the available actions at the current state
    print_actions_server_ = nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        PRINT_ACTIONS_SERVICE, [this](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
          if (!sm_->isRunning())
          {
            res.message = "SM is not running";
            res.success = false;
            ROS_ERROR_STREAM(res.message);
            return true;
          }

          std::vector<std::string> actions = sm_->getAvailableActions();
          if (actions.empty())
          {
            res.message = "No actions available within the current state";
            res.success = false;

            ROS_ERROR_STREAM(res.message);
            return true;
          }

          std::cout << "\nSM Actions: " << std::endl;
          for (const std::string& s : actions)
          {
            std::cout << "\t-" << s << std::endl;
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

class MockApplication
{
public:
  MockApplication(ros::NodeHandle nh)
  {
    continue_process_ = false;
    ready_ = false;
    process_msg_pub_ = nh.advertise<std_msgs::String>(PROCESS_EXECUTION_MSG, 1);
  }

  ~MockApplication() {}

  void resetProcess()
  {
    ready_ = true;
    continue_process_ = true;
    counter_ = 0;
    ROS_INFO_NAMED("Process", "Reseted process variables");
  }

  int getCounter() const { return counter_; }

  /**
   * @brief this is a blocking function
   * @return True on success, false otherwise
   */
  bool executeProcess()
  {
    if (!ready_)
    {
      return false;
    }

    ros::Duration process_pause(2.0);
    continue_process_ = true;
    while (continue_process_ && ros::ok())
    {
      std_msgs::String msg;
      msg.data = boost::str(boost::format("Incremented counter to %i") % counter_);
      process_msg_pub_.publish(msg);
      process_pause.sleep();
      counter_++;
    }
    return true;
  }

  void pauseProcess() { continue_process_ = false; }

  void haltProcess()
  {
    continue_process_ = false;
    ready_ = false;
    ROS_INFO_NAMED("Process", "Process halted");
    return;
  }

protected:
  ros::Publisher process_msg_pub_;
  std::atomic<bool> continue_process_;
  std::atomic<bool> ready_;
  int counter_;
};

int main(int argc, char** argv)
{
  using namespace scxml_core;

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
  if (!ph.getParam("state_machine_file", state_machine_file))
  {
    ROS_ERROR("failed to load state machine file");
    return -1;
  }

  // create state machine
  StateMachine* sm = new StateMachine();
  if (!sm->loadFile(state_machine_file))
  {
    ROS_ERROR("Failed to load state machine file %s", state_machine_file.c_str());
    return -1;
  }
  ROS_INFO("Loaded file");

  // adding application methods to SM
  MockApplication process_app(nh);
  bool success = false;
  std::vector<std::function<bool()> > functions = {

    // custom function invoked when the "st3Reseting" state is entered
    [&]() -> bool {
      return sm->addEntryCallback(
          "st3Reseting",
          [&](const Action& action) -> Response {
            // checking passed user data first
            if (!action.data.empty())
            {
              try
              {
                double secs = boost::any_cast<double>(action.data);
                ROS_INFO("State received time value of %f seconds", secs);
              }
              catch (boost::bad_any_cast& e)
              {
                ROS_WARN_STREAM(e.what());
              }
            }

            process_app.resetProcess();
            ros::Duration(3.0).sleep();
            // queuing action, should exit the state
            sm->postAction(Action{ .id = "trIdle" });
            return Response(true, 22.0);
          },
          false);  // false = runs sequentially, use for non-blocking functions
    },

    // custom function invoked when the "st3Execute" state is entered
    [&]() -> bool {
      return sm->addEntryCallback(
          "st3Execute",
          [&process_app](const Action& action) -> Response { return process_app.executeProcess(); },
          true);  // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st3Execute" state is exited
    [&]() -> bool {
      return sm->addExitCallback("st3Execute", [&process_app]() {
        process_app.pauseProcess();
        ROS_INFO_STREAM("Process counter at " << process_app.getCounter());
      });
    },

    // custom function invoked prior to entering the "st3Completing" state
    [&]() -> bool {
      return sm->addPreconditionCallback("st3Completing", [&process_app](const Action& action) -> Response {
        Response res;
        static const int REQ_VAL = 16;
        if (process_app.getCounter() >= REQ_VAL)
        {
          return true;
        }
        res.msg = boost::str(boost::format("Counter less than %i") % REQ_VAL);
        res.success = false;

        return res;
      });
    },

    // custom function invoked when the "st3Suspending" state is entered
    [&]() -> bool {
      return sm->addEntryCallback(
          "st3Suspending",
          [&](const Action& action) -> Response {
            ROS_INFO("Suspending process");
            process_app.haltProcess();
            ros::Duration(3.0).sleep();

            // queuing action, should exit the state
            sm->postAction(Action{ .id = "trSuspending" });
            Response res;
            res.success = true;
            return std::move(res);
          },
          true);  // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st3Completing" state is entered
    [&]() -> bool {
      return sm->addEntryCallback(
          "st3Completing",
          [&process_app](const Action& action) -> Response {
            Response res;
            res.success = true;
            res.data = ros::Time::now().toSec();
            return std::move(res);
          },
          false);  // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st2Clearing" state is entered, it will exit after waiting for 3 seconds
    [&]() -> bool {
      return sm->addEntryCallback(
          "st2Clearing",
          [&](const Action& action) -> Response {
            ROS_INFO("Clearing to enable process, please wait ...");
            ros::Duration(3.0).sleep();

            // queuing action, should exit the state
            sm->postAction(Action{ .id = "trStopped" });
            return true;
          },
          true);  // true = runs asynchronously, use for blocking functions
    },

    // custom function invoked when the "st2Clearing" state is exited
    [&]() -> bool {
      return sm->addExitCallback("st2Clearing",
                                 [&process_app]() { ROS_INFO("Done Clearing, Process is now good to go ..."); });
    },
  };

  // calling all functions and evaluating returned args
  if (!std::all_of(functions.begin(), functions.end(), [](auto& f) { return f(); }))
  {
    ROS_ERROR("Failed to setup application specific functions");
    return -1;
  }

  // create ROS interface
  ROSInterface ros_interface(nh, sm);

  // start sm
  if (!sm->start())
  {
    ROS_ERROR("Failed to start SM");
    return -1;
  }
  ROS_INFO("State Machine Ready");

  // main loop
  while (ros::ok())
  {
    app.processEvents(QEventLoop::AllEvents);
    throttle.sleep();
  }

  app.exit();
  spinner.stop();

  return 0;
}
