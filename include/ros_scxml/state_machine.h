/*
 * state_machine.h
 *
 *  Created on: Apr 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_ROS_SCXML_STATE_MACHINE_H_
#define INCLUDE_ROS_SCXML_STATE_MACHINE_H_

#include <list>
#include <mutex>
#include <memory>
#include <QScxmlStateMachine>
#include <private/qscxmlstatemachineinfo_p.h>
#include <private/qscxmlstatemachine_p.h>
#include <QTimer>
#include <QThread>
#include <QThreadPool>
#include <QtConcurrent/QtConcurrent>
#include <boost/none.hpp>
#include <boost/any.hpp>

namespace ros_scxml
{

struct Action
{
  std::string id;   /**@brief name of the event */
  boost::any data;  /**@brief optional data needed by the state */

  /**
   * @brief allows using the action as a map key by using the id member
   * @param action
   * @return
   */
  bool operator< (const Action& action) const
  {
    return action.id < this->id;
  }
};

struct Response
{
  /**
   * @brief Constructor for the response object
   * @param success   Set to true if the requested action was completed, use false otherwise.
   * @param data          Optional data that was generated from the requested transaction.
   */
  Response(bool success = true, boost::any data = boost::any(), std::string msg = ""):
    success(success),
    data(data),
    msg(msg)
  {

  }

  Response(const Response& obj):
    success(obj.success),
    data(obj.data),
    msg(obj.msg)
  {
  }

  ~Response()
  {

  }

  Response& operator=(const Response& obj)
  {
    success = obj.success;
    data = obj.data;
    msg = obj.msg;
    return *this;
  }

  Response& operator=(const bool& b)
  {
    this->success = b;
    return *this;
  }

  operator bool() const
  {
    return success;
  }

  bool success;
  boost::any data;
  std::string msg;
};

class StateMachine: public QObject
{
public:

  typedef std::function< Response (const Action& ) > PreconditionCallback;
  typedef std::function< Response (const Action& ) > EntryCallback;

private:

  class EntryCbHandler
  {
  public:
    EntryCbHandler(QThreadPool* thread_pool,EntryCallback cb,bool async_execution = false):
      cb_(cb),
      async_execution_(async_execution),
      tpool_(thread_pool)
    {

    }

    Response operator()(const Action& arg)
    {
      Response res;
      if(async_execution_)
      {
        QFuture<Response> future = QtConcurrent::run(tpool_,[this,arg](){
          return cb_(arg);
        });
        res = true;
      }
      else
      {
        res = cb_(arg);
      }
      return std::move(res);
    }

  private:
    bool async_execution_;
    EntryCallback cb_;
    QThreadPool* tpool_;

  };
  typedef std::shared_ptr<EntryCbHandler> EntryCbHandlerPtr;

  Q_OBJECT
public:

  StateMachine(double event_loop_period = 0.1);
  StateMachine(QScxmlStateMachine* sm, double event_loop_period = 0.1);
  virtual ~StateMachine();

  bool loadFile(const std::string& filename);

  bool start();
  bool stop();
  bool isRunning() const;

  /**
   * @brief immediately execute the action
   * @param action The action object
   * @return  A response object
   */
  Response execute(const Action& action);

  /**
   * @brief Adds the action to the queue
   * @param action  The action to execute
   */
  void postAction(Action action);

  /**
   * @brief adds a callback that gets invoked to check for pre-conditions prior to making a transition.
   * When the callback returns a valid result then the transition proceeds, otherwise the transition is
   * negated.
   * @param st_name The name of the state
   * @param cb      The callback to be invoked; it must be a non-blocking function.
   * @return  True on success, false otherwise
   */
  bool addPreconditionCallback(const std::string& st_name,PreconditionCallback cb);

  /**
   * @brief adds an callback that gets invoked when a state is entered
   * @param st_name         The name of the state
   * @param cb              The callback to be invoked; it can be a blocking function.
   * @param async_execution Set to true for blocking callbacks; the callback will be executed asynchronously
   * @return  True on success, false otherwise
   */
  bool addEntryCallback(const std::string& st_name,EntryCallback cb, bool async_execution = false);

  /**
   * @brief adds an callback that gets invoked when a state is exited
   * @param st_name The name of the state
   * @param cb      The callback to be invoked; it must be a non-blocking function.
   * @return  True on success, false otherwise
   */
  bool addExitCallback(const std::string& st_name,std::function< void ()> cb);

  /**
   * @brief provides a list of actions available at the current state
   * @return A vector of actions.
   */
  std::vector<std::string> getAvailableActions() const;
  std::vector<std::string> getActions(const std::string& state_name) const;
  std::string getCurrentState(bool full_name = false) const;
  std::vector<std::string> getCurrentStates(bool full_name = false) const;
  std::vector<std::string> getStates(bool full_name = false) const;
  bool hasAction(const std::string& state_name, const Action& action);
  bool hasState(const std::string state_name);

  /**
   * @brief checks if the SM is busy processing queued actions
   * @return True when busy, false otherwise.
   */
  bool isBusy() const;

  /**
   * @brief waits until the SM is no longer busy
   * @param timeout The time in seconds to wait
   * @return  True when the SM is no longer busy, false otherwise
   */
  bool wait(double timeout) const;

  signals:
  void state_entered(std::string);
  void state_exited(std::string);

protected:

  void signalSetup();
  Response executeAction(const Action& action);
  void processQueuedActions();
  std::vector<int> getTransitionsIDs(const QVector<int>& states) const;
  std::vector<int>  getValidTransitionIDs() const;
  void saveStateHistory();
  void clearStateHistory();

  // state machine members
  QScxmlStateMachineInfo* sm_info_;
  QScxmlStateMachine* sm_;
  QScxmlStateMachinePrivate* sm_private_;
  std::map<std::string,int> st_ids_map_;
  std::map<int, std::vector<int> > history_buffer_;

  // action queue members
  mutable std::mutex action_queue_mutex_;
  std::list<Action> action_queue_;

  // action execution members
  double event_loop_period_;
  QTimer* execute_action_timer_;
  mutable std::mutex execution_action_mutex_;
  std::atomic<bool> is_busy_;
  std::atomic<bool> all_states_entered_;
  std::map<std::string, PreconditionCallback> precond_callbacks_;
  std::map<std::string, EntryCbHandlerPtr> entry_callbacks_;
  std::map<std::string, std::function< void() > > exit_callbacks_;
  QThreadPool* async_thread_pool_;

};

} /* namespace ros_scxml */

#endif /* INCLUDE_ROS_SCXML_STATE_MACHINE_H_ */
