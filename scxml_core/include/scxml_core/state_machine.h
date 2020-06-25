/*
 * @author Jorge Nicho
 * @file state_machine.h
 * @date Apr 10, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Southwest Research Institute
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

#ifndef INCLUDE_SCXML_CORE_STATE_MACHINE_H_
#define INCLUDE_SCXML_CORE_STATE_MACHINE_H_

#include <list>
#include <mutex>
#include <memory>
#include <deque>
#include <future>
#include <QScxmlStateMachine>
#include <private/qscxmlstatemachineinfo_p.h>
#include <private/qscxmlstatemachine_p.h>
#include <QTimer>
#include <QThread>
#include <QThreadPool>
#include <QtConcurrent/QtConcurrent>
#include <boost/none.hpp>
#include <boost/any.hpp>
#include <log4cxx/logger.h>

namespace scxml_core
{
struct Action
{
  Action() = default;
  Action(std::string id, boost::any data = boost::any()) : id(id), data(data){};
  std::string id;  /**@brief name of the event */
  boost::any data; /**@brief optional data needed by the state */

  /**
   * @brief allows using the action as a map key by using the id member
   * @param action
   * @return
   */
  bool operator<(const Action& action) const { return action.id < this->id; }
};

struct Response
{
  /**
   * @brief Constructor for the response object
   * @param success   Set to true if the requested action was completed, use false otherwise.
   * @param data      Optional data that was generated from the requested transaction.
   * @param msg       Optional error message
   */
  Response(bool success = true, boost::any data = boost::any(), std::string msg = "")
    : success(success), data(data), msg(msg)
  {
  }

  Response(const Response& obj) : success(obj.success), data(obj.data), msg(obj.msg) {}

  ~Response() {}

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

  operator bool() const { return success; }

  bool success;
  boost::any data;
  std::string msg;
};

class StateMachine;

/**
 * @class state_machine::ResponseFuture
 * @brief A shared future with additional information regarding the Response object
 *        return by entry callbacks associated with states
 */
class ResponseFuture : public std::shared_future<Response>
{
public:
  /**
   * True if the future is not linked to the response produced by the state's entry callback
   * @return a boolean
   */
  bool isDetached() const { return is_detached_; }

  /**
   * @brief the name of the state that generated the response by invoking its entry callback
   * @return a string
   */
  std::string getState() const { return state_name_; }

  /**
   * @brief true if the state that produced the response is atomic
   * @return a boolean
   */
  bool isAtomic() const { return is_atomic_; }

  ~ResponseFuture() {}

private:
  friend class StateMachine;

  ResponseFuture(const std::shared_future<Response>& res_fut,
                 bool is_detached,
                 const std::string& state_name,
                 bool is_atomic);

  bool is_detached_;
  bool is_atomic_;
  std::string state_name_;
};

/**
 * @class state_machine::TransitionResult
 * @brief encapsulates the details of the state machine transition
 */
class TransitionResult
{
public:
  /**
   * @brief Evaluates to true when the transition succeeded, false otherwise
   */
  operator bool() const { return succeeded_; }

  /**
   * @brief A vector of responses produced by the entry callbacks associated with the states that were
   *        made active by this transition.  The front element corresponds to the atomic state's response
   * @return a vector of ResponseFutures
   */
  const std::vector<ResponseFuture>& getResponses() const { return responses_; }

  /**
   * @brief Returns the response produced by the callback associated with the atomic state
   *        made active by this transition.  Throws an exception when no response is available.
   * @return A reference to a response future
   */
  const ResponseFuture& getResponse() const;

  /**
   * @brief evaluates to true if there's a response waiting to be returned by the entry callback associated with
   *        the now active atomic state.  When true it should be safe to call the wait_for() and get() methods
   *        on the ResponseFuture
   * @return  a boolean
   */
  bool hasPendingResponse() const;

  /**
   * @brief the error message providing a brief description of the error that prevented the transition.
   * @return a string
   */
  std::string getErrorMessage() const { return err_msg_; }

private:
  friend class StateMachine;

  TransitionResult(bool succeeded, const std::vector<ResponseFuture>& responses, const std::string err_msg = "");

  bool succeeded_;
  std::vector<ResponseFuture> responses_;
  std::string err_msg_;
};

class StateMachine : public QObject
{
public:
  typedef std::function<Response(const Action&)> PreconditionCallback;
  typedef std::function<Response(const Action&)> EntryCallback;

private:
  class EntryCbHandler
  {
  public:
    EntryCbHandler(QThreadPool* thread_pool, EntryCallback cb, bool discard_response = false)
      : discard_response_(discard_response), cb_(cb), tpool_(thread_pool)
    {
    }

    /**
     * @brief invokes the state entry callback
     * @param arg The action
     * @return a future binded to the Response returned by the entry callback
     */
    std::shared_future<Response> operator()(const Action& arg);

    bool discard_response_;
    EntryCallback cb_;
    QThreadPool* tpool_;
  };
  typedef std::shared_ptr<EntryCbHandler> EntryCbHandlerPtr;

  Q_OBJECT
public:
  StateMachine(double event_loop_period = 0.1, log4cxx::LoggerPtr logger = nullptr);
  StateMachine(QScxmlStateMachine* sm, double event_loop_period = 0.1, log4cxx::LoggerPtr logger = nullptr);
  virtual ~StateMachine();

  bool loadFile(const std::string& filename);

  bool start();
  bool stop();
  bool isRunning() const;

  /**
   * @brief executes the action
   * @param action The action object
   * @param force Ignores the action queue and forces the execution of the requested action, recommended
   *              for critical tasks only.
   * @return  A transition result object
   */
  TransitionResult execute(const Action& action, bool force = false);

  /**
   * @brief Adds the action to the queue
   * @param action  The action to execute
   */
  void postAction(Action action);

  /**
   * @brief adds a callback that gets invoked to check for pre-conditions prior to transitioning into a state.
   * When the callback returns a valid result then the transition proceeds, otherwise the transition is
   * negated.
   * @param st_name The name of the state
   * @param cb      The callback to be invoked; long running blocking functions are not recommended.
   * @return  True on success, false otherwise
   */
  bool addPreconditionCallback(const std::string& st_name, PreconditionCallback cb);

  /**
   * @brief adds a callback that gets invoked asynchronously when a state is entered
   * @param st_name           The name of the state
   * @param cb                The callback to be invoked; it can be a blocking function.
   * @param discard_response  When true the callback will be executed asynchronously however the Response returned by
   * the callback won't be forwarded by the < b>execute()< /b> method back to the client code.
   * @return  True on success, false otherwise
   */
  bool addEntryCallback(const std::string& st_name, EntryCallback cb, bool discard_response = false);

  /**
   * @brief adds a callback that gets invoked when a state is exited
   * @param st_name The name of the state
   * @param cb      The callback to be invoked; long running blocking functions are not recommended.
   * @return  True on success, false otherwise
   */
  bool addExitCallback(const std::string& st_name, std::function<void()> cb);

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
  bool emitStateEnteredSignal();
  TransitionResult executeAction(const Action& action);
  void processQueuedActions();
  std::vector<int> getTransitionsIDs(const QVector<int>& states) const;
  std::vector<int> getValidTransitionIDs() const;

  // state machine members
  QScxmlStateMachineInfo* sm_info_;
  QScxmlStateMachine* sm_;
  QScxmlStateMachinePrivate* sm_private_;
  std::map<std::string, int> st_ids_map_;

  // action queue members
  mutable std::mutex action_queue_mutex_;
  std::list<Action> action_queue_;

  // action execution members
  double event_loop_period_;
  QTimer* execute_action_timer_;
  mutable std::mutex consuming_action_mutex_;

  std::atomic<bool> busy_executing_action_;
  std::atomic<bool> busy_consuming_entry_cb_;

  std::shared_future<Action> action_future_;
  using ResponseFuturesMap = std::map<int, std::shared_future<Response>>;
  std::promise<ResponseFuturesMap> responses_map_promise_;
  std::map<std::string, PreconditionCallback> precond_callbacks_;
  std::map<std::string, EntryCbHandlerPtr> entry_callbacks_;
  std::map<std::string, std::function<void()>> exit_callbacks_;
  QThreadPool* async_thread_pool_;
  mutable std::mutex entered_states_mutex_;
  std::deque<QScxmlStateMachineInfo::StateId> entered_states_queue_;
  log4cxx::LoggerPtr logger_;
};

} /* namespace scxml_core */

#endif /* INCLUDE_SCXML_CORE_STATE_MACHINE_H_ */
