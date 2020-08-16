/*
 * @author Jorge Nicho
 * @file state_machine.cpp
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

#include "scxml_core/state_machine.h"
#include <QCoreApplication>
#include <QString>
#include <QTime>
#include <QEventLoop>
#include <iostream>
#include <boost/format.hpp>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>

static const int WAIT_TRANSITION_PERIOD = 200;  // ms
static const int WAIT_QT_EVENTS = 20;           // ms

log4cxx::LoggerPtr createConsoleLogger(const std::string& logger_name)
{
  using namespace log4cxx;
  PatternLayoutPtr pattern_layout(new PatternLayout("[\%-5p] [\%c](L:\%L): \%m\%n"));
  ConsoleAppenderPtr console_appender(new ConsoleAppender(pattern_layout));
  log4cxx::LoggerPtr logger(Logger::getLogger(logger_name));
  logger->addAppender(console_appender);
  logger->setLevel(Level::getInfo());
  return logger;
}

static log4cxx::LoggerPtr DEFAULT_LOGGER = createConsoleLogger("Default");

namespace scxml_core
{
using SM = QScxmlStateMachine;
using SMInfo = QScxmlStateMachineInfo;
using TransitionEventsMap = std::map<std::string, std::vector<std::string>>;
using TransitionTable = std::map<std::string, std::map<std::string, std::vector<std::string>>>;

class ScopeExit
{
public:
  ScopeExit(std::atomic<bool>* b) : b_(b) { *b_ = true; }

  ~ScopeExit() { *(b_) = false; }

  std::atomic<bool>* b_;
};

std::string getStateFullName(const QScxmlStateMachineInfo* sm_info, const QScxmlStateMachineInfo::StateId id)
{
  using StateId = QScxmlStateMachineInfo::StateId;

  StateId parent_id = sm_info->stateParent(id);
  std::string full_name = sm_info->stateName(id).toStdString();
  while (parent_id != QScxmlStateMachineInfo::InvalidStateId)
  {
    std::string parent_name = sm_info->stateName(parent_id).toStdString();
    if (parent_name.empty())
    {
      break;
    }
    full_name = parent_name + "::" + full_name;
    parent_id = sm_info->stateParent(parent_id);
  }
  return full_name;
}

// TODO: Decide whether to use this or delete it.  Not being used at the moment
TransitionTable buildTransitionTable(const QScxmlStateMachineInfo* sm_info)
{
  TransitionTable table;
  QVector<QScxmlStateMachineInfo::TransitionId> transitions = sm_info->allTransitions();
  for (int i = 0; i < transitions.size(); i++)
  {
    auto transition_id = transitions[i];

    if (sm_info->transitionType(transition_id) == SMInfo::TransitionType::InvalidTransition)
    {
      continue;
    }

    QVector<QString> transition_events = sm_info->transitionEvents(transition_id);
    if (transition_events.empty())
    {
      continue;
    }

    // source states
    std::string src_st = sm_info->stateName(sm_info->transitionSource(transition_id)).toStdString();

    // getting target states
    std::vector<std::string> target_state_names = {};
    QVector<QScxmlStateMachineInfo::StateId> target_states = sm_info->transitionTargets(transition_id);
    std::transform(
        target_states.begin(),
        target_states.end(),
        std::back_inserter(target_state_names),
        [&sm_info](const QScxmlStateMachineInfo::StateId& s) { return sm_info->stateName(s).toStdString(); });
    if (target_states.empty())
    {
      continue;
    }

    // remove duplicates if any
    std::sort(target_state_names.begin(), target_state_names.end());
    target_state_names.erase(std::unique(target_state_names.begin(), target_state_names.end()),
                             target_state_names.end());

    // mapping transition events to target states
    TransitionEventsMap event_states_map = {};
    for (int j = 0; j < transition_events.size(); j++)
    {
      std::string tr_event_name = transition_events[j].toStdString();
      if (event_states_map.count(tr_event_name) > 0)
      {
        std::vector<std::string>& st_names = event_states_map.at(tr_event_name);
        st_names.insert(st_names.end(), target_state_names.begin(), target_state_names.end());
        std::sort(st_names.begin(), st_names.end());
        st_names.erase(std::unique(st_names.begin(), st_names.end()), st_names.end());
      }
      else
      {
        event_states_map.insert(std::make_pair(tr_event_name, target_state_names));
      }
    }

    if (table.count(src_st) == 0)
    {
      table.insert(std::make_pair(src_st, event_states_map));
    }
    else
    {
      TransitionEventsMap& m = table[src_st];
      for (TransitionEventsMap::value_type& kv : event_states_map)
      {
        if (m.count(kv.first) == 0)
        {
          m.insert(kv);
        }
        else
        {
          // adding to list of target states
          std::vector<std::string>& target_st_names = m.at(kv.first);
          target_st_names.insert(target_st_names.end(), kv.second.begin(), kv.second.end());

          // removing duplicates
          std::sort(target_st_names.begin(), target_st_names.end());
          target_st_names.erase(std::unique(target_st_names.begin(), target_st_names.end()), target_st_names.end());
        }
      }
    }
  }

  return table;
}

ResponseFuture::ResponseFuture(const std::shared_future<Response>& res_fut,
                               bool is_detached,
                               const std::string& state_name,
                               bool is_atomic)
  : std::shared_future<Response>(res_fut), is_detached_(is_detached), is_atomic_(is_atomic), state_name_(state_name)
{
}

TransitionResult::TransitionResult(bool succeeded,
                                   const std::vector<ResponseFuture>& responses,
                                   const std::string err_msg)
  : succeeded_(succeeded), responses_(responses), err_msg_(err_msg)
{
}

const ResponseFuture& TransitionResult::getResponse() const
{
  if (responses_.empty())
  {
    throw(std::runtime_error("Response unavailable, no entry callback was found for any of the active states"));
  }
  return responses_.front();
}

bool TransitionResult::hasPendingResponse() const { return !responses_.empty() && !responses_.front().isDetached(); }

std::shared_future<Response> StateMachine::EntryCbHandler::operator()(const Action& arg)
{
  // create promise and future to be forwarded
  std::shared_ptr<std::promise<Response>> promise_res = std::make_shared<std::promise<Response>>();
  std::shared_future<Response> future_res(promise_res->get_future());

  if (discard_response_)
  {
    // run in qt thread and return unbinded Response future
    QtConcurrent::run(tpool_, [this, arg]() { return cb_(arg); });
    Response res = true;
    promise_res->set_value(res);
  }
  else
  {
    QtConcurrent::run(tpool_, [this, arg, promise_res]() {
      try
      {
        promise_res->set_value(cb_(arg));
      }
      catch (std::future_error& /*e*/)
      {
        // Promise in entry callback already satisfied, no action needed
      }
    });
  }
  return future_res;
}

StateMachine::StateMachine(double event_loop_period, log4cxx::LoggerPtr logger)
  : sm_info_(nullptr)
  , sm_(nullptr)
  , event_loop_period_(event_loop_period)
  , async_thread_pool_(new QThreadPool(this))
  , logger_(logger ? logger : DEFAULT_LOGGER)
{
}

StateMachine::StateMachine(QScxmlStateMachine* sm, double event_loop_period, log4cxx::LoggerPtr logger)
  : sm_info_(new QScxmlStateMachineInfo(sm))
  , sm_(sm)
  , sm_private_(QScxmlStateMachinePrivate::get(sm))
  , event_loop_period_(event_loop_period)
  , async_thread_pool_(new QThreadPool(this))
  , logger_(logger ? logger : DEFAULT_LOGGER)
{
  QVector<QScxmlStateMachineInfo::StateId> states = sm_info_->allStates();
  std::for_each(states.begin(), states.end(), [&](const int& id) {
    st_ids_map_.insert(std::make_pair(sm_info_->stateName(id).toStdString(), id));
  });
  signalSetup();
}

StateMachine::~StateMachine() {}

bool StateMachine::loadFile(const std::string& filename)
{
  sm_ = SM::fromFile(QString::fromStdString(filename));
  if (sm_ == nullptr)
  {
    return false;
  }

  const auto errors = sm_->parseErrors();
  if (!errors.isEmpty())
  {
    for (const QScxmlError& error : errors)
    {
      LOG4CXX_ERROR(logger_, "SCXML Parser " << filename << error.toString().toStdString());
    }
    return false;
  }

  sm_info_ = new QScxmlStateMachineInfo(sm_);
  sm_private_ = QScxmlStateMachinePrivate::get(sm_);
  QVector<QScxmlStateMachineInfo::StateId> states = sm_info_->allStates();
  std::for_each(states.begin(), states.end(), [&](const int& id) {
    st_ids_map_.insert(std::make_pair(sm_info_->stateName(id).toStdString(), id));
  });
  signalSetup();
  return true;
}

bool StateMachine::start()
{
  action_queue_.clear();

  if (!sm_)
  {
    return false;
  }

  // setting up timer
  execute_action_timer_ = new QTimer(this);
  connect(execute_action_timer_, &QTimer::timeout, [&]() { processQueuedActions(); });

  execute_action_timer_->start(static_cast<int>(1000 * event_loop_period_));
  busy_executing_action_ = false;
  busy_consuming_entry_cb_ = false;

  sm_->start();
  QCoreApplication::processEvents(QEventLoop::AllEvents, WAIT_QT_EVENTS);

  // make sure at least one state is active
  auto names = sm_->activeStateNames();
  if (names.count() < 1)
  {
    LOG4CXX_ERROR(logger_, " No active states were found, at least the initial state was expected");
    return false;
  }

  return true;
}

bool StateMachine::stop()
{
  if (!sm_)
  {
    return false;
  }
  execute_action_timer_->stop();
  sm_->stop();
  return true;
}

TransitionResult StateMachine::execute(const Action& action, bool force)
{
  if (force)
  {
    LOG4CXX_WARN(logger_, "Forcing action " << action.id);
    return executeAction(action);
  }

  if (busy_executing_action_ || busy_consuming_entry_cb_)
  {
    std::string err_msg = "SM is busy";
    LOG4CXX_DEBUG(logger_, err_msg);
    return TransitionResult(false, {}, err_msg);
  }
  return executeAction(action);
}

void StateMachine::postAction(Action action)
{
  std::lock_guard<std::mutex> lock(action_queue_mutex_);
  action_queue_.push_back(action);
  LOG4CXX_DEBUG(logger_, "Posted action " << action.id);
}

bool StateMachine::isBusy() const
{
  std::lock_guard<std::mutex> lock(action_queue_mutex_);
  return !action_queue_.empty() || busy_executing_action_ || busy_consuming_entry_cb_;
}

bool StateMachine::wait(double timeout) const
{
  if (timeout <= 0)
  {
    timeout = std::numeric_limits<double>::infinity();
  }

  QTime stop_time = QTime::currentTime().addMSecs(static_cast<int>(timeout * 1000));
  while (QTime::currentTime() < stop_time)
  {
    if (!isBusy())
    {
      return true;
    }

    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
  }
  return false;
}

void StateMachine::processQueuedActions()
{
  using namespace boost;

  if (busy_executing_action_ || busy_consuming_entry_cb_)
  {
    LOG4CXX_DEBUG(logger_, __func__ << " is busy");
    return;
  }

  if (emitStateEnteredSignal())
  {
    return;  // allow for listeners to handle signal
  }

  Action action;
  {
    std::lock_guard<std::mutex> lock(action_queue_mutex_);
    if (action_queue_.empty())
    {
      return;
    }

    action = action_queue_.front();
    action_queue_.pop_front();
  }
  TransitionResult res = executeAction(action);
  return;
}

TransitionResult StateMachine::executeAction(const Action& action)
{
  std::lock_guard<std::mutex> lock(consuming_action_mutex_);

  ScopeExit scope_exit(&this->busy_executing_action_);  // sets the flag to busy

  std::vector<int> transition_ids = getValidTransitionIDs();

  // filter transitions in order to keep those caused by the requested action
  transition_ids.erase(std::remove_if(transition_ids.begin(),
                                      transition_ids.end(),
                                      [&](const int& t_id) {
                                        QVector<QString> events = sm_info_->transitionEvents(t_id);
                                        return std::find(events.begin(),
                                                         events.end(),
                                                         QString::fromStdString(action.id)) == events.end();
                                      }),
                       transition_ids.end());

  // check if valid transitions remain
  if (transition_ids.empty())
  {
    std::string err_msg =
        boost::str(boost::format("Action '%s' is not valid for any of the active states") % action.id);
    // res.success = false;
    LOG4CXX_ERROR(logger_, err_msg);

    // res_promise.set_value(res);
    return TransitionResult(false, {}, err_msg);
  }

  int current_src_st_id = sm_info_->transitionSource(transition_ids.front());
  std::string current_src_st_name = sm_info_->stateName(current_src_st_id).toStdString();

  // finding target states
  std::vector<int> target_state_ids;
  std::for_each(transition_ids.begin(), transition_ids.end(), [&](const int& t_id) {
    QVector<QScxmlStateMachineInfo::StateId> st_ids = sm_info_->transitionTargets(t_id);
    for (const int& st_id : st_ids)
    {
      target_state_ids.push_back(st_id);
    }
  });

  if (target_state_ids.empty())
  {
    std::string err_msg = boost::str(boost::format("No valid target states were found for transition %1% -> %2%") %
                                     current_src_st_name % action.id);
    return TransitionResult(false, {}, err_msg);
  }

  // checking precondition
  Response precond_res;
  if (!std::all_of(target_state_ids.begin(), target_state_ids.end(), [&](const int& st_id) -> bool {
        std::string st = sm_info_->stateName(st_id).toStdString();
        if (precond_callbacks_.count(st) == 0)
        {
          return true;  // no precondition
        }
        for (auto cb : precond_callbacks_[st])
        {
          precond_res = cb(action);
          if (!precond_res)
          {
            // break on the first failed precondition
            precond_res.msg = boost::str(boost::format("Precondition for state %s failed: %s") % st % precond_res.msg);
            break;
          }
        }

        return TransitionResult(precond_res.success, {}, precond_res.msg);
      }))
  {
    // precondition failed, not proceeding with transition
    LOG4CXX_ERROR(logger_, precond_res.msg);
    return TransitionResult(false, {}, precond_res.msg);
  }

  // setting up synchronization variables*
  std::promise<Action> action_promise;
  action_future_ = std::shared_future<Action>(action_promise.get_future());
  action_promise.set_value(action);
  responses_map_promise_ = std::promise<ResponseFuturesMap>();
  std::shared_future<ResponseFuturesMap> responses_map_future =
      std::shared_future<ResponseFuturesMap>(responses_map_promise_.get_future());

  // submitting event, entry callbacks registered in signalSetup() should be invoked
  LOG4CXX_DEBUG(logger_, "Submitting event with id: " << action.id);
  sm_->submitEvent(QString::fromStdString(action.id));

  // wait until transition is complete
  QTime stop_time = QTime::currentTime().addMSecs(WAIT_TRANSITION_PERIOD);
  bool transition_made = false;
  QVector<QScxmlStateMachineInfo::StateId> current_st_ids;
  while (QTime::currentTime() < stop_time && !transition_made)
  {
    QCoreApplication::processEvents(QEventLoop::AllEvents, WAIT_QT_EVENTS);
    transition_made =
        responses_map_future.wait_for(std::chrono::milliseconds(WAIT_QT_EVENTS)) == std::future_status::ready;
  }

  // resetting synchronization variables
  responses_map_promise_ = std::promise<ResponseFuturesMap>();
  action_future_ = std::shared_future<Action>();
  if (!transition_made)
  {
    QVector<QScxmlStateMachineInfo::StateId> current_st_ids = sm_info_->configuration();
    sm_->cancelDelayedEvent(QString::fromStdString(action.id));
    std::string current_states_str =
        std::accumulate(std::next(current_st_ids.begin()),
                        current_st_ids.end(),
                        sm_info_->stateName(current_st_ids.front()).toStdString(),
                        [&](std::string r, const int& s) { return r + ", " + sm_info_->stateName(s).toStdString(); });

    std::string target_states_str =
        std::accumulate(std::next(target_state_ids.begin()),
                        target_state_ids.end(),
                        sm_info_->stateName(target_state_ids.front()).toStdString(),
                        [&](std::string r, const int& s) { return r + ", " + sm_info_->stateName(s).toStdString(); });

    std::string err_msg = "SM timed out before finishing transition to states [" + target_states_str + "]";
    LOG4CXX_ERROR(logger_, err_msg);
    LOG4CXX_ERROR(logger_, "Current states are: " << current_states_str.c_str());
    return TransitionResult(false, {}, err_msg);
    ;
  }

  // retrieve responses now
  ResponseFuturesMap futures_map = responses_map_future.get();
  std::vector<ResponseFuture> responses_vec;
  for (auto& kv : futures_map)
  {
    int state_id = kv.first;
    std::shared_future<Response>& future = kv.second;
    const std::string& st_name = sm_info_->stateName(state_id).toStdString();
    bool is_detached = entry_callbacks_.at(st_name)->discard_response_;
    if (sm_private_->m_stateTable->state(state_id).isAtomic())
    {
      responses_vec.insert(responses_vec.begin(), ResponseFuture(future, is_detached, st_name, true));
    }
    else
    {
      responses_vec.push_back(ResponseFuture(future, is_detached, st_name, false));
    }
    LOG4CXX_DEBUG(logger_, "Finished executing entry callback for state " << st_name);
  }

  if (futures_map.empty())
  {
    return TransitionResult(true, {});
  }

  LOG4CXX_DEBUG(logger_, "Retrieved response structure from future");
  return TransitionResult(true, responses_vec);
}

void StateMachine::signalSetup()
{
  connect(sm_info_, &SMInfo::statesEntered, [this](const QVector<QScxmlStateMachineInfo::StateId>& states) {
    LOG4CXX_DEBUG(logger_, "Entered  statesEntered callback");
    ScopeExit scope_exit(&this->busy_consuming_entry_cb_);  // sets the flag to busy

    std::lock_guard<std::mutex> lock(entered_states_mutex_);
    entered_states_queue_.clear();
    for (const QScxmlStateMachineInfo::StateId& id : states)
    {
      entered_states_queue_.push_back(id);  // will emit entered signals from the processing thread
    }

    // attempting to get the action if one has been binded to the action_future in the executeAction method
    Action action;
    if (action_future_.valid())
    {
      action = action_future_.get();
    }

    // now storing all futures returned by the state entry callbacks
    ResponseFuturesMap futures_map;
    for (int id : states)
    {
      std::shared_future<Response> temp_res_future;
      const std::string& st_name = sm_info_->stateName(id).toStdString();
      if (entry_callbacks_.count(st_name) > 0)
      {
        temp_res_future = (*entry_callbacks_.at(st_name))(action);
        futures_map.insert(std::make_pair(id, temp_res_future));
      }
    }

    responses_map_promise_.set_value(futures_map);
    LOG4CXX_DEBUG(logger_, "All entered states processed");
  });

  connect(sm_info_, &SMInfo::statesExited, [this](const QVector<QScxmlStateMachineInfo::StateId>& states) {
    // invoke exit callbacks
    std::for_each(states.begin(), states.end(), [this](const QScxmlStateMachineInfo::StateId& id) {
      std::string st_name = sm_info_->stateName(id).toStdString();
      if (exit_callbacks_.count(st_name) > 0)
      {
        LOG4CXX_DEBUG(logger_, "Started exit callback for state " << st_name);
        exit_callbacks_.at(st_name)();
        LOG4CXX_DEBUG(logger_, "Finished exit callback for state " << st_name);
      }
    });

    for (const QScxmlStateMachineInfo::StateId& id : states)
    {
      emit this->state_exited(getStateFullName(sm_info_, id));
    }
  });
}

bool StateMachine::addPreconditionCallback(const std::string& st_name, PreconditionCallback cb)
{
  if (!hasState(st_name))
  {
    LOG4CXX_ERROR(logger_, boost::str(boost::format("State %s was not found in SM") % st_name));
    return false;
  }

  auto pc = precond_callbacks_.find(st_name);
  if (pc == precond_callbacks_.end())
  {
    precond_callbacks_.insert(std::make_pair(st_name, std::vector<PreconditionCallback>({cb})));
  }
  else
  {
    pc->second.push_back(cb);
  }

  return true;
}

bool StateMachine::addEntryCallback(const std::string& st_name, EntryCallback cb, bool discard_response)
{
  if (!hasState(st_name))
  {
    LOG4CXX_ERROR(logger_, boost::str(boost::format("State %s was not found in SM") % st_name));
    return false;
  }

  if (entry_callbacks_.count(st_name) > 0)
  {
    LOG4CXX_WARN(logger_, boost::str(boost::format("Entry callback for state %s will be replaced") % st_name));
  }
  entry_callbacks_.insert(
      std::make_pair(st_name, std::make_shared<EntryCbHandler>(async_thread_pool_, cb, discard_response)));

  return true;
}

bool StateMachine::addExitCallback(const std::string& st_name, std::function<void()> cb)
{
  if (!hasState(st_name))
  {
    LOG4CXX_ERROR(logger_, boost::str(boost::format("State %s was not found in SM") % st_name));
    return false;
  }
  if (exit_callbacks_.count(st_name) > 0)
  {
    LOG4CXX_WARN(logger_, boost::str(boost::format("Exit callback for state %s will be replaced") % st_name));
  }
  exit_callbacks_.insert(std::make_pair(st_name, cb));
  return true;
}

std::vector<std::string> StateMachine::getAvailableActions() const
{
  std::vector<std::string> action_ids = {};
  std::vector<int> transition_ids = getValidTransitionIDs();

  for (int t_id : transition_ids)
  {
    QVector<QString> event_names = sm_info_->transitionEvents(t_id);
    std::transform(event_names.begin(), event_names.end(), std::back_inserter(action_ids), [](const QString& s) {
      return s.toStdString();
    });
  }
  return action_ids;
}

std::vector<std::string> StateMachine::getActions(const std::string& state_name) const
{
  std::vector<std::string> action_ids = {};
  if (st_ids_map_.count(state_name) == 0)
  {
    return {};
  }

  std::vector<int> transition_ids = getTransitionsIDs({ st_ids_map_.at(state_name) });
  for (int t_id : transition_ids)
  {
    QVector<QString> event_names = sm_info_->transitionEvents(t_id);
    std::transform(event_names.begin(), event_names.end(), std::back_inserter(action_ids), [](QString& s) {
      return s.toStdString();
    });
  }
  return action_ids;
}

std::string StateMachine::getCurrentState(bool full_name) const
{
  QVector<SMInfo::StateId> state_ids = sm_info_->configuration();
  SMInfo::StateId current_st_id = state_ids.front();
  for (SMInfo::StateId id : state_ids)
  {
    if (sm_private_->m_stateTable->state(id).isAtomic())
    {
      current_st_id = id;
      break;
    }
  }
  return full_name ? getStateFullName(sm_info_, current_st_id) : sm_info_->stateName(current_st_id).toStdString();
}

std::vector<std::string> StateMachine::getStates(bool full_name) const
{
  QVector<SMInfo::StateId> state_ids = sm_info_->allStates();

  // sort by parenthood
  std::sort(state_ids.begin(), state_ids.end(), [this](SMInfo::StateId s1, SMInfo::StateId s2) {
    return sm_info_->stateChildren(s1).size() < sm_info_->stateChildren(s2).size();
  });

  std::vector<std::string> st_names;
  std::transform(
      state_ids.begin(), state_ids.end(), std::back_inserter(st_names), [this, full_name](const SMInfo::StateId& id) {
        return full_name ? getStateFullName(sm_info_, id) : sm_info_->stateName(id).toStdString();
      });
  return st_names;
}

bool StateMachine::hasState(const std::string state_name)
{
  std::vector<std::string> st_names_vec = getStates(false);
  bool found = std::any_of(
      st_names_vec.begin(), st_names_vec.end(), [&state_name](const std::string& st) { return state_name == st; });

  return found;
}

std::vector<std::string> StateMachine::getCurrentStates(bool full_name) const
{
  std::vector<std::string> st_names;
  QVector<SMInfo::StateId> state_ids = sm_info_->configuration();

  // sort by parenthood
  std::sort(state_ids.begin(), state_ids.end(), [this](SMInfo::StateId s1, SMInfo::StateId s2) {
    return sm_info_->stateChildren(s1).size() < sm_info_->stateChildren(s2).size();
  });

  std::transform(
      state_ids.begin(), state_ids.end(), std::back_inserter(st_names), [this, full_name](const SMInfo::StateId& id) {
        return full_name ? getStateFullName(sm_info_, id) : sm_info_->stateName(id).toStdString();
      });
  return st_names;
}

bool StateMachine::hasAction(const std::string& state_name, const Action& action)
{
  std::vector<std::string> actions = getActions(state_name);
  return std::any_of(actions.begin(), actions.end(), [&action](const std::string& id) { return action.id == id; });
}

bool StateMachine::isRunning() const
{
  if (!sm_)
  {
    LOG4CXX_ERROR(logger_, "QScxmlStateMachine has not been properly initialized");
    return false;
  }
  return sm_->isRunning();
}

bool StateMachine::emitStateEnteredSignal()
{
  bool emitted = false;
  std::lock_guard<std::mutex> lock(entered_states_mutex_);
  while (!entered_states_queue_.empty())
  {
    emitted = true;
    QScxmlStateMachineInfo::StateId id = entered_states_queue_.front();
    entered_states_queue_.pop_front();
    const std::string state_name = getStateFullName(sm_info_, id);
    LOG4CXX_DEBUG(logger_, boost::str(boost::format("Emitting state_entered signal for state %s") % state_name));
    QTimer::singleShot(10, [&, state_name]() { emit this->state_entered(state_name); });
  }
  return emitted;
}

std::vector<int> StateMachine::getTransitionsIDs(const QVector<int>& states) const
{
  using StateTable = QScxmlExecutableContent::StateTable;
  const StateTable* st_table = sm_private_->m_stateTable;

  std::vector<int> transition_ids;
  for (const int& id : states)
  {
    StateTable::Array st_transitions = st_table->array(st_table->state(id).transitions);
    if (!st_transitions.isValid())
    {
      continue;
    }

    transition_ids.insert(transition_ids.end(), st_transitions.begin(), st_transitions.end());
  }

  return transition_ids;
}

std::vector<int> StateMachine::getValidTransitionIDs() const
{
  using StateTable = QScxmlExecutableContent::StateTable;
  std::vector<std::string> action_names = {};

  const StateTable* st_table = sm_private_->m_stateTable;
  QScxmlDataModel* data_model = sm_private_->m_dataModel;
  const std::vector<int> transition_ids = getTransitionsIDs(sm_info_->configuration());
  std::vector<int> valid_transition_ids;
  if (transition_ids.empty())
  {
    return {};
  }

  for (const int& t_id : transition_ids)
  {
    const StateTable::Transition& t = st_table->transition(t_id);
    bool cond_valid = t.condition == -1;
    if (t.condition != -1)
    {
      cond_valid = data_model->evaluateToBool(t.condition, &cond_valid) && cond_valid;
    }

    if (!cond_valid)
    {
      continue;
    }

    valid_transition_ids.push_back(t_id);
  }
  return valid_transition_ids;
}

} /* namespace scxml_core */
