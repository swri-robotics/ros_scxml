/*
 * state_machine.cpp
 *
 *  Created on: Apr 10, 2019
 *      Author: jrgnicho
 */

#include "ros_scxml/state_machine.h"
#include <QCoreApplication>
#include <QTime>
#include <QEventLoop>
#include <boost/format.hpp>
#include <ros/console.h>

static const int WAIT_TRANSITION_PERIOD = 1000; // ms
static const int WAIT_QT_EVENTS = WAIT_TRANSITION_PERIOD/40.0; // ms

namespace ros_scxml
{

using SM = QScxmlStateMachine;
using SMInfo = QScxmlStateMachineInfo;
using TransitionEventsMap = std::map<std::string,std::vector<std::string>>;
using TransitionTable = std::map<std::string,std::map<std::string,std::vector<std::string>>>;

class ScopeExit
{
public:
  ScopeExit(std::atomic<bool>* b):
    b_(b)
  {
    *b_ = true;
  }

  ~ScopeExit()
  {
    *(b_) = false;
  }

  std::atomic<bool>* b_;
};

std::string getStateFullName(const QScxmlStateMachineInfo* sm_info, const QScxmlStateMachineInfo::StateId id)
{
  using StateId = QScxmlStateMachineInfo::StateId;

  StateId parent_id = sm_info->stateParent(id);
  std::string full_name = sm_info->stateName(id).toStdString();
  while(parent_id != QScxmlStateMachineInfo::InvalidStateId)
  {
    std::string parent_name = sm_info->stateName(parent_id).toStdString();
    if(parent_name.empty())
    {
      break;
    }
    full_name = parent_name + "::" + full_name;
    parent_id = sm_info->stateParent(parent_id);
  }
  return std::move(full_name);
}

TransitionTable buildTransitionTable(const QScxmlStateMachineInfo* sm_info)
{
  TransitionTable table;
  QVector<QScxmlStateMachineInfo::TransitionId> transitions = sm_info->allTransitions();
  for(std::size_t i = 0; i < transitions.size(); i++)
  {
    auto transition_id = transitions[i];

    if(sm_info->transitionType(transition_id) == SMInfo::TransitionType::InvalidTransition)
    {
      continue;
    }

    QVector<QString> transition_events = sm_info->transitionEvents(transition_id);
    if(transition_events.empty())
    {
      continue;
    }

    // source states
    std::string src_st = sm_info->stateName(sm_info->transitionSource(transition_id)).toStdString();

    // getting target states
    std::vector<std::string> target_state_names = {};
    QVector<QScxmlStateMachineInfo::StateId> target_states = sm_info->transitionTargets(transition_id);
    std::transform(target_states.begin(),target_states.end(),std::back_inserter(target_state_names),
                   [&sm_info](const QScxmlStateMachineInfo::StateId& s){
      return sm_info->stateName(s).toStdString();
    });
    if(target_states.empty())
    {
      continue;
    }

    // remove duplicates if any
    std::sort(target_state_names.begin(),target_state_names.end());
    target_state_names.erase(std::unique(target_state_names.begin(),target_state_names.end()),
                             target_state_names.end());

    // mapping transition events to target states
    TransitionEventsMap event_states_map = {};
    for(std::size_t j = 0; j < transition_events.size(); j++)
    {
      std::string tr_event_name = transition_events[j].toStdString();
      if(event_states_map.count(tr_event_name) > 0)
      {
        std::vector<std::string>& st_names = event_states_map.at(tr_event_name);
        st_names.insert(st_names.end(),target_state_names.begin(),target_state_names.end());
        std::sort(st_names.begin(),st_names.end());
        st_names.erase(std::unique(st_names.begin(),st_names.end()),st_names.end());
      }
      else
      {
        event_states_map.insert(std::make_pair(tr_event_name,target_state_names));
      }
    }

    if(table.count(src_st) == 0)
    {
      table.insert(std::make_pair(src_st,event_states_map));
    }
    else
    {
      TransitionEventsMap& m = table[src_st];
      for(TransitionEventsMap::value_type& kv : event_states_map)
      {
        if(m.count(kv.first) ==0)
        {
          m.insert(kv);
        }
        else
        {
          // adding to list of target states
          std::vector<std::string>& target_st_names = m.at(kv.first);
          target_st_names.insert(target_st_names.end(),kv.second.begin(),kv.second.end());

          // removing duplicates
          std::sort(target_st_names.begin(),target_st_names.end());
          target_st_names.erase(std::unique(target_st_names.begin(),target_st_names.end()),target_st_names.end());
        }
      }
    }
  }

  return std::move(table);
}

StateMachine::StateMachine(double event_loop_period):
  sm_(nullptr),
  sm_info_(nullptr),
  event_loop_period_(event_loop_period),
  async_thread_pool_(new QThreadPool(this))
{

}

StateMachine::StateMachine(QScxmlStateMachine* sm, double event_loop_period):
  sm_(sm),
  sm_private_(QScxmlStateMachinePrivate::get(sm)),
  sm_info_(new QScxmlStateMachineInfo(sm)),
  event_loop_period_(event_loop_period),
  async_thread_pool_(new QThreadPool(this))
{
  QVector<QScxmlStateMachineInfo::StateId> states = sm_info_->allStates();
  std::for_each(states.begin(),states.end(),[&](const int& id){
    st_ids_map_.insert(std::make_pair(sm_info_->stateName(id).toStdString(),id));
  });
  signalSetup();
}

StateMachine::~StateMachine()
{

}

bool StateMachine::loadFile(const std::string& filename)
{
  sm_ = SM::fromFile(QString::fromStdString(filename));
  if(sm_ == nullptr)
  {
    return false;
  }

  const auto errors = sm_->parseErrors();
  if (!errors.isEmpty())
  {
    for (const QScxmlError &error : errors)
    {

      ROS_ERROR("SCXML Parser [%s]: [%s]",
                filename.c_str(),
                error.toString().toStdString().c_str());
    }
    return false;
  }


  sm_info_ = new QScxmlStateMachineInfo(sm_);
  sm_private_ = QScxmlStateMachinePrivate::get(sm_);
  QVector<QScxmlStateMachineInfo::StateId> states = sm_info_->allStates();
  std::for_each(states.begin(),states.end(),[&](const int& id){
    st_ids_map_.insert(std::make_pair(sm_info_->stateName(id).toStdString(),id));
  });
  signalSetup();
  return true;
}

bool StateMachine::start()
{
  action_queue_.clear();

  if(!sm_ )
  {
    return false;
  }

  // setting up timer
  execute_action_timer_ = new QTimer(this);
  connect(execute_action_timer_,&QTimer::timeout,[&](){
    processQueuedActions();
  });

  execute_action_timer_->start(1000*event_loop_period_);
  is_busy_= false;
  sm_->start();
  return true;
}

bool StateMachine::stop()
{
  if(!sm_ )
  {
    return false;
  }
  execute_action_timer_->stop();
  sm_->stop();
  return true;
}

Response StateMachine::execute(const Action& action)
{
  return executeAction(action);
}


void StateMachine::postAction(Action action)
{
  std::lock_guard<std::mutex> lock(action_queue_mutex_);
  action_queue_.push_back(std::move(action));
  ROS_DEBUG("Posted action %s",action.id.c_str());
}

bool StateMachine::isBusy() const
{
  std::lock_guard<std::mutex> lock(action_queue_mutex_);
  return !action_queue_.empty() || is_busy_;
}

bool StateMachine::wait(double timeout) const
{
  if(timeout <= 0)
  {
    timeout = std::numeric_limits<double>::infinity();
  }

  QTime stop_time = QTime::currentTime().addMSecs(timeout * 1000);
  while(QTime::currentTime() < stop_time)
  {
    if(!isBusy())
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

  if(is_busy_)
  {
    ROS_DEBUG_STREAM_NAMED("SM",__func__<< " is busy");
    return;
  }

  if(emitStateEnteredSignal())
  {
    return; // allow for listeners to handle signal
  }

  Action action;
  {
    std::lock_guard<std::mutex> lock(action_queue_mutex_);
    if(action_queue_.empty())
    {
      return;
    }

    action  = action_queue_.front();
    action_queue_.pop_front();
  }
  Response res = executeAction(action);

  return;
}

void StateMachine::saveStateHistory()
{
  QVector<int> current_st_ids = sm_info_->configuration();
  const QScxmlExecutableContent::StateTable *state_table = sm_private_->m_stateTable;
  for(const int& id: current_st_ids)
  {
    const QScxmlExecutableContent::StateTable::Array children_st_ids = state_table->array(
        state_table->state(id).childStates);

    if(!children_st_ids.isValid())
    {
      continue;
    }

    // getting history states
    for (int k : children_st_ids)
    {
      std::vector<int> history_states;
      if (state_table->state(k).isHistoryState())
      {

        for(const int& s_id : current_st_ids)
        {
          if(!state_table->state(s_id).isAtomic())
          {
            continue;
          }

          if(sm_info_->stateParent(s_id) != id)
          {
            continue;
          }

          ROS_INFO_STREAM("Saved History state "<<sm_info_->stateName(s_id).toStdString());
          history_states.push_back(s_id);
        }

      }

      if(history_states.empty())
      {
        continue;
      }

      history_buffer_[k] = history_states;
    }
  }
}

void StateMachine::clearStateHistory()
{
  QVector<int> current_st_ids = sm_info_->configuration();
  const QScxmlExecutableContent::StateTable *state_table = sm_private_->m_stateTable;
  for(const int& id: current_st_ids)
  {
    const QScxmlExecutableContent::StateTable::Array children_st_ids = state_table->array(
        state_table->state(id).childStates);

    if(!children_st_ids.isValid())
    {
      continue;
    }

    // getting history states
    for (int k : children_st_ids)
    {
      if (state_table->state(k).isHistoryState() &&
          state_table->state(k).type == QScxmlExecutableContent::StateTable::State::DeepHistory &&
          history_buffer_.count(k) > 0)
      {
        history_buffer_.erase(k);
      }
    }
  }
}

Response StateMachine::executeAction(const Action& action)
{
  std::lock_guard<std::mutex> lock(execution_action_mutex_);
  ScopeExit scope_exit(&this->is_busy_); // sets the flag to busy

  Response res;

  // get transitions and check their events
  std::vector<int> transition_ids = getValidTransitionIDs();

  // filter transitions
  transition_ids.erase(std::remove_if(transition_ids.begin(),transition_ids.end(),[&](const int& t_id){
    QVector<QString> events = sm_info_->transitionEvents(t_id);
    return !std::any_of(events.begin(),events.end(),[&action](QString& s){
      return s.toStdString() == action.id;
    });
  }),transition_ids.end());

  // check if valid transitions remain
  if(transition_ids.empty())
  {
    res.msg = boost::str(boost::format("Action '%s' is not valid for any of the active states") % action.id);
    res.success = false;
    ROS_ERROR_STREAM(res.msg);
    return std::move(res);
  }

  int current_src_st_id = sm_info_->transitionSource(transition_ids.front());
  std::string current_src_st_name = sm_info_->stateName(current_src_st_id).toStdString();

  // finding target states
  std::vector<int> target_state_ids;
  std::for_each(transition_ids.begin(),transition_ids.end(),[&](const int& t_id){
    QVector<QScxmlStateMachineInfo::StateId> st_ids = sm_info_->transitionTargets(t_id);
    for(const int& st_id: st_ids)
    {
      if(sm_private_->m_stateTable->state(st_id).isHistoryState())
      {
        // check history buffer
        if(history_buffer_.count(st_id)>0)
        {
          target_state_ids.insert(target_state_ids.end(),
                                  history_buffer_.at(st_id).begin(), history_buffer_.at(st_id).end());
        }
      }
      else
      {
        target_state_ids.push_back(st_id);
      }
    }
  });


  if(target_state_ids.empty())
  {
    res.msg = boost::str(boost::format("No valid target states were found for transition %1% -> %2%") %
                         current_src_st_name % action.id);
    res.success = false;
    return std::move(res);
  }

  // checking precondition
  if(!std::all_of(target_state_ids.begin(),target_state_ids.end(),[&](const int& st_id) -> bool{
    std::string st = sm_info_->stateName(st_id).toStdString();
    if(precond_callbacks_.count(st) == 0)
    {
      return true; // no precondition
    }
    res = precond_callbacks_[st](action);
    if(!res)
    {
      res.msg = boost::str(boost::format("Precondition for state %s failed: %s") % st % res.msg);
    }
    return res;
  }))
  {
    ROS_ERROR_STREAM(res.msg);
    return std::move(res);  // precondition failed, not proceeding with transition
  }

  // clear history before transitioning
  clearStateHistory();

  // submitting event
  sm_->submitEvent(QString::fromStdString(action.id));

  // wait until QT makes the transition to the target state
  QTime stop_time = QTime::currentTime().addMSecs(WAIT_TRANSITION_PERIOD);
  bool transition_made = false;
  QVector<QScxmlStateMachineInfo::StateId> current_st_ids;
  while(QTime::currentTime() < stop_time)
  {
    QCoreApplication::processEvents(QEventLoop::AllEvents, WAIT_QT_EVENTS);
    current_st_ids = sm_info_->configuration();
    if(std::any_of(current_st_ids.begin(),current_st_ids.end(),[&target_state_ids](const int& st_id){
      return std::find(target_state_ids.begin(),target_state_ids.end(),st_id) != target_state_ids.end();
    }))
    {
      transition_made = true;
      break;
    }
  }

  if(!transition_made)
  {
    sm_->cancelDelayedEvent(QString::fromStdString(action.id));
    std::string current_states_str = std::accumulate(std::next(current_st_ids.begin()),current_st_ids.end(),
                                                     sm_info_->stateName(current_st_ids.front()).toStdString(),
                                                                         [&](std::string r, const int& s){
      return r + ", " + sm_info_->stateName(s).toStdString();
    });

    std::string target_states_str = std::accumulate(std::next(target_state_ids.begin()),target_state_ids.end(),
                                                    sm_info_->stateName(target_state_ids.front()).toStdString(),
                                                    [&](std::string r, const int& s){
      return r + ", " + sm_info_->stateName(s).toStdString();
    });

    res.msg = "SM timed out before finishing transition to states [" + target_states_str + "]";
    res.success = false;
    ROS_ERROR_STREAM(res.msg);
    ROS_ERROR("Current states are: %s",current_states_str.c_str());
    return std::move(res);
  }

  // save history
  saveStateHistory();

  // calling entry callbacks
  for(int id: current_st_ids)
  {
    Response st_res;
    const std::string& st_name = sm_info_->stateName(id).toStdString();
    if(entry_callbacks_.count(st_name) > 0)
    {
      st_res = (*entry_callbacks_.at(st_name))(action);
    }
    else
    {
      st_res.success = true;
    }

    if(sm_private_->m_stateTable->state(id).isAtomic())
    {
      res = std::move(st_res);
    }
  }

  return std::move(res);
}

bool StateMachine::addPreconditionCallback(const std::string& st_name, PreconditionCallback cb)
{
  if(!hasState(st_name))
  {
    ROS_ERROR("State %s was not found in SM",st_name.c_str());
    return false;
  }

  if(precond_callbacks_.count(st_name) > 0)
  {
    ROS_WARN("Pre-condition callback for state %s will be replaced", st_name.c_str());
  }
  precond_callbacks_.insert(std::make_pair(st_name,cb));

  return true;
}

bool StateMachine::addEntryCallback(const std::string& st_name, EntryCallback cb, bool async_execution)
{
  if(!hasState(st_name))
  {
    ROS_ERROR("State %s was not found in SM",st_name.c_str());
    return false;
  }

  if(entry_callbacks_.count(st_name) > 0)
  {
    ROS_WARN("Entry callback for state %s will be replaced", st_name.c_str());
  }
  entry_callbacks_.insert(std::make_pair(st_name,std::make_shared<EntryCbHandler>(async_thread_pool_,cb,async_execution)));

  return true;
}

bool StateMachine::addExitCallback(const std::string& st_name, std::function<void()> cb)
{
  if(!hasState(st_name))
  {
    ROS_ERROR("State %s was not found in SM",st_name.c_str());
    return false;
  }
  if(exit_callbacks_.count(st_name) > 0)
  {
    ROS_WARN("Exit callback for state %s will be replaced", st_name.c_str());
  }
  exit_callbacks_.insert(std::make_pair(st_name,cb));
  return true;
}

std::vector<std::string> StateMachine::getAvailableActions() const
{
  std::vector<std::string> action_ids = {};
  std::vector<int> transition_ids = getValidTransitionIDs();

  for(int t_id: transition_ids)
  {
    QVector<QString> event_names = sm_info_->transitionEvents(t_id);
    std::transform(event_names.begin(),event_names.end(),std::back_inserter(action_ids),[](const QString& s){
      return s.toStdString();
    });
  }
  return std::move(action_ids);
}

std::vector<std::string> StateMachine::getActions(const std::string& state_name) const
{
  std::vector<std::string> action_ids = {};
  if(st_ids_map_.count(state_name)==0)
  {
    return {};
  }

  std::vector<int> transition_ids = getTransitionsIDs({st_ids_map_.at(state_name)});
  for(int t_id : transition_ids)
  {
    QVector<QString> event_names = sm_info_->transitionEvents(t_id);
    std::transform(event_names.begin(),event_names.end(),std::back_inserter(action_ids),[](QString& s){
      return s.toStdString();
    });
  }
  return std::move(action_ids);
}

std::string StateMachine::getCurrentState(bool full_name) const
{
  QVector<SMInfo::StateId> state_ids = sm_info_->configuration();
  SMInfo::StateId current_st_id = state_ids.front();
  for(SMInfo::StateId id : state_ids)
  {
    QVector<SMInfo::StateId> children = sm_info_->stateChildren(id);
    if(children.empty())
    {
      current_st_id = id;
      break;
    }
  }
  return full_name ? getStateFullName(sm_info_,current_st_id) :
      std::move(sm_info_->stateName(current_st_id).toStdString());
}

std::vector<std::string> StateMachine::getStates(bool full_name) const
{
  QVector<SMInfo::StateId> state_ids = sm_info_->allStates();

  // sort by parenthood
  std::sort(state_ids.begin(),state_ids.end(),[this](SMInfo::StateId s1, SMInfo::StateId s2){
    return sm_info_->stateChildren(s1).size() < sm_info_->stateChildren(s2).size();
  });

  std::vector<std::string> st_names;
  std::transform(state_ids.begin(),state_ids.end(),std::back_inserter(st_names),
                 [this,full_name](const SMInfo::StateId& id){
    return full_name ? getStateFullName(sm_info_,id) : sm_info_->stateName(id).toStdString();
  });
  return std::move(st_names);
}

bool StateMachine::hasState(const std::string state_name)
{
  QStringList st_list = sm_->stateNames();
  return std::any_of(st_list.begin(),st_list.end(),[&state_name](QString& st){
    return state_name == st.toStdString();
  });
}

std::vector<std::string> StateMachine::getCurrentStates(bool full_name) const
{
  std::vector<std::string> st_names;
  QVector<SMInfo::StateId> state_ids = sm_info_->configuration();

  // sort by parenthood
  std::sort(state_ids.begin(),state_ids.end(),[this](SMInfo::StateId s1, SMInfo::StateId s2){
    return sm_info_->stateChildren(s1).size() < sm_info_->stateChildren(s2).size();
  });

  std::transform(state_ids.begin(),state_ids.end(),std::back_inserter(st_names),[this,full_name](const SMInfo::StateId& id){
    return full_name ? getStateFullName(sm_info_,id) :
        sm_info_->stateName(id).toStdString();
  });
  return std::move(st_names);
}

bool StateMachine::hasAction(const std::string& state_name, const Action& action)
{
  std::vector<std::string> actions = getActions(state_name);
  return std::any_of(actions.begin(),actions.end(),[&action](const std::string& id){
    return action.id == id;
  });
}

bool StateMachine::isRunning() const
{
  if(!sm_)
  {
    ROS_ERROR("QScxmlStateMachine has not been properly initialized");
    return false;
  }
  return sm_->isRunning();
}

bool StateMachine::emitStateEnteredSignal()
{
  bool emitted = false;
  std::lock_guard<std::mutex> lock(entered_states_mutex_);
  while(!entered_states_queue_.empty())
  {
    emitted = true;
    QScxmlStateMachineInfo::StateId id = entered_states_queue_.front();
    entered_states_queue_.pop_front();
    const std::string state_name = getStateFullName(sm_info_,id);
    ROS_DEBUG("Emitting state_entered signal for state %s",state_name.c_str());
    QTimer::singleShot(10,[&,state_name](){
      emit this->state_entered(state_name);
    });
  }
  return emitted;
}

void StateMachine::signalSetup()
{
  connect(sm_info_,&SMInfo::statesEntered,[this](const QVector<QScxmlStateMachineInfo::StateId>& states){
    std::lock_guard<std::mutex> lock(entered_states_mutex_);
    entered_states_queue_.clear();
    for(const QScxmlStateMachineInfo::StateId& id : states)
    {
      entered_states_queue_.push_back(id); // will emit entered signals from the processing thread
    }

  });

  connect(sm_info_,&SMInfo::statesExited,[this](const QVector<QScxmlStateMachineInfo::StateId>& states){

    // invoke exit callbacks
    std::for_each(states.begin(),states.end(),[this](const QScxmlStateMachineInfo::StateId& id){
      std::string st_name = sm_info_->stateName(id).toStdString();
      if(exit_callbacks_.count(st_name) > 0)
      {
        exit_callbacks_.at(st_name)();
      }
    });

    for(const QScxmlStateMachineInfo::StateId& id : states)
    {
      emit this->state_exited(getStateFullName(sm_info_,id));
    }
  });
}

std::vector<int> StateMachine::getTransitionsIDs(const QVector<int>& states) const
{
  using StateTable = QScxmlExecutableContent::StateTable;
  const StateTable* st_table = sm_private_->m_stateTable;
  QScxmlDataModel* data_model = sm_private_->m_dataModel;

  std::vector<int> transition_ids;
  for(const int& id: states)
  {
    StateTable::Array st_transitions = st_table->array(st_table->state(id).transitions);
    if (!st_transitions.isValid())
    {
        continue;
    }

    transition_ids.insert(transition_ids.end(),st_transitions.begin(),st_transitions.end());
  }

  return std::move(transition_ids);
}

std::vector<int> StateMachine::getValidTransitionIDs() const
{
  using StateTable = QScxmlExecutableContent::StateTable;
  std::vector<std::string> action_names = {};

  const StateTable* st_table = sm_private_->m_stateTable;
  QScxmlDataModel* data_model = sm_private_->m_dataModel;
  const std::vector<int> transition_ids = getTransitionsIDs(sm_info_->configuration());
  std::vector<int> valid_transition_ids;
  if(transition_ids.empty())
  {
    return {};
  }

  for(const int& t_id : transition_ids)
  {
    const StateTable::Transition &t = st_table->transition(t_id);
    bool cond_valid = t.condition == -1;
    if(t.condition != -1 )
    {
      cond_valid = data_model->evaluateToBool(t.condition, &cond_valid) && cond_valid;
    }

    if(!cond_valid)
    {
      continue;
    }

    valid_transition_ids.push_back(t_id);
  }
  return std::move(valid_transition_ids);
}

} /* namespace ros_scxml */

