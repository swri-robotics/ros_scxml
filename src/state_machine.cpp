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

static const int WAIT_TRANSITION_PERIOD = 500; // ms
static const int WAIT_QT_EVENTS = WAIT_TRANSITION_PERIOD/10.0; // ms

namespace ros_scxml
{

using SM = QScxmlStateMachine;
using SMInfo = QScxmlStateMachineInfo;
using TransitionEventsMap = std::map<std::string,std::vector<std::string>>;

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

StateMachine::TransitionTable buildTransitionTable(const QScxmlStateMachineInfo* sm_info)
{
  StateMachine::TransitionTable table;
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
  async_thread_pool_(new QThreadPool(this)),
  ttable_(buildTransitionTable(sm_info_))
{
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

  ttable_ = buildTransitionTable(sm_info_);
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
  std::lock_guard<std::mutex> lock(action_queue_mutex_);

  if(is_busy_)
  {
    ROS_DEBUG_STREAM_NAMED("SM",__func__<< " is busy");
    return;
  }

  if(action_queue_.empty())
  {
    return;
  }

  auto action  = action_queue_.front();
  action_queue_.pop_front();
  Response res = executeAction(action);

  return;
}

Response StateMachine::executeAction(const Action& action)
{
  std::lock_guard<std::mutex> lock(execution_action_mutex_);
  ScopeExit scope_exit(&this->is_busy_); // sets the flag to busy

  Response res;
  std::vector<std::string> current_states = getCurrentStates();
  std::string current_src_state;
  if(!std::any_of(current_states.begin(), current_states.end(),[&](const std::string& st){
    if(hasAction(st,action) && validateTransition(action))
    {
      current_src_state = st;
      return true;
    }
    return false;
  }))
  {
    res.msg = boost::str(boost::format("Action '%s' is not valid for any of the active states") % action.id);
    res.success = false;
    ROS_ERROR_STREAM(res.msg);
    return std::move(res);
  }

  // finding target state
  std::vector<std::string> target_states = ttable_[current_src_state].at(action.id);
  if(target_states.empty())
  {
    res.msg = boost::str(boost::format("No valid target states were found for transition %1% -> %2%") %
                         current_src_state % action.id);
    res.success = false;
    return std::move(res);
  }

  // checking precondition
  if(!std::all_of(target_states.begin(),target_states.end(),[&](const std::string& st) -> bool{
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

  // submitting event
  sm_->submitEvent(QString::fromStdString(action.id));

  // wait until QT makes the transition to the target state
  QTime stop_time = QTime::currentTime().addMSecs(WAIT_TRANSITION_PERIOD);
  bool transition_made = false;
  std::vector<std::string> new_current_states;
  while(QTime::currentTime() < stop_time)
  {
    QCoreApplication::processEvents(QEventLoop::AllEvents, WAIT_QT_EVENTS);
    new_current_states = getCurrentStates();
    if(std::any_of(new_current_states.begin(),new_current_states.end(),[&target_states](const std::string& st){
      return std::find(target_states.begin(),target_states.end(),st) != target_states.end();
    }))
    {
      transition_made = true;
      break;
    }
  }

  if(!transition_made)
  {
    sm_->cancelDelayedEvent(QString::fromStdString(action.id));
    std::string current_states_str = std::accumulate(std::next(current_states.begin()),current_states.end(),
                                             current_states.front(),[](std::string r, const std::string& s){
      return r + ", " + s;
    });

    std::string target_states_str = std::accumulate(std::next(target_states.begin()),target_states.end(),
                                                    target_states.front(),[](std::string r, const std::string& s){
      return r + ", " + s;
    });
    res.msg = "SM timed out before finishing transition to states [" + target_states_str + "]";
    res.success = false;
    ROS_ERROR_STREAM(res.msg);
    ROS_ERROR("Current states are: %s",current_states_str.c_str());
    return std::move(res);
  }

  // calling entry callback
  res.success = true;
  std::vector<Response> responses;
  std::reverse(std::begin(new_current_states),std::end(new_current_states));
  for(const std::string& st : new_current_states)
  {
    Response new_res;
    if(entry_callbacks_.count(st) > 0)
    {
      new_res = (*entry_callbacks_.at(st))(action);
    }
    else
    {
      new_res.success = true;
    }

    if(!new_res)
    {
      ROS_ERROR_STREAM(res.msg);
    }

    responses.push_back(std::move(new_res));
  }

  // choosing the first bad result if there is one
  res = std::accumulate(responses.begin(),responses.end(),responses.front(),[](Response result, const Response& r){
    return (!result) ? result : r;
  });

  return res;
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
  QVector<SMInfo::StateId> state_ids = sm_info_->configuration();
  std::vector<std::string> action_ids = {};
  for(SMInfo::StateId id : state_ids)
  {
    std::vector<std::string> st_actions = getActions(sm_info_->stateName(id).toStdString());
    action_ids.insert(action_ids.end(),st_actions.begin(),st_actions.end());
  }

  action_ids.erase(std::remove_if(action_ids.begin(),action_ids.end(),[this](const std::string& id){
    return !validateTransition(Action{.id = id});
  }),action_ids.end());
  return std::move(action_ids);
}

std::vector<std::string> StateMachine::getActions(const std::string& state_name) const
{

  std::vector<std::string> action_ids = {};
  if(ttable_.count(state_name) == 0)
  {
    return {};
  }

  const TransitionEventsMap& transitions_map = ttable_.at(state_name);
  std::transform(transitions_map.begin(),transitions_map.end(),std::back_inserter(action_ids),
                 [](const TransitionEventsMap::value_type& kv){
    return kv.first;
  });
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

void StateMachine::signalSetup()
{
  connect(sm_info_,&SMInfo::statesEntered,[this](const QVector<QScxmlStateMachineInfo::StateId>& states){
    for(const QScxmlStateMachineInfo::StateId& id : states)
    {
      emit this->state_entered(getStateFullName(sm_info_,id));
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

bool StateMachine::validateTransition(const Action& action) const
{
  using StateTable = QScxmlExecutableContent::StateTable;
  QVector<SMInfo::StateId> states = sm_info_->configuration();
  const StateTable* st_table = sm_private_->m_stateTable;
  QScxmlDataModel* data_model = sm_private_->m_dataModel;
  bool valid = false;
  for(SMInfo::StateId& id: states)
  {
    const StateTable::Array transitions = st_table->array(st_table->state(id).transitions);
    if (!transitions.isValid())
    {
        continue;
    }
    std::vector<int> sorted_transitions(transitions.size(), -1);
    std::copy(transitions.begin(), transitions.end(), sorted_transitions.begin());
    int selected_tr_id = StateTable::InvalidIndex;
    for (int tr_id : sorted_transitions)
    {
      QVector<QString> event_names = sm_info_->transitionEvents(tr_id);
      if(std::any_of(event_names.begin(),event_names.end(),[&action](QString& n){
        return n.toStdString() == action.id;
      }))
      {
        selected_tr_id = tr_id;
        break;
      }
    }

    if(selected_tr_id == -1)
    {
      continue;
    }

    const StateTable::Transition &t = st_table->transition(selected_tr_id);
    if (t.condition == -1)
    {
      valid = true; // no condition assigned
    }
    else
    {
      bool ok = false;
      valid = data_model->evaluateToBool(t.condition, &ok) && ok;
    }
    break;
  }

  return valid;
}

} /* namespace ros_scxml */

