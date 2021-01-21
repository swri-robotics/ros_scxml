#include <scxml_core/scxml_sm_interface.h>
#include <sstream>
#include <tinyxml2.h>

static const char* SCXML_ELEMENT = "scxml";
static const char* STATE_ELEMENT = "state";
static const char* STATE_ID_ATTRIBUTE = "id";
static const char* HISTORY_STATE_ELEMENT = "history";
static const char* HISTORY_STATE_ID_ATTRIBUTE = "id";
static const char* TRANSITION_ELEMENT = "transition";
static const char* EVENT_ATTRIBUTE = "event";

/**
 * @brief Recursively adds states and transitions to the map
 * @param state
 * @param map
 */
static void getStateTransitionsRecursive(tinyxml2::XMLElement* state,
                                         scxml_core::StateTransitionMap& map,
                                         QSet<QString> inherited_events)
{
  using namespace tinyxml2;

  // Get the ID of this state element
  QString state_id;
  {
    const char* id = state->Attribute(STATE_ID_ATTRIBUTE);
    if (!id)
      throw std::runtime_error("'" + std::string(STATE_ELEMENT) + "' element does not have '" +
                               std::string(STATE_ID_ATTRIBUTE) + "' attribute");

    state_id = QString(id);
  }

  // Add this state to the map with its inherited events
  map[state_id] = inherited_events;

  // Add all the transitions for this state
  XMLElement* transition = state->FirstChildElement(TRANSITION_ELEMENT);
  while (transition)
  {
    // Get the name of the event associated with this transition
    const char* event = transition->Attribute(EVENT_ATTRIBUTE);
    if (!event)
      throw std::runtime_error("'" + std::string(TRANSITION_ELEMENT) + "' element does not have '" +
                               std::string(EVENT_ATTRIBUTE) + "' attribute");

    // Add the event name to the map
    map.at(state_id).insert(event);

    // Get the next transition element
    transition = transition->NextSiblingElement(TRANSITION_ELEMENT);
  }

  // Recurse if this node has nested state elements
  XMLElement* child_state_element = state->FirstChildElement(STATE_ELEMENT);
  while (child_state_element)
  {
    getStateTransitionsRecursive(child_state_element, map, map.at(state_id));
    child_state_element = child_state_element->NextSiblingElement(STATE_ELEMENT);
  }

  // Get the history states
  XMLElement* history = state->FirstChildElement(HISTORY_STATE_ELEMENT);
  while (history)
  {
    const char* id = history->Attribute(HISTORY_STATE_ID_ATTRIBUTE);
    if (!id)
      throw std::runtime_error("'" + std::string(HISTORY_STATE_ELEMENT) + "' element does not have '" +
                               std::string(HISTORY_STATE_ID_ATTRIBUTE) + "' attribute");

    // Add this state to the map
    map[QString(id)] = QSet<QString>{};

    // History states do not have transitions or nested states, so no need to recurse into it
    history = history->NextSiblingElement(HISTORY_STATE_ELEMENT);
  }
}

namespace scxml_core
{
StateTransitionMap getStateTransitionMap(const std::string& scxml_file)
{
  using namespace tinyxml2;

  // Create an XML document
  XMLDocument doc;
  if (doc.LoadFile(scxml_file.c_str()) != XMLError::XML_SUCCESS)
    throw std::runtime_error("Failed to load document");

  XMLElement* scxml = doc.FirstChildElement(SCXML_ELEMENT);
  if (!scxml)
    throw std::runtime_error("Scxml document contains no '" + std::string(SCXML_ELEMENT) + "' element");

  XMLElement* state = scxml->FirstChildElement(STATE_ELEMENT);
  if (!state)
    throw std::runtime_error("'" + std::string(SCXML_ELEMENT) + "' has no child '" + std::string(STATE_ELEMENT) +
                             "' elements");

  // Call the recursive function on each of the top level states
  StateTransitionMap map;
  while (state)
  {
    getStateTransitionsRecursive(state, map, QSet<QString>{});
    state = state->NextSiblingElement(STATE_ELEMENT);
  }

  return map;
}

ScxmlSMInterface::ScxmlSMInterface(const std::string& scxml_file)
  : sm_(QScxmlStateMachine::fromFile(QString::fromStdString(scxml_file)))
  , state_transition_map_(scxml_core::getStateTransitionMap(scxml_file))
{
  if (!sm_)
    throw std::runtime_error("Failed to create state machine");

  // Double check that the states in the state machine correspond to those in the map
  QStringList states = sm_->stateNames(false);
  for (auto it = state_transition_map_.begin(); it != state_transition_map_.end(); ++it)
  {
    if (!states.contains(it->first))
      throw std::runtime_error("State machine does not contain state '" + it->first.toStdString() + "'");
  }
  for (const QString& state : states)
  {
    if (state_transition_map_.find(state) == state_transition_map_.end())
      throw std::runtime_error("State transition map does not contain state '" + state.toStdString() + "'");
  }
}

void ScxmlSMInterface::addOnEntryCallback(const QString& state, const std::function<void()>& callback)
{
  if (state_transition_map_.find(state) == state_transition_map_.end())
    throw std::runtime_error("State '" + state.toStdString() + "' is not known");
  sm_->connectToState(state, QScxmlStateMachine::onEntry(callback));
}

void ScxmlSMInterface::addOnExitCallback(const QString& state, const std::function<void()>& callback)
{
  if (state_transition_map_.find(state) == state_transition_map_.end())
    throw std::runtime_error("State '" + state.toStdString() + "' is not known");
  sm_->connectToState(state, QScxmlStateMachine::onExit(callback));
}

void ScxmlSMInterface::submitEvent(const QString& event)
{
  QStringList active_states = sm_->activeStateNames();
  for (const QString& state : active_states)
  {
    if (state_transition_map_.at(state).contains(event))
    {
      sm_->submitEvent(event);
      return;
    }
  }

  std::stringstream ss;
  ss << "Transition '" << event.toStdString() << "' was not defined for active states [ ";
  for (const QString& state : active_states)
  {
    ss << state.toStdString() << " ";
  }
  ss << "]";
  throw std::runtime_error(ss.str());
}

}  // namespace scxml_core
