#pragma once
#include <QScxmlStateMachine>
#include <QString>
#include <QSet>
#include <QFuture>
#include <set>
#include <scxml_core/scxml_sm_interface.h>

namespace scxml_core
{
/** @brief Container for states and their associated transitions */
using StateTransitionMap = std::map<QString, std::set<std::pair<QString, QString>>>;

/** @brief Creates a map of known states and transition events associated with those states */
StateTransitionMap getStateTransitionMap(const std::string& scxml_file);

/**
 * @brief Interface for the QScxmlStateMachine class
 * @details The QScxmlStateMachine class does not provide much error feedback if callbacks are assigned to incorrect
 * states or if events are submitted to states that do not have associated transitions. This class maintains a map of
 * known states and transitions and ensures that basic state machine operations occur correctly
 */
class ScxmlSMInterface
{
public:
  ScxmlSMInterface(const std::string& scxml_file);

  /**
   * @brief checks if an event exists
   */
  bool eventExists(const QString& event, std::set<std::pair<QString, QString>> events);

  /**
   * @brief gets the state to which a desired transition occurs
   * @param search_text - insert transition text you'd like to match
   * @throws if you don't have that transition, it will return itself as it's neighbor
   */

  const QString getNeighbor(const QString& state, const QString& search_text);
  /**
   * @brief Adds a callback to the input state that will be invoked on entry to the state
   * @param async - flag for executing the input callback asynchronously
   * @throws exception if the state does not exist in the state machine
   */

  void addOnEntryCallback(const QString& state, const std::function<void()>& callback, bool async = false);

  /**
   * @brief Adds a callback to the input state that will be invoked when leaving the state
   * @throws exception if the state does not exist in the state machine
   */
  void addOnExitCallback(const QString& state, const std::function<void()>& callback);

  /**
   * @brief Submits an event to move the state machine to a different state
   * @param force - force the submission of the event, even if the asynchronous task isn't finished
   * @return True if the asynchronous callback for the current state was finished and the event could be posted, false
   * otherwise
   * @throws if the event is not a valid transition
   */
  bool submitEvent(const QString& event, bool force = false);

  inline const QScxmlStateMachine* getSM() const { return sm_; }
  inline QScxmlStateMachine* getSM() { return sm_; }
  inline StateTransitionMap getStateTransitionMap() const { return state_transition_map_; }

  /** @brief Provides access to the future of an asynchronous callback for the input state */
  inline QFuture<void>& getStateFuture(const QString& state) { return future_map_.at(state); }

protected:
  QScxmlStateMachine* sm_;
  const StateTransitionMap state_transition_map_;
  std::map<QString, QFuture<void>> future_map_;
};

}  // namespace scxml_core
