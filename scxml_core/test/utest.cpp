#include <scxml_core/scxml_sm_interface.h>

#include <iostream>
#include <QtTest/QtTest>
#include <thread>
#include <random>

using namespace scxml_core;

static const QList<QPair<QString, QString>> SM_SEQUENCE{
  { "trAborted", "st2Aborted" },      { "userClear", "st2Clearing" },   { "trStopped", "st2Stopped" },
  { "userReset", "st3Reseting" },     { "trIdle", "st3Idle" },          { "userStart", "st3Starting" },
  { "trExecute", "st3Execute" },      { "pause", "st2Pause" },          { "resume", "st3Execute" },
  { "trExecuting", "st3Completing" }, { "trCompleting", "st3Complete" }
};

class UTest : public QObject
{
  Q_OBJECT

private slots:
  void badInputs()
  {
    try
    {
      ScxmlSMInterface sm("bad_file");

      // If we made it this far, we fail
      QVERIFY(false);
    }
    catch (const std::exception&)
    {
    }

    try
    {
      auto map = getStateTransitionMap("bad_file");

      // If we made it this far, we fail
      QVERIFY(false);
    }
    catch (const std::exception&)
    {
    }
  }

  void badCallbacksAndEvents()
  {
    ScxmlSMInterface sm(SCXML_FILE);
    auto cb = []() {};
    const QString bad_state_name("state_name_definitely_not_in_diagram");
    QVERIFY_EXCEPTION_THROWN(sm.addOnEntryCallback(bad_state_name, cb), std::runtime_error);
    QVERIFY_EXCEPTION_THROWN(sm.addOnEntryCallback(bad_state_name, cb), std::runtime_error);

    const QString bad_event("event_definitely_not_in_diagram");
    QVERIFY_EXCEPTION_THROWN(sm.submitEvent(bad_event), std::runtime_error);
  }

  void runArbitrarySequence()
  {
    ScxmlSMInterface sm(SCXML_FILE);
    const StateTransitionMap map = sm.getStateTransitionMap();

    for (auto it = map.begin(); it != map.end(); ++it)
    {
      const QString& state_name = it->first;
      auto entry_cb = [state_name]() { std::cout << state_name.toStdString() << std::endl; };
      sm.addOnEntryCallback(state_name, entry_cb);
    }

    // Start the state machine and allow some time for the Qt thread to register the start event
    sm.getSM()->setRunning(true);
    QTest::qWait(100);
    QVERIFY(sm.getSM()->isRunning());

    // Issue some number of transitions
    for (int i = 0; i < 10; ++i)
    {
      QStringList active_states = sm.getSM()->activeStateNames();
      QVERIFY(active_states.size() > 0);

      // Get the available events for the first active state
      const QSet<QString> available_events = map.at(active_states.at(0));
      if (available_events.empty())
      {
        std::cout << "State '" << active_states.at(0).toStdString() << "' has no available events" << std::endl;
        break;
      }

      // Choose a random event to submit
      static std::mt19937 gen(1);
      std::uniform_int_distribution<int> dist(0, available_events.size() - 1);
      const QString event = available_events.toList().at(dist(gen));

      std::cout << "Event: " << event.toStdString() << std::endl;
      QVERIFY(sm.submitEvent(event));
      QTest::qWait(100);
    }
  }

  void runPlannedSequence()
  {
    ScxmlSMInterface sm(SCXML_FILE);
    StateTransitionMap map = sm.getStateTransitionMap();

    for (auto it = map.begin(); it != map.end(); ++it)
    {
      const QString& state_name = it->first;
      auto entry_cb = [state_name]() { std::cout << state_name.toStdString() << std::endl; };
      sm.addOnEntryCallback(state_name, entry_cb);
    }

    // Start the state machine and allow some time for the Qt thread to register the start event
    sm.getSM()->setRunning(true);
    QTest::qWait(100);
    QVERIFY(sm.getSM()->isRunning());

    // Submit the events and check that the state machine moves to the corresponding state
    for (int i = 0; i < SM_SEQUENCE.size(); ++i)
    {
      QVERIFY(sm.submitEvent(SM_SEQUENCE.at(i).first));
      QTest::qWait(100);

      QStringList active_states = sm.getSM()->activeStateNames();
      QVERIFY(active_states.at(0) == SM_SEQUENCE.at(i).second);
    }
  }

  void runPlannedSequenceAsync()
  {
    ScxmlSMInterface sm(SCXML_FILE);
    // Get only the lowest level children states
    const QStringList states = sm.getSM()->stateNames(true);

    for (const QString& state_name : states)
    {
      // Create an asynchronous callback
      auto entry_cb = [state_name]() {
        std::cout << state_name.toStdString() << std::endl;
        // Wait for a while
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Callback complete" << std::endl;
      };

      // Add the entry callback
      sm.addOnEntryCallback(state_name, entry_cb, true);
    }

    // Start the state machine and allow some time for the Qt thread to register the start event
    sm.getSM()->setRunning(true);
    QTest::qWait(100);
    QVERIFY(sm.getSM()->isRunning());

    // Submit the events and check that the state machine moves to the corresponding state
    for (int i = 0; i < SM_SEQUENCE.size(); ++i)
    {
      // Issue the transition a few times when we know that the entry callbacks are not complete
      for (int j = 0; j < 5; ++j)
      {
        QVERIFY(!sm.submitEvent(SM_SEQUENCE.at(i).first));
      }

      // Wait for the task to finish
      QStringList active_states = sm.getSM()->activeStateNames();
      sm.getStateFuture(active_states.at(0)).waitForFinished();

      // Submit the event, now that we know the callback is complete
      QVERIFY(sm.submitEvent(SM_SEQUENCE.at(i).first));
      QTest::qWait(100);

      // Check that the expected state is now active
      active_states = sm.getSM()->activeStateNames();
      QVERIFY(active_states.at(0) == SM_SEQUENCE.at(i).second);
    }
  }
};

QTEST_MAIN(UTest);
#include "utest.moc"
