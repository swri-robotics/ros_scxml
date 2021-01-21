#include <scxml_core/scxml_sm_interface.h>

#include <iostream>
#include <QtTest/QtTest>

using namespace scxml_core;

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
    catch(const std::exception&) {}

    try
    {
      auto map = getStateTransitionMap("bad_file");

      // If we made it this far, we fail
      QVERIFY(false);
    }
    catch(const std::exception&) {}
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

    // Issue some number of transitions
    for (int i = 0; i < 10; ++i)
    {
      QStringList active_states = sm.getSM()->activeStateNames();
      QVERIFY(active_states.size() > 0);

      // Get the available events for the first active state
      QSet<QString> available_events;
      available_events = map.at(active_states.at(0));
      if (available_events.empty())
      {
        std::cout << "State '" << active_states.at(0).toStdString() << "' has no available events" << std::endl;
        break;
      }

      // Submit the first available event
      sm.submitEvent(*available_events.begin());
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

    // Define a list of events and corresponding states from the test state machine
    QList<QPair<QString, QString>> list{ { "trAborted", "st2Aborted" },
                                         { "userClear", "st2Clearing" },
                                         { "trStopped", "st2Stopped" },
                                         { "userReset", "st3Reseting" },
                                         { "trIdle", "st3Idle" },
                                         { "userStart", "st3Starting" },
                                         { "trExecute", "st3Execute" },
                                         { "pause", "st2Pause" },
                                         { "resume", "st3Execute" },
                                         { "trExecuting", "st3Completing" },
                                         { "trCompleting", "st3Complete" }
    };

    // Submit the events and check that the state machine moves to the corresponding state
    for (int i = 0; i < list.size(); ++i)
    {
      sm.submitEvent(list.at(i).first);
      QTest::qWait(100);

      QStringList active_states = sm.getSM()->activeStateNames();
      QVERIFY(active_states.at(0) == list.at(i).second);
    }
  }
};

QTEST_MAIN(UTest);
#include "utest.moc"

