#include <scxml_core/scxml_sm_interface.h>
#include <QCoreApplication>
#include <iostream>
#include <sstream>

int main(int argc, char** argv)
{
  try
  {
    if (argc != 2)
      throw std::runtime_error("Please provide SCXML file location as first argument");

    // Load the state machine
    scxml_core::ScxmlSMInterface interface(argv[1]);

    // Add a simple state callback for each state
    const scxml_core::StateTransitionMap map = interface.getStateTransitionMap();
    for (auto it = map.begin(); it != map.end(); ++it)
    {
      QString state_name = it->first;
      auto cb = [state_name]() { std::cout << "Entered state '" << state_name.toStdString() << "'" << std::endl; };
      interface.addOnEntryCallback(state_name, cb);
    }

    // Create the Qt application
    QCoreApplication app(argc, argv);

    std::cout << "Starting the state machine" << std::endl;
    interface.getSM()->setRunning(true);
    app.processEvents();
    std::cout << "Enter the numeric index of the desired event to execute" << std::endl;

    while (true)
    {
      // Process the Qt events
      app.processEvents();

      // Get the active state and available events
      QStringList active_states = interface.getSM()->activeStateNames();
      const QString& current_state = active_states.at(0);
      std::set<std::pair<QString, QString>> available_events = map.at(current_state);

      std::stringstream ss;
      ss << "Available events: [ ";
      for (auto& pair : available_events)
      {
        const QString& event = pair.first;
        ss << event.toStdString() << " ";
      }
      ss << "]";

      //      // Get user input as to which event to execute
      //      bool done = false;
      //      while (!done)
      //      {
      //        std::cout << ss.str() << std::endl;

      //        // Get the character input
      //        auto input = std::cin.get();
      //        // Throw away the enter input
      //        std::cin.get();

      //        if (std::isdigit(input))
      //        {
      //          int idx = static_cast<int>(input) - 48;
      //          if (idx < available_events.size())
      //          {
      //            interface.submitEvent(available_events.toList().at(idx));
      //            done = true;
      //          }
      //          else
      //          {
      //            std::cout << "Index " << idx << " was not in range [0, " << available_events.size() - 1 << "]" <<
      //            std::endl;
      //          }
      //        }
      //        else
      //        {
      //          std::cout << "Input must be numeric" << std::endl;
      //        }
      //      }
    }
  }
  catch (const std::exception& ex)
  {
    std::cerr << "Error: " << ex.what() << std::endl;
    return -1;
  }
}
