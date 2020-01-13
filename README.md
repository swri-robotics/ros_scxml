# ros_scxml
Lightweight finite state machine library that uses the [SCXML](https://commons.apache.org/proper/commons-scxml/guide/scxml-documents.html) standard

---
## Prerequisites
### QT 5
  The QScxml library is only available from version Qt 5.7 and up, this implementation currently uses Qt 5.12.2 which
  can be downloaded from [here](http://download.qt.io/official_releases/qt/5.12/5.12.2/).  Run the instalation script 
  with root access and follow the on screen instructions.
  In order properly configure this library for cmake and so that it can be used as a shared library then the `PATH` and `LD_LIBRARY_PATH` 
  must be set.  For instance if you installed QT in the `/opt/QT` system directory then you would add set the variables as follows:
  ```
  export PATH="/opt/Qt/5.12.2/gcc_64/lib/cmake:$PATH"
  export LD_LIBRARY_PATH=/opt/Qt/5.12.2/gcc_64/lib:$LD_LIBRARY_PATH
  ```

---
## Demo program
### Description
The `demo_scxml_state_machine` ROS node shows how to use the State Machine library with ROS and process specific code.  A key part of this code is in the section that adds custom functions which get invoked when specific states are entered or exited; inspect this section in order to understand how this is accomplished:
```cpp
  // adding application methods to SM
  MockApplication process_app(nh);
  bool success = false;
  std::vector< std::function<bool ()> > functions = {
  
    // custom function invoked when the "st3Reseting" state is entered
    [&]() -> bool{
      return sm->addEntryCallback("st3Reseting",[&](const Action& action) -> Response{
        process_app.resetProcess();
        ros::Duration(3.0).sleep();
        // queuing action, should exit the state
        sm->postAction(Action{.id="trIdle"});
        return true;
      },false); // false = runs sequentially, use for non-blocking functions
    },
    
    // custom function invoked when the "st3Execute" state is entered
    [&]() -> bool{
      return sm->addEntryCallback("st3Execute",[&process_app](const Action& action) -> Response{
        return process_app.executeProcess();
      },true); // true = runs asynchronously, use for blocking functions
    },
    
    // custom function invoked when the "st3Execute" state is exited
    [&]() -> bool{
      return sm->addExitCallback("st3Execute",[&process_app](){
        process_app.haltProcess();
      });
    },
    
    // custom function invoked when the "st2Clearing" state is entered, it will exit after waiting for 3 seconds
    [&]() -> bool{
      return sm->addEntryCallback("st2Clearing",[&](const Action& action) -> Response{
        ROS_INFO("Clearing to enable process, please wait ...");
        ros::Duration(3.0).sleep();
        ROS_INFO("Done Clearing");

        // queuing action, should exit the state
        sm->postAction(Action{.id="trStopped"});
        return true;
      }, true); // true = runs asynchronously, use for blocking functions
    }
  };
```

---
### Workspace Setup
#### catkin (ROS1)
1. Blacklist ros2 package, from the repo directory run the following:
    ```
    cat catkin-ignore.txt | xargs -n 1 catkin config -a --blacklist
    ```

2. Build the workspace
    ```
    catkin build
    ```

#### colcon (ROS2)
These steps use the [mixin feature of colcon](https://github.com/colcon/colcon-mixin-repository) to ignore ros1 (catkin) packages. More on that [here](https://colcon.readthedocs.io/en/released/reference/verb/mixin.html)
1. Create a *mixin* directory in your workspace directory
	```
	mkdir mixin
	```
2. Create an **index.yaml** file with the following content
	```
	mixin:
    - skip.mixin
    ```
    
3. Copy the **skip.mixin** file of the repo into the mixin directory
	```
	cp src/ros_scxml/skip.mixin mixin
	```
	
4. Add the mixin to the colcon workspace
	```
	colcon mixin add skip file://`pwd`/mixin/index.yaml
	colcon mixin update skip
	```
		
	NOTE: If you need to blacklist other packages then edit the **mixin/skip.mixin** file and add those
	packages to the `packages-skip` list
	
5. Build colcon environment
	```
	colcon build --symlink-install --mixin skip
	```

> These instructions were inspired by this [ros answers post](https://answers.ros.org/question/306624/ignore-package-in-colcon-but-not-catkin/)
---
### RUN Demo
#### ROS 1
1. Start the roscore
2. Go to the resource directory:
    ```
    roscd ros_scxml/resource/
    ```
3. Open the **demo_packml_sm.scxml** file with QTCreator to display a graphical depiction of the SM workflow.
4. Run the node with the demo scxml file:
    ```
   rosrun ros_scxml demo_scxml_state_machine _state_machine_file:=demo_packml_sm.scxml
    ```
5. Echo the current state:
    ```
    rostopic echo /current_state
    ```
6. Call the service to print the available actions at the current state:
    ```
    rosservice call /print_actions "{}" 
    ```
    
    The SM terminal should display something as follows:
    ```
    SM Actions: 
	    -userClear
	    -trAborted
    ```
    This means that the actions available at the current state are **trAborted** and **userClear**
7. Publish an action from the list to the state machine, for instance the **userClear** action will be requested as follows:
    ```
    rostopic pub -1 /execute_state std_msgs/String "data: 'userClear'" 
    ```

    This should cause the state machine to go into the **st2Clearing** state.  A few seconds after that it should go into the **st2Stopped** state as the custom entry callback function automatically posts an action.

8. Repeat steps 6 and 7 to experiment with other states and actions, use the SM graph shown in QTCreator to verify the workflow.
9. Additionally, the SM will publish a message to the **/process_msg** topic only when it's in the **st3Execute** state.  When the SM is in that state echo the topic as follows:
    ```
    rostopic echo /process_msg
    ```

#### ROS 2

1. From the *rclcpp_scxml_demos* directory run the demo node as follows
    ```
    ros2 run rclcpp_scxml_demos demo_scxml_state_machine __params:=resource/params.yaml
    ```
2. From another terminal echo the current state
    ```
    ros2 topic echo /current_state 
    ```
  You should get the name of the current state printed in the terminal

3. Print current actions
    ```
    ros2 service call /print_actions std_srvs/srv/Trigger
    ```
  In the node terminal you should see a list of available actions within the current state

4. Execute action
    ```
    ros2 topic pub -1 /execute_action std_msgs/msg/String '{data: trAborted}'
    ```
  If the action is valid you should see the following confirmation message in the node terminal 
  *"Action trAborted successfully executed"*