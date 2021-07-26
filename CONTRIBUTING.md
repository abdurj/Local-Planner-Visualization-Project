# Local Planner
To add a new local planner, you must implement a class with the following methods:
- update_pose(initial_pose, goal_pose)
- start()
These methods are called when the user clicks the simulate button, or when the user updates the configuration by dragging and dropping the start or end goals.
You will also need to add a new conditional for the new planner in the init_state, renderState, simulateState, update_pose methods in base.py to reflect your new planner.

## Configurable parameters
To add configurable parameters to your local planner, you must add a new State Enum to the State class, and create a dict of pygame_gui elements to reflect your parameters. Then you must implement the interactions of the buttons in the on_ui_event() method in base.py and send these updated parameters to your planner.

Consult the current implementations for examples.
