## Summary


- **complete_run.sh** - bash script that calls the respective launch files and ros nodes to perform the model recording and registration in sequence.
- Once the entry points and ball points are selected by registration, then it is published over the topic 'A_final' & 'A_ball'

## How it does?

- Launches the `auto_pcd_recorder.launch` which executes two nodes, 


	- `quintic_trajectory_publisher_sim.py` - performs quintic trajectory
	- `saving_pcd_jointstates.py` - records the point cloud data and saves it in the specified directory

- And, once the model recording is done which we check by getting the rosparam `model_recording_done` as  true, then 
- We execute the  `model_registration_node_for_integration.py` which performs the model registration and publishes entry and ball poses over the topics `A_entry` & `A_ball` respectively.
		      



