# online_activity_recognition

requirements: 

-  OpenNI skeleton tracker [here](https://github.com/OMARI1988/skeleton_tracker) 

To run: 

```
rosrun online_activity_recognition activity_action.py

rosrun actionlib axclient.py /recognise_action
```

Goal:


time = 60
waypoint = 'test' 



If you have human body pose estimate data stored offline, you can run with goal.waypoint='offline'.
