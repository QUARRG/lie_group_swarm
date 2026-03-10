# Decentralized Swarm Control Via SO(3) Embeddings for 3D Trajectories

* This code generates periodic 3D trajectories for multi-agent systems by distorting a circle using the Lie group SO(3). The agents navigate free of collisions with minimal exchange of information, making this technique decentralized and scalable. This approach was validaded via simulation and with a swarm of Crazyflies

![Drone gif](/media/3D_encirclement.gif)

# Simulation
To run the simulation:

```
python3 crazy_encirclement/encirclement_sim.py
```
# Crazyflies workspace 
## Clone the [crazyswarm2](https://github.com/dimitriasilveria/crazyswarm2)  and the [motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking/tree/73da55a005c3c8ccd26e108ba627dbcd438f30f9) repositories
## Then, select the crazyflies you intend to fly in the configuration file:
1. Go to the file [crazyflie.yaml](crazyswarm2/crazyflie/config/crazyflies.yaml) 
2. Under the drone's name, set the flag "enable"
3. Open a terminal, go to the src folder of your workspace:

    ```
    cd ~/ros2_ws/src
    ```

3. Run the command, to build the crazyflie package : 

    ```
    colcon build --packages-select crazyflie
    ```

## Running all the launch files and nodes
* If you haven't add the source commands to your .bash.rc, don't forget to run 
```source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/local_setup.bash``` in all the terminal that you use for the steps in this section


1- For safety, first open a terminal and run the node that sends a landing command to all the drones:
```
ros2 run controller_pkg landing
```
* To send the landing command, click on the terminal where this node is runnig and press ```Enter``` 
  
2- On another terminal, run the [encirclement_launch.py](crazy_encirclement/launch/encirclement_launch.py) file:

``` 
ros2 launch crazy_encirclement encirclement_launch.py
```

* This launch runs the motion capture, the watch dog, the crazy server, the agents order node, and the crazy_encirclement. 

6- After all of them took off, in another terminal, run the the node that sends the drone a flag to start the encirclement trajectory:

```
ros2 run controller_pkg encircling
```
* Click on the terminal where this node is running and press ```Enter``` to start the encirclement.

**If anything goes wrong, click on the terminal where the landing node is running and press Enter.**

# Citation

If you use this code, please cite:
```bibtex
@ARTICLE{11260939,
  author={Silveria, Dimitria and Cabral, Kleber and Jardine, Peter T. and Givigi, Sidney},
  journal={IEEE Robotics and Automation Letters}, 
  title={Decentralized Swarm Control Via SO(3) Embeddings for 3D Trajectories}, 
  year={2026},
  volume={11},
  number={1},
  pages={842-849},
  keywords={Trajectory;Three-dimensional displays;Angular velocity;Lie groups;Deformation;Algebra;Quaternions;Aerospace electronics;Vehicle dynamics;Vectors;Autonomous aerial vehicles;decentralized control;lie groups;swarm robotics},
  doi={10.1109/LRA.2025.3634882}}
```