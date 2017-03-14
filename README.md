# hratc2017
The Anti-Personnel Mine Ban Convention is one of the world's most widely accepted treaties with 162 States Parties agreeing not to use, stockpile, produce or transfer antipersonnel mines. Although this has had profound impact worldwide, there is a non-negligible conflict legacy that still has provoked terrible consequences. According to the UN Mine Action Service, landmines kill 15,000-20,000 people every year (mostly children) and maim countless more across 78 countries. Demining efforts cost US$ 300-1000 per mine, and, for every 5000 mines cleared, one person is killed and two are injured. Thus, clearing post-combat regions of landmines has proven to be a difficult, risky, dangerous and expensive task with enormous social implications for civilians. Motivated by these considerations, the IEEE Robotics &amp; Automation Society’s Special Interest Group on Humanitarian Technology (RAS–SIGHT) is inviting the academic and non-academic community to participate in the fourth Humanitarian Robotics and Automation Technology Challenge (HRATC) at the 2017 International Conference on Robotics and Automation (ICRA’17).   This fourth HRATC edition will follow the footsteps of previous ones and continue to focus on promoting the development of new strategies for autonomous landmine detection using a mobile (ground) robot. HRATC Challenge has three phases: 1) Simulation Phase, 2) Testing Phase, and 3) Finals Phase. The strategies developed by the participating teams will be according to the following criteria: exploration time and environmental coverage; detection and classification quality; and landmine avoidance. Teams will be progressively eliminated after each phase and the remaining teams would move on to the next phase culminating in the Challenge (Finals) phase at ICRA’17.

### Install and run
- In order to execute this code, we need install some dependencies. Run the follow commands in a terminal:

```
sudo apt-get install ros-indigo-twist-mux
sudo apt-get install ros-indigo-navigation
sudo apt-get install ros-indigo-hector-slam
```

- Unzip the source code in `[your_catkin_workspace]/src`
- Compile the packages with the command in `your_catkin_workspace` folder:

```
catkin_make
```

- Run the simulator in a terminal:

```
roslaunch hratc2017_framework run_simulation.launch
```

- Run the program using a roslaunch file in a terminal:

```
roslaunch hratc2017_entry entry.launch simulation:=true
```

- Otherwise, run the following command for real robot aplication:

```
roslaunch hratc2017_entry entry.launch
```
