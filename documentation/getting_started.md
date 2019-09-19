# Getting Started
本页说明本软件的下载安装、如何运行MIT Controller、如果创建用户自己的Controller。

# Install dependencies

依赖包:
```
sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
```

Others:
- LCM 1.3.1 (it says Java 6, but you can use newer) (https://lcm-proj.github.io/)
- Qt 5.10.0 or newer (requires the gamepad library) (https://www.qt.io/download-qt-installer)
- Eigen (http://eigen.tuxfamily.org/)

NOTE: on Ubuntu 18.10 or 19.04, you may instead install Qt with
```
sudo apt install libqt5 libqt5gamepad5
```


# 下载及构建代码

```
git clone https://github.com/mit-biomimetics/Cheetah-Software.git
cd Cheetah-Software
cd scripts # for now, you must actually go into this folder
./make_types.sh # you may see an error like `rm: cannot remove...` but this is okay
cd ..
mkdir build
cd build
cmake .. # there are still some warnings here
make -j
```


# 测试

在 `common` 目录下有些测试代码。在build目录下, 可以运行 `common/test-common`。这里有两个测试一般会失败:

- OSQP - 这个solver本身存在不确定性，重试下应该会正常
- CASADI - 这个solver运行时加载了一个库，有时会在寻找它是报错，不过没关系，因为我们并没有用到这个Solver。

# 手柄控制器
我们使用罗技 F310 手柄控制器，它的背面有一个开关，这开关应该在“X”位置。如果切换了这个开关，手柄需要重新连接。手柄正面靠近模式切换按钮的LED灯需要是熄灭的。
(https://www.amazon.com/Logitech-940-000110-Gamepad-F310/dp/B003VAHYQY)


# 仿真示例
仿真器默认配置在`config/simulator-defaults.yaml` 和 `config/default-terrain.yaml`。默认配置在大多数情况都是正常的。 `default-terrain` 文件已经注释举例了如何去增加mesh、box和stairs，此文件也可以配置地面摩擦力。

运行仿真器前，先插入手柄，然后在 `build` 目录下运行`sim/sim` 选择 "Mini Cheetah" 和 "Simulator"，然后点击 "Start"。  你应该将看到以下输出信息：

```
[GameController] Found 1 joystick
```
左面板允许你调整仿真器配置，最常用的设置是 `simulation_speed`。默认设置是仿真器启动时从 `simulator-defaults.yaml`加载的。这个配置文件的结构一定不能动。如果增删了参数，你必须重新编译 `sim` 。任意时间你都可以保存和加载这些参数，除了 `use_spring_damper` 这个参数必须重启仿真器才会生效。

中央面板允许你调整机器人配置（不由Controller指定）。默认值当仿真器启动时从`mini-cheetah-defatuls.yaml`加载，重启仿真器时也会重新加载默认配置。这个配置文件的结构一定不能动。如果增删了参数，你必须重新编译 `sim` 。当前很多配置并没有用，将来也会被去除。最常用的配置是
-   `cheater_mode`，这用于发送机器人当前的代码、位置、航向、速度, 
-   `controller_dt`, 用于改变控制代码的执行频率，（本项配置还需要更多的测试验证）

右面板允许你调整称为“User Parameters”的运动控制参数。如果你的控制代码不涉及控制参数，控制参数将被忽略。如果你的控制代码涉及控制参数，你必须加载“User Parameters”，否则你的controller将不会启动。


运行机器人控制代码需要执行 `user/MIT_Controller/mit_ctrl m s`，其中参数 `m` 表示mini-cheetah， `s` 表示需要连接到仿真器。这种使用方式用到共享内存技术，以实现与仿真器通讯。仿真器需要已经开始运行，机器人需要移动到一个可行的位置。在仿真器窗口中央列，设置控制模式（control mode）为10。一旦机器人停止运动，设置控制模式为1，然后设置为4，机器人将会开始Trotting。

你可以使用手柄控制机器人运动。你会看到两个机器人，灰色的是仿真出来的实际位置，红色的被状态估计器预估出来的位置。开启 "cheater_mode" 会使红色和灰色机器人的位置一致。可以拖动滚动窗口；按住 `t` 可以加速仿真执行；按住空格键可以开启自由相机模式，通过 w,a,s,d,r,f 键移动相机, 鼠标拖动调整方向。


# Lightweight Communications and Marshalling
我们使用 LCM (https://lcm-proj.github.io/) 连接控制接口和实际的mini cheetah 硬件，也是在仿真时的调试工具。`make_types.sh` 脚本运行LCM工具生成包含LCM数据类型的C++头文件，当仿真器运行时，你可以运行 `scripts/launch_lcm_spy.sh` 去打开LCM工具，它会显示仿真器和控制器详细的信息，你可以点击数据流并图表化，这是非常Nice的调试方式。还有一个叫`lcm-logger` 的可以可在LCM数据到文件。


# Writing a Robot Controller
To add your own robot controller, you should add a folder under `Cheetah-Software/user`, and add the folder to the `CMakeLists.txt` in `user`.  The `JPos_Controller` is an example of a very simple controller.  The `JPosUserParameters.h` file has an example of declaring two user parameters which can be adjusted from the simulator interface, but using user parameters is optional.  The `JPos_Controller.hpp` and `JPos_Controller.cpp` files are the actual controller, which should extend `RobotController`.  Notice that in the `JPos_Controller.hpp` file, the `getUserControlParameters` method retuns a pointer to the user parameters.  If you do not use user parameters, your `getUserControlParameters` should return `nullptr`.  Finally, your `main` function must be like the example main function in `main.cpp`.

The `runController` method of your controller will be called automatically, at 1 kHz.  Here, you have access to the following:

- `_quadruped` : contains constant parameters about the robot (link lengths, gear ratios, inertias...).  The `getHipLocation` function returns the location of the "hip" in the body coordinate system.  The x-axis points forward, y-axis to the left, and z-axis up.  The legs are ordered like this

```
FRONT
1 0  RIGHT
3 2
BACK
```

- `_model` : a dynamics model of the robot.  This can be used to compute forward kinematics, Jacobians, etc...
- `_legController`: Interface to the robot's legs. This data is syncronized with the hardware at around 700 Hz. There are multiple ways to control the legs, and the result from all the controllers are added together.
    - `commands[leg_id].tauFeedForward` : Leg torque (Nm, at the joint).  Order is ab/ad, hip, knee.
    - `commands[leg_id].forceFeedForward` : Force to apply at foot (N), in hip frame. (Same orientation as body frame, origin is the hip)
    - `commands[leg_id].qDes` : Desired joint position for joint PD controller (radians). Order is ab/ad, hip, knee.  `(0,0,0)` is leg pointing straight down.
    - `commands[leg_id].qdDes` : Desired joint velocity for joint PD controller (rad/sec).
    - `commands[leg_id].pDes, vDes` : Desired foot position/velocity for cartesian PD controller (meters, hip frame)
    - `commands[leg_id].kpCartesian, kdCartesian, kpJoint, kdJoint` : Gains for PD controllers (3x3 matrix).  Use the diagonal entries only.
    - `datas[leg_id].q` : Leg joint encoder (radians).  Order is ab/ad, hip, knee.  `(0,0,0)` is leg pointing straight down.
    - `datas[leg_id].qd` : Leg joint velocity (radians/sec).  Same order as `q`.
    - `datas[leg_id].p`  : Foot cartesian position, in hip frame. (Same orientation as body frame, origin is the hip)
    - `datas[leg_id].v`  : Foot cartesian velocity, in hip frame. 
    - `datas[leg_id].tau` : Estimate of motor torque from combination of all controllers
The joint PD control actually runs at 40 kHz on the motor controllers.
- `_stateEstimate, _stateEstimatorContainer` The result and interface for the provided state estimator.  If you provide the contact state of the robot (which feet are touching the ground), it will determine the robot's position/velocity in the world.
- `_driverCommand` : inputs from the game pad.
- `_controlParameters` : values from the center robot control parameters panel
- `_visualizationData` : interface to add debugging visualizations to the simulator window
- `_robotType` : If you are the mini Cheetah or Cheetah 3 robot.


If you would like to see more of how this works, look at the `robot` folder.  The `RobotRunner` class actually runs the control code, and connects it with either the `HardwareBridge` or `SimulationBridge`.  The code in the `rt` folder actually interacts with the hardware.

# User Parameters
User Parameters are settings which are specific to the controller you are running.  The list of user parameters and their values are defined in a yaml file.  On startup, the file `config/default-user.yaml` is loaded into the simulator. Currently you must manually make sure that your currently loaded list of user parameters (on the right of the simulator window) matches the user parameters you define in your controller.  If these do not match, the controller will see the mismatch and print an error like 

`parameter cmpc_gait wasn't found in parameter collection user-parameters`

In this case, you should edit `config/default-user.yaml` to have the same parameters as your controller's user parameters. After changing `default-user.yaml`, you must either restart the simulator, or use the "Load" button to reload the parameters.

If you do not want to use user parameters, you can simply put

`__collection-name__: user-parameters`

as the only thing in `config/default-user.yaml`.  Then, in your `RobotController` class, you should provide the method

```
  // indicate that there are no user control parameters for this controller
  virtual ControlParameters* getUserControlParameters() {
    return nullptr;
  }
```


# Errors
If the controller software encounters an error where it cannot determine what to do safely (such as incompatible control parameters), it will stop itself.  Currently it will crash with `terminate called after throwing an instance of ....` and should print the exception and an error message. 

If you see the controller crash with `CRASH: Caught 11` (segfault)  or `terminate called without active exception`, it is because the code has actually crashed.  If this crash is not caused by your controller code, this is a bug, and you should create a Github issue.

If the controller stops responding for more than 1 second (it has crashed, or is stuck in a loop), the simulator will go into an error state.  It will print `[ERROR] Timed out waiting for message from robot!  Did it crash?`. 

If the simulator itself throws an exception or crashes, this is a bug, and you should create a Github issue.

# Recovering from Errors
Currently, the most reliable way to recover from errors is:

- Kill the controller code
- Click "Stop" in the simulator
- Click "Start" in the simulator
- Restart the controller code


If you ever get the simulator in a state where you can't click "Stop" to reset the simulation (or the simulator crashes), it is a bug and you should open a Github issue.



# Unfinished
- Verify that the controller update rate can be changed in simulation/robot hardware
- Using the State Estimator (and disabling it if you don't want it)
- Using the Dynamics
- Final update rate to legs.
- Are other people going to use the wireless RC controller?  If so, add it to the robot controller.
- Safety checks
- Visualization data - not everything is implemented and they don't work on the robot


# Running on the robot differences
Running on the robot is very similar to running the simulator.  You will still have the gamepad, user parameters, and robot parameters.  In the simulation window, you will see only the state estimate of the robot, and cheater mode will not work.  Currently debugging visualizations don't work.




