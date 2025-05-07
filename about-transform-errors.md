# About Transform Errors

Those red TF² messages look scary, but in this particular setup they’re (unfortunately) “normal”.
Here’s what’s happening and what you can do about it.

---

### Why you see the spam

| What you see                                                         | Why it happens                                                                                                                                                                                                                                                                                                                  |
| -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `TF NAN_INPUT / TF_DENORMALIZED_QUATERNION` on the four wheel frames | At the very first simulation ticks the `gazebo_ros2_control` plugin hasn’t populated the joint-state array yet, so all wheel joint positions come through as **NaN**. `robot_state_publisher` immediately tries to turn those NaNs into transforms, TF² rejects them, and every Nav 2 node that listens to TF shouts a warning. |
| “Authority undetectable”                                             | The offending transform is coming straight from `robot_state_publisher`; TF just doesn’t know which node injected it because the message was invalid.                                                                                                                                                                           |

The AWS DeepRacer maintainers are aware of it and have an entry in the repo FAQ that says exactly this — it’s a **known issue and does **not** affect the vehicle’s motion** ([github.com][1]).

So, unless you’re seeing other failures (e.g. the car never moves, costmaps stay empty, etc.) you can safely ignore the log noise.

---

### How to verify it’s benign

1. **Watch the TF tree after a couple of seconds**

   ```bash
   ros2 run tf2_tools view_frames
   ```

   You should see a healthy tree rooted at `map → odom → base_link`; the wheel links will be present and publishing sane numbers. If they are, you’re good.

2. **Confirm Nav 2 is alive**

   - In RViz, set the fixed frame to `map`.
   - Send a 2-D Nav Goal.
     The DeepRacer model should plan and start rolling.

3. **Quick sanity check**

   ```bash
   ros2 topic echo -n 5 /tf | grep -E "left_front_wheel|right_front_wheel"
   ```

   After the first handful of messages you should no longer see `nan` in the output.

---

### Getting a clean log (optional)

If the spam really bothers you, here are a couple of work-arounds you can drop into your workspace:

| Approach                            | Idea                                                                                                                                                                 | Where to change                                                                                                            |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| **Delay `robot_state_publisher`**   | Start it 1–2 s _after_ the Gazebo entity is spawned so the joint array is already initialised.                                                                       | In `deepracer_bringup/launch/nav_amcl_demo_sim.launch.py`, wrap `robot_state_publisher` in a `TimerAction(period=2.0, …)`. |
| **Initialise joint positions to 0** | In the robot URDF/Xacro, set `initial_position="0"` (or a `<state>` block) for the four wheel joints.                                                                | `deepracer_description/urdf/deepracer.xacro`                                                                               |
| **Update gazebo_ros2_control**      | The pinned commit `04b2c6f…` predates a small bug-fix that stops NaNs leaking out. Testing with any commit ≥ `c3c7e24` on the `ros2` branch eliminates the warnings. | Replace the `git reset --hard` line with a checkout of a newer commit and rebuild.                                         |

None of those change the vehicle’s behaviour — they just hush the console.

---

### The bigger picture

- **What the error means in TF in general** — TF will drop any transform whose translation or quaternion contains NaNs; that’s what the docs refer to as `TF_NAN_INPUT` and `TF_DENORMALIZED_QUATERNION` .
- In most robots this would signal a serious math error, but in the DeepRacer sim it’s really just an artefact of the start-up race between Gazebo and the TF publishers.

Give the launch a few seconds, send a goal from RViz, and you should see the car moving despite the early warnings. If it doesn’t move, let me know what you observe (costmap status, controller feedback, etc.) and we can dig deeper.

[1]: https://github.com/aws-deepracer/aws-deepracer/blob/main/frequently_asked_questions.md "aws-deepracer/frequently_asked_questions.md at main · aws-deepracer/aws-deepracer · GitHub"
