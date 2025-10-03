# hypr.bot

Bootstrap repo for a personal robot assistant (JetAuto Pro + Vision Robotic Arm, Jetson Orin Nano Super).  
Includes: system config (YAML/JSON), schema validation, ROS 2 Nav2 bring-up, teach-by-instruction module, and CI to auto-bump platform version.

## Structure
```
config.yaml            # Human-friendly master config (v1.2)
config.json            # Mirror JSON
schema/
  personal_robot_assistant.schema.json
tools/
  bump_platform_version.py
params/
  nav2_params.yaml
launch/
  nav2_bringup.launch.py
nodes/
  intent_navigator.py
  teach_task_manager.py
.github/workflows/
  config-validate-and-bump.yml
```

## Quick Start
1. **Clone & push** this repo to GitHub.
2. Enable **Actions** in the GitHub UI.
3. Launch Nav2 with:
   ```bash
   ros2 launch launch/nav2_bringup.launch.py
   ```
4. Run the example nodes:
   ```bash
   ros2 run your_pkg intent_navigator
   ros2 run your_pkg teach_task_manager
   ```
   (or integrate these files into your ROS 2 package and add entry points).

## Teach-by-Instruction
NLU should publish a JSON payload to `/assistant/teach_task` like:
```json
{
  "intent": "teach_task",
  "slots": {
    "goal": "pick_up_objects",
    "object_types": ["toys"],
    "area": "this_room",
    "destination": "toy_bin_by_door",
    "constraints": ["be_gentle"]
  }
}
```
The `teach_task_manager` will summarize, confirm, compile a plan, and ask whether to save the new task as a reusable skill.

## CI
- Validates `config.yaml` and `config.json` against the JSON Schema.
- On `main` pushes, opens a PR that bumps `identity.platform_version` if configs changed.

---

**Note:** Adjust topics, frames, and parameters to match your robot. Replace `YOUR_ORG/YOUR_ROBOT_REPO` in `config.yaml` with your actual repo.
