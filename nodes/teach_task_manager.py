# nodes/teach_task_manager.py
import rclpy, json
from rclpy.node import Node
from std_msgs.msg import String

class TeachTaskManager(Node):
    def __init__(self):
        super().__init__('teach_task_manager')
        # NLU publishes structured teach intent as JSON on /assistant/teach_task
        self.sub = self.create_subscription(String, '/assistant/teach_task', self.on_teach_intent, 10)
        self.pub_dialogue_out = self.create_publisher(String, '/assistant/dialogue/out', 10)
        self.pub_plan_compile = self.create_publisher(String, '/assistant/plan/compile', 10)  # downstream planner/BT
        self.pub_skills_save = self.create_publisher(String, '/assistant/skills/save', 10)
        self.pending_save = None  # {"name":..., "task":...}

    def on_teach_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Bad teach_task payload: {e}')
            return

        slots = data.get('slots', {})
        missing = [s for s in ['goal','area'] if s not in slots or not slots[s]]
        if missing:
            q = f"I need {', '.join(missing)}. For example, where exactly is the destination or which area?"
            self.pub_dialogue_out.publish(String(data=q))
            return

        goal = slots['goal']; area = slots['area']
        object_types = ', '.join(slots.get('object_types', ['objects']))
        destination = slots.get('destination', 'the designated bin')
        constraints = ', '.join(slots.get('constraints', [])) or 'none'

        summary = (f"I will {goal.replace('_',' ')} in {area}, handling {object_types} "
                   f"and placing them in {destination}. Constraints: {constraints}. Shall I proceed?")
        self.pub_dialogue_out.publish(String(data=summary))

        plan = {
            "type": goal,
            "params": {
                "labels": slots.get('object_types', []),
                "source_area": area,
                "destination": destination,
                "constraints": slots.get('constraints', [])
            }
        }
        self.pub_plan_compile.publish(String(data=json.dumps(plan)))

        ask = "Should I remember this as a reusable task for the future? If so, what name should I use?"
        self.pub_dialogue_out.publish(String(data=ask))
        self.pending_save = {"task": plan}

    def on_dialogue_in(self, text: str):
        # Wire this to a subscription on '/assistant/dialogue/in' if you have an ASR/NLU loop
        if not self.pending_save: 
            return
        t = text.strip().lower()
        if t.startswith("yes"):
            name = t.replace("yes", "").replace("call it", "").strip(" ,.") or "learned_task"
            self.pending_save["name"] = name
            self.pub_skills_save.publish(String(data=json.dumps({"save_skill": self.pending_save})))
            self.pub_dialogue_out.publish(String(data=f"Saved as '{name}'."))
            self.pending_save = None
        elif t.startswith("no"):
            self.pub_dialogue_out.publish(String(data="Okay, I won't save this task for later."))
            self.pending_save = None

def main():
    rclpy.init()
    n = TeachTaskManager()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
