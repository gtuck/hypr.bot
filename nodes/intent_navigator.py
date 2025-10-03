# nodes/intent_navigator.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class IntentNavigator(Node):
    def __init__(self):
        super().__init__('intent_navigator')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.est_speaker_pose_sub = self.create_subscription(
            PoseStamped, '/perception/speaker_pose', self._on_speaker_pose, 10)
        self.est_pose = None
        self.estop_sub = self.create_subscription(Bool, '/safety/e_stop', self._on_estop, 10)
        self.estopped = False

    def _on_estop(self, msg: Bool):
        self.estopped = msg.data
        if self.estopped:
            self.get_logger().warn('E-STOP engaged; canceling navigation goals.')
            # TODO: cancel goals via proper client handle when available

    def _on_speaker_pose(self, pose: PoseStamped):
        self.est_pose = pose

    async def navigate_to_speaker(self):
        if self.estopped:
            self.get_logger().warn('Blocked by e-stop.')
            return
        if self.est_pose is None:
            self.get_logger().warn('No speaker pose estimate yet.')
            return
        await self.nav_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = self.est_pose
        self.get_logger().info('On my way.')
        send_future = self.nav_client.send_goal_async(goal)
        await send_future
        result_future = send_future.result().get_result_async()
        await result_future
        self.get_logger().info('Arrived at caller.')

def main():
    rclpy.init()
    n = IntentNavigator()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
