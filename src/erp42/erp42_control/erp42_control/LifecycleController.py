from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy


class LifecycleController:
    def __init__(self, node):
        self.node = node
        self.cli_activate = None

    def activate_node(self, target_node: str = "/pcl_localization", retries: int = 3) -> ChangeState.Response:
        """
        target_node: 활성화할 Lifecycle 노드 이름 (예: '/pcl_localization')
        retries: 재시도 횟수 (기본 3회)
        """
        service_name = f"{target_node}/change_state"
        self.cli_activate = self.node.create_client(ChangeState, service_name)

        if not self.cli_activate.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"{service_name} 서비스가 없습니다")

        last_exc = None
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE  # 3 = Activate 전이

        for attempt in range(1, retries + 1):
            try:
                fut = self.cli_activate.call_async(req)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=0.5)
                resp = fut.result()

                if resp is None:
                    raise RuntimeError("서비스 응답이 없습니다")

                if not resp.success:
                    raise RuntimeError("Activate 전환 실패")

                self.node.get_logger().info(
                    f"{target_node} 노드가 Active 상태로 전이되었습니다 ✅"
                )
                return resp

            except Exception as e:
                last_exc = e
                self.node.get_logger().warn(
                    f"activate_node 시도 {attempt} 실패: {e}"
                )
                continue

        raise RuntimeError(
            f"{target_node} 활성화 실패: {retries}회 시도 후에도 반응 없음"
        ) from last_exc
