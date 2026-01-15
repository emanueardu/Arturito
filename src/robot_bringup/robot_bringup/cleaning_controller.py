from typing import List, Optional, Set

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int16, String
from std_srvs.srv import SetBool


def _normalize_state_list(values: List[str]) -> Set[str]:
    return {str(v).strip().lower() for v in values if str(v).strip()}


class CleaningController(Node):
    """Automates vacuum/brush and motor speed based on cleaning mode."""

    def __init__(self) -> None:
        super().__init__('cleaning_controller')

        self._cleaning_pwm = int(self.declare_parameter('cleaning_pwm', 200).value)
        self._travel_pwm = int(self.declare_parameter('travel_pwm', 255).value)
        cleaning_states_param = self.declare_parameter(
            'cleaning_states', ['cleaning', 'rutina', 'routine']
        ).value
        self._cleaning_states = _normalize_state_list(cleaning_states_param)
        self._mode_topic = str(self.declare_parameter('mode_topic', 'routine_state').value or '')
        self._mode_bool_topic = str(
            self.declare_parameter('mode_bool_topic', 'cleaning_active').value or ''
        )
        self._publish_interval = float(self.declare_parameter('publish_interval_sec', 5.0).value)

        self._vacuum_pub = self.create_publisher(Bool, '/vacuum/enabled', 10)
        self._brush_pub = self.create_publisher(Bool, '/brush/enabled', 10)
        self._pwm_pub = self.create_publisher(Int16, '/motors/max_pwm', 10)
        self._state_pub = self.create_publisher(Bool, 'cleaning_controller/active', 10)

        if self._mode_topic:
            self.create_subscription(String, self._mode_topic, self._mode_string_cb, 10)
        if self._mode_bool_topic:
            self.create_subscription(Bool, self._mode_bool_topic, self._mode_bool_cb, 10)

        self.create_service(SetBool, 'set_cleaning_mode', self._srv_set_cleaning_mode)

        self._cleaning_active: Optional[bool] = None
        self._republish_timer = self.create_timer(self._publish_interval, self._republish_state)

        self._set_state(False, reason='inicio')

    # region Callbacks ------------------------------------------------------
    def _mode_string_cb(self, msg: String) -> None:
        data = msg.data.strip().lower()
        desired = data in self._cleaning_states
        self._set_state(desired, reason=f"modo='{data}'")

    def _mode_bool_cb(self, msg: Bool) -> None:
        self._set_state(bool(msg.data), reason='modo_bool')

    def _srv_set_cleaning_mode(self, request: SetBool.Request, response: SetBool.Response):
        desired = bool(request.data)
        self._set_state(desired, reason='servicio')
        response.success = True
        response.message = 'Modo limpieza activado' if desired else 'Modo limpieza desactivado'
        return response

    def _republish_state(self) -> None:
        if self._cleaning_active is None:
            return
        self._publish_outputs(self._cleaning_active, log=False)

    # endregion -------------------------------------------------------------

    def _set_state(self, active: bool, reason: Optional[str] = None) -> None:
        if self._cleaning_active == active:
            return
        self._cleaning_active = active
        human = 'activado' if active else 'desactivado'
        if reason:
            self.get_logger().info(f'Rutina de limpieza {human} ({reason}).')
        else:
            self.get_logger().info(f'Rutina de limpieza {human}.')
        self._publish_outputs(active, log=False)

    def _publish_outputs(self, active: bool, log: bool = True) -> None:
        self._vacuum_pub.publish(Bool(data=active))
        self._brush_pub.publish(Bool(data=active))
        target_pwm = self._cleaning_pwm if active else self._travel_pwm
        self._pwm_pub.publish(Int16(data=int(target_pwm)))
        self._state_pub.publish(Bool(data=active))
        if log:
            self.get_logger().debug(
                'Publicado vacuum=%s brush=%s pwm=%d',
                active,
                active,
                target_pwm,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
