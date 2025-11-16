#!/usr/bin/env python3
import os
import time
import shlex
import subprocess
import rclpy
from rclpy.node import Node

class StackManager(Node):
    """
    Starts and supervises the patient-side processes:
      1) patient_mqtt_bridge
      2) network_detector
      3) motion_detector (param: profile=beginner|advanced)
      4) anomaly_orchestrator
      5) patient_node
    """

    def __init__(self):
        super().__init__('stack_manager')

        default_profile = os.environ.get('PATIENT_PROFILE', 'beginner')
        self.declare_parameter('profile', default_profile)
        self.profile = self.get_parameter('profile').get_parameter_value().string_value
        if self.profile not in ('beginner','advanced'):
            self.get_logger().warn(f"[stack_manager] Unknown profile '{self.profile}', falling back to 'beginner'")
            self.profile = 'beginner'

        self.get_logger().info(f"[stack_manager] starting patient stack with profile={self.profile}")

        self.procs = [
            {
                'name': 'patient_mqtt_bridge',
                'cmd' : ['ros2','run','patient_engine','patient_mqtt_bridge'],
                'popen': None, 'backoff': 0.5
            },
            {
                'name': 'network_detector',
                'cmd' : ['ros2','run','patient_engine','network_detector'],
                'popen': None, 'backoff': 0.5
            },
            {
                'name': 'motion_detector',
                'cmd' : ['ros2','run','patient_engine','motion_detector','--ros-args','-p',f'profile:={self.profile}'],
                'popen': None, 'backoff': 0.5
            },
            {
                'name': 'anomaly_orchestrator',
                'cmd' : ['ros2','run','patient_engine','anomaly_orchestrator'],
                'popen': None, 'backoff': 0.5
            },
            {
                'name': 'patient_node',
                'cmd' : ['ros2','run','patient_engine','patient_node'],
                'popen': None, 'backoff': 0.5
            },
        ]

        for proc in self.procs:
            self._start(proc)

        self.create_timer(1.0, self._supervise_tick)

    def _start(self, proc):
        self.get_logger().info(f"[stack_manager] launching: {' '.join(shlex.quote(x) for x in proc['cmd'])}")
        try:
            proc['popen'] = subprocess.Popen(proc['cmd'])
        except Exception as e:
            self.get_logger().error(f"[stack_manager] failed to start {proc['name']}: {e}")
            proc['popen'] = None

    def _supervise_tick(self):
        for proc in self.procs:
            p = proc['popen']
            if p is None:
                self._restart_with_backoff(proc)
                continue

            rc = p.poll()
            if rc is None:
                continue

            self.get_logger().warn(f"[stack_manager] '{proc['name']}' exited (rc={rc}). restarting…")
            proc['popen'] = None
            self._restart_with_backoff(proc)

    def _restart_with_backoff(self, proc):
        delay = proc['backoff']
        self.get_logger().info(f"[stack_manager] waiting {delay:.1f}s to restart '{proc['name']}'")
        time.sleep(delay)
        proc['backoff'] = min(max(delay*2, 0.5), 8.0)
        self._start(proc)

    def destroy_node(self):
        self.get_logger().info("[stack_manager] shutting down children…")
        for proc in self.procs:
            p = proc['popen']
            if p and (p.poll() is None):
                try:
                    p.terminate()
                except Exception:
                    pass
        time.sleep(1.0)
        for proc in self.procs:
            p = proc['popen']
            if p and (p.poll() is None):
                try:
                    p.kill()
                except Exception:
                    pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StackManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — stopping stack_manager")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
