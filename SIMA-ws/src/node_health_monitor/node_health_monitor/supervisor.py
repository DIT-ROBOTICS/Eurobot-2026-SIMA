import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import signal
import yaml
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

class NodeSupervisor(Node):
    def __init__(self):
        super().__init__('node_health_monitor')
        
        # 1. read config file
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        # If no path is specified, try to auto-locate the config under the share directory
        if not config_file:
            pkg_share = get_package_share_directory('node_health_monitor')
            config_file = os.path.join(pkg_share, 'config', 'target_nodes.yaml')

        self.get_logger().info(f'Loading config from: {config_file}')
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        # 2. Initialize state
        self.process_map = {}   # Store subprocess objects
        self.node_states = {}   # Store node states (retry_count, status)
        self.log_history = []   # Store log history

        # Set log output path (output to logs folder under the node_health_monitor package)
        self.log_dir = os.path.join(os.getcwd(), 'logs')
        os.makedirs(self.log_dir, exist_ok=True)

        start_time_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        report_filename = f"health_report_{start_time_str}.txt"
        self.report_path = os.path.join(self.log_dir, report_filename)

        with open(self.report_path, 'w') as f:
            f.write(f"========== MONITORING SESSION STARTED: {start_time_str} ==========\n")
            f.write(f"Config loaded from: {config_file}\n")

        self.get_logger().info(f"Report will be appended to: {self.report_path}")

        # 3. Start all nodes
        self.start_all_nodes()

        # 4. Set up periodic checks (check every 5 seconds)
        self.timer = self.create_timer(5.0, self.monitor_callback)
        self.get_logger().info('Supervisor initialized and monitoring...')

    def start_all_nodes(self):
        for node_cfg in self.config['nodes']:
            name = node_cfg['name']
            self.node_states[name] = {'retries': 0, 'status': 'PENDING', 'start_time': time.time()}
            self.spawn_process(node_cfg)

    def spawn_process(self, node_cfg):
        name = node_cfg['name']
        cmd = ["ros2", "run", node_cfg['package'], node_cfg['executable']]
        
        try:
            # Start subprocess
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # Change to subprocess.PIPE to capture output
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid # Ensure the entire process group can be killed
            )
            self.process_map[name] = proc
            self.log_event(f"[START] Node '{name}' started with PID {proc.pid}")
        except Exception as e:
            self.log_event(f"[ERROR] Failed to start '{name}': {str(e)}")

    def monitor_callback(self):
        # Get the names of active nodes in the current ROS Graph
        # get_node_names_and_namespaces returns list of (name, namespace)
        active_nodes_raw = self.get_node_names_and_namespaces()
        active_node_names = [n[0] for n in active_nodes_raw]

        all_checks_done = True
        
        for node_cfg in self.config['nodes']:
            name = node_cfg['name']
            proc = self.process_map.get(name)
            state = self.node_states[name]

            # If already marked as DEAD (exceeded retry count), skip further checks
            if state['status'] == 'DEAD':
                continue

            # Check 1: Is the process crashed (Process Level)
            if proc.poll() is not None:
                self.handle_failure(node_cfg, "Process Crash (Exit Code: {})".format(proc.returncode))
                all_checks_done = False
                continue

            # Check 2: Is it present in the ROS Graph (ROS Level)
            # Need to allow some startup buffer time
            time_since_start = time.time() - state['start_time']
            if time_since_start > node_cfg.get('startup_delay', 3.0):
                if name not in active_node_names:
                    self.handle_failure(node_cfg, "Missing from ROS Graph")
                    all_checks_done = False
                else:
                    state['status'] = 'HEALTHY'
            else:
                # Still in startup buffer period, temporarily consider healthy
                pass

        # If all nodes are healthy or dead, generate a report
        self.generate_report()

    def handle_failure(self, node_cfg, reason):
        name = node_cfg['name']
        state = self.node_states[name]
        
        self.log_event(f"[FAIL] Node '{name}': {reason}")

        # Try restart logic
        if state['retries'] < 3:
            state['retries'] += 1
            self.log_event(f"[ACTION] Restarting '{name}' (Attempt {state['retries']}/3)...")

            # 1. Ensure old process is killed
            old_proc = self.process_map.get(name)
            if old_proc and old_proc.poll() is None:
                os.killpg(os.getpgid(old_proc.pid), signal.SIGTERM)

            # 2. Restart the process
            state['start_time'] = time.time()
            self.spawn_process(node_cfg)
        else:
            state['status'] = 'DEAD'
            self.log_event(f"[CRITICAL] Node '{name}' failed 3 times. Giving up.")

    def log_event(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_str = f"[{timestamp}] {message}"
        self.log_history.append(log_str)
        self.get_logger().info(message)

    def generate_report(self):
        with open(self.report_path, 'a') as f:
            # 1. Write header
            f.write("\n" + "="*50 + "\n")
            f.write(f"CHECK INTERVAL: {datetime.now().strftime('%H:%M:%S')}\n")
            f.write("-" * 50 + "\n")
            
            # 2. Write node status summary
            f.write("NODE STATUS SUMMARY:\n")
            for name, state in self.node_states.items():
                status_str = state['status']
                retry_str = f"(Retries: {state['retries']}/3)"
                
                f.write(f"  {name:<20} : {status_str} {retry_str}\n")
            
            if self.log_history:
                f.write(f"  Last Event: {self.log_history[-1]}\n")

    def destroy_node(self):
        # When the Monitor exits, ensure all child processes are killed
        self.get_logger().info("Shutting down supervisor, killing child processes...")
        for proc in self.process_map.values():
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NodeSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()