import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import threading

# ================= Setting Area =================

# Define the 4 robots
ROBOT_FLEET = {
    "Robot 1": (2, ""),  # (Domain_ID, Namespace)
    "Robot 2": (3, ""),
    "Robot 3": (4, ""),
    "Robot 4": (5, ""),
}

# Define the parameters to tune
# Each entry defines one tunable parameter
# Format: {
#   "label": Display name,
#   "node": Target node name (Note: if it's a plugin, usually under the costmap node),
#   "param": Full parameter name (including plugin namespace),
#   "min": Minimum value, "max": Maximum value, "default": Initial display value
# }
PARAMS_CONFIG = [
    {
        "label": "Max Velocity (m/s)",
        "node": "controller_server",
        "param": "Diff.max_v",
        "min": 0.0, "max": 1.5, "default": 0.4
    },
    {
        "label": "Sensor Layer : Obstacle Radius (m)",
        "node": "global_costmap/global_costmap",
        "param": "sensor_layer.obstacle_radius",
        "min": 0.01, "max": 0.3, "default": 0.08
    },
    {
        "label": "Sensor Layer : Inflation Radius (m)",
        "node": "global_costmap/global_costmap",
        "param": "sensor_layer.inflation_radius",
        "min": 0.01, "max": 0.5, "default": 0.21
    }
]

# ===========================================

class RobotClient:
    """Responsible for handling connections and multiple node services for a single robot"""
    def __init__(self, name, domain_id, namespace):
        self.name = name
        self.domain_id = domain_id
        self.namespace = namespace
        
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node(f"tuner_client_{domain_id}", context=self.context)
        
        # Store Service Clients for different nodes, format: {"node_name": client}
        self.service_clients = {}

    def get_client(self, target_node):
        """Get or create a Service Client for the specified node"""
        if target_node in self.service_clients:
            return self.service_clients[target_node]
        
        # Construct service name
        if self.namespace:
            srv_name = f"/{self.namespace}/{target_node}/set_parameters"
        else:
            srv_name = f"/{target_node}/set_parameters"
            
        client = self.node.create_client(SetParameters, srv_name)
        self.service_clients[target_node] = client
        
        # Try to connect (only check on first creation)
        # Do not block here, check when sending instead
        return client

    def send_param(self, target_node, param_name, value):
        client = self.get_client(target_node)
        
        if not client.service_is_ready():
            # Try waiting a bit, if still not ready then give up
            if not client.wait_for_service(timeout_sec=0.1):
                return f"❌ {self.name}: Connection Failed ({target_node})"

        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        param = Parameter(name=param_name, value=val)
        req.parameters.append(param)
        
        # Asynchronous send
        client.call_async(req)
        return None # Successfully sent (does not guarantee setting success, only that the request was sent)

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi sima Parameter Tuner")
        self.root.geometry("800x600")
        
        self.clients = []
        self.init_robots()
        self.build_ui()

    def init_robots(self):
        print("Initializing robot connections...")
        for name, (did, ns) in ROBOT_FLEET.items():
            client = RobotClient(name, did, ns)
            self.clients.append(client)

    def build_ui(self):
        # Label
        ttk.Label(self.root, text="Parameter Console", font=("Arial", 14, "bold")).pack(pady=10)
        
        # Scrollable area (for many parameters)
        canvas = tk.Canvas(self.root)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Dynamically create control blocks for each parameter
        for config in PARAMS_CONFIG:
            self.create_slider_group(scrollable_frame, config)

        # Status Log Area
        ttk.Label(self.root, text="Log:", font=("Arial", 10, "bold")).pack(pady=(10,0))
        self.status_text = tk.Text(self.root, height=8, width=50)
        self.status_text.pack(pady=10, padx=10)
        self.log("System ready.")

    def create_slider_group(self, parent, config):
        frame = ttk.LabelFrame(parent, text=config['label'], padding=10)
        frame.pack(fill="x", padx=10, pady=5)
        
        # Value display Label
        val_label = ttk.Label(frame, text=f"Value: {config['default']}")
        val_label.pack(anchor="w")
        
        # Slider
        slider = ttk.Scale(
            frame, 
            from_=config['min'], 
            to=config['max'], 
            orient='horizontal', 
            length=350
        )
        slider.set(config['default']) # Set default value
        slider.pack(fill="x", pady=5)

        # Display value changes (do not send yet)
        slider.configure(command=lambda v, l=val_label: l.configure(text=f"Value: {float(v):.3f}"))

        # Send on mouse release (pass config so callback knows which parameter)
        slider.bind("<ButtonRelease-1>", lambda event, s=slider, c=config: self.on_slider_release(s, c))

    def on_slider_release(self, slider, config):
        val = slider.get()
        self.log(f"Setting [{config['label']}] -> {val:.3f}")
        self.broadcast_param(config, val)

    def broadcast_param(self, config, value):
        t = threading.Thread(target=self._broadcast_worker, args=(config, value))
        t.start()

    def _broadcast_worker(self, config, value):
        target_node = config['node']
        param_name = config['param']
        
        for client in self.clients:
            try:
                err = client.send_param(target_node, param_name, value)
                if err:
                    self.root.after(0, self.log, err)
                else:
                    self.root.after(0, self.log, f"-> {client.name} (ID {client.domain_id}) OK")
            except Exception as e:
                self.root.after(0, self.log, f"-> {client.name} Exception: {e}")

    def log(self, msg):
        self.status_text.insert(tk.END, msg + "\n")
        self.status_text.see(tk.END)

    def on_close(self):
        for c in self.clients:
            c.destroy()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()