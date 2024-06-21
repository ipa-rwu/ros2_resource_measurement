import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Float64MultiArray
import psutil
import os

class Process:
    def __init__(self, parent, id, cpu_topic, memory_topic, network_topic, disk_io_topic, cmdline):
        self.id = id
        self.cmdline_arg_match = cmdline
        self.cpu_value = 0.0
        self.memory_value = 0.0
        self.network_value = [0.0, 0.0]  # [bytes_sent, bytes_recv]
        self.disk_io_value = [0.0, 0.0]  # [read_bytes, write_bytes]
        self.node = parent
        self.node.get_logger().warn('Init %s (cmdline args: "%s")' % (self.id, cmdline))
        self.pub_cpu_measure_ = self.node.create_publisher(Float64, cpu_topic.replace("-", "_"), 10)
        self.pub_memory_measure_ = self.node.create_publisher(Float64, memory_topic.replace("-", "_"), 10)
        self.pub_network_measure_ = self.node.create_publisher(Float64MultiArray, network_topic.replace("-", "_"), 10)
        self.pub_disk_io_measure_ = self.node.create_publisher(Float64MultiArray, disk_io_topic.replace("-", "_"), 10)

def split_process_arg(name: str):
    return name.split("@")

class MeasureProcess(Node):
    def __init__(self):
        super().__init__("measure_process")
        # Params
        self.declare_parameter("process_name", "python3@ros2")
        self.declare_parameter("process_period", 0.5)

        # Subscription
        self.status_ = self.create_subscription(String, "status", self.status_callback, 10)

        # Publisher
        self.pub_cpu_measure_ = self.create_publisher(Float64MultiArray, "cpu_stats", 10)
        self.pub_memory_measure_ = self.create_publisher(Float64MultiArray, "memory_stats", 10)
        self.pub_network_measure_ = self.create_publisher(Float64MultiArray, "network_stats", 10)
        self.pub_disk_io_measure_ = self.create_publisher(Float64MultiArray, "disk_io_stats", 10)

        # Variables
        self.cpu_value = 0.0
        self.memory_value = 0.0

        self.initialize()

    def initialize(self):
        self.get_logger().info("Measure Process::initialize() ok.")
        # Read Params
        self.process_name = self.get_parameter("process_name").get_parameter_value().string_value
        self.proc_list = self.process_name.split(", ")
        print(self.proc_list)
        self.process_list = list()
        for process in self.proc_list:

            lst = split_process_arg(process)

            process = lst[0]
            if len(lst) > 1:
                cmdline = lst[1]
            else:
                cmdline = ""
            print("process: " + process + " cmdline: " + str(cmdline))
            proc = Process(self, process, process + "_cpu", process + "_memory", process + "_network", process + "_disk_io", cmdline)
            self.process_list.append(proc)
        process_period = self.get_parameter("process_period").get_parameter_value().double_value

        self.timer = self.create_timer(process_period, self.iterate)

    def status_callback(self, msg):
        self.get_logger().info('Status: "%s"' % msg.data)

    def get_process_network_io(self, pid):
        net_dev_path = f"/proc/{pid}/net/dev"
        if not os.path.exists(net_dev_path):
            return [0, 0]
        with open(net_dev_path, 'r') as f:
            data = f.readlines()
        # print (f"data {data}")
        print(f"pid{pid}")
        bytes_sent, bytes_recv = 0, 0
        for line in data[2:]:
            parts = line.split()
            # print(f"parts {parts}")
            if len(parts) < 17:
                continue
            bytes_recv += int(parts[1])
            bytes_sent += int(parts[9])
        return [bytes_sent, bytes_recv]

    def do_measure(self):
        
        pi = psutil.process_iter(['pid', 'name', 'cmdline'])
        print(psutil.cpu_percent(percpu=True))
        for proc in pi:
            for process in self.process_list:
                if process.id not in proc.info['name']:
                    continue
                if process.cmdline_arg_match and process.cmdline_arg_match not in ' '.join(proc.info['cmdline']):
                    continue
                
                # CPU and memory
                cpu_value = proc.cpu_percent()
                memory_info = proc.memory_info()
                memory_value = memory_info.rss / (1024 * 1024)  

                # Network I/O
                network_value = self.get_process_network_io(proc.info['pid'])
                network_value[0] /= (1024 * 1024)  
                network_value[1] /= (1024 * 1024)  

                # Disk I/O
                try:
                    io_counters = proc.io_counters()
                    disk_io_value = [io_counters.read_bytes / (1024 * 1024), io_counters.write_bytes / (1024 * 1024)]  
                except psutil.AccessDenied:
                    disk_io_value = [0, 0]

                process.cpu_value += cpu_value
                process.memory_value += memory_value
                process.network_value[0] += network_value[0]
                process.network_value[1] += network_value[1]
                process.disk_io_value[0] += disk_io_value[0]
                process.disk_io_value[1] += disk_io_value[1]
                print(f"process id: {process.id}")

        cpu_msg = Float64MultiArray()
        memory_msg = Float64MultiArray()
        network_msg = Float64MultiArray()
        disk_io_msg = Float64MultiArray()
        
        for process in self.process_list:
            cpu_msg.data.append(process.cpu_value)
            memory_msg.data.append(process.memory_value)
            network_msg.data.extend(process.network_value)
            disk_io_msg.data.extend(process.disk_io_value)
            self.get_logger().debug("Process %s - CPU: %f, Memory: %f, Network: %s, Disk I/O: %s" % (
                process.id, process.cpu_value, process.memory_value, str(process.network_value), str(process.disk_io_value)))
            process.cpu_value = 0.0
            process.memory_value = 0.0
            process.network_value = [0.0, 0.0]
            process.disk_io_value = [0.0, 0.0]

        self.get_logger().debug("Msg: %s" % str(cpu_msg))

        self.pub_cpu_measure_.publish(cpu_msg)
        self.pub_memory_measure_.publish(memory_msg)
        self.pub_network_measure_.publish(network_msg)
        self.pub_disk_io_measure_.publish(disk_io_msg)

    def iterate(self):
        self.do_measure()

def main(args=None):
    rclpy.init(args=args)
    measure_process_node = MeasureProcess()
    rclpy.spin(measure_process_node)

    measure_process_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
