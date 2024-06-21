import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RecordCpuMemoryNetworkDiskUsage(Node):
    def __init__(self):
        super().__init__('record_cpu_memory_network_disk_usage')
        # Params
        self.declare_parameter('cpu_output_file', '/tmp/cpu_usage.txt')
        self.declare_parameter('memory_output_file', '/tmp/memory_usage.txt')
        self.declare_parameter('network_output_file', '/tmp/network_usage.txt')
        self.declare_parameter('disk_io_output_file', '/tmp/disk_io_usage.txt')
        self.declare_parameter('initial_delay', 10)  # [s]
        self.declare_parameter('number_samples', 30)
        self.declare_parameter('cpu_topic_name', 'cpu_stats')
        self.declare_parameter('memory_topic_name', 'memory_stats')
        self.declare_parameter('network_topic_name', 'network_stats')
        self.declare_parameter('disk_io_topic_name', 'disk_io_stats')
        self.declare_parameter('cpu_topic_array_index', 0)
        
        self.declare_parameter('memory_topic_array_index', 0)

        self.declare_parameter('network_topic_array_indices', [0, 1])  
        self.declare_parameter('disk_io_topic_array_indices', [0, 1]) 

        # Variables
        self.cpu_iters_count = 0
        self.memory_iters_count = 0
        self.network_iters_count = 0
        self.disk_io_iters_count = 0
        self.cpu_out_fil = None
        self.memory_out_fil = None
        self.network_out_fil = None
        self.disk_io_out_fil = None

        self.initialize()

    def initialize(self):
        self.get_logger().info('RecordCpuMemoryNetworkDiskUsage::initialize() ok.')
        # Read Params
        self.cpu_output_file = self.get_parameter('cpu_output_file').get_parameter_value().string_value
        self.memory_output_file = self.get_parameter('memory_output_file').get_parameter_value().string_value
        self.network_output_file = self.get_parameter('network_output_file').get_parameter_value().string_value
        self.disk_io_output_file = self.get_parameter('disk_io_output_file').get_parameter_value().string_value
        self.cpu_topic_array_index = self.get_parameter('cpu_topic_array_index').get_parameter_value().integer_value
        self.memory_topic_array_index = self.get_parameter('memory_topic_array_index').get_parameter_value().integer_value
        self.network_topic_array_indices = self.get_parameter('network_topic_array_indices').get_parameter_value().integer_array_value
        self.disk_io_topic_array_indices = self.get_parameter('disk_io_topic_array_indices').get_parameter_value().integer_array_value
        self.initial_delay = self.get_parameter('initial_delay').get_parameter_value().integer_value
        self.number_samples = self.get_parameter('number_samples').get_parameter_value().integer_value
        self.cpu_topic_name = self.get_parameter('cpu_topic_name').get_parameter_value().string_value
        self.memory_topic_name = self.get_parameter('memory_topic_name').get_parameter_value().string_value
        self.network_topic_name = self.get_parameter('network_topic_name').get_parameter_value().string_value
        self.disk_io_topic_name = self.get_parameter('disk_io_topic_name').get_parameter_value().string_value

        # Subscriptions
        self.sub_cpu_ = self.create_subscription(Float64MultiArray, self.cpu_topic_name, self.cpu_callback, 1)
        self.sub_memory_ = self.create_subscription(Float64MultiArray, self.memory_topic_name, self.memory_callback, 1)
        self.sub_network_ = self.create_subscription(Float64MultiArray, self.network_topic_name, self.network_callback, 1)
        self.sub_disk_io_ = self.create_subscription(Float64MultiArray, self.disk_io_topic_name, self.disk_io_callback, 1)

        # Open the files for writing
        self.cpu_out_fil = open(self.cpu_output_file, 'wt')
        self.memory_out_fil = open(self.memory_output_file, 'wt')
        self.network_out_fil = open(self.network_output_file, 'wt')
        self.disk_io_out_fil = open(self.disk_io_output_file, 'wt')

    def cpu_callback(self, msg):
        cpu_usage = msg.data[self.cpu_topic_array_index]

        self.cpu_iters_count += 1
        if self.cpu_iters_count < self.initial_delay:
            return

        self.get_logger().info(f'cpu_callback: {cpu_usage}% [{self.cpu_iters_count} / {self.initial_delay} / {self.number_samples}]')

        if self.cpu_out_fil:
            self.cpu_out_fil.write(str(cpu_usage) + '\n')

        if self.cpu_iters_count >= self.number_samples + self.initial_delay:
            self.get_logger().info('ALL CPU SAMPLES GRABBED. Shutting down CPU logging')
            if self.cpu_out_fil:
                self.cpu_out_fil.close()
                self.cpu_out_fil = None
            self.destroy_subscription(self.sub_cpu_)
            self.check_shutdown()

    def memory_callback(self, msg):
        memory_usage = msg.data[self.memory_topic_array_index]

        self.memory_iters_count += 1
        if self.memory_iters_count < self.initial_delay:
            return

        self.get_logger().info(f'memory_callback: {memory_usage}MB [{self.memory_iters_count} / {self.initial_delay} / {self.number_samples}]')

        # Write to the file if it's open
        if self.memory_out_fil:
            self.memory_out_fil.write(str(memory_usage) + '\n')

        if self.memory_iters_count >= self.number_samples + self.initial_delay:
            self.get_logger().info('ALL MEMORY SAMPLES GRABBED. Shutting down memory logging')
            if self.memory_out_fil:
                self.memory_out_fil.close()
                self.memory_out_fil = None
            self.destroy_subscription(self.sub_memory_)
            self.check_shutdown()

    def network_callback(self, msg):
        bytes_sent = msg.data[self.network_topic_array_indices[0]]
        bytes_recv = msg.data[self.network_topic_array_indices[1]]

        self.network_iters_count += 1
        if self.network_iters_count < self.initial_delay:
            return

        self.get_logger().info(f'network_callback: {bytes_sent} bytes sent, {bytes_recv} bytes received [{self.network_iters_count} / {self.initial_delay} / {self.number_samples}]')

        # Write to the file if it's open
        if self.network_out_fil:
            self.network_out_fil.write(f'{bytes_sent},{bytes_recv}\n')

        if self.network_iters_count >= self.number_samples + self.initial_delay:
            self.get_logger().info('ALL NETWORK SAMPLES GRABBED. Shutting down network logging')
            if self.network_out_fil:
                self.network_out_fil.close()
                self.network_out_fil = None
            self.destroy_subscription(self.sub_network_)
            self.check_shutdown()

    def disk_io_callback(self, msg):
        read_bytes = msg.data[self.disk_io_topic_array_indices[0]]
        write_bytes = msg.data[self.disk_io_topic_array_indices[1]]

        self.disk_io_iters_count += 1
        if self.disk_io_iters_count < self.initial_delay:
            return

        self.get_logger().info(f'disk_io_callback: {read_bytes} bytes read, {write_bytes} bytes written [{self.disk_io_iters_count} / {self.initial_delay} / {self.number_samples}]')

        # Write to the file if it's open
        if self.disk_io_out_fil:
            self.disk_io_out_fil.write(f'{read_bytes},{write_bytes}\n')

        if self.disk_io_iters_count >= self.number_samples + self.initial_delay:
            self.get_logger().info('ALL DISK I/O SAMPLES GRABBED. Shutting down disk I/O logging')
            if self.disk_io_out_fil:
                self.disk_io_out_fil.close()
                self.disk_io_out_fil = None
            self.destroy_subscription(self.sub_disk_io_)
            self.check_shutdown()

    def check_shutdown(self):
        if all(f is None for f in [self.cpu_out_fil, self.memory_out_fil, self.network_out_fil, self.disk_io_out_fil]):
            self.get_logger().info('All samples grabbed, shutting down the node.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RecordCpuMemoryNetworkDiskUsage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
