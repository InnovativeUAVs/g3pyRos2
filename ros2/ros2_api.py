# this file from chatgpt

from g3py import G3App
from ros2_data_subscriber import ROS2DataSubscriber
import threading
import rclpy

# Set up g3py app
app = G3App(__name__)

# Global variable to store the ROS 2 node instance
ros2_node = None

# Endpoint to get ROS 2 data
@app.route('/ros2_data', methods=['GET'])
def get_ros2_data():
    global ros2_node
    if ros2_node:
        data = ros2_node.get_data()  # Get the data from ROS 2 subscriptions
        return {
            "status": "success",
            "data": data
        }
    else:
        return {
            "status": "error",
            "message": "ROS 2 node not initialized"
        }

def start_ros2_node():
    rclpy.init()
    global ros2_node
    ros2_node = ROS2DataSubscriber()
    rclpy.spin(ros2_node)
    ros2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the ROS 2 node in a separate thread
    ros2_thread = threading.Thread(target=start_ros2_node)
    ros2_thread.start()
    
    # Run the g3py app (which will expose the /ros2_data endpoint)
    app.run()
