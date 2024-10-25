from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import rclpy
from geometry_msgs.msg import Twist

app = Flask(__name__)
socketio = SocketIO(app)
rclpy.init(args=None)

node = rclpy.create_node('turtle_velocity_publisher')

# Publishing at a rate of 2 Hz
rate = node.create_rate(10)

# WebSocket event to receive joystick data from the client
@socketio.on('joystick_data')
def handle_joystick_data(data):
    x = data.get('x')
    y = data.get('y')
    topic = data.get('topic')
    print(data)

   
    # emit('response', {'status': 'success', 'x': x, 'y': y})  # Respond back to the client (optional)
    if x == "0" or y == "0":
        return
    
    publisher = node.create_publisher(Twist, topic, 10)
    msg = Twist()
    msg.linear.x = int(y)/100
    msg.angular.z = -int(x)/100
    
    publisher.publish(msg)
    print('Publishing: Linear Velocity: %.2f, Angular Velocity: %.2f' % (msg.linear.x, msg.angular.z))
@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
     socketio.run(app, host='0.0.0.0', port=5000, debug=True)
