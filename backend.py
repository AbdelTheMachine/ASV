from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import math
from picarx import Picarx
from vilib import Vilib

# Initialize Flask app and SocketIO for real-time updates
app = Flask(__name__)
socketio = SocketIO(app)

# Initialize the Picar-X robot
px = Picarx()

# Robot state tracking: position, heading, map data, and mode
robot_state = {
    "x": 0.0,
    "y": 0.0,
    "heading": 0.0,
    "walls": [],
    "mode": "manual",
    "pins": []
}

# Constants for movement and detection
STEP_DISTANCE = 5.0
TURN_ANGLE = math.radians(15)
OBSTACLE_THRESHOLD = 15.0
pan_angle = 0
tilt_angle = 0

# Movement control functions 
def forward():
    px.set_dir_servo_angle(0)
    px.forward(80)

def backward():
    px.set_dir_servo_angle(0)
    px.backward(80)

def turn_left():
    px.set_dir_servo_angle(-30)
    px.forward(80)
    time.sleep(0.3)

def turn_right():
    px.set_dir_servo_angle(30)
    px.forward(80)
    time.sleep(0.3)

def stop():
    px.stop()

def get_ultrasonic_distance():
    return round(px.get_distance(), 2)

# Position update based on movement
def update_position(action):
    if action == "forward":
        robot_state["x"] += STEP_DISTANCE * math.cos(robot_state["heading"])
        robot_state["y"] += STEP_DISTANCE * math.sin(robot_state["heading"])
    elif action == "backward":
        robot_state["x"] -= STEP_DISTANCE * math.cos(robot_state["heading"])
        robot_state["y"] -= STEP_DISTANCE * math.sin(robot_state["heading"])
    elif action == "left":
        robot_state["heading"] += TURN_ANGLE
    elif action == "right":
        robot_state["heading"] -= TURN_ANGLE

# Map environment based on distance sensor
def update_map_data():
    distance = get_ultrasonic_distance()
    angle = robot_state["heading"]
    wall_x = robot_state["x"] + distance * math.cos(angle)
    wall_y = robot_state["y"] + distance * math.sin(angle)
    robot_state["walls"].append({"x": wall_x, "y": wall_y})
    return {
        "robot": {
            "x": robot_state["x"],
            "y": robot_state["y"],
            "heading": robot_state["heading"]
        },
        "walls": robot_state["walls"]
    }

# Autonomous behavior step 
def autonomous_step():
    distance = get_ultrasonic_distance()
    if distance < OBSTACLE_THRESHOLD:
        turn_left()
        update_position("left")
    else:
        forward()
        update_position("forward")
    stop()

# Loop to handle autonomous and mapping modes
def background_loop():
    while True:
        mode = robot_state["mode"]

        if mode == "autonomous":
            data = update_map_data()
            autonomous_step()

            # Detect orange object and drop a pin
            if Vilib.detect_obj_parameter['object']:
                robot_state["pins"].append({
                    "x": robot_state["x"],
                    "y": robot_state["y"],
                    "label": "orange"
                })

            socketio.emit("map_update", {**data, "pins": robot_state["pins"]})

        elif mode == "mapping":
            data = update_map_data()
            stop()
            socketio.emit("map_update", {**data, "pins": robot_state["pins"]})

        time.sleep(0.5)

# Flask Web Routes 
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/toggle_mode", methods=["POST"])
def toggle_mode():
    mode = request.json.get("mode")
    robot_state["mode"] = mode
    return jsonify({"status": "ok", "mode": mode})

# Handle manual control input 
@app.route("/control", methods=["POST"])
def control():
    if robot_state["mode"] != "manual":
        return jsonify({"error": "Not in manual mode"}), 403

    key = request.json.get("action").lower()
    global pan_angle, tilt_angle

    # Movement and camera control
    if key == 'w':
        px.set_dir_servo_angle(0)
        px.forward(80)
        update_position("forward")
    elif key == 's':
        px.set_dir_servo_angle(0)
        px.backward(80)
        update_position("backward")
    elif key == 'a':
        px.set_dir_servo_angle(-30)
        px.forward(80)
        update_position("left")
    elif key == 'd':
        px.set_dir_servo_angle(30)
        px.forward(80)
        update_position("right")
    elif key == 'i':
        tilt_angle = min(tilt_angle + 5, 30)
        px.set_cam_tilt_angle(tilt_angle)
    elif key == 'k':
        tilt_angle = max(tilt_angle - 5, -30)
        px.set_cam_tilt_angle(tilt_angle)
    elif key == 'l':
        pan_angle = min(pan_angle + 5, 30)
        px.set_cam_pan_angle(pan_angle)
    elif key == 'j':
        pan_angle = max(pan_angle - 5, -30)
        px.set_cam_pan_angle(pan_angle)
    elif key == 'x':
        stop()
    else:
        return jsonify({"error": "Invalid key"}), 400

    return jsonify({"status": "ok"})

# Program entry point 
if __name__ == "__main__":
    # Start camera and orange object detection
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)
    Vilib.color_detect("orange")

    # Start background thread for autonomous/mapping updates
    threading.Thread(target=background_loop, daemon=True).start()

    # Run Flask server
    socketio.run(app, host="0.0.0.0", port=5050)
