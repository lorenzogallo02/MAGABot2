# flask library to control MAGAbot with ESP32 through a flask web server
# picamera2 library for camera control
# io, time, threading, requests, signal, subprocess, atexit for various utilities
# torch, cv2 for machine learning and image processing
# PIL for image handling

from flask import Flask, render_template_string, Response, request, jsonify
from picamera2 import Picamera2
import io, time, threading, requests, signal, subprocess, atexit, torch, cv2
from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights
from torchvision import transforms
from PIL import Image
import pygame

app = Flask(__name__)  # Flask web server for MAGAbot control

ESP32_IP = '192.168.2.34' # Replace with your ESP32 IP address
ESP32_URL = f"http://{ESP32_IP}/post_message" 
ESP32_BTN_URL = f"http://{ESP32_IP}/button_state" 
ESP32_BUMPER_URL = f"http://{ESP32_IP}/bumper_state"  # For bumper control 
STOP_PAYLOAD = bytes([0x86, 0x00, 0x00, 0x00, 0x00])  
MOTOR_BYTE = 0x86  # Command byte for motor control

mode = 'stopped'

# Initialize Picamera2

picam2 = Picamera2() 
picam2.configure(picam2.create_video_configuration(main={"size": (160, 120)},))  
picam2.start()

follow_thread = None
stop_follow = threading.Event()
current_proc = None
joystick_thread = None
stop_joystick = threading.Event()

WIDTH, HEIGHT = 320, 240  # For follow person
weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
model = ssdlite320_mobilenet_v3_large(weights=weights)
model.eval()
transform = weights.transforms()

def button_monitor():
    """Continuously poll the ESP32 /button_state endpoint.
       If we detect a transition from pressed → released,
       kill any active mode threads and zero the motors."""
    prev_pressed = True
    while True:
        try:
            resp = requests.get(ESP32_BTN_URL, timeout=0.2)
            pressed = resp.json().get('pressed', False)
        except requests.RequestException:
            pressed = False

        # on release (went from True→False)
        if prev_pressed and not pressed:
            # stop joystick
            global joystick_thread, stop_joystick
            if joystick_thread and joystick_thread.is_alive():
                stop_joystick.set()
                joystick_thread.join()
                joystick_thread = None
                stop_joystick.clear()

            # stop follow
            global follow_thread, stop_follow
            if follow_thread and follow_thread.is_alive():
                stop_follow.set()
                follow_thread.join()
                follow_thread = None
                stop_follow.clear()

            # zero the motors
            stop_motors()

        prev_pressed = pressed
        time.sleep(0.1)  # poll every 100 ms

def bumper_monitor():
    last_hit = False   
    while True:
        try:
            r = requests.get(ESP32_BUMPER_URL, timeout=0.2)
            data = r.json()       # {"hit":"true"/"false","which":"BOTH"/ "L" / "R"}
            print(data)
            hit = data.get('hit', False)

            if not last_hit and hit :   # Bumper hit detected
                print('bumper hit detected')
                direction = data.get("which", "BOTH")
                go_back(direction)

            last_hit = hit  # Update last_hit status

        except requests.RequestException:
            last_hit = False
        time.sleep(0.1)


# MOTORS CONTROL
def send_command(left_speed, left_dir, right_speed, right_dir):  # Send motor control commands to ESP32
    payload = bytes([MOTOR_BYTE, left_speed, left_dir, right_speed, right_dir])  # Create payload
    try:
        requests.post(ESP32_URL, data=payload, timeout=0.1)
    except: pass

def stop_motors():
    send_command(0, 0, 0, 0) 

def go_back(direction):  # Go back when bumper is hit
   global mode, joystick_thread, follow_thread
   
   old_mode = mode

   stop_motors()  # Stop motors first
   time.sleep(1)

   if joystick_thread and joystick_thread.is_alive():
       stop_joystick.set()
       joystick_thread.join()
       joystick_thread = None
       stop_joystick.clear()
       print('joystick mode stopped')
       

   if follow_thread and follow_thread.is_alive():
       stop_follow.set()
       follow_thread.join()
       follow_thread = None
       stop_follow.clear()
       print('follow mode stopped')

   if direction == 'L':          # left bumper → back + veer right
       # left wheel reverse fast, right wheel reverse slower
       send_command(20, 1, 10, 1) 
   elif direction == 'R':        # right bumper → back + veer left
       send_command(10, 1, 20, 1)
   else:                         # BOTH → straight back
       send_command(20, 1, 20, 1)

   time.sleep(2)
   stop_motors()
   
   # resume previous mode
   if old_mode == 'joystick':
       joystick_thread = threading.Thread(target=joystick_control_loop, daemon=True)
       joystick_thread.start()
       print('joystick mode resumed')
   elif old_mode == 'follow':
       follow_thread = threading.Thread(target=follow_person, daemon=True)
       follow_thread.start()
       print('follow mode resumed')
   else:
       mode = 'stopped'


atexit.register(stop_motors)  # Ensure motors stop on exit


# WEB SERVER 
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <title>MAGAbot Control Panel</title>
  <style>
    body { font-family: sans-serif; text-align: center; margin-top: 50px; background: #f2f2f2; }
    h1 { color: #333; }
    #status { margin: 20px; font-size: 1.2em; color: #222; }
    .btn { padding: 15px 30px; font-size: 1.2em; margin: 10px; border: none; border-radius: 10px;
           cursor: pointer; transition: background 0.3s; }
    .joystick { background: #4CAF50; color: white; }
    .follow   { background: #FF9800; color: white; }
    .active   { box-shadow: 0 0 20px rgba(0,0,0,0.3); }
    img { border: 1px solid #ccc; border-radius: 10px; margin-bottom: 20px; }
  </style>
</head>
<body>
  <h1>MAGAbot Control Panel</h1>
  <h3>🔴 Live Camera Feed</h3>
  <img src="/video_feed" width="640" height="480">
  <div id="status">Current mode: <strong id="modeDisplay">Stopped</strong></div>
  <button class="btn joystick" onclick="setMode('joystick')" id="joystickBtn">Joystick Mode</button>
  <button class="btn follow"   onclick="setMode('follow')"   id="followBtn">Follow Person</button>

  <script>
    let currentMode = 'stopped';
    function setMode(mode) {
      fetch('/mode', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({mode})
      })
      .then(r=>r.json())
      .then(j=>{
        if(j.ok) {
          currentMode = mode;
          updateUI();
        } else {
          alert('Mode switch failed: '+ j.error);
        }
      })
      .catch(e=>alert('Error: '+e));
    }
    function updateUI() {
      const names = { joystick:'Joystick Mode', follow:'Follow Person', stopped:'Stopped' };
      document.getElementById('modeDisplay').textContent = names[currentMode]||'Unknown';
      ['joystickBtn','followBtn'].forEach(id=> document.getElementById(id).classList.remove('active'));
      if(currentMode==='joystick') document.getElementById('joystickBtn').classList.add('active');
      if(currentMode==='follow')   document.getElementById('followBtn').classList.add('active');
    }
  </script>
</body>
</html>
"""

NO_ROBOT_PAGE = """
<!DOCTYPE html>
<html>
  <head>
    <title>No Robot</title>
    <style>
      body { font-family: sans-serif; text-align: center; margin-top: 100px; }
    </style>
  </head>
  <body>
    <h1>No desktop robot detected</h1>
    <p>Please positionate a robot on the platform to enable the control panel.</p>
  </body>
</html>
"""

# VIDEO-STREAMING 
def generate_frames(): # Generate frames for video streaming
    while True:
        img = picam2.capture_array()             # raw RGB numpy array
        # downscale to 160×120 or 320×240 first:
        small = cv2.resize(img, (160, 120))
        # encode to JPEG in one call
        ret, buf = cv2.imencode('.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if not ret:
            continue
        frame = buf.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        # no sleep() here—rely on the camera’s ~30 fps



@app.route('/video_feed') # Route for video feed 
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/') # Main route for the control panel (2 buttons)
def index():
    try:
        # Ask the ESP32 if the button is pressed
        resp = requests.get(ESP32_BTN_URL, timeout=0.2)
        data = resp.json()  # expects { "pressed": true } or { "pressed": false }

        if data.get('pressed', False):
            # Button is pressed → show the normal MAGAbot control panel
            return render_template_string(HTML_PAGE)
        else:
            # Button is NOT pressed → show “No desktop robot detected”
            return NO_ROBOT_PAGE

    except requests.RequestException:
        # If we cannot reach the ESP32, also show “No desktop robot detected”
        return NO_ROBOT_PAGE


def joystick_control_loop(): # Joystick control loop for manual control of MAGAbot

    left_speed, left_dir, right_speed, right_dir = 0, 0, 0, 0  # Initialize motor speeds and directions
    stop_joystick.clear()
    stop_motors() # Ensure motors are stopped initially
    
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    # local copies
    max_speed = 20
    dead_zone = 0.1
    exponential_factor = 2
    start_dead_zone = 0.25
    
    print("joystick mode")
    
    moved = False
    while not moved and not stop_joystick.is_set():
        pygame.event.pump()
        print("joystick not moved")
        ly = joystick.get_axis(1)
        ry = joystick.get_axis(4)
        stop_joystick.clear()
        print(ly)
        print(ry)
        
        if abs(ly) > start_dead_zone or abs(ry) > start_dead_zone:
            moved = True
            
            
            print("joystick moved")
        else:
            stop_motors()
        time.sleep(0.02)

    
    while not stop_joystick.is_set():
        pygame.event.pump()
        print("second while")
        # axes
        lx = joystick.get_axis(0)
        ly = joystick.get_axis(1)
        rx = joystick.get_axis(3)
        ry = joystick.get_axis(4)

        # dead-zone
        ly = 0 if abs(ly) < dead_zone else ly
        ry = 0 if abs(ry) < dead_zone else ry

        # scaling
        left_speed  = int(max_speed * (abs(ly) ** exponential_factor))
        right_speed = int(max_speed * (abs(ry) ** exponential_factor))

        # directions
        left_dir  = 0 if ly < 0 else 1 if ly > 0 else 0
        right_dir = 0 if ry < 0 else 1 if ry > 0 else 0

        # stop if near zero
        if abs(ly) < dead_zone:
            left_speed = 0; left_dir = 0
        if abs(ry) < dead_zone:
            right_speed = 0; right_dir = 0

        # send out
        send_command(left_speed, left_dir, right_speed, right_dir)

        time.sleep(0.01)

    # once stopped
    pygame.quit()

# FOLLOW-PERSON THREAD
def follow_person():      # Follow a person using the camera and SSD model
    global stop_follow
    MAX_SPEED = 30
    MIN_SPEED = 5
    CENTER_MARGIN = 20 
    detect_every = 3 
    LOSS_TIMEOUT = 2.0 
    TARGET_LABEL = 1
    CONF_THRESH = 0.5

    prev_error = 0.0
    prev_time = time.time()
    last_seen = 0
    frame_count = 0 
    detections = []
    prev_best_cx = None  #

    while not stop_follow.is_set(): 
        img = picam2.capture_array()
        img = cv2.resize(img, (WIDTH, HEIGHT))
        if img.shape[2] == 4:
            img = img[:, :, :3]
        now = time.time()
        best_cx, best_score = None, 0.0
        new_detection = False

        if frame_count % detect_every == 0:
            pil_img = Image.fromarray(img)
            inp = transform(pil_img).unsqueeze(0)

            with torch.no_grad():
                out = model(inp)[0]

            detections = [
                (box, scr) for box, lab, scr in zip(out["boxes"], out["labels"], out["scores"])
                if int(lab) == TARGET_LABEL and scr > CONF_THRESH
            ]

            if detections:
                new_detection = True

            for box, scr in detections:
                x1, y1, x2, y2 = map(int, box.tolist())
                cx = (x1 + x2) // 2
                if scr > best_score:
                    best_score = scr
                    best_cx = cx

        if best_cx is None and prev_best_cx and (now - last_seen < LOSS_TIMEOUT):
            best_cx = prev_best_cx
        elif best_cx is None:
            prev_best_cx = None

        if new_detection and best_cx is not None:
            last_seen = now
            prev_best_cx = best_cx

        if best_cx is not None:
            err = (WIDTH // 2) - best_cx
            dt = now - prev_time if now != prev_time else 1e-3
            prev_error, prev_time = err, now

            norm = abs(err) / (WIDTH // 2)
            speed = int(MAX_SPEED - (MAX_SPEED - MIN_SPEED) * norm)
            rot = int((MAX_SPEED - MIN_SPEED) * norm)

            if abs(err) < CENTER_MARGIN:
                vL = vR = speed
            elif err > 0:
                vL, vR = speed - rot, speed + rot
            else:
                vL, vR = speed + rot, speed - rot

            dirL = 0 if vL >= 0 else 1
            dirR = 0 if vR >= 0 else 1

            send_command(abs(vL), dirL, abs(vR), dirR)
        else:
            stop_motors()

        frame_count += 1
        time.sleep(0.04)

# MODE-SWITCHING
@app.route('/mode', methods=['POST'])
def switch_mode():
    global mode, joystick_thread, stop_joystick, follow_thread, stop_follow

    # 0) Don’t allow mode changes unless button still pressed
    try:
        resp = requests.get(ESP32_BTN_URL, timeout=0.5)
        if not resp.json().get("pressed", False):
            return jsonify(ok=False, error="Robot removed from the platform"), 403
    except requests.RequestException:
        return jsonify(ok=False, error="Cannot reach robot"), 503

    data = request.get_json() or {}
    new_mode = data.get('mode')

    # 1) Stop any running joystick thread
    if joystick_thread and joystick_thread.is_alive():
        stop_joystick.set()
        joystick_thread.join()
        joystick_thread = None
        stop_joystick.clear()

    # 2) Stop any running follow thread
    if follow_thread and follow_thread.is_alive():
        stop_follow.set()
        follow_thread.join()
        follow_thread = None
        stop_follow.clear()

    # 3) Always zero motors on mode change
    stop_motors()

    # 4) Start the requested mode
    mode = new_mode
    if mode == 'joystick':
        joystick_thread = threading.Thread(target=joystick_control_loop, daemon=True)
        joystick_thread.start()
    elif mode == 'follow':
        follow_thread = threading.Thread(target=follow_person, daemon=True)
        follow_thread.start()
    else:
        mode = 'stopped'

    return jsonify(ok=True)

if __name__ == '__main__':
    # background watcher
    monitor = threading.Thread(target=button_monitor, daemon=True)
    monitor.start()
    # background bumper monitor
    bumper = threading.Thread(target=bumper_monitor, daemon=True)   
    bumper.start()
    # now run Flask
    app.run(host='0.0.0.0', port=5000) 



