from fastapi import FastAPI, Response, Request
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import paho.mqtt.client as mqtt
import threading

app = FastAPI()

# MQTT Broker Configuration
MQTT_BROKER = "localhost"  # Use "10.7.233.217" if running on a different machine
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/cam_0"
MQTT_COMMAND_TOPIC = "esp32/cam_commands"

# Variable to store the latest image
latest_image = None
image_lock = threading.Lock()

class Command(BaseModel):
    command: str

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    global latest_image
    if msg.topic == MQTT_TOPIC:
        with image_lock:
            latest_image = msg.payload
        print(f"Received image of size {len(msg.payload)} bytes")
    elif msg.topic == MQTT_COMMAND_TOPIC:
        print(f"Received command: {msg.payload.decode()}")

def mqtt_loop():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()

# Start MQTT client in a separate daemon thread
mqtt_thread = threading.Thread(target=mqtt_loop)
mqtt_thread.daemon = True
mqtt_thread.start()

# Enhanced HTML frontend with Bootstrap and JavaScript for arrow keys
html_content = """
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-CAM Live Stream</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
    <style>
        body { 
            background-color: #f8f9fa; 
            display: flex; 
            flex-direction: column; 
            align-items: center; 
            padding-top: 50px;
        }
        .container { 
            max-width: 800px; 
            text-align: center; 
        }
        img { 
            width: 100%; 
            border-radius: 10px; 
            box-shadow: 0 4px 8px rgba(0,0,0,0.2); 
        }
        .header { 
            margin-bottom: 20px; 
        }
    </style>
    <script>
        function updateImage() {
            var img = document.getElementById("cam_image");
            img.src = "/latest_image?" + new Date().getTime();
        }
        setInterval(updateImage, 100); // Update every 100 ms

        document.addEventListener('keydown', function(event) {
            let command = '';
            switch(event.key) {
                case 'ArrowUp':
                    command = 'UP';
                    break;
                case 'ArrowDown':
                    command = 'DOWN';
                    break;
                case 'ArrowLeft':
                    command = 'LEFT';
                    break;
                case 'ArrowRight':
                    command = 'RIGHT';
                    break;
                default:
                    return; // Ignore other keys
            }
            fetch('/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ command: command })
            });
        });

        document.addEventListener('keyup', function(event) {
            // Send "STOP" command 5 times
            for (let i = 0; i < 5; i++) {
                setTimeout(() => {
                    fetch('/command', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({ command: 'STOP' })
                    });
                }, i * 50); // 50 ms interval between each command
            }
        });
    </script>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1 class="mb-3">ESP32-CAM Live Stream</h1>
            <p class="text-muted">Real-time video feed from your ESP32-CAM device</p>
            <p>Use arrow keys on your keyboard to send commands.</p>
        </div>
        <div class="card">
            <img id="cam_image" src="/latest_image" alt="Camera Feed" class="card-img-top">
            <div class="card-body">
                <p class="card-text">Refreshing every 100 ms</p>
            </div>
        </div>
    </div>
    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
def read_root():
    return html_content

@app.get("/latest_image")
def get_latest_image():
    with image_lock:
        if latest_image:
            return Response(content=latest_image, media_type="image/jpeg")
        else:
            return Response(content=b"", media_type="image/jpeg")

@app.post("/command")
def send_command(cmd: Command):
    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.publish(MQTT_COMMAND_TOPIC, cmd.command)
    client.disconnect()
    return {"status": "Command sent"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8002)
