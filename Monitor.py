import paho.mqtt.client as mqtt
from datetime import datetime
import time

# 1. Configuration
BROKER = "broker.hivemq.com"
TOPIC_RPM = "my_unique_test_999/sensor/rpm"
TOPIC_TEMP = "my_unique_test_999/sensor/temperature"
TOPIC_PH = "my_unique_test_999/sensor/ph"

# --- Controller Configuration (Targets & Gains) ---
PID_CONFIG = {
    TOPIC_TEMP: {
        "name": "TEMP", "target": 30.0, "Kp": 2.0, "Ki": 0.5, 
        "min_safe": 25.0, "max_safe": 35.0
    },
    TOPIC_RPM: {
        "name": "RPM",  "target": 700.0, "Kp": 0.5, "Ki": 0.1, 
        "min_safe": 300.0, "max_safe": 1000.0  # +/- 20 RPM safety range
    },
    TOPIC_PH: {
        "name": "pH",   "target": 5.0,   "Kp": 10.0, "Ki": 2.0, 
        "min_safe": 4.5,   "max_safe": 5.5
    }
}

# --- State Management ---
# Stores the integral sum and last timestamp for EACH sensor independently
controller_state = {
    TOPIC_TEMP: {"integral_sum": 0.0, "last_time": None},
    TOPIC_RPM:  {"integral_sum": 0.0, "last_time": None},
    TOPIC_PH:   {"integral_sum": 0.0, "last_time": None}
}

def on_connect(client, userdata, flags, rc):
    print(f"Connected to {BROKER} with result code {rc}")
    client.subscribe(TOPIC_RPM)
    client.subscribe(TOPIC_TEMP)
    client.subscribe(TOPIC_PH)
    print("Listening for bioreactor data...")

def on_message(client, userdata, msg):
    current_time = time.time()
    topic = msg.topic
    
    # Check if this topic is in our configuration
    if topic in PID_CONFIG:
        cfg = PID_CONFIG[topic]
        state = controller_state[topic]
        
        try:
            val = float(msg.payload.decode())
        except ValueError:
            return # Skip if not a number

        # --- PI CALCULATIONS ---
        # 1. Calculate Time Delta (dt)
        if state["last_time"] is None:
            dt = 0.0
        else:
            dt = current_time - state["last_time"]
        
        state["last_time"] = current_time

        # 2. Calculate Error (Target - Current)
        error = cfg["target"] - val

        # 3. Proportional Term
        p_term = cfg["Kp"] * error

        # 4. Integral Term
        # Only accumulate if time has actually passed (dt > 0)
        if dt > 0:
            state["integral_sum"] += error * dt
        i_term = cfg["Ki"] * state["integral_sum"]

        # 5. Total Control Output
        control_output = p_term + i_term

        # --- LOGGING & ALERTS ---
        print(f"[{cfg['name']}] Val: {val:.2f} | Err: {error:.2f} | Out: {control_output:.2f} (P:{p_term:.2f} I:{i_term:.2f})")

        # Safety Check
        if val < cfg["min_safe"] or val > cfg["max_safe"]:
            print(f"⚠️  {cfg['name']} ALERT: {val:.2f} is outside safety limits!")

        # Write to CSV
        with open("data_log.csv", "a") as file:
            # Format: Time, SensorName, Value, ControlOutput, P_Term, I_Term
            log_line = f"{datetime.now()},{cfg['name']},{val},{control_output:.2f},{p_term:.2f},{i_term:.2f}\n"
            file.write(log_line)

# Main Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("Monitor Connecting...")
client.connect(BROKER, 1883, 60)
client.loop_forever()
