import paho.mqtt.client as mqtt
import time
import random

# 1. Configuration
BROKER = "broker.hivemq.com"
TOPIC_RPM = "my_unique_test_999/sensor/rpm"
TOPIC_TEMP = "my_unique_test_999/sensor/temperature"
TOPIC_PH = "my_unique_test_999/sensor/ph"

client = mqtt.Client()
print("Simulator connecting to broker...")
client.connect(BROKER, 1883, 60)
client.loop_start()
print("Simulator connected! Sending data...")

# Initialize variables (Start near targets)
# Targets: Temp=30, RPM=130, pH=5.0
current_temp = 30.0 
current_ph = 5.0
current_rpm = 700.0

while True:
    # 2. Update simulated values (Random Walk)
    # Drifts slightly to force the PI controller to react
    current_temp += random.uniform(-0.5, 0.5) 
    current_ph   += random.uniform(-0.05, 0.05)
    current_rpm  += random.uniform(-15, 15)

    # Keep values within physical limits (clamping)
    current_temp = max(20, min(40, current_temp))
    current_ph   = max(3, min(7, current_ph))
    current_rpm  = max(0, min(300, current_rpm))

    # 3. Publish
    client.publish(TOPIC_RPM, round(current_rpm, 1))
    client.publish(TOPIC_TEMP, round(current_temp, 2))
    client.publish(TOPIC_PH, round(current_ph, 2))

    print(f"Sent: RPM={current_rpm:.1f}, Temp={current_temp:.2f}, pH={current_ph:.2f}")
    
    # Wait 2 seconds
    time.sleep(2)
