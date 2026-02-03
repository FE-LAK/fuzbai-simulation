#!/usr/bin/env python3
import requests
import time
import math

TEAM_URL = "http://127.0.0.1:8080"  # Red team (8081 for blue)

def main():
    print(f"Connecting to {TEAM_URL}...")

    while True:
        try:
            # Get camera state
            response = requests.get(f"{TEAM_URL}/Camera/State")

            if response.status_code != 200:
                print(f"Server error: {response.status_code}")
                continue

            state = response.json()
            ball_y = state['camData'][0]['ball_y']
            print(f"{ball_y=:.3f}")

            # Send actions
            action = {
                "commands": [
                    {
                        "driveID": 1,  # Goalkeeper
                        "translationTargetPosition": (math.sin(time.time()) + 1) / 2,
                        "translationVelocity": 1.0,
                        "rotationTargetPosition": 0.05 * math.sin(10 * time.time()),
                        "rotationVelocity": 1.0,
                    },
                    {
                        "driveID": 4,  # Attacker
                        "translationTargetPosition": (math.sin(2 * time.time()) + 1) / 2,
                        "translationVelocity": 1.0,
                        "rotationTargetPosition": 0.05 * math.sin(20 * time.time()),
                        "rotationVelocity": 1.0,
                    }
                ]
            }

            requests.post(f"{TEAM_URL}/Motors/SendCommand", json=action)
            time.sleep(0.016)  # ~60Hz

        except requests.RequestException as e:
            print(f"Error: {e}")
            time.sleep(1)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
