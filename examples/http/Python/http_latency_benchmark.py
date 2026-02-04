import requests
import time

def requests_thread():
    session = requests.Session()

    # Exponential moving average factor
    alpha = 0.0001
    smoothed = 0.0
    while True:
        try:
            # Measure round-trip time for camera state request
            start = time.perf_counter()
            response = session.get("http://127.0.0.1:8080/Camera/State", timeout=5)
            response.raise_for_status()

            # We must Parse the JSON as it's part of the real-world overhead
            state = response.json()

            elapsed_micros = (time.perf_counter() - start) * 1_000_000

            # Update smoothed latency with exponential moving average
            smoothed += alpha * (elapsed_micros - smoothed)

            # Extract some data to ensure the request was valid
            ball_x = state['camData'][0]['ball_x']
            ball_y = state['camData'][0]['ball_y']

            print(f"x={ball_x:.3f} y={ball_y:.3f} perf={smoothed:.3f}")

        except requests.exceptions.RequestException as e:
            print(f"Error: {e}")
            time.sleep(1)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    requests_thread()
