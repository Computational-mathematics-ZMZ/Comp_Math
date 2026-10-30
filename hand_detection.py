
import cv2
import mediapipe as mp
from collections import deque, Counter
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import os

# ==============================
# Project paths
# ==============================
PROJECT_DIR = r"D:\Drone_Project\Matlab_1"
MODEL_PATH = os.path.join(PROJECT_DIR, "hand_landmarker.task")
COMMAND_FILE = os.path.join(PROJECT_DIR, "command.txt")

# Hand height thresholds
UP_THRESHOLD = 0.38
DOWN_THRESHOLD = 0.62

# ==============================
# Check model file
# ==============================
if not os.path.isfile(MODEL_PATH):
    raise FileNotFoundError(
        f"Model file not found: {MODEL_PATH}"
    )

# ==============================
# MediaPipe setup
# ==============================
options = vision.HandLandmarkerOptions(
    base_options=python.BaseOptions(
        model_asset_path=MODEL_PATH
    ),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=1
)

detector = vision.HandLandmarker.create_from_options(options)
camera = cv2.VideoCapture(0)

history = deque(maxlen=5)
frame_number = 0
last_command = "STOP"


# ==============================
# Count raised fingers
# ==============================
def count_fingers(hand, handedness):
    fingers = 0

    # Index, middle, ring and little fingers
    for tip, joint in [
        (8, 6),
        (12, 10),
        (16, 14),
        (20, 18)
    ]:
        if hand[tip].y < hand[joint].y:
            fingers += 1

    # Thumb
    if handedness == "Right":
        if hand[4].x > hand[3].x:
            fingers += 1
    else:
        if hand[4].x < hand[3].x:
            fingers += 1

    return fingers


# ==============================
# Main camera loop
# ==============================
try:
    if not camera.isOpened():
        raise RuntimeError(
            "Cannot open camera. Check camera permissions."
        )

    while camera.isOpened():
        success, frame = camera.read()

        if not success:
            print("Cannot read camera frame.")
            break

        # Mirror the camera
        frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb
        )

        frame_number += 1
        timestamp_ms = frame_number * 33

        result = detector.detect_for_video(
            image,
            timestamp_ms
        )

        command = "No Hand"
        height_label = ""

        # ==============================
        # Hand detection
        # ==============================
        if result.hand_landmarks:
            hand = result.hand_landmarks[0]
            h, w, _ = frame.shape

            # Draw hand landmarks
            for point in hand:
                cv2.circle(
                    frame,
                    (int(point.x * w), int(point.y * h)),
                    4,
                    (0, 255, 0),
                    -1
                )

            handedness = (
                result.handedness[0][0].category_name
            )

            fingers = count_fingers(
                hand,
                handedness
            )

            # Average palm position
            palm_y = sum(
                hand[i].y
                for i in [0, 5, 9, 13, 17]
            ) / 5.0

            # ==============================
            # Open palm: altitude control
            # ==============================
            if fingers == 5:

                if palm_y < UP_THRESHOLD:
                    detected = "UP"
                    height_label = "Hand high: UP"

                elif palm_y > DOWN_THRESHOLD:
                    detected = "DOWN"
                    height_label = "Hand low: DOWN"

                else:
                    detected = "STOP"
                    height_label = "Open palm: STOP"

            # ==============================
            # Finger gestures: direction
            # ==============================
            else:
                commands = {
                    1: "FORWARD",
                    2: "BACKWARD",
                    3: "RIGHT",
                    4: "LEFT"
                }

                detected = commands.get(
                    fingers,
                    "STOP"
                )

                height_label = (
                    f"Raised fingers: {fingers}"
                )

            # Smooth commands to reduce flickering
            history.append(detected)

            command = Counter(
                history
            ).most_common(1)[0][0]

        else:
            history.clear()

        # ==============================
        # Write command for MATLAB
        # ==============================
        if command != last_command:
            with open(
                COMMAND_FILE,
                "w",
                encoding="utf-8"
            ) as file:
                file.write(command)

            last_command = command

        # ==============================
        # Display information
        # ==============================
        cv2.putText(
            frame,
            f"Command: {command}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            height_label,
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2
        )

        cv2.putText(
            frame,
            "Press Q to quit",
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            "Drone Hand Control",
            frame
        )

        # Press Q to exit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    # Send STOP before closing
    try:
        with open(
            COMMAND_FILE,
            "w",
            encoding="utf-8"
        ) as file:
            file.write("STOP")
    except OSError:
        pass

    camera.release()
    detector.close()
    cv2.destroyAllWindows()

