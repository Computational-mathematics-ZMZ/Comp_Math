import cv2
import mediapipe as mp
from collections import deque, Counter
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

options = vision.HandLandmarkerOptions(
    base_options=python.BaseOptions(
        model_asset_path=r"C:\Users\SHAHED AL-TAIE\PycharmProjects\RobotProject\hand_landmarker.task"
    ),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=1
)

detector = vision.HandLandmarker.create_from_options(options)
camera = cv2.VideoCapture(0)

history = deque(maxlen=7)
frame_number = 0

while camera.isOpened():
    success, frame = camera.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = mp.Image(
        image_format=mp.ImageFormat.SRGB,
        data=rgb
    )

    frame_number += 1
    result = detector.detect_for_video(image, frame_number * 33)
    command = "No Hand"

    if result.hand_landmarks:
        hand = result.hand_landmarks[0]
        h, w, _ = frame.shape

        for point in hand:
            cv2.circle(
                frame,
                (int(point.x * w), int(point.y * h)),
                5, (0, 255, 0), -1
            )

        fingers = 0

        # Four fingers
        for tip, joint in [(8, 6), (12, 10), (16, 14), (20, 18)]:
            if hand[tip].y < hand[joint].y:
                fingers += 1

        # Thumb: check which hand MediaPipe detected
        label = result.handedness[0][0].category_name

        if label == "Right":
            if hand[4].x > hand[3].x:
                fingers += 1
        else:
            if hand[4].x < hand[3].x:
                fingers += 1

        commands = {
            5: "STOP",
            1: "FORWARD",
            2: "BACKWARD",
            3: "RIGHT",
            4: "LEFT"
        }

        detected = commands.get(fingers, "Unknown")
        history.append(detected)

        # Use the most common recent result
        command = Counter(history).most_common(1)[0][0]

    else:
        history.clear()

    cv2.putText(
        frame, command, (20, 50),
        cv2.FONT_HERSHEY_SIMPLEX, 1.2,
        (0, 255, 0), 3
    )

    cv2.imshow("Hand Control", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
detector.close()
cv2.destroyAllWindows()