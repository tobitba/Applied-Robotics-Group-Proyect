from smc import load_config
import cv2
import mediapipe as mp
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.6, min_tracking_confidence=0.6)

while True:
    success, frame = cap.read()
    if not success:
        continue

    h, w, _ = frame.shape

    # Convert for mediapipe
    RGB_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(RGB_frame)

    # ------------------------------
    # Process hand detection
    # ------------------------------
    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            index_knuckle = hand_landmarks.landmark[6]  # index PIP
            index_tip = hand_landmarks.landmark[8]  # index tip

    # --------------------------------------
    # ALWAYS SHOW FRAME (even with no hands)
    # --------------------------------------
    cv2.imshow("capture image", frame)

    # Quit
    if cv2.waitKey(1) == ord("q"):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()

