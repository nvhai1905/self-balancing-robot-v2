import cv2
import mediapipe as mp
import numpy as np
import socket
import struct

# TCP server setup
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9999   # Choose any free port

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print(f"Server listening on {HOST}:{PORT}...")
conn, addr = server_socket.accept()
print(f"Client connected from {addr}")

# Khởi tạo MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Mở webcam
cap = cv2.VideoCapture(0)

def count_fingers(landmarks):
    # Danh sách các điểm mốc cho đầu ngón tay và khớp
    finger_tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
    finger_pips = [3, 6, 10, 14, 18]  # Các khớp tương ứng
    count = 0

    # Đếm các ngón tay (trừ ngón cái)
    for tip, pip in zip(finger_tips[1:], finger_pips[1:]):
        if landmarks[tip].y < landmarks[pip].y:  # Đầu ngón tay cao hơn khớp
            count += 1

    # Xử lý ngón cái (dùng khoảng cách hoặc góc)
    thumb_tip = landmarks[4]
    thumb_ip = landmarks[3]
    wrist = landmarks[0]
    if (thumb_tip.x - wrist.x) > 0.1:  # Điều kiện cho ngón cái (tùy chỉnh)
        count += 1

    return count

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Chuyển đổi sang RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)

        # Vẽ điểm mốc và đếm ngón tay
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Đếm số ngón tay
                num_fingers = count_fingers(hand_landmarks.landmark)
                
                # Hiển thị số ngón tay trên màn hình
                cv2.putText(frame, f'Fingers: {num_fingers}', (10, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # Ghi số ngón tay ra file để Winform đọc
                with open("gesture.txt", "w") as f:
                   f.write(str(num_fingers))

        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        data = jpeg.tobytes()
        # Send length of data first, then the data itself
        try:
            conn.sendall(struct.pack(">L", len(data)) + data)
        except Exception as e:
            print("Client disconnected.")
            break

        # Hiển thị khung hình
        cv2.imshow('Hand Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Giải phóng tài nguyên
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    conn.close()
    server_socket.close()