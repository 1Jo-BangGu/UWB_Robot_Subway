import cv2

# 영상 스트림 URL
stream_url = 'http://192.168.213.19:5000/stream/global'

# OpenCV로 영상 스트림 열기
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print('스트림을 열 수 없습니다.')
    exit(1)
else:
    print('스트림 열기 성공')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print('프레임을 읽을 수 없습니다.')
        break

    cv2.imshow('Stream from 192.168.213.5000', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
