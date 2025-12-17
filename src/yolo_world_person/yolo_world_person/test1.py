import cv2

def test_camera_indices():
    for index in range(5):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            print(f"카메라가 인덱스 {index}에서 열렸습니다")
            ret, frame = cap.read()
            if ret:
                print(f"프레임 캡처 성공: 해상도 {frame.shape}")
            cap.release()
        else:
            print(f"인덱스 {index}는 열 수 없습니다.")

if __name__ == "__main__":
    test_camera_indices()