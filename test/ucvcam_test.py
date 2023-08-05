import cv2


def main():
    camera_num = 0  # 카메라 인덱스 (일반적으로 0부터 시작)

    # 카메라 열기
    cap = cv2.VideoCapture(camera_num)

    if not cap.isOpened():
        print("Failed to open camera.")
        return

    # 카메라 해상도 설정 (IMX291 카메라의 해상도는 1920x1080입니다.)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 이미지 캡처
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture image.")
            cap.release()
            return

        print(frame.shape)
        # 이미지 표시/
        # bgr_frame = cv2.cvtColor(frame[:,:,0], cv2.COLOR_GRAY2BGR)
        cv2.imshow("Captured Image", frame)
        # cv2.imwrite("asd.png",frame)
        cv2.waitKey(1)
    cv2.destroyAllWindows()

    # 카메라 닫기
    cap.release()


if __name__ == "__main__":
    main()