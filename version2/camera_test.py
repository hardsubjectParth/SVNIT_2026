import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise RuntimeError("Camera not detected")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("IMX219", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
