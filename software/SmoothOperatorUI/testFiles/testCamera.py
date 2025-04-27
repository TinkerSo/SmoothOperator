import cv2
from pyzbar.pyzbar import decode

def read_qr_code():
    # Open the default camera (0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Scanning for QR codes... Press 'q' to quit.")

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Convert to grayscale for better accuracy
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect and decode QR codes
        qr_codes = decode(gray)

        for qr_code in qr_codes:
            qr_data = qr_code.data.decode('utf-8')
            qr_rect = qr_code.rect

            # Draw rectangle around detected QR code
            cv2.rectangle(frame, (qr_rect.left, qr_rect.top), 
                          (qr_rect.left + qr_rect.width, qr_rect.top + qr_rect.height), 
                          (0, 255, 0), 2)

            # Display QR code data
            cv2.putText(frame, qr_data, (qr_rect.left, qr_rect.top - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            print(f"QR Code Detected: {qr_data}")

        # Show the video feed
        cv2.imshow("QR Code Scanner", frame)

        # Break loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    read_qr_code()
