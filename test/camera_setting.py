import cv2

# Function to update camera settings dynamically
def update_camera_settings(cap, brightness, contrast, saturation, sharpness, 
                           focus,afocus, exposure, aexposure,pexposure, aperture):
    # Set camera properties based on slider values
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_SATURATION, saturation)
    cap.set(cv2.CAP_PROP_SHARPNESS, sharpness)
    cap.set(cv2.CAP_PROP_FOCUS, focus)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, afocus)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, aexposure)
    cap.set(cv2.CAP_PROP_XI_EXP_PRIORITY, pexposure)
    cap.set(cv2.CAP_PROP_APERTURE, aperture)

# Initialize the camera
cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

# Check if camera is opened correctly
if not cap.isOpened():
    print("Error: Camera not found or unable to open.")
    exit()

# Create a window
cv2.namedWindow("Camera Settings")

# Create trackbars (sliders) for adjusting various settings
cv2.createTrackbar("Brightness", "Camera Settings", 128, 255, lambda x: None)  # Default 128
cv2.createTrackbar("Contrast", "Camera Settings", 128, 255, lambda x: None)    # Default 128
cv2.createTrackbar("Saturation", "Camera Settings", 128, 255, lambda x: None)  # Default 128
cv2.createTrackbar("Sharpness", "Camera Settings", 128, 255, lambda x: None)   # Default 128
cv2.createTrackbar("Focus", "Camera Settings", 0, 255, lambda x: None)  # Default 0
cv2.createTrackbar("AutoFocus", "Camera Settings", 1, 1, lambda x: None)  # Default 0
cv2.createTrackbar("Exposure", "Camera Settings", 250, 2047, lambda x: None)  # Default 250
cv2.createTrackbar("AutoExposure", "Camera Settings", 3, 3, lambda x: None)  # Default 250
cv2.createTrackbar("ProgramExposure", "Camera Settings", 0, 1, lambda x: None)  # Default 250
cv2.createTrackbar("Aperture", "Camera Settings", 0, 255, lambda x: None)  # Default 0

# Main loop for capturing video and adjusting settings
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Get current values from the trackbars
    brightness = cv2.getTrackbarPos("Brightness", "Camera Settings")
    contrast = cv2.getTrackbarPos("Contrast", "Camera Settings")
    saturation = cv2.getTrackbarPos("Saturation", "Camera Settings")
    sharpness = cv2.getTrackbarPos("Sharpness", "Camera Settings")
    focus = cv2.getTrackbarPos("Focus", "Camera Settings")
    afocus = cv2.getTrackbarPos("AutoFocus", "Camera Settings")
    exposure = cv2.getTrackbarPos("Exposure", "Camera Settings")
    aexposure = cv2.getTrackbarPos("AutoExposure", "Camera Settings")
    pexposure = cv2.getTrackbarPos("ProgramExposure", "Camera Settings")
    aperture = cv2.getTrackbarPos("Aperture", "Camera Settings")

    # Apply the settings to the camera
    update_camera_settings(cap, brightness, contrast, saturation, sharpness, 
                           focus,afocus, exposure, aexposure,pexposure, aperture)

    # Display the resulting frame
    cv2.imshow('Camera Feed', frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
