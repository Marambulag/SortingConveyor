import cv2
import numpy as np
import time
import serial

# Color HSV thresholds and debug colors
colors_config = {
    "green": {
        "hsv_lower": (35, 70, 70),
        "hsv_upper": (85, 255, 255),
        "debug_color_bgr": (0, 255, 0)
    },
    "yellow": {
        "hsv_lower": (20, 100, 100),
        "hsv_upper": (35, 255, 255),
        "debug_color_bgr": (0, 255, 255)
    },
    "red": {
        "hsv_lower1": (0, 100, 100),
        "hsv_upper1": (10, 255, 255),
        "hsv_lower2": (160, 100, 100),
        "hsv_upper2": (180, 255, 255),
        "debug_color_bgr": (0, 0, 255)
    },
    "blue": {
        "hsv_lower": (95, 100, 100),
        "hsv_upper": (135, 255, 255),
        "debug_color_bgr": (255, 0, 0)
    }
}

# 2-bit binary codes for colors
color_to_binary = {
    "green": 0b00,   # 0 decimal
    "blue":  0b01,   # 1 decimal
    "red":   0b10,   # 2 decimal
    "yellow":0b11    # 3 decimal
}

ordered_colors = list(colors_config.keys())

# Morphological operation parameters
MORPH_KERNEL_SIZE = (3, 3)
OPENING_ITERATIONS = 2
DILATE_ITERATIONS = 3
DISTANCE_TRANSFORM_THRESHOLD_MULTIPLIER = 0.4
MIN_CONTOUR_AREA = 300
MAX_CENTROID_DISTANCE = 50  

# Tracking structures
object_trackers = {color: {} for color in ordered_colors}
object_next_id = {color: 0 for color in ordered_colors}
color_counts = {color: 0 for color in ordered_colors}

def get_color_mask(hsv_img, color_name):
    config = colors_config[color_name]
    if color_name == "red":
        lower1 = np.array(config["hsv_lower1"])
        upper1 = np.array(config["hsv_upper1"])
        lower2 = np.array(config["hsv_lower2"])
        upper2 = np.array(config["hsv_upper2"])
        mask1 = cv2.inRange(hsv_img, lower1, upper1)
        mask2 = cv2.inRange(hsv_img, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        lower = np.array(config["hsv_lower"])
        upper = np.array(config["hsv_upper"])
        mask = cv2.inRange(hsv_img, lower, upper)
    return mask

def separate_objects(mask):
    kernel = np.ones(MORPH_KERNEL_SIZE, np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=OPENING_ITERATIONS)
    sure_bg = cv2.dilate(opening, kernel, iterations=DILATE_ITERATIONS)
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist_transform, DISTANCE_TRANSFORM_THRESHOLD_MULTIPLIER * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    _, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown == 255] = 0
    img_for_watershed = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
    markers = cv2.watershed(img_for_watershed, markers)
    boxes = []
    for marker_id in np.unique(markers):
        if marker_id <= 1:
            continue
        component_mask = np.uint8(markers == marker_id) * 255
        contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                boxes.append((x, y, w, h))
    return boxes

# Initialize serial port
try:
    ser = serial.Serial('/dev/ttyUSB', 115200, timeout=1)  # Adjust to your serial port (e.g., 'COM3' on Windows)
    time.sleep(2) 
    print("Serial port opened")
except Exception as e:
    print(f"Could not open serial port: {e}")
    ser = None

# Open camera
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("Error: Cannot open camera index 2 (HBCAM)")
    exit()

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
center_line = frame_width // 2

print("HBCAM opened successfully. Press ESC to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name in ordered_colors:
        mask = get_color_mask(hsv, color_name)
        boxes = separate_objects(mask)

        new_objects = []
        for (x, y, w, h) in boxes:
            cX, cY = x + w // 2, y + h // 2
            side = "left" if cX < center_line else "right"
            new_objects.append({"centroid": (cX, cY), "side": side, "box": (x, y, w, h)})

        updated_ids = set()
        for obj in new_objects:
            cX, cY = obj["centroid"]
            min_dist = float("inf")
            matched_id = None

            for obj_id, tracked in object_trackers[color_name].items():
                tX, tY = tracked["centroid"]
                dist = np.sqrt((cX - tX) ** 2 + (cY - tY) ** 2)
                if dist < min_dist and dist < MAX_CENTROID_DISTANCE:
                    min_dist = dist
                    matched_id = obj_id

            if matched_id is not None:
                prev_side = object_trackers[color_name][matched_id]["side"]
                new_side = obj["side"]
                if prev_side == "right" and new_side == "left":
                    color_counts[color_name] += 1
                    msg = f"{color_name.upper()} ID {matched_id} crossed L->R | Total: {color_counts[color_name]}"
                    print(msg)
                    # Send 2-bit color code over serial port as a single byte
                    if ser and ser.is_open:
                        code = color_to_binary[color_name]
                        ser.write(bytes([code]))
                object_trackers[color_name][matched_id] = {
                    "centroid": obj["centroid"],
                    "side": new_side
                }
                updated_ids.add(matched_id)
            else:
                new_id = object_next_id[color_name]
                object_next_id[color_name] += 1
                object_trackers[color_name][new_id] = {
                    "centroid": obj["centroid"],
                    "side": obj["side"]
                }

            x, y, w, h = obj["box"]
            color_bgr = colors_config[color_name]["debug_color_bgr"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
            cv2.putText(frame, f"{color_name}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            cv2.circle(frame, obj["centroid"], 5, color_bgr, -1)

    # Draw center line for crossing
    cv2.line(frame, (center_line, 0), (center_line, frame_height), (0, 255, 255), 2)

    # Display counts
    y0 = 50
    for color_name in ordered_colors:
        text = f"{color_name.capitalize()} count: {color_counts[color_name]}"
        cv2.putText(frame, text, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors_config[color_name]["debug_color_bgr"], 2)
        y0 += 30

    cv2.imshow("Color Object Crossing Counter", frame)
    if cv2.waitKey(1) == 27:  # ESC key to quit
        break

cap.release()
cv2.destroyAllWindows()

if ser and ser.is_open:
    ser.close()
    print("Serial port closed")
