import cv2

img = cv2.imread("test.jpeg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

dicts = [
    cv2.aruco.DICT_4X4_50,
    cv2.aruco.DICT_5X5_50,
    cv2.aruco.DICT_5X5_100,
    cv2.aruco.DICT_6X6_250
]

for d in dicts:
    aruco_dict = cv2.aruco.getPredefinedDictionary(d)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        print("Detected with:", d, "IDs:", ids.flatten())
