"""OpenCV-only template crop selector used by the CLI client."""
import cv2


class Template:
    @staticmethod
    def select_crop(image):
        selection = {"start": None, "crop": None}
        shown = image.copy()

        def mouse(event, x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                selection["start"] = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and selection["start"]:
                x0, y0 = selection["start"]
                x_min, x_max = sorted((x0, x))
                y_min, y_max = sorted((y0, y))
                if x_max > x_min and y_max > y_min:
                    selection["crop"] = (x_min, x_max, y_min, y_max)

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", mouse)
        print("Drag over the object, then press q. Esc cancels.", flush=True)
        try:
            while True:
                frame = shown.copy()
                if selection["crop"]:
                    x_min, x_max, y_min, y_max = selection["crop"]
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max),
                                  (0, 255, 0), 2)
                cv2.imshow("image", frame)
                key = cv2.waitKey(20) & 0xFF
                if key == 27:
                    return None
                if key == ord("q") and selection["crop"]:
                    return selection["crop"]
        finally:
            cv2.destroyAllWindows()
