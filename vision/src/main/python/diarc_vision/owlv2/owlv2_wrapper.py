
import numpy as np
import cv2
import torch
from typing import List

from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

class Owlv2:
    def __init__(self, device: str = "cpu"):
        self.device = device
        self.processor = AutoProcessor.from_pretrained("google/owlv2-base-patch16-ensemble")
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained("google/owlv2-base-patch16-ensemble").to(self.device)

    def detect(self, image_rgb: np.ndarray, labels: List[str], conf_thresh: float):
        inputs = self.processor(text=labels, images=image_rgb, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        target_sizes = torch.tensor([image_rgb.shape[:2]], device=self.device)  # (h, w)
        results = self.processor.post_process_object_detection(outputs, target_sizes=target_sizes)[0]

        boxes = results["boxes"].detach().cpu().numpy()  # (x1,y1,x2,y2)
        scores = results["scores"].detach().cpu().numpy()
        class_ids = results["labels"].detach().cpu().numpy()

        # if no detections
        if len(boxes) == 0:
            return np.zeros((0,4),dtype=np.float32), np.zeros((0,),dtype=np.float32), []

        # Perform NMS
        # The function returns the indices of the boxes to keep
        indices = cv2.dnn.NMSBoxes(bboxes=boxes, scores=scores, score_threshold=conf_thresh, nms_threshold=0.5)

        # Flatten the indices array if it's not empty
        if len(indices) > 0:
            indices = indices.flatten()

        return boxes[indices], scores[indices], [labels[i] for i in class_ids[indices]]

if __name__ == '__main__':
    # Test image path
    image_path = "/home/evan/Pictures/test.jpg"

    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image from {image_path}")
        exit(1)

    # Resize image to 640x480
    image = cv2.resize(image, (640, 480))

    owlv2 = Owlv2()
    boxes, scores, labels = owlv2.detect(image, ["face", "person"], "cpu", 0.2)

    # Draw detection results on image
    for box, score, label in zip(boxes, scores, labels):
        x1, y1, x2, y2 = box.astype(int)
        # Draw rectangle
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # Create label text with confidence score
        label_text = f"{label}: {score:.2f}"
        # Draw label background
        (text_width, text_height), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(image, (x1, y1 - text_height - 4), (x1 + text_width, y1), (0, 255, 0), -1)
        # Draw text
        cv2.putText(image, label_text, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Display the image
    cv2.imshow("Test Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
