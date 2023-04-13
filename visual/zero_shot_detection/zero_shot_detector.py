from transformers import OwlViTProcessor, OwlViTForObjectDetection
from segment_anything import build_sam, SamPredictor 
import skimage
import numpy as np
from PIL import Image
import torch
import matplotlib.pyplot as plt
import time
import cv2

from transformers.image_utils import ImageFeatureExtractionMixin


class ZeroShotDetector:
    def __init__(self):
        ## OWL ViT
        self.mixin = ImageFeatureExtractionMixin()
        # Use GPU if available
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        self.model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")
        self.processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
        self.image_size = self.model.config.vision_config.image_size
        # Set model in evaluation mode
        self.model.to(self.device)
        self.model.eval()
        ## SAM
        self.sam_checkpoint = "sam_vit_h_4b8939.pth"
        self.predictor = SamPredictor(build_sam(checkpoint=self.sam_checkpoint).to(self.device))
        # self.predictor = SamPredictor(build_sam(checkpoint=self.sam_checkpoint))

    def detect(self, images, text_queries):
        inputs = self.processor(text=text_queries, images=images, return_tensors="pt").to(self.device)
        # Get predictions
        with torch.no_grad():
            outputs = self.model(**inputs)

        ## convert results back to original sizes
        # Target image sizes (height, width) to rescale box predictions [batch_size, 2]
        target_sizes = torch.Tensor([img.size[::-1] for img in images]).to(self.device)

        # Convert outputs (bounding boxes and class logits) to COCO API
        results = self.processor.post_process(outputs=outputs, target_sizes=target_sizes)

        boxes_pool = [results[i]["boxes"] for i in range(len(images))]
        scores_pool = [results[i]["scores"] for i in range(len(images))]
        labels_pool = [results[i]["labels"] for i in range(len(images))]

        return labels_pool, scores_pool, boxes_pool
    
    def segment(self, image, boxes):
        self.predictor.set_image(image)
        transformed_boxes = self.predictor.transform.apply_boxes_torch(boxes, image.shape[:2])
        with torch.no_grad():
            masks, _, _ = self.predictor.predict_torch(
                point_coords = None,
                point_labels = None,
                boxes = transformed_boxes,
                multimask_output = False,
            )
        return masks


    def draw_prediction(self, image, text_queries, labels, scores, boxes):
        image = np.array(image)
        color_text = [0, 0, 255]
        # color_bbox = [51, 51, 255]
        color_bbox = [102, 204, 0]
        color_bg = [255, 255, 255]
        image_s = image.copy()
        pad = 30
        image_s = cv2.copyMakeBorder(image_s, pad, pad, pad, pad, cv2.BORDER_CONSTANT,
            value=color_bg)
        image_s = cv2.cvtColor(image_s, cv2.COLOR_BGR2RGB)
        # Threshold to eliminate low probability predictions
        score_threshold = 0.1
        for score, box, label in zip(scores, boxes, labels):
            if score < score_threshold:
                continue
            box = [round(i, 2) for i in box.tolist()]
            x0, y0, x1, y1 = box

            cv2.rectangle(image_s, (int(x0)+pad, int(y0)+pad), (int(x1)+pad, int(y1)+pad), color_bbox, thickness=2)
            # For the text background
            # Finds space required by the text so that we can put a background with that amount of width.
            (w_t, h_t), _ = cv2.getTextSize(f"{text_queries[label]}: {score:1.2f}", cv2.FONT_HERSHEY_PLAIN, 1.0, 1)
            image_s = cv2.rectangle(image_s, (int(x0)+pad, int(y0 - 20)+pad), (int(x0 + w_t)+pad, int(y0)-2+pad), color_bg, -1)

            # Prints the text.    
            image_s = cv2.putText(image_s, f"{text_queries[label]}: {score:1.2f}", (int(x0)+pad, int(y0)-5+pad),
                                cv2.FONT_HERSHEY_PLAIN, 1.0, color_text, 1)
            
        cv2.imshow('Detection', image_s)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def plot_predictions(self, input_image, text_queries, outputs):
        input_image = self.mixin.resize(input_image, self.image_size)
        input_image = np.asarray(input_image).astype(np.float32) / 255.0
        # Threshold to eliminate low probability predictions
        score_threshold = 0.1

        # Get prediction logits
        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()

        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        boxes = outputs["pred_boxes"][0].cpu().detach().numpy()

        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.imshow(input_image, extent=(0, 1, 1, 0))
        ax.set_axis_off()

        for score, box, label in zip(scores, boxes, labels):
            if score < score_threshold:
                continue

            cx, cy, w, h = box
            ax.plot([cx-w/2, cx+w/2, cx+w/2, cx-w/2, cx-w/2],
                    [cy-h/2, cy-h/2, cy+h/2, cy+h/2, cy-h/2], "r")
            ax.text(
                cx - w / 2,
                cy + h / 2 + 0.015,
                f"{text_queries[label]}: {score:1.2f}",
                ha="left",
                va="top",
                color="red",
                bbox={
                    "facecolor": "white",
                    "edgecolor": "red",
                    "boxstyle": "square,pad=.3"
                })
        plt.show()

    def show_mask(self, mask, ax, random_color=False):
        if random_color:
            color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
        else:
            color = np.array([30/255, 144/255, 255/255, 0.6])
        h, w = mask.shape[-2:]
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        ax.imshow(mask_image)

    def show_box(self, box, ax, label):
        x0, y0 = box[0], box[1]
        w, h = box[2] - box[0], box[3] - box[1]
        ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2)) 
        ax.text(x0, y0, label)

    def draw_segmentation(self, masks, text_queries, scores, boxes, labels):
        boxes = self.predictor.transform.apply_boxes_torch(boxes, image.shape[:2])
        # draw output image
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        for mask in masks:
            self.show_mask(mask.cpu().numpy(), plt.gca(), random_color=True)
        for score, box, label in zip(scores, boxes, labels):
            box = [round(i, 2) for i in box.tolist()]
            self.show_box(np.array(box), plt.gca(), f"{text_queries[label]}: {score:1.2f}")
        plt.show()

if __name__=="__main__":
    detector = ZeroShotDetector()
    # Download sample image
    image = skimage.data.astronaut()
    image_pil = Image.fromarray(np.uint8(image)).convert("RGB")

    image2 = skimage.data.coffee()
    image2_pil = Image.fromarray(np.uint8(image2)).convert("RGB")

    text_queries = ["human face", "rocket", "nasa badge", "star-spangled banner"]
    text_queries2 = ["coffee mug", "plate", "spoon"]
    start_time = time.time()
    labels_pool, scores_pool, boxes_pool = detector.detect([image_pil, image2_pil], [text_queries, text_queries2])
    print(time.time()-start_time)
    detector.draw_prediction(image, text_queries, labels_pool[0], scores_pool[0], boxes_pool[0])
    detector.draw_prediction(image2, text_queries2, labels_pool[1], scores_pool[1], boxes_pool[1])

    masks = detector.segment(image, boxes_pool[0])
    detector.draw_segmentation(masks, text_queries, scores_pool[0], boxes_pool[0], labels_pool[0])

