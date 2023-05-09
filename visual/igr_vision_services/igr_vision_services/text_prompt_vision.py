import os, sys

import numpy as np
import json
import torch
from PIL import Image

from torchvision import models

# Grounding DINO
import torchvision.transforms as T
import torch.nn.functional as F
from groundingdino.models import build_model
from groundingdino.util.slconfig import SLConfig
from groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap

# segment anything
from segment_anything import build_sam, SamPredictor 
import cv2
import numpy as np
import matplotlib.pyplot as plt

from ament_index_python.packages import get_package_share_directory

PACKAGE = "igr_vision_services"
ROOT = get_package_share_directory(PACKAGE)
CONFIG = "GroundingDINO_SwinT_OGC.py"

class TextPromptVision:
    def __init__(self):
        # params
        self.config_file = os.path.join(ROOT, 'config', CONFIG)  # change the path of the model config file
        self.grounded_checkpoint = os.path.join(ROOT, 'models', 'groundingdino_swint_ogc.pth')  # change the path of the model
        self.sam_checkpoint = os.path.join(ROOT, "models", "sam_vit_h_4b8939.pth")
        self.input_image = None
        self.text_prompt = None
        self.output_dir = "vision_outputs"
        self.box_threshold = 0.3
        self.text_threshold = 0.25

        # Use GPU if available
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        # self.device = torch.device("cpu")

        # load models
        self.cls_model = models.resnet101(pretrained=True)
        self.cls_model.eval()
        self.cls_model.to(self.device)
        self.dino_model = self.load_model()
        # self.sam_predictor = SamPredictor(build_sam(checkpoint=self.sam_checkpoint).to(self.device))
        self.sam_predictor = SamPredictor(build_sam(checkpoint=self.sam_checkpoint).to("cpu"))

        # member variables
        self.last_masks = None
        self.last_pred_phases = None

    def detect(self, image_list, text_prompt_list):
        assert len(image_list) == len(text_prompt_list), "The number of images must match the number of text prompts."

        text_prompt_list = ['.'.join(text_prompt) for text_prompt in text_prompt_list]
        print(text_prompt_list)

        # load images
        image_pil_list = [Image.fromarray(np.uint8(image_np)).convert("RGB") for image_np in image_list]
        transform = T.Compose(
            [
                T.Resize((800, 800)),
                T.ToTensor(),
                T.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )
        image_pil_resized_list = [transform(image_pil) for image_pil in image_pil_list]

        image_batch = torch.stack(image_pil_resized_list, dim=0)
        
        boxes_filt_list, pred_phrases_list = self.get_grounding_output(image_batch, text_prompt_list)

        final_boxes_filt_list = [
            torch.stack([
                torch.cat((box[:2] * torch.Tensor([image_pil.size[0], image_pil.size[1]]) - box[2:] * torch.Tensor([image_pil.size[0], image_pil.size[1]]) / 2, 
                        box[:2] * torch.Tensor([image_pil.size[0], image_pil.size[1]]) + box[2:] * torch.Tensor([image_pil.size[0], image_pil.size[1]]) / 2))
                for box in boxes_filt
            ])
            for image_pil, boxes_filt in zip(image_pil_list, boxes_filt_list)
        ]

        return final_boxes_filt_list, pred_phrases_list
    
    def segment(self, image_np, boxes, pred_phrases):
        boxes = boxes.to("cpu")
        self.sam_predictor.set_image(image_np)
        transformed_boxes = self.sam_predictor.transform.apply_boxes_torch(boxes, image_np.shape[:2])
        transformed_boxes = transformed_boxes.to("cpu")
        with torch.no_grad():
            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords = None,
                point_labels = None,
                boxes = transformed_boxes,
                multimask_output = False,
            )
        masks_save = masks.cpu().numpy()
        masks_save = masks_save.squeeze(axis=0)
        self.last_masks = (masks_save * 255).astype(np.uint8)
        pred_phrases = [phrase.split('(')[0] for phrase in pred_phrases]
        self.last_pred_phases = pred_phrases
        return masks
    
    def get_features(self, object):
        if (self.last_pred_phases is None):
            raise("No previous segmentation.")
        try:
            indices = [i for i, element in enumerate(self.last_pred_phases) if element == object]
        except ValueError:
            raise("Given text not in prompt.")
        centroids = []
        principal_axes = []

        for index in indices:
            binary_mask = self.last_masks[index]
    
            # Calculate the image moments
            moments = cv2.moments(binary_mask)

            # Calculate the centroid
            centroid_x = moments['m10'] / moments['m00']
            centroid_y = moments['m01'] / moments['m00']

            # Calculate the second-order central moments
            mu20 = moments['mu20'] / moments['m00']
            mu02 = moments['mu02'] / moments['m00']
            mu11 = moments['mu11'] / moments['m00']

            # Calculate the orientation angle (in radians)
            angle = 0.5 * np.arctan2(2 * mu11, mu20 - mu02)

            # Calculate the unit vector representing the principal axis
            centroids.append((centroid_x, centroid_y))
            principal_axes.append((np.cos(angle), np.sin(angle)))
        return centroids, principal_axes

    # utils
    def draw_detection(self, image, boxes_filt, pred_phrases):
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        for box, label in zip(boxes_filt, pred_phrases):
            self.show_box(box.numpy(), plt.gca(), label)
        plt.axis('off')
        plt.show()

    def draw_segmentation(self, image, boxes, masks, pred_phrases, save=False, output_name="grounded_sam_output.png"):
        # draw output image
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        for mask in masks:
            self.show_mask(mask.cpu().numpy(), plt.gca(), random_color=True)
        for box, label in zip(boxes, pred_phrases):
            self.show_box(box.numpy(), plt.gca(), label)
        
        plt.axis('off')
        if save:
            plt.savefig(
            os.path.join(self.output_dir, output_name), 
            bbox_inches="tight", dpi=300, pad_inches=0.0
            )
            self.save_mask_data(self.output_dir, masks, boxes, pred_phrases)
        plt.show()
    
    def draw_features(self, image, centroids, principal_axes):
        # Convert the image to color if it's grayscale
        if len(image.shape) == 2 or image.shape[2] == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        plt.figure(figsize=(10, 10))
        for centroid, principal_axis in zip(centroids, principal_axes):
            centroid_x, centroid_y = centroid

            # Draw the centroid as a small circle
            plt.scatter(centroid_x, centroid_y, s=50, c='g', marker='o')

            # Calculate the start and end points of the principal axis line
            length = 50  # Length of the principal axis line
            start_point = (centroid_x - principal_axis[0] * length, centroid_y - principal_axis[1] * length)
            end_point = (centroid_x + principal_axis[0] * length, centroid_y + principal_axis[1] * length)

            # Draw the principal axis line
            plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], 'r-', linewidth=2)

        # Display the image with the drawn features
        plt.imshow(image)
        plt.show()

    def load_model(self):
        args = SLConfig.fromfile(self.config_file)
        args.device = self.device
        model = build_model(args)
        checkpoint = torch.load(self.grounded_checkpoint, map_location=self.device)
        load_res = model.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
        model.to(self.device)
        model.eval()
        return model

    def get_grounding_output(self, images, caption_list, with_logits=True):
        assert len(caption_list) == images.size(0), "The number of captions must match the number of images."

        caption_list = [caption.lower().strip() + "." if not caption.lower().strip().endswith(".") else caption.lower().strip() for caption in caption_list]

        self.dino_model, images = self.dino_model.to(self.device), images.to(self.device)

        with torch.no_grad():
            outputs = self.dino_model(images, captions=caption_list)

        logits, boxes = outputs["pred_logits"].cpu().sigmoid(), outputs["pred_boxes"].cpu()
        filt_mask = logits.max(dim=2)[0] > self.box_threshold
        logits_filt_list, boxes_filt_list = logits[filt_mask], boxes[filt_mask]

        # get phrase
        tokenlizer = self.dino_model.tokenizer
        tokenized_list = [tokenlizer(caption) for caption in caption_list]

        # build pred
        pred_phrases_list = [
            [
                get_phrases_from_posmap(logit > self.text_threshold, tokenized, tokenlizer) + (f"({str(logit.max().item())[:4]})" if with_logits else "")
                for logit, box in zip(logits_filt, boxes_filt)
            ]
            for logits_filt, boxes_filt, tokenized in zip(logits_filt_list.split(filt_mask.sum(dim=1).tolist()), boxes_filt_list.split(filt_mask.sum(dim=1).tolist()), tokenized_list)
        ]

        return boxes_filt_list.split(filt_mask.sum(dim=1).tolist()), pred_phrases_list

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

    def save_mask_data(self, output_dir, mask_list, box_list, label_list):
        value = 0  # 0 for background

        mask_img = torch.zeros(mask_list.shape[-2:])
        for idx, mask in enumerate(mask_list):
            mask_img[mask.cpu().numpy()[0] == True] = value + idx + 1
        plt.figure(figsize=(10, 10))
        plt.imshow(mask_img.numpy())
        plt.axis('off')
        plt.savefig(os.path.join(output_dir, 'mask.png'), bbox_inches="tight", dpi=300, pad_inches=0.0)

        json_data = [{
            'value': value,
            'label': 'background'
        }]
        for label, box in zip(label_list, box_list):
            value += 1
            name, logit = label.split('(')
            logit = logit[:-1] # the last is ')'
            json_data.append({
                'value': value,
                'label': name,
                'logit': float(logit),
                'box': box.numpy().tolist(),
            })
        with open(os.path.join(output_dir, 'mask.json'), 'w') as f:
            json.dump(json_data, f)

    def resize_mask(self, mask, square_size=200, isbool=False):
        # Calculate the scaling factors for width and height
        height, width = mask.shape[:2]
        image = (mask * 255).astype(np.uint8)
        scale_factor = min(square_size / width, square_size / height)

        # Calculate the new width and height
        new_width = int(width * scale_factor)
        new_height = int(height * scale_factor)
        resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

        if not isbool:
            resized_mask = (resized_image > 127).astype(int)
        else:
            resized_mask = (resized_image > 127).astype(bool)

        return resized_mask


