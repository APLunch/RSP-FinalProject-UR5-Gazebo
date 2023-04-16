import os

import time
import numpy as np
import json
import torch
from PIL import Image

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

# test
import skimage

class ZeroShotVision:
    def __init__(self):
        # params
        self.config_file = "config/GroundingDINO_SwinT_OGC.py"  # change the path of the model config file
        self.grounded_checkpoint = "groundingdino_swint_ogc.pth"  # change the path of the model
        self.sam_checkpoint = "sam_vit_h_4b8939.pth"
        self.input_image = None
        self.text_prompt = None
        self.output_dir = ""
        self.box_threshold = 0.3
        self.text_threshold = 0.25

        # Use GPU if available
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        # load models
        self.dino_model = self.load_model()
        self.sam_predictor = SamPredictor(build_sam(checkpoint=self.sam_checkpoint).to(self.device))
        # self.sam_predictor = SamPredictor(build_sam(checkpoint=self.sam_checkpoint).to("cpu"))
        
    def detect(self, image_list, text_prompt_list):
        assert len(image_list) == len(text_prompt_list), "The number of images must match the number of text prompts."

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

        final_boxes_filt_list = []
        for image_pil, boxes_filt in zip(image_pil_list, boxes_filt_list):
            size = image_pil.size
            H, W = size[1], size[0]
            
            # Scale the bounding boxes
            for i in range(boxes_filt.size(0)):
                boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])

            # Recalculate box center and dimensions
            for i in range(boxes_filt.size(0)):
                boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
                boxes_filt[i][2:] += boxes_filt[i][:2]

            final_boxes_filt_list.append(boxes_filt)

        return final_boxes_filt_list, pred_phrases_list
    
    def segment(self, image_np, boxes):
        # boxes = boxes.to("cpu")
        self.sam_predictor.set_image(image_np)
        transformed_boxes = self.sam_predictor.transform.apply_boxes_torch(boxes, image_np.shape[:2])
        # transformed_boxes = transformed_boxes.to("cpu")
        with torch.no_grad():
            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords = None,
                point_labels = None,
                boxes = transformed_boxes,
                multimask_output = False,
            )
        return masks
    
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
        plt.show()
        if save:
            plt.savefig(
            os.path.join(self.output_dir, output_name), 
            bbox_inches="tight", dpi=300, pad_inches=0.0
            )
            self.save_mask_data(self.output_dir, masks, boxes, pred_phrases)
    
    # utils
    def load_model(self):
        args = SLConfig.fromfile(self.config_file)
        args.device = self.device
        model = build_model(args)
        # checkpoint = torch.load(model_checkpoint_path, map_location="cpu")
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
        logits_filt_list, boxes_filt_list = [logits[i][filt_mask[i]] for i in range(logits.size(0))], [boxes[i][filt_mask[i]] for i in range(boxes.size(0))]

        # get phrase
        tokenlizer = self.dino_model.tokenizer
        tokenized_list = [tokenlizer(caption) for caption in caption_list]

        # build pred
        pred_phrases_list = [
            [
                get_phrases_from_posmap(logit > self.text_threshold, tokenized, tokenlizer) + (f"({str(logit.max().item())[:4]})" if with_logits else "")
                for logit, box in zip(logits_filt, boxes_filt)
            ]
            for logits_filt, boxes_filt, tokenized in zip(logits_filt_list, boxes_filt_list, tokenized_list)
        ]

        return boxes_filt_list, pred_phrases_list

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

if __name__=="__main__":
    vision = ZeroShotVision()
    # Download sample image
    image1 = skimage.data.astronaut()

    image2 = skimage.data.coffee()

    image3 = Image.open("test_imgs/balls.jpg")
    image3 = np.array(image3)

    text_prompt1 = "human face.rocket.nasa badge.star-spangled banner"
    text_prompt2 = "coffee mug.plate.spoon"
    text_prompt3 = "box.black ball"

    start_time = time.time()
    boxes_filt_list, pred_phrases_list = vision.detect([image1, image2, image3], [text_prompt1, text_prompt2, text_prompt3])
    print(time.time()-start_time)

    vision.draw_detection(image1, boxes_filt_list[0], pred_phrases_list[0])
    vision.draw_detection(image2, boxes_filt_list[1], pred_phrases_list[1])
    vision.draw_detection(image3, boxes_filt_list[2], pred_phrases_list[2])


    # masks = vision.segment(image1, boxes_filt_list[0])
    # vision.draw_segmentation(image1, boxes_filt_list[0], masks, pred_phrases_list[0])

    # masks = vision.segment(image2, boxes_filt_list[1])
    # vision.draw_segmentation(image2, boxes_filt_list[1], masks, pred_phrases_list[1])

    masks = vision.segment(image3, boxes_filt_list[2])
    vision.draw_segmentation(image3, boxes_filt_list[2], masks, pred_phrases_list[2])