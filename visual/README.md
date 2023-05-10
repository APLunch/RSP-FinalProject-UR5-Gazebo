# Vision Installation

## install dependencies
```bash
pip install opencv-python pycocotools matplotlib onnxruntime onnx Pillow ipykernel
```

## install torch
- if cuda
```bash
pip3 install torch torchvision torchaudio
```
- if cpu
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

## install grounded-sam

```bash
pip install git+https://github.com/IDEA-Research/GroundingDINO.git
pip install git+https://github.com/facebookresearch/segment-anything.git
```

## download models

### direct to the path
```bash
cd ./visual/igr_vision_services/models
```
### wget model checkpoints
```bash
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
wget https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```
