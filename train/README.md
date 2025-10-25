## 🎯 YOLO Training Environment

This folder provides a Python environment to quickly train YOLO models using the `ultralytics` library.
It uses uv for package management and virtual environments.

## 📋 Requirements

### Prerequisites:

- Python: v3.13 or newer
- uv: The Python package installer and resolver.

### Project Dependencies (uv managed):

- opencv-python >= 4.12.0.88
- ultralytics >= 8.3.204

## 🚀 Training

### 1. Prepare the Dataset

Place your dataset folder (in YOLO format) inside the train/dataset/ directory.
Your dataset must contain a data.yaml file that points to the train, valid, and test subfolders.

#### Expected Directory Structure:
.
├── train/
│   └── dataset/      
│       ├── data.yaml   
│       ├── train/      
│       ├── valid/     
│       └── test/     
│
├── train.py
├── detect.py
└── ... (other project files)

### 2. Install Dependencies

Simply run:
`uv sync`

### 3. Start training

To begin training, run the `train.py` script using uv:
`uv run train.py`

Note: Make sure the paths inside the script are correct.

### 4. Check the Results

The training results (model weights, graphs, and logs) will be saved in the runs/detect/ folder (e.g., runs/detect/train1/, runs/detect/train2/, etc.).

## 🧪 How to Test the Model

After training, you can use the detect.py script to run inference.

- Copy the Model Weights file in your latest training results folder. Copy it to the project's root directory.
- Replace trainXXX with the name of your training run folder (e.g., train1, train2):

`cp ./runs/detect/train1/weights/best.pt ./best.pt`

Run the detection script:
`uv run detect.py`

Note: Make sure the paths inside the script are correct. 
