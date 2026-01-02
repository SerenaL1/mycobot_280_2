from ultralytics import YOLO
import torch
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def train_custom_yolo():
    """
    Initializes and trains a custom YOLOv8 model.
    """
    # 1. Select the base model
    # This will download yolov8m.pt if not present
    model = YOLO('yolov8m.pt')
    
    # 2. Check for GPU
    device = 0 if torch.cuda.is_available() else 'cpu'
    if device == 'cpu':
        logging.warning("CUDA not available. Training on CPU will be extremely slow.")
        
    logging.info(f"Starting training on device: {device}")

    try:
        # 3. Train the model
        # This is the core command
        results = model.train(
            data='/home/serena-liu/Desktop/yolotray/data.yaml',
            epochs=30,
            imgsz=640,
            batch=16,  # Adjust based on GPU VRAM. -1 auto-batches.
            device=device,
            name='yolov8m_custom_run_2', # Experiment name
            patience=10, # Stop training if no improvement after 20 epochs
            exist_ok=True # Overwrite existing experiment
        )
        
        logging.info(f"Training complete. Results saved to {results.save_dir}")
        logging.info(f"Best model saved at: {results.best}")

    except Exception as e:
        logging.error(f"An error occurred during training: {e}", exc_info=True)

if __name__ == '__main__':
    train_custom_yolo()
