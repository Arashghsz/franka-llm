import os
import yaml
from pathlib import Path
from ultralytics import YOLO


def create_dataset_config(data_dir: str = '/home/arash/franka-multiagent-manipulation/datasets/battery_assembly'):
    """Create YOLO dataset configuration"""
    
    config = {
        'path': data_dir,
        'train': 'images/train',
        'val': 'images/val',
        'test': 'images/test',
        'nc': 27,  # Number of classes
        'names': [
            'battery_aa', 'battery_aaa', 'battery_9v', 'battery_coin', 'battery_alkaline',
            'battery_spring', 'battery_contact', 'battery_terminal', 'contact_pin',
            'pcb', 'pcb_track', 'resistor', 'capacitor', 'diode', 'led', 'connector',
            'screw', 'bolt', 'nut', 'washer', 'spacer', 'clip', 'rivet',
            'screwdriver', 'wrench', 'pliers', 'drill_bit', 'soldering_iron',
            'wire', 'cable', 'housing', 'cover', 'bracket', 'fastener'
        ]
    }
    
    # Save config
    config_path = os.path.join(data_dir, 'data.yaml')
    os.makedirs(data_dir, exist_ok=True)
    
    with open(config_path, 'w') as f:
        yaml.dump(config, f)
    
    print(f"✓ Dataset config created at {config_path}")
    return config_path


def train_model(data_config: str, model_name: str = 'yolov8x', epochs: int = 100):
    """Fine-tune YOLOv8 on battery assembly dataset"""
    
    # Load base model
    model = YOLO(f'{model_name}.pt')
    
    # Train
    print("=" * 60)
    print("FINE-TUNING YOLOv8 on Battery Assembly Dataset")
    print("=" * 60)
    
    results = model.train(
        data=data_config,
        epochs=epochs,
        imgsz=640,
        batch=16,
        patience=20,
        device=0,  # GPU device
        project='/home/arash/franka-multiagent-manipulation/runs/detect',
        name='battery_assembly',
        plots=True,
        save=True,
        cache=True,
    )
    
    print("\n" + "=" * 60)
    print("TRAINING COMPLETE")
    print("=" * 60)
    print(f"Best model: {results.save_dir}/weights/best.pt")
    
    return results


def export_model(model_path: str):
    """Export model to ONNX/TensorRT for inference optimization"""
    
    model = YOLO(model_path)
    
    print("Exporting model to ONNX format...")
    export_path = model.export(format='onnx')
    print(f"✓ ONNX model exported: {export_path}")
    
    return export_path


def main(args=None):
    """Main entry point for CLI"""
    print("""
    
    SETUP INSTRUCTIONS:
    
    1. Create dataset directory structure:
       /home/arash/franka-multiagent-manipulation/datasets/battery_assembly/
       ├── images/
       │   ├── train/
       │   ├── val/
       │   └── test/
       └── labels/
           ├── train/
           ├── val/
           └── test/
    
    2. Place annotated images in images/{train,val,test}/
    3. Place corresponding YOLO .txt annotations in labels/{train,val,test}/
    
    4. Run: python3 -m franka_vision_detection.table_detector_finetuner
    
    ANNOTATION FORMAT:
    Each .txt file should contain:
    <class_id> <x_center> <y_center> <width> <height>
    (normalized coordinates 0-1)
    
    YOLO CLASS IDS (27 classes):
    0: battery_aa         1: battery_aaa        2: battery_9v
    3: battery_coin       4: battery_alkaline   5: battery_spring
    6: battery_contact    7: battery_terminal   8: contact_pin
    9: pcb               10: pcb_track         11: resistor
    12: capacitor        13: diode             14: led
    15: connector        16: screw             17: bolt
    18: nut              19: washer            20: spacer
    21: clip             22: rivet             23: screwdriver
    24: wrench           25: pliers            26: drill_bit
    27: soldering_iron   28: wire              29: cable
    30: housing          31: cover             32: bracket
    33: fastener
    
    To start fine-tuning:
    1. Create your dataset following the structure above
    2. Run: python3 table_detector_finetuner.py
    """)


if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
