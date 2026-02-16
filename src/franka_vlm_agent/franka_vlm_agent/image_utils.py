#!/usr/bin/env python3
"""
Image utilities for VLM processing
Handles image encoding, saving, and marker drawing
"""

import cv2
import numpy as np
import base64
from pathlib import Path
from datetime import datetime


def encode_image_to_base64(image: np.ndarray) -> str:
    """
    Encode OpenCV image to base64 string
    
    Args:
        image: OpenCV image (BGR format)
        
    Returns:
        Base64 encoded string
    """
    _, buffer = cv2.imencode('.jpg', image)
    return base64.b64encode(buffer).decode('utf-8')


def draw_center_marker(image: np.ndarray, center: list, color=(0, 255, 0), size=20, thickness=3) -> np.ndarray:
    """
    Draw a crosshair marker at the center point
    
    Args:
        image: OpenCV image
        center: [x, y] pixel coordinates
        color: BGR color tuple (default: green)
        size: Size of the crosshair in pixels
        thickness: Line thickness
        
    Returns:
        Image with marker drawn
    """
    img_marked = image.copy()
    x, y = int(center[0]), int(center[1])
    
    # Draw crosshair
    cv2.line(img_marked, (x - size, y), (x + size, y), color, thickness)
    cv2.line(img_marked, (x, y - size), (x, y + size), color, thickness)
    
    # Draw circle around center
    cv2.circle(img_marked, (x, y), size // 2, color, thickness)
    
    # Add text label
    cv2.putText(img_marked, f'({x}, {y})', (x + size + 5, y - size), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    return img_marked


def save_debug_image(image: np.ndarray, object_name: str, center: list = None, 
                     debug_dir: Path = None) -> str:
    """
    Save image with optional marker to debug directory
    
    Args:
        image: OpenCV image
        object_name: Name of the object (used in filename)
        center: Optional [x, y] coordinates to mark
        debug_dir: Directory to save images (default: workspace_root/debug_images)
        
    Returns:
        Path to saved image
    """
    # Determine save directory
    if debug_dir is None:
        # Default to workspace root / debug_images
        current_file = Path(__file__).resolve()
        workspace_root = current_file.parents[4]
        debug_dir = workspace_root / 'debug_images'
    
    debug_dir.mkdir(exist_ok=True, parents=True)
    
    # Create annotated image
    if center is not None:
        img_to_save = draw_center_marker(image, center)
    else:
        img_to_save = image.copy()
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    safe_object_name = object_name.replace(' ', '_')
    filename = f'{safe_object_name}_{timestamp}.jpg'
    filepath = debug_dir / filename
    
    # Save image
    cv2.imwrite(str(filepath), img_to_save)
    
    return str(filepath)


def parse_vlm_response(vlm_response: str) -> dict:
    """
    Parse VLM JSON response and handle different coordinate formats
    
    Args:
        vlm_response: JSON string from VLM
        
    Returns:
        Parsed dictionary with normalized center coordinates
    """
    import json
    
    try:
        data = json.loads(vlm_response)
    except json.JSONDecodeError:
        return None
    
    center = data.get('center')
    
    # Handle null/None center
    if center is None or (isinstance(center, list) and None in center):
        return None
    
    # Convert bounding box [x1, y1, x2, y2] to center [x, y]
    if isinstance(center, list) and len(center) == 4:
        x1, y1, x2, y2 = center
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        data['center'] = [center_x, center_y]
        data['bounding_box'] = [x1, y1, x2, y2]
        return data
    
    # Already in center format [x, y]
    elif isinstance(center, list) and len(center) == 2:
        return data
    
    return None
