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


def draw_center_marker(image: np.ndarray, center: list, object_name: str = None, 
                      depth: float = None, color=(0, 255, 255), size=50, thickness=4) -> np.ndarray:
    """
    Draw a HIGHLY VISIBLE crosshair marker at the center point
    
    Args:
        image: OpenCV image
        center: [x, y] pixel coordinates
        object_name: Optional name of the detected object
        depth: Optional depth value at center in meters
        color: BGR color tuple (default: bright cyan/yellow)
        size: Size of the crosshair in pixels
        thickness: Line thickness
        
    Returns:
        Image with marker drawn
    """
    img_marked = image.copy()
    x, y = int(center[0]), int(center[1])
    
    # Use bright cyan for maximum visibility
    main_color = (0, 255, 255)  # Cyan - visible on most backgrounds
    outline_color = (255, 0, 255)  # Magenta outline for extra contrast
    
    # Draw thick magenta outline first
    outline_size = size + 5
    cv2.line(img_marked, (x - outline_size, y), (x + outline_size, y), outline_color, thickness + 4)
    cv2.line(img_marked, (x, y - outline_size), (x, y + outline_size), outline_color, thickness + 4)
    cv2.circle(img_marked, (x, y), size, outline_color, thickness + 4)
    
    # Draw main cyan crosshair on top
    cv2.line(img_marked, (x - size, y), (x + size, y), main_color, thickness)
    cv2.line(img_marked, (x, y - size), (x, y + size), main_color, thickness)
    
    # Draw circles - outer ring and filled center
    cv2.circle(img_marked, (x, y), size // 2, main_color, thickness)
    cv2.circle(img_marked, (x, y), 8, (0, 0, 255), -1)  # Red filled center dot
    cv2.circle(img_marked, (x, y), 8, (255, 255, 255), 2)  # White outline on center dot
    
    # Add info text with high-contrast background
    info_lines = [f'Pixel: ({x}, {y})']
    if object_name:
        info_lines.insert(0, f'Object: {object_name}')
    if depth is not None:
        info_lines.append(f'Depth: {depth:.3f} m')
    
    # Draw text with thick background for maximum visibility
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.8
    text_thickness = 2
    y_offset = max(50, y - size - 15)  # Make sure text is visible
    
    for i, line in enumerate(info_lines):
        text_y = y_offset + (i * 35)
        # Get text size for background
        (text_width, text_height), _ = cv2.getTextSize(line, font, font_scale, text_thickness)
        # Draw thick black background with white border
        bg_x1 = x + size + 15
        bg_y1 = text_y - text_height - 8
        bg_x2 = bg_x1 + text_width + 16
        bg_y2 = text_y + 8
        # White border
        cv2.rectangle(img_marked, (bg_x1 - 3, bg_y1 - 3), (bg_x2 + 3, bg_y2 + 3), (255, 255, 255), -1)
        # Black background
        cv2.rectangle(img_marked, (bg_x1, bg_y1), (bg_x2, bg_y2), (0, 0, 0), -1)
        # Cyan text
        cv2.putText(img_marked, line, (bg_x1 + 8, text_y), font, font_scale, main_color, text_thickness)
    
    return img_marked


def save_debug_image(image: np.ndarray, object_name: str, center: list = None, 
                     depth: float = None, debug_dir: Path = None, model_name: str = None) -> str:
    """
    Save image with optional marker to debug directory
    
    Args:
        image: OpenCV image
        object_name: Name of the object (used in filename)
        center: Optional [x, y] coordinates to mark
        depth: Optional depth value at center in meters
        debug_dir: Directory to save images (default: workspace_root/debug_images)
        model_name: Optional model name to include in filename
        
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
        img_to_save = draw_center_marker(image, center, object_name=object_name, depth=depth)
    else:
        img_to_save = image.copy()
    
    # Generate filename with timestamp and model name
    timestamp = datetime.now().strftime('%b%d_%H-%M-%S')  # e.g., Feb17_18-30-45
    safe_object_name = object_name.replace(' ', '_')
    
    if model_name:
        # Keep full model name including size (e.g., llama3.2-vision:90b)
        # Replace : with - for filesystem compatibility
        model_safe = model_name.replace(':', '-')
        filename = f'{safe_object_name}_{model_safe}_{timestamp}.jpg'
    else:
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
