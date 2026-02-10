/**
 * Camera Module
 * Handles camera feed display
 */

class CameraModule {
    constructor() {
        this.canvas = document.getElementById('cameraCanvas');
        this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
        this.isRunning = false;
    }

    startFeed() {
        if (!this.canvas || !this.ctx) {
            console.warn('Camera canvas not found');
            return;
        }
        
        this.isRunning = true;
        this.messageCount = 0;
        
        // Subscribe to annotated image from YOLO detection
        if (window.ros && window.ros.isConnected()) {
            console.log('ROS connected, attempting subscription...');
            
            try {
                // Use arrow function to preserve 'this' context
                const callback = (msg) => {
                    this.messageCount++;
                    console.log('MESSAGE ARRIVED!', msg);
                    this.displayImage(msg);
                };
                
                window.ros.subscribe(
                    '/detection/annotated_image',
                    'sensor_msgs/Image',
                    callback
                );
                console.log('✓ Subscribed to /detection/annotated_image');
            } catch (e) {
                console.error('Failed to subscribe:', e);
            }
        } else {
            console.warn('ROS not connected, retrying in 1 second');
            setTimeout(() => this.startFeed(), 1000);
            return;
        }

        // Set canvas size
        this.canvas.width = 640;
        this.canvas.height = 480;
        
        // Draw placeholder
        this.drawPlaceholder();
        
        // Add heartbeat to detect if messages are coming
        this.heartbeat = setInterval(() => {
            if (this.messageCount === 0) {
                console.warn('⚠️ No image messages received in 3 seconds - Check if YOLO detector is publishing');
            } else {
                console.log(`✓ Received ${this.messageCount} images in last 3 seconds (${Math.round(this.messageCount/3)} fps)`);
            }
            this.messageCount = 0;
        }, 3000);
    }

    stopFeed() {
        this.isRunning = false;
        if (this.heartbeat) {
            clearInterval(this.heartbeat);
        }
        if (window.ros && window.ros.isConnected()) {
            try {
                window.ros.unsubscribe('/detection/annotated_image');
            } catch (e) {
                console.error('Error unsubscribing:', e);
            }
        }
    }

    displayImage(imageMsg) {
        try {
            if (!this.ctx || !this.canvas) return;
            
            console.log('Image received:', {
                width: imageMsg.width,
                height: imageMsg.height,
                encoding: imageMsg.encoding,
                dataLength: imageMsg.data ? imageMsg.data.length : 0,
                dataType: imageMsg.data ? imageMsg.data.constructor.name : 'undefined'
            });
            
            const width = imageMsg.width;
            const height = imageMsg.height;
            const encoding = imageMsg.encoding || 'bgr8';
            let data = imageMsg.data;
            
            if (!data) {
                console.warn('No image data received');
                return;
            }
            
            // Convert base64 string to Uint8Array if needed
            if (typeof data === 'string') {
                console.log('Converting base64 to binary...');
                const binary = atob(data);
                data = new Uint8Array(binary.length);
                for (let i = 0; i < binary.length; i++) {
                    data[i] = binary.charCodeAt(i);
                }
                console.log('Conversion complete, data length:', data.length);
            } else if (!(data instanceof Uint8Array)) {
                data = new Uint8Array(data);
            }
            
            // Create ImageData from the raw data
            const imageData = this.ctx.createImageData(width, height);
            const imgDataArray = imageData.data;
            
            if (encoding === 'rgb8') {
                // RGB: convert to RGBA
                for (let i = 0, j = 0; i < data.length; i += 3, j += 4) {
                    imgDataArray[j] = data[i];        // R
                    imgDataArray[j + 1] = data[i + 1]; // G
                    imgDataArray[j + 2] = data[i + 2]; // B
                    imgDataArray[j + 3] = 255;         // A
                }
            } else if (encoding === 'bgr8') {
                // BGR: convert to RGBA
                for (let i = 0, j = 0; i < data.length; i += 3, j += 4) {
                    imgDataArray[j] = data[i + 2];     // R (from B position)
                    imgDataArray[j + 1] = data[i + 1]; // G
                    imgDataArray[j + 2] = data[i];     // B (from R position)
                    imgDataArray[j + 3] = 255;         // A
                }
            } else if (encoding === 'rgba8') {
                // Already RGBA
                imgDataArray.set(data);
            } else if (encoding === 'bgra8') {
                // BGRA: convert to RGBA
                for (let i = 0; i < data.length; i += 4) {
                    imgDataArray[i] = data[i + 2];     // R
                    imgDataArray[i + 1] = data[i + 1]; // G
                    imgDataArray[i + 2] = data[i];     // B
                    imgDataArray[i + 3] = data[i + 3]; // A
                }
            } else if (encoding === 'mono8' || encoding === 'mono16') {
                // Grayscale: convert to RGBA
                for (let i = 0, j = 0; i < data.length; i++, j += 4) {
                    imgDataArray[j] = data[i];
                    imgDataArray[j + 1] = data[i];
                    imgDataArray[j + 2] = data[i];
                    imgDataArray[j + 3] = 255;
                }
            } else {
                console.warn('Unknown encoding:', encoding, 'Trying BGR fallback');
                // Fallback to BGR
                for (let i = 0, j = 0; i < data.length; i += 3, j += 4) {
                    imgDataArray[j] = data[i + 2];
                    imgDataArray[j + 1] = data[i + 1];
                    imgDataArray[j + 2] = data[i];
                    imgDataArray[j + 3] = 255;
                }
            }
            
            // Update canvas dimensions if needed
            if (this.canvas.width !== width || this.canvas.height !== height) {
                this.canvas.width = width;
                this.canvas.height = height;
            }
            
            // Draw the image
            this.ctx.putImageData(imageData, 0, 0);
            console.log('✓ Image rendered successfully');
            
        } catch (error) {
            console.error('Error displaying image:', error, imageMsg);
        }
    }

    drawPlaceholder() {
        if (!this.ctx || !this.canvas) return;
        
        const w = this.canvas.width;
        const h = this.canvas.height;
        
        this.ctx.fillStyle = '#000';
        this.ctx.fillRect(0, 0, w, h);
        
        this.ctx.fillStyle = '#fff';
        this.ctx.font = '20px Arial';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('Camera Feed Offline', w / 2, h / 2);
    }

    captureFrame() {
        if (!this.canvas) return null;
        return this.canvas.toDataURL('image/png');
    }
}
