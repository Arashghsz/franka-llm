/**
 * Chat Module
 * Handles chat interface and LLM communication
 */

class ChatModule {
    constructor() {
        this.chatHistory = document.getElementById('chatHistory');
        this.messages = [];
        if (this.chatHistory) {
            this.setupROSSubscriptions();
        }
    }

    setupROSSubscriptions() {
        // Subscribe to responses from web_handler
        if (window.ros && window.ros.isConnected()) {
            window.ros.subscribe('/web/response', 'std_msgs/String', (message) => {
                try {
                    const data = JSON.parse(message.data);
                    const sender = data.sender || 'robot';
                    const text = data.message || '';
                    
                    if (text) {
                        // Remove the processing indicator if it exists
                        const processingMessages = Array.from(this.chatHistory.querySelectorAll('.message-container.system'))
                            .filter(m => m.textContent.includes('Processing'));
                        if (processingMessages.length > 0) {
                            processingMessages[processingMessages.length - 1].remove();
                        }
                        
                        this.addMessage(text, sender);
                    }
                    
                    console.log('Response received:', data);
                } catch (e) {
                    console.error('Error parsing response:', e);
                }
            });
            
            console.log('Subscribed to /web/response');
        }
    }

    sendMessage(userMessage) {
        // Add user message to display
        this.addMessage(userMessage, 'user');

        // Send to coordinator via ROS
        if (window.ros && window.ros.isConnected()) {
            this.sendToLLM(userMessage);
        } else {
            this.addMessage('⚠️ System offline: Cannot send message. Please ensure ROSBridge is running.', 'system');
        }
    }

    addMessage(text, sender = 'robot') {
        if (!this.chatHistory) {
            console.warn('Chat history element not found');
            return;
        }
        
        const messageDiv = document.createElement('div');
        messageDiv.className = `message-container ${sender}`;
        
        const bubble = document.createElement('div');
        bubble.className = 'message-bubble';
        bubble.innerHTML = `<p>${this.escapeHtml(text)}</p>`;
        
        messageDiv.appendChild(bubble);
        this.chatHistory.appendChild(messageDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
        this.messages.push({ text, sender, timestamp: new Date() });
    }

    sendToLLM(message) {
        // Publish to ROS topic: /web/request with JSON format
        if (window.ros && window.ros.isConnected()) {
            const request = {
                type: 'chat',
                data: {
                    message: message,
                    timestamp: new Date().toISOString()
                }
            };
            
            window.ros.publish('/web/request', 'std_msgs/String', {
                data: JSON.stringify(request)
            });
            console.log('Chat request sent to coordinator:', request);
        }

        // Show thinking indicator
        this.addMessage('⏳ Processing your request...', 'system');
    }

    escapeHtml(text) {
        const map = {
            '&': '&amp;',
            '<': '&lt;',
            '>': '&gt;',
            '"': '&quot;',
            "'": '&#039;'
        };
        return text.replace(/[&<>"']/g, m => map[m]);
    }

    getHistory() {
        return this.messages;
    }

    clearHistory() {
        if (!this.chatHistory) {
            console.warn('Chat history element not found');
            return;
        }
        
        this.messages = [];
        this.chatHistory.innerHTML = '';
        this.addMessage('👋 Welcome! I\'m your Franka robot assistant. You can ask me to perform tasks like "pick up the red cube" or "move to home position".', 'robot');
    }
}
