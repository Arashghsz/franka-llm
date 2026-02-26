/**
 * Chat Module
 * Handles chat interface and LLM communication
 */

class ChatModule {
    constructor() {
        this.chatHistory = document.getElementById('chatHistory');
        this.logsContent = document.getElementById('logsContent');
        this.clearLogsBtn = document.getElementById('clearLogsBtn');
        this.messages = [];
        this.subscribed = false;
        
        if (this.chatHistory) {
            // Clear default welcome and add our own
            this.chatHistory.innerHTML = '';
            this.showWelcomeMessage();
            
            // Setup clear logs button
            if (this.clearLogsBtn) {
                this.clearLogsBtn.addEventListener('click', () => this.clearLogs());
            }
            
            // Try to setup subscriptions now
            this.setupROSSubscriptions();
            
            // Also retry after a delay to ensure connection is ready
            setTimeout(() => this.setupROSSubscriptions(), 1000);
            setTimeout(() => this.setupROSSubscriptions(), 3000);
            
            // Add initial log entry after a short delay
            setTimeout(() => {
                if (this.logsContent) {
                    this.logsContent.innerHTML = '';
                    this.addLogMessage('Web dashboard initialized', '🌐 System');
                }
            }, 100);
        }
    }
    
    showWelcomeMessage() {
        // Simple loading message that will be replaced by actual welcome from backend
        const loadingText = '⏳ Initializing system and loading configuration...';
        this.addMessage(loadingText, 'robot', null, '🤖 Franka Assistant');
        
        // Mark this as the placeholder to be replaced
        const lastMessage = this.chatHistory.lastElementChild;
        if (lastMessage) {
            lastMessage.id = 'welcome-placeholder';
        }
    }
    
    showTypingIndicator() {
        // Remove any existing typing indicator
        this.hideTypingIndicator();
        
        if (!this.chatHistory) {
            console.error('chatHistory not found');
            return;
        }
        
        const typingDiv = document.createElement('div');
        typingDiv.className = 'message-container robot';
        typingDiv.id = 'typing-indicator';
        
        const indicator = document.createElement('div');
        indicator.className = 'typing-indicator';
        indicator.innerHTML = `
            <div class="typing-dots">
                <span class="typing-dot"></span>
                <span class="typing-dot"></span>
                <span class="typing-dot"></span>
            </div>
            <span class="typing-text">Franka is thinking...</span>
        `;
        
        typingDiv.appendChild(indicator);
        this.chatHistory.appendChild(typingDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
        
        console.log('Typing indicator shown');
    }
    
    hideTypingIndicator() {
        const typingIndicator = document.getElementById('typing-indicator');
        if (typingIndicator) {
            typingIndicator.remove();
            console.log('Typing indicator removed');
        }
    }

    setupROSSubscriptions() {
        // Don't subscribe multiple times
        if (this.subscribed) {
            return;
        }
        
        // Subscribe to responses from web_handler
        if (window.ros && window.ros.isConnected()) {
            this.subscribed = true;
            
            window.ros.subscribe('/web/response', 'std_msgs/String', (message) => {
                try {
                    const data = JSON.parse(message.data);
                    const msgType = data.type || 'message';
                    const sender = data.sender || 'robot';
                    const agentName = data.agent_name || null;
                    const text = data.message || '';
                    const image = data.image || null;

                    // Handle different message types
                    if (msgType === 'confirmation_request') {
                        // Hide typing indicator for confirmation
                        this.hideTypingIndicator();
                        // Show inline confirmation in chat
                        this.addConfirmationMessage(data);
                    } else if (msgType === 'log') {
                        // Show as a subtle log message (don't hide typing indicator for logs)
                        this.addLogMessage(text, agentName);
                    } else if (text) {
                        // Hide typing indicator only when actual message arrives
                        this.hideTypingIndicator();
                        // Regular message - display with optional image and agent name
                        // Replace loading placeholder if it exists
                        const placeholder = document.getElementById('welcome-placeholder');
                        if (placeholder) {
                            placeholder.remove();
                        }
                        this.addMessage(text, sender, image, agentName);
                    }
                    
                    console.log('Response received:', data);
                } catch (e) {
                    console.error('Error parsing response:', e);
                    console.error('Raw message:', message);
                }
            });
            
            console.log('Subscribed to /web/response');
        } else {
            console.log('Waiting for ROS connection to subscribe to /web/response');
        }
    }
    
    addConfirmationMessage(data) {
        const action = data.action || 'perform action';
        const target = data.target || 'object';
        const message = data.message || `Ready to ${action} ${target}`;
        const agentName = data.agent_name || '⚙️ Motion Controller';
        
        // Create confirmation message with inline buttons
        const messageDiv = document.createElement('div');
        messageDiv.className = 'message-container system confirmation';
        
        const bubble = document.createElement('div');
        bubble.className = 'message-bubble';
        
        // Add agent name badge
        if (agentName) {
            const badge = document.createElement('div');
            badge.className = 'agent-badge';
            badge.textContent = agentName;
            bubble.appendChild(badge);
        }
        
        // Add formatted message text with markdown support
        const textDiv = document.createElement('div');
        textDiv.className = 'message-text';
        textDiv.innerHTML = this.formatMessage(message);
        bubble.appendChild(textDiv);
        
        // Add inline confirmation buttons
        const buttonContainer = document.createElement('div');
        buttonContainer.className = 'confirmation-buttons';
        
        const approveBtn = document.createElement('button');
        approveBtn.className = 'btn-approve';
        approveBtn.innerHTML = '✅ Approve & Execute';
        approveBtn.onclick = () => {
            buttonContainer.remove();
            this.sendConfirmation(true);
            this.addLogMessage('Motion approved by user', '✓ User');
        };
        
        const cancelBtn = document.createElement('button');
        cancelBtn.className = 'btn-cancel';
        cancelBtn.innerHTML = '❌ Cancel';
        cancelBtn.onclick = () => {
            buttonContainer.remove();
            this.sendConfirmation(false);
            this.addLogMessage('Motion cancelled by user', '✗ User');
        };
        
        buttonContainer.appendChild(cancelBtn);
        buttonContainer.appendChild(approveBtn);
        bubble.appendChild(buttonContainer);
        
        messageDiv.appendChild(bubble);
        this.chatHistory.appendChild(messageDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    addLogMessage(text, agentName = null) {
        if (!this.logsContent) {
            console.warn('Logs content element not found!');
            return;
        }
        
        const logEntry = document.createElement('div');
        logEntry.className = 'log-entry';
        
        // Add timestamp first
        const timeSpan = document.createElement('span');
        timeSpan.className = 'log-timestamp';
        const now = new Date();
        timeSpan.textContent = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
        logEntry.appendChild(timeSpan);
        
        if (agentName) {
            const agentSpan = document.createElement('span');
            agentSpan.className = 'log-agent';
            agentSpan.textContent = agentName;
            logEntry.appendChild(agentSpan);
        }
        
        const textSpan = document.createElement('span');
        textSpan.className = 'log-text';
        textSpan.textContent = text;
        logEntry.appendChild(textSpan);
        
        this.logsContent.appendChild(logEntry);
        this.logsContent.scrollTop = this.logsContent.scrollHeight;
        
        console.log('Log added:', agentName, text);
    }
    
    clearLogs() {
        if (this.logsContent) {
            this.logsContent.innerHTML = '';
        }
    }
    
    formatMessage(text) {
        // Simple markdown-like formatting
        text = this.escapeHtml(text);
        // Remove ### headers - just make them bold
        text = text.replace(/###+ (.*?)$/gm, '<strong>$1</strong>');
        // Bold: **text**
        text = text.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
        // Bullet points
        text = text.replace(/^• /gm, '<span class="bullet">•</span> ');
        // Line breaks
        text = text.replace(/\n/g, '<br>');
        return text;
    }
    
    sendConfirmation(confirmed) {
        if (window.ros && window.ros.isConnected()) {
            const request = {
                type: 'confirmation',
                confirmed: confirmed,
                timestamp: new Date().toISOString()
            };
            
            window.ros.publish('/web/request', 'std_msgs/String', {
                data: JSON.stringify(request)
            });
            
            console.log('Confirmation sent:', confirmed);
        }
    }

    sendMessage(userMessage) {
        // Add user message to display
        this.addMessage(userMessage, 'user');

        // Send to coordinator via ROS
        if (window.ros && window.ros.isConnected()) {
            this.sendToLLM(userMessage);
        } else {
            this.addMessage('System offline: Cannot send message. Please ensure ROSBridge is running.', 'system');
        }
    }

    addMessage(text, sender = 'robot', imageData = null, agentName = null) {
        if (!this.chatHistory) {
            console.warn('Chat history element not found');
            return;
        }
        
        const messageDiv = document.createElement('div');
        messageDiv.className = `message-container ${sender}`;
        
        const bubble = document.createElement('div');
        bubble.className = 'message-bubble';
        
        // Add agent name badge if provided
        if (agentName) {
            const badge = document.createElement('div');
            badge.className = 'agent-badge';
            badge.textContent = agentName;
            bubble.appendChild(badge);
        }
        
        // Add text content with formatting
        const textDiv = document.createElement('div');
        textDiv.className = 'message-text';
        textDiv.innerHTML = this.formatMessage(text);
        bubble.appendChild(textDiv);
        
        // Add timestamp
        const timestamp = document.createElement('div');
        timestamp.className = 'message-timestamp';
        const now = new Date();
        timestamp.textContent = now.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
        bubble.appendChild(timestamp);
        
        // Add image if provided
        if (imageData) {
            const imageEl = document.createElement('img');
            imageEl.src = imageData;
            imageEl.className = 'message-image';
            imageEl.alt = 'Vision result';
            imageEl.onclick = () => {
                // Open image in new tab on click
                window.open(imageData, '_blank');
            };
            bubble.appendChild(imageEl);
        }
        
        messageDiv.appendChild(bubble);
        this.chatHistory.appendChild(messageDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
        this.messages.push({ text, sender, imageData, agentName, timestamp: new Date() });
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
            
            // Show typing indicator
            console.log('Showing typing indicator...');
            this.showTypingIndicator();
            
            window.ros.publish('/web/request', 'std_msgs/String', {
                data: JSON.stringify(request)
            });
            console.log('Chat request sent to coordinator:', request);
        }

        // Show data flow start instead of generic "Processing"
        this.addLogMessage('Request sent to Web Handler', 'Browser');
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

    clearHistory() {
        if (!this.chatHistory) return;
        this.messages = [];
        this.chatHistory.innerHTML = '';
        this.addMessage('Chat cleared. Ask me anything — I can describe the scene, find objects, or execute motions.', 'robot', null, 'Franka Assistant');
    }
}
