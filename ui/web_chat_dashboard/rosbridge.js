/**
 * ROSBridge Module
 * Handles ROS2 communication via ROSBridge WebSocket
 */

class ROSBridge {
    constructor(config = {}) {
        this.url = config.url || 'ws://localhost:9090';
        this.connected = false;
        this.subscribers = new Map();      // topic -> callbacks[]
        this.subscriptionTypes = new Map(); // topic -> messageType (for reconnect)
        this.publishers = new Map();
        this.services = new Map();

        this.connect();
    }

    connect() {
        try {
            this.ws = new WebSocket(this.url);

            this.ws.onopen = () => {
                console.log('Connected to ROSBridge');
                this.connected = true;
                // Re-send all subscription requests to the server after (re)connect
                this.subscriptionTypes.forEach((type, topic) => {
                    this.send({ op: 'subscribe', topic: topic, type: type });
                });
                if (this.subscriptionTypes.size > 0) {
                    console.log(`Re-subscribed to ${this.subscriptionTypes.size} topic(s) after connect`);
                }
            };

            this.ws.onmessage = (event) => {
                this.handleMessage(event.data);
            };

            this.ws.onerror = (error) => {
                console.error('ROSBridge error:', error);
            };

            this.ws.onclose = () => {
                console.log('Disconnected from ROSBridge');
                this.connected = false;
                setTimeout(() => this.connect(), 3000);
            };
        } catch (error) {
            console.error('Failed to connect to ROSBridge:', error);
        }
    }

    handleMessage(data) {
        try {
            const message = JSON.parse(data);
            
            if (message.op === 'publish' && this.subscribers.has(message.topic)) {
                const callbacks = this.subscribers.get(message.topic);
                callbacks.forEach(cb => cb(message.msg));
            }
        } catch (error) {
            console.error('Error handling ROSBridge message:', error);
        }
    }

    subscribe(topic, messageType, callback) {
        // Track the type for reconnect re-subscription
        this.subscriptionTypes.set(topic, messageType);

        if (!this.subscribers.has(topic)) {
            this.subscribers.set(topic, []);
        }
        this.subscribers.get(topic).push(callback);

        // Send subscribe op now if already connected; onopen handles the rest
        if (this.connected && this.ws.readyState === WebSocket.OPEN) {
            this.send({ op: 'subscribe', topic: topic, type: messageType });
        }
    }

    unsubscribe(topic) {
        this.send({
            op: 'unsubscribe',
            topic: topic
        });
        this.subscribers.delete(topic);
    }

    publish(topic, messageType, message) {
        const request = {
            op: 'publish',
            topic: topic,
            type: messageType,
            msg: message
        };
        this.send(request);
    }

    callService(service, serviceType, request) {
        return new Promise((resolve, reject) => {
            const id = Math.random().toString(36).substr(2, 9);
            const req = {
                op: 'call_service',
                service: service,
                type: serviceType,
                id: id,
                args: request
            };

            // Store callback for response
            this.services.set(id, { resolve, reject });
            this.send(req);

            // Timeout after 10 seconds
            setTimeout(() => {
                if (this.services.has(id)) {
                    this.services.delete(id);
                    reject(new Error('Service call timeout'));
                }
            }, 10000);
        });
    }

    send(message) {
        if (this.connected && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(message));
        }
    }

    isConnected() {
        return this.connected;
    }
}
