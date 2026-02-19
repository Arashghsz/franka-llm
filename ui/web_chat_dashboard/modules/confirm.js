/**
 * Confirmation Module
 * Handles user confirmation dialogs
 */

class ConfirmModule {
    constructor() {
        this.resolver = null;
        this.setupModal();
    }
    
    setupModal() {
        // Create modal overlay if it doesn't exist
        if (!document.getElementById('confirmModal')) {
            const modalHTML = `
                <div id="confirmModal" class="confirm-modal">
                    <div class="confirm-modal-overlay"></div>
                    <div class="confirm-modal-content">
                        <div class="confirm-modal-header">
                            <h3>⚠️ Confirm Action</h3>
                        </div>
                        <div class="confirm-modal-body">
                            <p id="confirmMessage"></p>
                        </div>
                        <div class="confirm-modal-footer">
                            <button id="confirmNo" class="btn btn-cancel">❌ Cancel</button>
                            <button id="confirmYes" class="btn btn-confirm">✅ Approve</button>
                        </div>
                    </div>
                </div>
            `;
            document.body.insertAdjacentHTML('beforeend', modalHTML);
            
            // Add styles
            const style = document.createElement('style');
            style.textContent = `
                .confirm-modal {
                    display: none;
                    position: fixed;
                    top: 0;
                    left: 0;
                    width: 100%;
                    height: 100%;
                    z-index: 10000;
                }
                .confirm-modal.show {
                    display: block;
                }
                .confirm-modal-overlay {
                    position: absolute;
                    top: 0;
                    left: 0;
                    width: 100%;
                    height: 100%;
                    background: rgba(0, 0, 0, 0.7);
                }
                .confirm-modal-content {
                    position: relative;
                    background: var(--card-bg, #1e1e2e);
                    border: 2px solid var(--accent-color, #3b82f6);
                    border-radius: 12px;
                    max-width: 500px;
                    margin: 100px auto;
                    padding: 0;
                    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.5);
                }
                .confirm-modal-header {
                    padding: 20px 24px;
                    border-bottom: 1px solid rgba(255, 255, 255, 0.1);
                }
                .confirm-modal-header h3 {
                    margin: 0;
                    color: var(--text-primary, #ffffff);
                    font-size: 1.3em;
                }
                .confirm-modal-body {
                    padding: 24px;
                }
                .confirm-modal-body p {
                    margin: 0;
                    color: var(--text-secondary, #b0b0c0);
                    font-size: 1.1em;
                    line-height: 1.6;
                    white-space: pre-wrap;
                }
                .confirm-modal-footer {
                    padding: 20px 24px;
                    border-top: 1px solid rgba(255, 255, 255, 0.1);
                    display: flex;
                    gap: 12px;
                    justify-content: flex-end;
                }
                .confirm-modal-footer .btn {
                    padding: 10px 24px;
                    border: none;
                    border-radius: 8px;
                    font-size: 1em;
                    font-weight: 600;
                    cursor: pointer;
                    transition: all 0.2s;
                }
                .confirm-modal-footer .btn-cancel {
                    background: rgba(239, 68, 68, 0.2);
                    color: #ef4444;
                }
                .confirm-modal-footer .btn-cancel:hover {
                    background: rgba(239, 68, 68, 0.3);
                }
                .confirm-modal-footer .btn-confirm {
                    background: var(--accent-color, #3b82f6);
                    color: white;
                }
                .confirm-modal-footer .btn-confirm:hover {
                    background: var(--accent-hover, #2563eb);
                }
            `;
            document.head.appendChild(style);
        }
        
        this.modal = document.getElementById('confirmModal');
        this.messageEl = document.getElementById('confirmMessage');
        this.yesBtn = document.getElementById('confirmYes');
        this.noBtn = document.getElementById('confirmNo');

        if (this.yesBtn && this.noBtn) {
            this.yesBtn.addEventListener('click', () => this.resolve(true));
            this.noBtn.addEventListener('click', () => this.resolve(false));
        }
    }

    show(message) {
        if (!this.modal || !this.messageEl) {
            console.warn('Confirmation modal elements not found');
            return Promise.resolve(true);
        }
        return new Promise((resolve) => {
            this.resolver = resolve;
            this.messageEl.textContent = message;
            this.modal.classList.add('show');
        });
    }

    resolve(result) {
        if (this.modal) {
            this.modal.classList.remove('show');
        }
        if (this.resolver) {
            this.resolver(result);
            this.resolver = null;
        }
    }
}
