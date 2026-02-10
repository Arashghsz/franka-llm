#!/usr/bin/env python3
"""
Check VLM status and health
"""

import requests
import subprocess
import sys


def check_ollama_running():
    """Check if Ollama is running"""
    try:
        response = requests.get('http://localhost:11434/api/tags', timeout=2)
        if response.status_code == 200:
            print("Ollama is RUNNING")
            models = response.json().get('models', [])
            if models:
                print(f"   Available models: {len(models)}")
                for model in models:
                    print(f"   - {model['name']}")
            return True
        else:
            print("Ollama API error")
            return False
    except requests.exceptions.ConnectionError:
        print("Ollama NOT RUNNING")
        print("   Start with: ollama serve")
        return False
    except Exception as e:
        print(f"Error checking Ollama: {str(e)}")
        return False


def check_llava_model():
    """Check if LLaVA model is available"""
    try:
        response = requests.get('http://localhost:11434/api/tags', timeout=2)
        models = response.json().get('models', [])
        for model in models:
            if 'llava' in model['name'].lower():
                print(f"✅ LLaVA model found: {model['name']}")
                return True
        print("LLaVA model NOT FOUND")
        print("   Pull with: ollama pull llava:7b")
        return False
    except Exception as e:
        print(f"Error checking model: {str(e)}")
        return False


def test_ollama_inference():
    """Test VLM inference with a simple text query"""
    try:
        print("\n🧪 Testing VLM inference...")
        payload = {
            'model': 'llava:7b',
            'prompt': 'What color is the yellow?',
            'stream': False
        }
        response = requests.post(
            'http://localhost:11434/api/generate',
            json=payload,
            timeout=30
        )
        if response.status_code == 200:
            result = response.json()
            print(f"VLM responded: {result.get('response', '')[:50]}...")
            return True
        else:
            print(f"VLM API error: {response.status_code}")
            return False
    except Exception as e:
        print(f"Error testing inference: {str(e)}")
        return False


def main():
    print("\n" + "="*60)
    print("VLM HEALTH CHECK")
    print("="*60 + "\n")
    
    # Check Ollama
    ollama_ok = check_ollama_running()
    if not ollama_ok:
        print("\nFAILED: Ollama is not running!")
        print("\nStart Ollama with:")
        print("   ollama serve")
        return False
    
    # Check LLaVA model
    model_ok = check_llava_model()
    if not model_ok:
        print("\nLLaVA model not found. Pull with:")
        print("   ollama pull llava:7b")
        return False
    
    # Test inference
    inference_ok = test_ollama_inference()
    
    if ollama_ok and model_ok and inference_ok:
        print("\n" + "="*60)
        print("ALL CHECKS PASSED - VLM is ready for ROS2!")
        print("="*60 + "\n")
        return True
    else:
        print("\n" + "="*60)
        print("SOME CHECKS FAILED - See above for fixes")
        print("="*60 + "\n")
        return False


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
