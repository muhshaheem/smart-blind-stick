import cv2
import serial
import gtts
from playsound import playsound
import requests
import time
import threading
import math
from grove_vision_ai import GroveVisionAI  # You'll need to install this SDK

class SmartBlindStick:
    def __init__(self):
        # Predefined locations (latitude, longitude, name)
        self.locations = {
            "hospital": (12.9716, 77.5946),  # Example coordinates
            "bus_stand": (12.9720, 77.5930),
            "shopping_mall": (12.9730, 77.5950),
            "park": (12.9700, 77.5935),
            "home": (12.9690, 77.5920)
        }
        
        # Initialize components
        self.gps_serial = serial.Serial('COM3', 9600)  # Adjust port
        self.vision_ai = GroveVisionAI()
        self.ultrasonic_serial = serial.Serial('COM4', 9600)  # For ultrasonic
        
        # Button setup (using keyboard input for simulation)
        self.button_pressed = False
        
        # Current position
        self.current_lat = 0
        self.current_lon = 0
        
        # Safety distance (in meters)
        self.safe_distance = 2.0
        
    def get_gps_data(self):
        """Read GPS data from NEO-6M module"""
        try:
            line = self.gps_serial.readline().decode('utf-8').strip()
            if line.startswith('$GPGGA'):
                data = line.split(',')
                if data[2] and data[4]:  # If latitude and longitude available
                    lat = float(data[2][:2]) + float(data[2][2:])/60
                    lon = float(data[4][:3]) + float(data[4][3:])/60
                    
                    # Adjust for hemisphere
                    if data[3] == 'S':
                        lat = -lat
                    if data[5] == 'W':
                        lon = -lon
                    
                    self.current_lat = lat
                    self.current_lon = lon
                    return True
            return False
        except Exception as e:
            print(f"GPS Error: {e}")
            return False
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two coordinates in meters"""
        R = 6371000  # Earth's radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def get_nearest_location(self):
        """Find nearest predefined location"""
        if not self.current_lat or not self.current_lon:
            return "Unknown location"
        
        nearest_name = "Unknown"
        min_distance = float('inf')
        
        for name, (lat, lon) in self.locations.items():
            distance = self.calculate_distance(
                self.current_lat, self.current_lon, lat, lon
            )
            if distance < min_distance:
                min_distance = distance
                nearest_name = name
        
        return f"Near {nearest_name.replace('_', ' ')}. Distance: {min_distance:.1f} meters"
    
    def text_to_speech(self, text):
        """Convert text to speech and play"""
        try:
            tts = gtts.gTTS(text=text, lang='en')
            tts.save("output.mp3")
            playsound("output.mp3")
        except Exception as e:
            print(f"TTS Error: {e}")
    
    def detect_obstacles(self):
        """Use Grove Vision AI for object detection"""
        try:
            # Get detection results from Vision AI
            results = self.vision_ai.get_detection_results()
            
            obstacles = []
            for obj in results:
                if obj['confidence'] > 0.6:  # Confidence threshold
                    obstacles.append(obj['name'])
            
            return obstacles
        except Exception as e:
            print(f"Vision AI Error: {e}")
            return []
    
    def read_ultrasonic(self):
        """Read ultrasonic sensor distance"""
        try:
            distance = float(self.ultrasonic_serial.readline().decode().strip())
            return distance
        except:
            return float('inf')
    
    def monitor_obstacles(self):
        """Continuous obstacle monitoring"""
        while True:
            # Check ultrasonic sensor
            distance = self.read_ultrasonic()
            if distance < self.safe_distance:
                self.text_to_speech(f"Warning! Obstacle {distance:.1f} meters ahead")
            
            # Check vision AI for specific obstacles
            obstacles = self.detect_obstacles()
            if obstacles:
                obstacle_text = "Warning! " + ", ".join(obstacles) + " ahead"
                self.text_to_speech(obstacle_text)
            
            time.sleep(1)
    
    def button_handler(self):
        """Handle button press for location announcement"""
        print("Press 'L' to hear current location, 'Q' to quit:")
        while True:
            key = input().lower()
            if key == 'l':
                if self.get_gps_data():
                    location_info = self.get_nearest_location()
                    print(f"Location: {location_info}")
                    self.text_to_speech(location_info)
                else:
                    self.text_to_speech("GPS signal not available")
            elif key == 'q':
                break
    
    def run(self):
        """Main program loop"""
        print("Smart Blind Stick System Started")
        
        # Start obstacle monitoring in separate thread
        obstacle_thread = threading.Thread(target=self.monitor_obstacles)
        obstacle_thread.daemon = True
        obstacle_thread.start()
        
        # Start button handler
        self.button_handler()
        
        # Cleanup
        self.gps_serial.close()
        self.ultrasonic_serial.close()

if __name__ == "__main__":
    stick = SmartBlindStick()
    stick.run()