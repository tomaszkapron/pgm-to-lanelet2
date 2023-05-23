import cv2
from Track import LoopTrack

class TrackGeneratorApp:
    def __init__(self):
        self.generatorRunning = True
        
        self.track = LoopTrack()
        self.track_img = cv2.imread("imola (1).pgm")
        self.screen_name = "Track Generator"
        self.screen = cv2.namedWindow(self.screen_name, cv2.WINDOW_AUTOSIZE)
        
        cv2.setMouseCallback(self.screen_name, self.mouseCallback)
        
    def run(self):
        while self.generatorRunning:
            canvas = self.track_img.copy()
            self.drawTrack(canvas)
                        
            result = cv2.bitwise_and(self.track_img, canvas)
            cv2.imshow(self.screen_name, result)

            key = chr(cv2.waitKey(1) & 0xFF)
            self.track.handle_keyboard_input(key)
            if key == 'q' or key == chr(27):
                self.generatorRunning = False
            
            
    def drawTrack(self, canvas: cv2):
        self.track.draw(canvas)
       
    def mouseCallback(self, event, x, y, flags, param):
        self.track.update(event, x, y, flags, param)

        