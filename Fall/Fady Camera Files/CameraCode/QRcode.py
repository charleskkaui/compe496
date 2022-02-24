from gplozero import LED, Button, Buzzer
import cv2
import re


#using the make pHat board pins
led = LED()
sw1 = Button()
buzzer = Buzzer()

cap = cv2.VideoCapture(0)

    global sw1Push
    sw1Push = True
    
sw1.Pressed = sw1Push
sw1.Push = False


print("reading the QR code with rpi camera")
print("push the sw1 button to scan")


while True:
        
        if sw1Push == True;
            led.toggle()
            
        _, img = cap.read()
        data, bbox, _ = detector.detectAndDecod(img)
        
        if bbox is not None:
            for i in range(len(bbox)):
                cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]), color = (255, 0, 0), thickness=2)
                    
        cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    
        if data:
            sw1Push = False
            buzzer.beep(0.1, 0.1, 1)
            print("Data succeeded: " + data)
            led.off()
            data = ""
        cv2.imshow("code detect", img)
        
        else:
            cap.read()
            cv2.destroyAllWindows()
            
        if cv2.waitKey(1) == ord("q")

led.off()
cap.release()
cv2.destroyAllWindows()

                    