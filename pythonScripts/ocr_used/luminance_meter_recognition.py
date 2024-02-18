import cv2  
import pytesseract  
  
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files (x86)\Tesseract-OCR\tesseract.exe'


cap = cv2.VideoCapture(0)  
cap.set(3,320)
cap.set(4,240)
#cap.set(15, 0.001)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, -1);
custom_config = r'--psm 6 --psm 13 --oem 3 -c tessedit_char_whitelist=0123456789.'

while True:  
    ret, frame = cap.read()  
    im_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    im_at_mean = cv2.adaptiveThreshold(im_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 7, 7)
    #text = pytesseract.image_to_string(im_at_mean, lang='eng') 
    text = pytesseract.image_to_string(im_at_mean, lang='eng', config=custom_config)
    print(text)  
  
    cv2.imshow('frame', im_at_mean)  
    if cv2.waitKey(1) & 0xFF == ord('q'):  
        break  
  
cap.release()  
cv2.destroyAllWindows()