import numpy as np
import cv2

#Arzena a imagem.png em img
flag, img = cv2.imread('imagem.png')

#Gray passa a ser a imagem em escala de cinza
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#Todos os bits acima de 127 são pintados de branco e todos abaixo de 127 sao pintados de preto
ret, thresh = cv2.threshold(gray, 127, 255, 0)

#Armazena todos os contornos numa lista chamada contorno
contorno, hierarquia = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

print("Numero de contornos na imagem: " + str(len(contorno)))

#Mostra a imagem orignial
cv2.imshow('Original', img)

#Pinta todos os pixels admitidos como contorno
cv2.drawContours(img, contorno, -1, (0, 0, 255), 3)

#Mostra a imagem com o contorno pintado
cv2.imshow('Contorno', img)

cv2.waitKey(0)

# Windows > Linux
cv2.destroyAllWindows()
