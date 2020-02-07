import numpy as np
import cv2

#Seta a variavel video para receber dados da camera 0 (default)
video = cv2.VideoCapture(0)

#Como é video, é necessario um while true para pegar novos frames sempre
while(True):

    #flag só indica se deu certo ou nao. Img é o frame que voce recebeu da câmera
    flag, img = video.read()

    #Se flag existir e for != 0, está tudo certo
    if(flag):

        #Abre uma aba chamada 'Camera' e linka ela à imagem que voce acabou de receber da camera
        cv2.imshow('Camera', img)

        #Caso a tecla q seja pressionada, sai do while true e acaba o programa
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #Se o flag não existir ou for 0, aborta o codigo
    else:
        print("Deu ruuuuim!")
        cv2.waitKey(10)

#Quando a tecla q for pressionada, deslinka a video da camera
video.release()

#Destroi todas as abas criadas
cv2.destroyAllWindows()
