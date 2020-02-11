import numpy as np
import cv2

#vincula a webcam na posição 0 (0 por padrao) à variavel video
video = cv2.VideoCapture(0)

# Como vai ser em tempo real, é necessário um while true para pegar o máximo de frames que o clock permitir
while(True):

    #flag só indica se deu certo ou nao. Img é o frame que ele leu da webcam, ou seja, uma imagem
    flag, img = video.read()

    if(flag):

        #Lê a mesma imagem, porem em escala de cinza
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Filtra a imagem, borrando ela um pouco
        img_filtered = cv2.GaussianBlur(gray, (5, 5), 0)
        # img_bin = cv2.threshold(img_filtered, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        #Cria uma imagem 'edges' somente com as bordas das coisas
        edges = cv2.Canny(img_filtered,100,200)

        #Filtra ainda mais essas bordas, e consegue o contorno das coisas
        contornos, _ = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Abre uma janela chamda 'Original', contendo a imagem inicial da Camera
        cv2.imshow("Original", img)

        #Para cada contorno checa o numero de bordas
        for contorno in contornos:
            approx = cv2.approxPolyDP(contorno, 0.01* cv2.arcLength(contorno, True), True)
            cv2.drawContours(img, [approx], 0, (0, 0, 0), 3)
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 5

            #Se forem 3 bordas, é um triangulo
            if len(approx) == 3:
                cv2.putText(img, "Triangulo", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

            #Se forem 4 bordas, pode ser um quadrado ou um retangulo
            elif len(approx) == 4:
                x1 ,y1, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w)/h
                #Se a proporção da base pela altura estiver entre 0.95 e 1.05, é um quadrado
                if aspectRatio >= 0.95 and aspectRatio <= 1.05:
                  cv2.putText(img, "Quadrado", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                #Senão, é um retangulo
                else:
                  cv2.putText(img, "Retangulo", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

            #Se forem 5 bordas, é um pentagono
            elif len(approx) == 5:
                cv2.putText(img, "Pentagono", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

            #Se forem 6 bordas, é um pentagono
            elif len(approx) == 6:
                cv2.putText(img, "Hexagono", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

            #Se forem 10 bordas, é uma estrela
            elif len(approx) == 10:
                cv2.putText(img, "Estrela", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

            #Se nao se encaixa em nenhum dos outros, é um circulo
            # else:
            #     cv2.putText(img, "Circulo", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

        #Cria uma janela chamada formas que tem as bordas pintadas e nomeia cada forma geometrica
        cv2.imshow("Formas", img)

        #Se a tecla 'q' for pressionada, o programa se encerra
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #Se a flag for 0 ou nao existir, não foi possivel abrir a camera ou ela nao esta conectada
    else:
        print("Deu ruim kkkk")

#fecha todas as janelas contendo imagens abertas
cv2.destroyAllWindows()
