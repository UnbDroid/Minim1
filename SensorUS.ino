    #include <NewPing.h>

    #define TRIGGER_PIN  34
    #define US_FRONT     37
    #define US_SIDE      36
    #define MAX_DISTANCE 200

    #define direita 0
    #define esquerda 1
    #define frente 2
    #define tras 3
    #define ultra_f 4
    #define ultra_l 5

    int leitura_max = 0;
    int leitura_min = 1000;
    int contador = 0;
    int iteracoes = 5;
    int vetor_leituras_esquerda[5];
    int vetor_leituras_direita[5];
    int vetor_leituras_us_front[5];
    int vetor_leituras_us_lat[5];
    int validacao = 0;
    int leitura_dir = 0;
    int leitura_esq = 0;
    int leitura_front = 0;
    int leitura_lat = 0;


    NewPing usFront(TRIGGER_PIN, US_FRONT, MAX_DISTANCE);
    NewPing usSide(TRIGGER_PIN, US_SIDE, MAX_DISTANCE);

    int media_vetor(int direcao){
      int i=0, pos_min, pos_max;
      int minimo = 1000;
      int maximo = 0;
      int total = 0;

      if(direcao == 0){
        for(i=0; i<iteracoes; i++){
          if(vetor_leituras_direita[i] < minimo){
            minimo = vetor_leituras_direita[i];
            pos_max = i;
          }
          if(vetor_leituras_direita[i] > maximo){
            maximo = vetor_leituras_direita[i];
            pos_min = i;
          }
        }

        if(pos_min == pos_max){
          pos_max++;
        }

        vetor_leituras_direita[pos_min] = 0;
        vetor_leituras_direita[pos_max] = 0;

        for(i=0; i<iteracoes; i++){
          total += vetor_leituras_direita[i];
        }
      }else if(direcao == 1){
        for(i=0; i<iteracoes; i++){
          if(vetor_leituras_esquerda[i] < minimo){
            minimo = vetor_leituras_esquerda[i];
            pos_max = i;
          }
          if(vetor_leituras_esquerda[i] > maximo){
            maximo = vetor_leituras_esquerda[i];
            pos_min = i;
          }
        }

        if(pos_min == pos_max){
          pos_max++;
        }

        vetor_leituras_esquerda[pos_min] = 0;
        vetor_leituras_esquerda[pos_max] = 0;

        for(i=0; i<iteracoes; i++){
          total += vetor_leituras_esquerda[i];
        }
      }else if(direcao == 4){
        for(i=0; i<iteracoes; i++){
          if(vetor_leituras_us_front[i] < minimo){
            minimo = vetor_leituras_us_front[i];
            pos_max = i;
          }
          if(vetor_leituras_us_front[i] > maximo){
            maximo = vetor_leituras_us_front[i];
            pos_min = i;
          }
        }

        if(pos_min == pos_max){
          pos_max++;
        }

        vetor_leituras_us_front[pos_min] = 0;
        vetor_leituras_us_front[pos_max] = 0;

        for(i=0; i<iteracoes; i++){
          total += vetor_leituras_us_front[i];
        }
      }else if(direcao == 5){
        for(i=0; i<iteracoes; i++){
          if(vetor_leituras_us_lat[i] < minimo){
            minimo = vetor_leituras_us_lat[i];
            pos_max = i;
          }
          if(vetor_leituras_us_lat[i] > maximo){
            maximo = vetor_leituras_us_lat[i];
            pos_min = i;
          }
        }

        if(pos_min == pos_max){
          pos_max++;
        }

        vetor_leituras_us_lat[pos_min] = 0;
        vetor_leituras_us_lat[pos_max] = 0;

        for(i=0; i<iteracoes; i++){
          total += vetor_leituras_us_lat[i];
        }
      }

      return total/(iteracoes-2);
    }

    void le_ultra(){
      int f, l;
      if((f = usFront.ping_cm()) > 0 && (l = usSide.ping_cm()) > 0){
        vetor_leituras_us_front[contador] = f;
        vetor_leituras_us_lat[contador] = l;
        contador++;
      }


      while(contador<5){
        if((f = usFront.ping_cm()) > 0 && (l = usSide.ping_cm()) > 0){
          vetor_leituras_us_front[contador] = f;
          vetor_leituras_us_lat[contador] = l;
          contador++;
        }
      }

      if(contador == 5){
        contador = 0;
      }
    }

    void setup() {
      Serial.begin(9600);
    }

    void loop() {
      delay(500);

      le_ultra();

      leitura_front = media_vetor(4);
      leitura_lat = media_vetor(5);

      int i=0;
      Serial.print("Frente: ");
      Serial.print(leitura_front);
      Serial.println("cm");
      for(i=0; i<4; i++){
      Serial.print(vetor_leituras_us_front[i]);
      Serial.print(" ");
      }
      Serial.println(vetor_leituras_us_front[4]);
      Serial.print("Lado: ");
      Serial.print(leitura_lat);
      Serial.println("cm");
      for(i=0; i<4; i++){
      Serial.print(vetor_leituras_us_lat[i]);
      Serial.print(" ");
      }
      Serial.println(vetor_leituras_us_lat[4]);
    }
