# Tradução de textos e comentários em 'Code.ino'

## Cabeçalho

Filament drying box controller.  
  
Este código foi baseado neste projeto:  
[Caixa Secadora de Filamentos Dry Fila Box](https://marlonnardi.com/2023/10/10/o-problema-que-acaba-com-suas-impressoes-3d-caixa-secadora-de-filamentos-dry-fila-box/)
  
O projeto consiste em um elemento de aquecimento controlado por um PID e ventiladores para manter a temperatura  
dentro de um ambiente fechado para ajudar a secar filamentos plasticos para impressoras 3D.  
O código e o PID controlam o elemento de aquecimento baseado na temperatura ambiente.  

Partes utilizadas:

- Arduino Pro Mini 5V
- Sensor de temperatura e humidade DHT22
- Um hot end com um dissipador de alumínio
- Sensor de temperatura DS18B20 para o dissipador
- Um ventilador 12V de 80mm/80mm
- Fonte de 12V / 10A
- Um modulo de encoder giratório digital KY040
- Um display LCD 20x4 com modulo I2C.
  
Em nossos experimentos, com todo o aparato o pico de corrente foi de aproximadamente 4.4 amperes.  
  
Conecções:

- DHT22 - pino de dados                : 12
- Hot end (controle do mosfet)         : 11
- Ventilador (controle do transistor)  : 10
- DS18B20 - pino de dados              : 9
- Encoder - pino CLK (A)               : 5
- Encoder - pino DT (B)                : 6
- Encoder - pino SW (chave)            : 2
  
O diagrama do circuito pode ser encontrado no post o qual nosso projeto se baseia.  
  
Este código é distribuido sob a licença MIT.  
  
## Comentários

### \[1\]

Nós tentamos usar a versão Arduino da biblioteca *FreeRTOS* para utilizar as funções de manipulação de texto e mecanismos de sincronização, mas não funcionou bem com a placa que estamos usando, o Pro Mini 5V.  
Eu também aprendi como o microcontrolador agenda execução de código e como *interrupts* funcionam, o que invalida o uso de semáforos e *mutexes* de qualquer maneira, já que todo output (serial e LCD) tem que vir do loop principal.

### \[2\]

Definindo níveis de temperatura.

### \[3\]

Definindo pinos do sensor DHT22 e o modo de operação.

### \[4\]

Pinos de IO.

### \[5\]

Valor PWM do pino do ventilador.

### \[6\]

O encoder rotativo KY040 usa chaves internas para sinalizar mudanças.  
Como toda chave mecânica eles podem sinalizar mais de um acionamento, efeito conhecido como *bouncing*, isto é, você gira o encoder ou aperta o botão e ele manda mais de um sinal.  
A biblioteca *KY040.h* faz um trabalho fantástico evitando *bounces* quando girando o encoder, mas não suporta a chave. Por isso nós implementamos um algorítmo de *debouncing* bem rudimentar que rejeita sinais extra que venham antes do tempo estabelecido passar.

### \[7\]

Define por quanto tempo devemos segurar o botão do encoder apertado para mudar o estado da luz da tela LCD.

### \[8\]

Uma enumeração representando os modos do ventilador do dissipador.

- ON    : O ventilador está sempre ligado
- OFF   : O ventilador está sempre desligado
- AUTO  : O ventilador está ligado ou desligado de acordo com nosso algorítmo

### \[9\]

Variáveis globais.

- bool startup: Usada para determinar se o sistema foi iniciado pela primeira vez
- bool fan_on: *True* se o ventilador do dissipador está ligado
- float temperature: A temperatura ambiente
- float relative_humidity: A humidade relativa
- float absolute_humidity: A humidade absoluta
- float heatsink_temp: A temperatura do dissipador
- int enc_pina_prev: Variável para armazenar o estado anterior do pino *A* (CLK) do encoder
- double input, set_point, output: Variáveis usadas pelo controlador PID. input == temperatura, set_point == alvo, output == saída do PID
- volatile double target_temp: A temperatura alvo definida pela posição do encoder
- volatile bool isr_invalidate_next: Usada para invalidar o processamento de *interrupts* para o mesmo pino para evitar *bouncing*
- volatile bool backlight_on: Possui o valor usado para determinar se ligamos ou desligamos a luz de fundo da tela LCD
- heatsink_fan_mode_t hs_prev_mode: O modo anterior do ventilador
- volatile heatsink_fan_mode_t hs_fan_mode O modo de operação do ventilador usada pela *ISR*
- volatile unsigned long debounce_millis: Guarda em milisegundos o tempo desde quando o controlador foi iniciado para que possamos aplicar o *debouncer*

### \[10\]

Criando dispositivos.

- DeviceAddress tsens_addr: Utilizado pelo sensor de temperatura do dissipador
- OneWire one_wire(PIN_TSENS): Utilizado pelo sensor de temperatura do dissipador
- DallasTemperature tsens(&one_wire): Utilizado pelo sensor de temperatura do dissipador
- DHT_Unified dht(DHTPIN, DHTTYPE): Utilizado pelo sensor DHT22
- LiquidCrystal_I2C lcd(0x27, 20, 4): Utilizado para controlar a interface *I2C* da tela LCD
- PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT): O controlador PID. 'P_ON_M' para medida proporcional, isso ajuda evitar altas oscilações quando a temperatura está chegando ou passando do alvo.
- KY040 encoder(PIN_ENC_CLK, PIN_ENC_DT): O encoder usado para controlar a temperatura alvo

### \[11\]

Caractere especial para o display que representa o símbolo '°'.  

### \[12\]

```c
/**
* @brief Habilita interrupts para o pino.
*
* @param pin O número GPIO do pino para o qual habilitar *interrupts*.
* @returns Nada.
*/
void pci_setup(byte pin) {
  // Mudando o bit correspondente ao pino.
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));

  // Limpando interrupts.
  PCIFR |= bit(digitalPinToPCICRbit(pin));

  // Habilitando interrupts para o grupo.
  PCICR |= bit(digitalPinToPCICRbit(pin));
}
```

### \[13\]

```c
/**
* @brief A rotina de serviço de *interrupt* (Interrupt service routine - ISR) para ser chamada quando o estado dos pinos habilitados para *interrupt* mudar.
* Essa rotina cuida do input do usuário no encoder digital rotativo. O comportamento é:
*     - Se rotacionado no sentido horário: Aumenta a temperatura alvo
*     - Se rotacionado no sentido anti-horário: Diminui a temperatura alvo
*     - Se o botão for pressionado (por menos de 1 segundo): Muda o modo de operação do ventilador
*     - Se o botão for pressionado (por mais de um segundo): Liga ou desliga a luz de fundo do LCD
*/
ISR(PCINT2_vect) {
  // Variável indicando que é a primeira vez que a ISR foi engatilhada.
  static bool first = true;

  // Pegando o estado do encoder para checar se ele foi rotacionado e garantindo que nossa temperatura alvo está dentro dos nossos limites.
  switch (encoder.getRotation()) {
    case KY040::CLOCKWISE:
      target_temp = target_temp < MAX_TEMP ? target_temp + 1 : MAX_TEMP;
      break;

    case KY040::COUNTERCLOCKWISE:
      target_temp = target_temp > MIN_TEMP ? target_temp - 1 : MIN_TEMP;
      break;
  }

  // Checando se o botão do encoder foi pressionado.
  if (digitalRead(PIN_ENC_SW) == LOW) {
    // Os milisegundos atuais desde que o controlador foi ligado.
    // Nós usamos para comparar com o estado anterior.
    unsigned long current_millis = micros() / 1000;
    
    // O timeout em microsegundos desde que nós chegamos nesse bloco de controle.
    unsigned long bl_timeout = 0;

    // Este loop espera até que o usuário solte o botão ou até o timeout expirar, determinando
    // a operação que iremos executar em seguida.
    do {
      // "Heeeeeey you shouldn't wait in an ISR maaaaaaaaaaan..." Yeah yeah master of all that is technological, don't tell me what to do.
      delayMicroseconds(100);
      bl_timeout += 100;
    } while (digitalRead(PIN_ENC_SW) == LOW && bl_timeout / 1000 < BACKLIGHT_OFF_TRIGGER_MS);

    if (bl_timeout / 1000 >= BACKLIGHT_OFF_TRIGGER_MS) {
      // O usuário segurou o botão por mais do que 'BACKLIGHT_OFF_TRIGGER_MS', então nós mudamos o estado da luz de fundo do LCD.
      backlight_on = !backlight_on;

      // Já que nós esperamos por 'BACKLIGHT_OFF_TRIGGER_MS', nosso tempo do debouncer já passou os 'DEBOUNCE_MS', então se nos temos um 'bounce' nós vamos acabar mudando o estado do ventilador. Nós usamos esta variável para invalidar a próxima mudança no pino, se ela acontecer antes do periodo 'DEBOUNCE_MS' passar. Nós não invalidamos após o período passar porque será um input de usuário legítimo.
      isr_invalidate_next = true;
    }
    else {
      // Se o período de 'debouncing' passo, ou esta é a primeira vez que apertamos o botão nós mudamos o modo do ventilador.
      if (isr_invalidate_next) {
        isr_invalidate_next = false;

        // A flag de invalidação está setada, Nós estamos dentro do período de 'debouncing'?
        if (current_millis - debounce_millis < DEBOUNCE_MS)
          // Sim. Vamos para o final.
          goto BTN_END;
        else
          // Não. Mudamos o modo do ventilador.
          goto CHANGE_FAN_MODE;
      }
      else {
        // A flag de invalidação não está setada, então nós checamos o período de 'debouncing' e mudamos o modo do ventilador, se aplicável.
        if ((current_millis - debounce_millis >= DEBOUNCE_MS || first))
          goto CHANGE_FAN_MODE;
        else
          goto BTN_END;
      }

      CHANGE_FAN_MODE:
      switch (hs_fan_mode) {
        case heatsink_fan_mode_t::AUTO:
          hs_fan_mode = heatsink_fan_mode_t::ON;
          break;

        case heatsink_fan_mode_t::ON:
          hs_fan_mode = heatsink_fan_mode_t::OFF;
          break;

        case heatsink_fan_mode_t::OFF:
          hs_fan_mode = heatsink_fan_mode_t::AUTO;
          break;
      }

      // Salvando os milisegundos, já que mudamos o modo.
      debounce_millis = current_millis;

      BTN_END:
      if (first)
        first = false;
    }
  }
}
```

### \[14\]

```c
void setup() {
  // Inicializando a comunicação serial.
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println("####################\n##    Starting    ##\n####################");
  Serial.println();
  Serial.println();
  Serial.println();

  // Mudando o modo dos pinos GPIO.
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_H_ELEMENT, OUTPUT);

  // Habilitando 'interrupts' para os pinos do encoder.
  pci_setup(PIN_ENC_CLK);
  pci_setup(PIN_ENC_DT);
  pci_setup(PIN_ENC_SW);

  // Desligando o ventilador do dissipador.
  analogWrite(PIN_FAN, 0);

  // Setando o 'setpoint' para a temperatura média entre o mínimo e maximo e configurando o PID.
  set_point = target_temp = ((MAX_TEMP - MIN_TEMP) / 2) + MIN_TEMP;
  pid.SetMode(AUTOMATIC);

  // Inicializando os sensores.
  dht.begin();
  tsens.begin();

  // Inicializando a tela LCD.
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, customChar);
}
```

### \[15\]

```c
void loop() {
  static bool our_backlight_on;
  static heatsink_fan_mode_t our_fan_mode;

  // Lendo as variáveis voláteis setadas pela ISR.
  // Durante a leitura nós desabilitamos 'interrupts' (cli()) para evitar 'race conditions'.
  cli();
  set_point = target_temp;          // Mudando o 'setpoint' do PID baseado na temperatura alvo.
  our_fan_mode = hs_fan_mode;       // Salvando o modo do ventilador para que possamos analizar.
  our_backlight_on = backlight_on;  // Salvando o estado da luz de fundo do LCD.
  sei();                            // Re-habilitando 'interrupts'.

  // Nós imprimimos todo o status duas vezes no loop. Fazemos isso porque quando os sensores estão funcionand leva algum tempo para fazer todas as leituras, cálculos e controlar a temperatura.
  // Se o usuário girar o encoder vai levar mais tempo para que a nova temperatura alvo reflita na tela LCD.
  // Imprimindo o status aqui torna tudo mais responsivo, mas imprime os valores antigos. Por isso imprimimos uma segunda vez mais tarde.
  print_status();

  // Gerenciando a luz de fundo do LCD de acordo com o estado.
  if (our_backlight_on)
    lcd.backlight();
  else
    lcd.noBacklight();

  if (our_fan_mode != hs_prev_mode) {
    // O modo do ventilador mudou, então imprimimos o estado na tela LCD.
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Heatsink fan mode:");

    switch (our_fan_mode) {
      case heatsink_fan_mode_t::AUTO:
        lcd.setCursor(8, 2);
        lcd.print("AUTO");
        break;

      case heatsink_fan_mode_t::ON:
        lcd.setCursor(9, 2);
        lcd.print("ON");
        break;

      case heatsink_fan_mode_t::OFF:
        lcd.setCursor(8, 2);
        lcd.print("OFF");
        break;
    }

    delay(1000);
    lcd.clear();

    // Salvando o modo atual.
    hs_prev_mode = our_fan_mode;
  }

  // Lendo a temperatura ambiente.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  temperature = event.temperature;

  // Lendo a humidade relativa do ambiente.
  dht.humidity().getEvent(&event);
  relative_humidity = event.relative_humidity;

  // Lendo a temperatura do dissipador.
  heatsink_temp = get_heatsink_temperature();

  if (!isnan(temperature) && !isnan(relative_humidity)) {
    // Nós temos a temperatura e humidade relativa. Vamos calcular a humidade absoluta e controlar o PID.
    absolute_humidity = get_abs_humidity();
    control_temperature();
  }
  else {
    temperature = 0.0;
    relative_humidity = 0.0;
  }

  // Algorítimo para decidir se ligamos ou desligamos o ventilador do dissipador.
  // Fazemos isso porque o elemento de aquecimento não consegue acompanhar a diminuição da temperatura quando a temperatura é muito baixa. Então nós ligamos ou desligamos o ventilador para obter uma transferencia de temperatura mais eficiente.

  // Workflow:
  //    - Se nós acabamos de inicializar esperamos até que o dissipador alcance nossa temperatura alvo máximo antes de ligar o ventilador.
  //    - Checamos então se o ventilador está ligado.
  //        - Se estiver ligado, a temperatura do dissipador é menor que a nossa temperatura alvo + o diferencial e a temperatura ambiente é menor que a temperatura alvo menos a diferença significa que a câmara e o dissipador estão frios de mais, então desligamos o ventilador.
  //        - Se o ventilador estiver desligado e a temperatura do dissipador atingiu nosso máximo nós ligamos o ventilador.
  //
  // Isso significa que nós permitiremos que o elemento aquecedor permaneça quente enquanto a temperatura ambiente é baixa de mais e quando a temperatura estiver perto o suficiente do alvo nós mantemos o ventilador ligado para melhorar a circulação.
  //
  // Se o algorítmo é utilizado é determinado pelo valor de 'hs_fan_mode'.
  switch (our_fan_mode) {
    // Modo 'ON'. O ventilador ficará sempre ligado.
    case heatsink_fan_mode_t::ON:
      {
        if (!fan_on) {
          analogWrite(PIN_FAN, HS_FAN_SPEED);
          fan_on = true;
        }
      }
      break;

    // Modo 'OFF'. O ventilador ficará sempre desligado.
    case heatsink_fan_mode_t::OFF:
      {
        if (fan_on) {
          analogWrite(PIN_FAN, 0);
          fan_on = false;
        }
      }
      break;

    // Modo 'AUTO'. O ventilador vai ficar ligado ou desligado de acordo com as condições ambientais.
    case heatsink_fan_mode_t::AUTO:
      {
        if (startup) {
          // Nós acabamos de iniciar, então somente ligamos o ventilador se a temperatura do dissipador é maior do que nosso máximo, ou a temperatura ambiente é maior do que o alvo, menos o diferencial.
          if (heatsink_temp >= HS_MAX_TEMP || temperature >= set_point - TEMP_TGT_DIFF) {
            analogWrite(PIN_FAN, HS_FAN_SPEED);
            startup = false;
            fan_on = true;
          }
          else {
            // Nós acabamos de iniciar, mas nossas temperaturas não são ideais. Então desligamos o ventilador.
            if (fan_on) {
              analogWrite(PIN_FAN, 0);
              fan_on = false;
            }
          }
        }
        else {
          // Nós já passamos da fase de inicialização, então se o ventilador estiver ligado nós desligamos quando a temperatura do dissipador é menor do que a temperatura ambiente mais o valor de 'cuttoff' e a temperatura ambiente é menor do que a temperatura alvo, menos o diferencial.
          if (fan_on && heatsink_temp <= temperature + HS_TEM_DIFF_CUT && temperature <= set_point - TEMP_TGT_DIFF) {
            analogWrite(PIN_FAN, 0);
            fan_on = false;
          }

          // O ventilador está desligado, então nós ligamos se a temperatura do dissipador é maior do que nosso máximo ou a temperatura ambiente é maior do que nosso alvo menos o diferencial.
          else if (!fan_on && (heatsink_temp >= HS_MAX_TEMP || temperature >= set_point - TEMP_TGT_DIFF)) {
            analogWrite(PIN_FAN, HS_FAN_SPEED);
            fan_on = true;
          }
        }
      }
      break;
  }

  // Imprimindo o status de novo.
  print_status();

  // Dormindo. Este período é as vezes insuficiente para que os sensores atualizem, o que faz com que não possamos ler as temperaturas, mas isso é só de vez em quando. Para ser honesto não existe mal em aumentar o período aqui, a não ser alterar a responsividade do input do usuário.
  delay(50);
}
```

### \[16\]

```c
/**
* @brief Imprime o status para a tela LCD. O status inclui as temperaturas ambiente, alvo e do dissipador, umidade relativa e absoluta, status do ventilador do dissipador, e a porcentagem do PWM sendo aplicado no elemento aquecedor.
*
* @returns Nada.
*/
void print_status(void) {
  static uint8_t prev_power = 0;

  // Imprimindo a temperatura ambiente.
  lcd.print("Tmp:");
  lcd.print(temperature);
  lcd.write(0);
  lcd.print("C/");
  lcd.print(set_point);
  lcd.write(0);
  lcd.print("C");

  // Imprimindo a umidade relativa e absoluta.
  lcd.setCursor(0, 1);
  lcd.print("RH:");
  lcd.print(relative_humidity);
  lcd.print("% AH:");
  lcd.print(absolute_humidity);
  lcd.print("%");

  // Imprimindo a temperatura do dissipador.
  lcd.setCursor(0, 2);
  lcd.print("HS temp:");
  lcd.print(heatsink_temp);
  lcd.write(0);
  lcd.print("C");

  // Imprimindo se o ventilador está ligado e a potencia equivalente do valor do PWM no elemento aquecedor.
  lcd.setCursor(0, 3);
  lcd.print("Fan on:");
  lcd.print(fan_on ? "Yes" : "No ");
  lcd.print(" PWR:");

  uint8_t current_power = (uint8_t)((output / 255) * 100);

  // Se temos menos dígitos nós limpamos a tela LCD para evitar células fantasma.
  if (get_digit_count(current_power) < get_digit_count(prev_power))
    lcd.clear();

  lcd.print(current_power < 100 ? current_power : 100);
  lcd.print("%");

  prev_power = current_power;

  // Setando o cursor para o início do display.
  lcd.home();
}
```

### \[17\]

```c
/**
* @brief Controla o elemento aquecedor com o controlador PID.
*
* @returns Nada.
*/
void control_temperature(void) {

  // Antigo método de setar a temperatura alvo, usando um potenciômetro e entrada analógica.

  // Lendo os valores no potenciômetro e calculando a porcentagem, ou posição que ele esta.
  // O 'cast' para 'float' é importante aqui antes de fazer os cálculos.
  // int potValue = analogRead(PIN_POT);
  // float percent = (float)potValue / 1023;

  // Aplicando a porcentagem na diferença entre a temperatura alvo mínimo e máximo.
  // Então setamos o valor do 'setpoint' antes de chamar 'pid.Compute().
  // Nós fazemos o cast para 'int' aqui antes do cast para 'double' para que mantemos a temperatura sempre em incrementos de um grau. Isso é útil porque já que estamos usando um simples potenciômetro e leituras analógicas nós estamos sucetíveis a flutuações e interferencia. Em uma versão futura o potenciômetro será substituido por um encoder rotativo digital para manter tudo mais estável.
  // set_point = (double)((int)(MIN_TEMP + ((MIN_TEMP - MAX_TEMP) * percent)));

  if (!isnan(temperature)) {
    // Computando a saída PWM e aplicando no pino do elemento aquecedor.
    input = temperature;
    pid.Compute();
    analogWrite(PIN_H_ELEMENT, output);
  }
}
```

### \[18\]

```c
/**
* @brief Retorna a temperatura do dissipador em graus célsius lendo o sensor DS18B20.
*
* @returns A temperatura em graus célsius..
*/
float get_heatsink_temperature(void) {
  // Pedindo ao sensor para calcular a temperatura.
  tsens.requestTemperatures();
  if (!tsens.getAddress(tsens_addr, 0))
    return 0.0;

  // Convertendo a temperatura para célsius e retornando.
  return tsens.getTempC(tsens_addr, 1);
}
```

### \[19\]

```c
/**
* @brief Converte a umidade relativa para umidade absoluta.
*
* @returns A umidade absoluta.
*/
float get_abs_humidity(void) {
  return ((6.112 * (pow(EULER, ((17.67 * temperature) / (temperature + 243.5)))) * relative_humidity * 2.1674) / (273.15 + temperature));
}
```

### \[20\]

```c
/**
* @brief Retorna o número de digitos para um certo 'unsigned byte' (número positivo de 8 bits).
*
* @param[in] number O número para contar os dígitos.
* @returns O número de dígitos.
*/
uint8_t get_digit_count(uint8_t number) {
  // Não é necessariamente bonito, mas é rápido, já que sabemos que nosso número é pequeno.
  // https://stackoverflow.com/questions/1068849/how-do-i-determine-the-number-of-digits-of-an-integer-in-c
  if (number < 10) return 1;
  if (number < 100) return 2;

  return 3;
}
```
