#include <FlexCAN.h>
#include <LiquidCrystal.h>

//definicje wartości do resetowania mikroprocesora
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

//definicja wartości dla numerów pinów odpowiedzialnych za odczytywanie położenia joysticku
#define sensorPinV 33
#define sensorPinH 34

LiquidCrystal lcd(5, 6, 24, 25, 26, 27); //definicja obiektu wyświetlacza (RS, E, D4, D5, D6, D7)
int pos = 90; //ustalenie wartości początkowej pozycji serwa
CAN_message_t msg;// inicjalizacja obiektu do obsługi magistrali CAN
uint8_t state = 0;// zmienna stanu do obsługi głównej pętli switch
uint8_t debug = 1;//zmienna debugowania

/*
 * Funkcja wysyłania danych do głowicy za pomocą magistrali CAN
 */
void sendMsg(CAN_message_t &frame)
{
  //kod debugowania
  if (debug)
  {
    Serial.println("Now sending frame");
  }
  //kod debugowania
  Can0.write(frame);// zadanie wiadomości do obiektu magistrali CAN
}

/*
 * Klasa zawierająca metody obsługi wiadomości przychodzących
 * 
 * Dziedziczy z klasy CANListener w celu zapewnienia odbioru wiadomości poprzez
 * przerwanie
 */
class Reception : public CANListener
{
  public:
    void printFrame(CAN_message_t &frame, int mailbox); 
    void gotFrame(CAN_message_t &frame, int mailbox);
};

/*
 * Metoda przetwarzania odebranej ramki
 */
void Reception::printFrame(CAN_message_t &frame, int mailbox)
{
  if (debug)
  {
    Serial.println("Now in printFrame");
  }
  Serial.print("ID: ");
  Serial.print(frame.id, HEX);
  Serial.print(" Data: ");
  for (int c = 0; c < frame.len; c++)
  {
    Serial.print(frame.buf[c], HEX);
    Serial.write(' ');
  }
  Serial.print("Mailbox ID: ");
  Serial.print(mailbox);
  Serial.write('\r');
  Serial.write('\n');
  msg = frame;
  state = 1;
}

/*
 * Metoda obsługi odbioru ramki wykorzystujaca przerwanie
 */
void Reception::gotFrame(CAN_message_t &frame, int mailbox)
{
  if (debug)
  {
    Serial.println("Now in gotFrame");
  }
  printFrame(frame, mailbox);
}

/*
 * Główna klasa zawierająca metody sterujące
 */
class MainBody : public Reception
{
  
  public:
    volatile unsigned long rangeAvalue; //deklaracja globalnej zmiennej klasy MainBody do przechowywania odległości do obiektu A
    volatile unsigned long rangeBvalue; //deklaracja globalnej zmiennej klasy MainBody do przechowywania odległości do obiektu B
    volatile double distanceABvalue; //deklaracja globalnej zmiennej klasy MainBody do przechowywania odległości pomiędzy obiektem A i B
    int angl; //deklaracja globalnej zmiennej klasy MainBody do przechowywania kąta pokonanego przez układ pomiarowy pomiędzy obiektem A i B

  public:
    void getPos(CAN_message_t &frame);
    void rangeA();
    void rangeB();
    void distanceAB();
};

/*
 * Metoda zamieniająca ruch joysticka na odległość kątową dla serwomechanizmów
 */
void MainBody::getPos(CAN_message_t &frame)
{
  float sensorValue; //zmienna przechowująca wartość odczytaną przetwornika pomiarowego
  sensorValue = analogRead(sensorPinV); //odczyt wartości rezystancji joysticka dla zadanej płaszczyzny
  sensorValue = map(sensorValue, 0, 1023, -1, 1); //mapowanie odczytanej wartości do wartości pomiędzy [-2, 2]
  msg.buf[0] = msg.buf[0] + sensorValue; //dodanie do zmiennej przechowującej wartość kąta zmiany wprowadzanej przez joystick
  if (sensorValue != 0)
  {
    if (msg.buf[0] > 179) //sprawdzenie warunku przekroczenia zakresu ruchu serwa w górę
    {
      msg.buf[0] = 179; //jeśli zakres przekroczony ustaw maksymalną wartość
    }
    if (msg.buf[0] < 2) //sprawdzenie przekroczenia zakresu ruchu serwa w dół
    {
      msg.buf[0] = 2; // jeśli zakres przekroczony ustawiamy minimalną wartość
    }
  }
  //delay(10); //gdyby coś nie nadążało

  sensorValue = analogRead(sensorPinH);
  sensorValue = map(sensorValue, 0, 1023, -1, 1);
  msg.buf[1] = msg.buf[1] + sensorValue;
  if (sensorValue != 0)
  {
    if (msg.buf[1] > 179) //sprawdzenie warunku przekroczenia zakresu ruchu serwa w górę
    {
      msg.buf[1] = 179; //jeśli zakres przekroczony ustaw maksymalną wartość
    }
    if (msg.buf[1] < 2) //sprawdzenie przekroczenia zakresu ruchu serwa w dół
    {
      msg.buf[1] = 2; // jeśli zakres przekroczony ustawiamy minimalną wartość
    }
  }
  //delay(10); //gdyby coś nie nadążało
}

/*
 * Metoda wykonująca pomiar do obiektu oznaczonego jako "A"
 */
void MainBody::rangeA()
{
  msg.buf[7] = 1;//zadanie wartosci wyzwalającej pomiar do odpowiedniego bufora na interfejsie CAN
  msg.buf[6] = 1;//zadanie wartości do zmiennej synchronizacyjnej
  sendMsg(msg); //wysłanie komunikatu do głowicy
  //sprawdzanie warunku odebrania wiadomości zwrotnej,
  //msg.buf[7] zerowany jest po stronie głowicy po odczytaniu wartości z dalmierza, podobnie jak msg.buf[6]
  Serial.println("Waiting for response...");
  noInterrupts(); //chwilowe wyłączenie obsługi przetwań na czas oczekiwania na odpowiedź zwrotną z głowicy
  while (msg.buf[7] != 0 && msg.buf[6] != 0)// oczekiwanie na odpowiedź z głowicy
  {
    digitalWrite(13, HIGH);
    delay(5);
    Serial.print(".");
  }
  digitalWrite(13, LOW);
  interrupts(); //ponowne włączenie głowicy 
  //reasemblacja słowa bitowego podzielonego w celu przesłania przez interfejs CAN
  unsigned long highWord = word(msg.buf[2], msg.buf[3]);
  unsigned long lowWord = word(msg.buf[4], msg.buf[5]);
  rangeAvalue = highWord << 16 | lowWord;
  //kod debugowania
  if (debug)
  {
    Serial.println(rangeAvalue);
  }
  //kod debugowania
  
  //wyświetlenie zmierzonej odległości na wyświetlaczu
  int disp = rangeAvalue;
  lcd.clear();//czyszczenie wyświetlacza
  lcd.print("Pomiar do A:");
  lcd.setCursor(0, 1);//zadanie pozycji karetki na nową linię
  int v = lcd.print(disp);//odczyt długości sekwencji znaków
  lcd.setCursor(v, 1);//zadanie pozycji karetki na pozycję za wyświetlonym wynikiem pomiaru
  lcd.print(" cm");
  //koniec kodu wyświetlania na wyświetlaczu
  
  angl = msg.buf[0];//przypisanie do zmiennej wartości kąta zadanej do serwa ruchu poziomego
  state = 0;//zmiana stanu układu na stan 0
}

/*
 * Metoda wykonująca pomiar do obiektu oznaczonego jako "B"
 */
void MainBody::rangeB()
{
  msg.buf[7] = 1;//zadanie wartosci wyzwalającej pomiar do odpowiedniego bufora na interfejsie CAN
  msg.buf[6] = 1;//zadanie wartości do zmiennej synchronizacyjnej
  sendMsg(msg);//wysłanie komunikatu do głowicy
  //sprawdzanie warunku odebrania wiadomości zwrotnej,
  //msg.buf[7] zerowany jest po stronie głowicy po odczytaniu wartości z dalmierza, podobnie jak msg.buf[6]
  Serial.println("Waiting for response...");
  noInterrupts();//chwilowe wyłączenie obsługi przetwań na czas oczekiwania na odpowiedź zwrotną z głowicy
  while (msg.buf[7] != 0 && msg.buf[6] != 0)// oczekiwanie na odpowiedź z głowicy
  {
    digitalWrite(13, HIGH);
    delay(5);
    Serial.print(".");
  }
  digitalWrite(13, LOW);
  interrupts();//ponowne włączenie głowicy 
  unsigned long highWord = word(msg.buf[2], msg.buf[3]);
  unsigned long lowWord = word(msg.buf[4], msg.buf[5]);
  rangeBvalue = highWord << 16 | lowWord;
  //wyświetlenie zmierzonej odległości na wyświetlaczu
  int disp = rangeBvalue;
  lcd.clear();
  lcd.print("Pomiar do B:");
  lcd.setCursor(0, 1);
  int v = lcd.print(disp);
  lcd.setCursor(v, 1);
  lcd.print(" cm");
  delay(2000);// dwie sekundy na odczyt odległości do obiektu B dla większej ergonomiczności
  //koniec kodu wyświetlania na wyświetlaczu
  
  //kod debugowania
  if (debug) {
    Serial.println("In B");
    Serial.println(rangeBvalue);
  }
  //kod debugowania
  
  //przypisanie do zmiennej drugiej wartości kąta odczytanej po wyznaczeniu odległości do punktu B
  int anglB = msg.buf[0];
  //sprawdzenie warunków różnicy kątów
  //gdy kąt pierwszy jest większy od drugiego
  
  //kod debugowania
  if (debug) {
    Serial.println("kątA");
    Serial.println(angl);
    Serial.println("kątB");
    Serial.println(anglB);
  }
  //kod debugowania
  
  if (angl > anglB)
  {
    angl = angl - anglB;
  } else
    //gdy kąt pierwszy jest mniejszy lub równy drugiemu
  {
    angl = anglB - angl;
  }
  //kod debugowania
  if (debug) {
    Serial.println("kątAB");
    Serial.println(angl);
  }
  //kod debugowania
  state = 4;
}

/*
 * Funkcja wyliczająca dystans pomiędzy obiektem A i B w lini prostej
 */
void MainBody::distanceAB()
{
  double w = angl * (PI / 180);//zamiana stopni na radiany
  distanceABvalue = sqrt(((pow(rangeAvalue, 2) + pow(rangeBvalue, 2)) - (2 * rangeAvalue * rangeBvalue * cos(w))));// funkcja wyliczająca (twierdzenie cosinusów)
  
  //kod debugowania
  if (debug) {
    Serial.println("In AB");
    Serial.println(distanceABvalue);
  }
  //kod debugowania

  //wyświetlanie obliczonego wyniku
  lcd.clear();
  lcd.print("Range A to B:");
  double disp = distanceABvalue;
  lcd.setCursor(0, 1);
  int v = lcd.print(disp) + 3;
  lcd.setCursor(v, 1);
  lcd.print(" cm");
  //koniec kodu wyświetlania wyniku

  //zerowanie zmiennych pomiarowych
  rangeAvalue = 0;
  rangeBvalue = 0;
  distanceABvalue = 0;
  angl = 0;
  
  state = 0;//zmiana stanu układu na stan 0
}

//globalne wywołanie metod, aby można było z nich skorzystać w dowolnym miejscu programu
Reception reception;
MainBody mainBody;


//funkcja inicjalizująca
//--------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); //Prędkość portu szeregowego

  pinMode (18, INPUT_PULLDOWN); // Pin 18 do resetu
  pinMode (19, INPUT_PULLDOWN); // Pin 19 do odczytania odległości do punktu B
  pinMode (20, INPUT_PULLDOWN); // Pin 20 do odczytania odległości do punktu A

  attachInterrupt(digitalPinToInterrupt(18), resetFunction, RISING); //przerwanie od przycisku odpowiadającego zareset
  attachInterrupt(digitalPinToInterrupt(19), rangeBHandler, RISING); //przerwanie od przycisku odpowiadającego za pomiar do punktu B
  attachInterrupt(digitalPinToInterrupt(20), rangeAHandler, RISING); //przerwanie od przycisku odpowiadającego za pomiar do punktu A

  Can0.begin(1000000);//inicjalizacja interfejsu CAN
  Can0.attachObj(&reception);//przypisanie klasy do obiektu w bibliotece CANListener odpowiedzialnego za wyzwolenie przerwania
  reception.attachGeneralHandler();//przypisanie wszystkich dostępnych mailbox-ów do odbierania (można filtrować wiadomości
                                   //przypisując je do konkretnych mailboxów)
                                   
  //zadanie wartości podstawowych pozycji serw dla obiektu wiaddomości magistrali CAN
  msg.buf[0] = 90; //pozycja serwa V
  msg.buf[1] = 90; //pozycja serwa H
  msg.ext = 0; //flaga rozszerzonego ID magistrali CAN
  msg.id = 0x100; // numer ID wiadomości potrzebny do celów identyfikacyjnych protokołu CAN
  msg.len = 8; // długość wiadomości
  // pozostałe bufory
  msg.buf[2] = 0;
  msg.buf[3] = 0;
  msg.buf[4] = 0;
  msg.buf[5] = 0;
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  //inicjalizacja wyświetlacza
  Serial.println("LCD initialization");
  lcd.begin(16, 2); //Deklaracja typu
  lcd.setCursor(5, 0); //Ustawienie kursora
  lcd.print("Witaj!"); //Wyświetlenie tekstu
  delay(1000);
  lcd.clear();
  lcd.print("Wykonaj pierwszy");
  lcd.setCursor(5, 1);
  lcd.print("pomiar");

  pinMode(13, OUTPUT);//inicjalizacja diody wskazującej działanie systemu
  delay(30);
}

//główna pętla programu
//--------------------------------------------------------------------------------------------------
void loop()
{
  int pos1;
  int pos2;
  switch (state)
  {
    //stan domyslny układu
    default:
      digitalWrite(13, HIGH);// zapalenie diody na mikroukładzie
      pos1 = msg.buf[0];
      pos2 = msg.buf[1];
      mainBody.getPos(msg);
      if (msg.buf[0] != pos1 || msg.buf[1] != pos2) //sprawdzenie warunku zmiany pozycji dla jednego z serw
      {
        msg.buf[6] = 1;//zadanie wartości do zmiennej synchronizującej
                       //jest ona zerowana po stronie głowicy
        sendMsg(msg);//wysłanie komunikatu do głowicy
        state = 1;//zmiana stanu modułu na stan 1
        //delay(20); //gdyby coś nie nadążało
      }

      //indykatory pracy mikroprocesora (gdy procesor przetwarza dioda mruga, a na terminalu szeregowym wyświetlane są kropki)
      Serial.println(".");
      delay(5);
      digitalWrite(13, LOW);
      break;
    //stan nr 1 układu
    case 1:
      //oczekiwanie na wiadomość zwrotną z głowicy
      Serial.println("Waiting for response");
      while (msg.buf[6] != 0)
      {
        digitalWrite(13, HIGH);
        delay(5);
        Serial.print(".");
      }
      Serial.println("Responded");
      digitalWrite(13, LOW);//gdy układ otrzyma wiadomość z głowicy gaśnie dioda 
                            //(ciągłe jasne świecenie wskazuje na błąd w komunikacji)
      state = 0;//zmiana stanu układu na stan 0
      break;
    //stan nr 2 układu
    case 2:
      mainBody.rangeA();//wywołanie funkcji do pomiaru odległości do obiektu A
      break;
    //stan nr 3 układu
    case 3:
      mainBody.rangeB();//wywołanie funkcji do pomiaru odległości do obiektu B
      break;
    //stan nr 4 układu
    case 4:
      mainBody.distanceAB();//wywołanie funkcji obliczającej odległość liniową pomiędzy obiektem A i B
      break;
  }
}
/*
 * Funkcje wywoływane przerwaniem
 */
void rangeAHandler()//zmiana stanu układu na "2" aby wykonał pomiar do obiektu A
{
  state = 2;
}

void rangeBHandler()//zmiana stanu układu na "3" aby wykonał pomiar do obiektu B
{
  state = 3;
}

void resetFunction()//reset całego systemu
{
  msg.buf[6] = 2;//zadanie do zmiennej synchronizującej wartości wywołującej reset po stronie głowicy
  sendMsg(msg);//wysłanie wiadomości
  delay(100);//drobne opóźnienie, aby przed resetem układ zdążył nadać wiadomość
  CPU_RESTART;//wywołanie resetu wpisaniem pod odpowiedni adres rejestru wartości resetu. 
}
