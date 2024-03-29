
#include <TinyGPS.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <SlowMotionServo.h>
#include <Servo.h>
 SoftwareSerial SIM900(19, 18);
 /*-------------------------------------------------------------------*/
const String PHONE = "your number"; // change this
/*-------------------------------------------------------------------*/
const int BUZZER = 10 ;
/*-------------------------------------------------------------------*/
float distance ;                          

/*-------------------------------------------------------------------*/
int angle_barriere_fermee = 40; // angle du servo pour mettre la barriere en position basse
int angle_barriere_ouverte = 110; // angle du servo pour mettre la barriere en position haute
boolean barriere_ouverte; // variable à 1 si les barrières sont ouvertes et à 0 si elles sont fermées
int vitesse_barriere = 70; // temps entre 2 degrés
int angle_barriere;
boolean etat_signal = false;  // une variable contenat 0 si les leds sont eteintes et 1 si elles sont allumlées
unsigned long compteur_temps; // Une varible qui va permettre de compter le temps pour faire clignoter à intervale fixe les leds
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Création d'une instance pour l'objet TinyGPS
TinyGPS gps;
float latitude, longitude;
/*----------------------------------------------------*/
const byte pb1 = 22; // pedale de detection 
const byte pb2 = 23; //pedale de rearement
/*----------------------------------------------------*/
const int barriereA = 6; // le servoA est reliée à l'entrée/sortie numérique 2 de la carte Arduino
const int barriereB = 13; // le servoB est reliée à l'entrée/sortie numérique 4 de la carte Arduino
const int train=21 ; // le servo c qui joue le role du train est lié à la broche 21 de la carte arduino 
/*----------------------------------------------------*/
//leds qui vont clignotent
int led1 = 8;
int led2= 9 ;
/*----------------------------------------------------*/
/*-------------------------------------------------------*/
/* Les servos */
Servo servoA;
Servo servoB;
Servo servoC; // le train  
/*-------------------------------------------------------*/
int pinPotar=A10; //pin de potentiometre 
/*-------------------------------------------------------*/
// Alarm
int buzzer_timer = 0;
bool alarm = false;
boolean send_alert_once = true;
//----------------------------------
// keep track of time, after how much time the messages are sent
 
// Tracks the time since last event fired
boolean multiple_sms = false;
unsigned long previousMillis=0;
unsigned long int previoussecs = 0; 
unsigned long int currentsecs = 0; 
 unsigned long currentMillis = 0; 
 int secs = 0; 
 int pmints = 0; 
 int mints = 0; // current mints
 int interval= 1 ; // updated every 1 second
//--------------------------------------------------------------
// Size of the geo fence (in meters)qui represente la distance entre le PN et la pedale d'annonce
const float maxDistance = 1400 ;
//--------------------------------------------------------------
float initialLatitude = 10.853739;  // latitude initial 36.853739
float initialLongitude = 8.092840; // longitude initial 11.092840
// Ceci est l'endroit où nous déclarons le prototypes pour la fonction 
// qui utilisera la bibliothéque TinyGPS et controlera l'affichage 
void getgps(TinyGPS &gps);
void setup()
{

  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT);
  // make the pushbutton's pin an input:
  pinMode(pb1, INPUT);
  pinMode(pb2, INPUT);
  // les leds output
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  // On relie caque objet servo à la broche qui lui a été désignée
  servoA.attach(barriereA);
  servoB.attach(barriereB);
  servoC.attach(train);
  // On place les barrières en position ouverte
  servoA.write(angle_barriere_ouverte);
  servoB.write(angle_barriere_ouverte);
  barriere_ouverte = true; 
  SIM900.begin(9600);  
  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(2,1);
  lcd.print("En attente ...");
}
 /*****************************************************************************************
 * sendAlert() Function : quand le train entre dans la zone d'annoncement
*****************************************************************************************/
void sendAlert()
{
  //return;
  String sms_data;
  sms_data = "Alert! le train est arrivée .\r";
  sms_data += "http://maps.google.com/maps?q=loc:";
  sms_data += String(latitude) + "," + String(longitude);
 
  //return;
  SIM900.print("AT+CMGF=1\r");
  delay(1000);
   SIM900.print("AT+CMGS=\""+PHONE+"\"\r");
  delay(1000);
   SIM900.print(sms_data);
  delay(100);
   SIM900.write(0x1A); //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
 Serial.println("SMS Sent Successfully.");
  
}

 
// La fonction get gps va obtenir et imprimer les valeurs que nous voulons
void getgps(TinyGPS &gps)
{
  // appelle de la fonction qui récupère la latitude et la longitude
  gps.f_get_position(&latitude, &longitude);
  
  // clear LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print ("localisation du train le plus proche");
   lcd.setCursor(0,1);
  lcd.print("Lat: "); 
  lcd.print(latitude,5); 
  lcd.setCursor(0,2); 
  lcd.print("Long: "); 
  lcd.print(longitude,5);
}
   void SendSMS()
{ 
  
  SIM900.println("AT+CMGF=1");    //To send SMS in Text Mode
  delay(1000);
  SIM900.println("AT+CMGS=\"+21621242393\"\r"); //Change to destination phone number 
  delay(1000);
  SIM900.print("https://www.google.com/maps/?q=");
  //Serial.print("Latitude :");
  SIM900.print(latitude, 6);
  //Serial.print("Longitude:");
  SIM900.print(",");
  SIM900.print(longitude, 6);
  delay(1000);
  SIM900.println((char)26); //the stopping character Ctrl+Z
  delay(1000);  
}
 void Signal() {
 
      digitalWrite(led1,HIGH);
      digitalWrite(led2,HIGH);
      delay (100) ;
      digitalWrite(led1,LOW);
      digitalWrite(led2,LOW);
      delay (100); 
    }
/*-------------------------------------------------------*/
//fonction de sonnerie
/*-------------------------------------------------------*/

void son(){
  tone(BUZZER, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(BUZZER);     // Stop sound...
  delay(1000);        // ...for 1sec
}
/*-------------------------------------------------------
  fonction d'affichage lcd
/*-------------------------------------------------------*/
void affichage () {
  getDistance ;  
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.println("soyez prudents ! "); 
  delay (500) ;
  lcd.setCursor(0,1); 
  lcd.print("le train est a : " );
  lcd.setCursor(0,2); 
  lcd.print(distance);
  lcd.print("metres" );
  delay (500);
  lcd.clear();
  lcd.setCursor(0,2);
  lcd.println("stop ! " );
  delay (200);
}

/*****************************************************************************************
* getDistance() function : pour calculer la distance entre le train et le passage a niveau
*****************************************************************************************/
 
// Calculate distance between two points
float getDistance(float flat1, float flon1, float flat2, float flon2) {
 
  // Variables
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
 
  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));
 
  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;
 
  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters
 
  return dist_calc;
}
 /*--------------------------------------------------------------
fonction s'execute dans le cas ou les pedales ne fonctionne pas (GEOFENCING)
 /--------------------------------------------------------------*/
 void GEOFENCING () {
  getgps(gps) ;
  //--------------------------------------------------------------
  float distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);
  //--------------------------------------------------------------
  Serial.print("Latitude= "); Serial.println(latitude, 6);
  Serial.print("Lngitude= "); Serial.println(longitude, 6);
  Serial.print("initialLatitude= "); Serial.println(initialLatitude, 6);
  Serial.print("initialLngitude= "); Serial.println(initialLongitude, 6);
  Serial.print("current Distance= "); Serial.println(distance);
  //--------------------------------------------------------------
  // Set alarm on?
  if(distance < maxDistance) {
    multiple_sms = true;
    //------------------------------------------
    if(send_alert_once == true){
      digitalWrite(BUZZER, HIGH);
      sendAlert();
      alarm = true;
      send_alert_once = false;
      buzzer_timer = millis();
      
    }
    //------------------------------------------
  }
  else{
    send_alert_once = true;
    multiple_sms = false;
  }
  //--------------------------------------------------------------
 
  // Handle alarm
  if (alarm == true) {
    if (millis() - buzzer_timer > 5000) {
      digitalWrite(BUZZER, LOW);
      alarm = false;
      buzzer_timer = 0;
      
    }
  }
  if ( multiple_sms = true)
  {
       currentMillis = millis();
       currentsecs = currentMillis / 1000; 
       if ((unsigned long)(currentsecs - previoussecs) >= interval) {
       secs = secs + 1; 
 
       if ( secs >= 20)
       {
        sendAlert();
        multiple_sms = false;
        secs = 0;
       }
    }
  }
  //--------------------------------------------------------------  
  while(SIM900.available()){
    Serial.println(SIM900.readString());
  }
  //--------------------------------------------------------------
  while(Serial.available())  {
    SIM900.println(Serial.readString());
  }
  //--------------------------------------------------------------
}

/*-------------------------------------------------------
  fonction de mouvement du train 
/*-------------------------------------------------------*/
void mvmttrain() {
int valeurPotar=analogRead (pinPotar); // lire la valeur de potentiomtre
int angleC=map(valeurPotar , 0 , 1023 , 0 ,179); // transformation en angle 
servoC.write (angleC);}
//--------------------------------------------------------------

void loop()
{
  bool newData = false;
  int buttonState1 = digitalRead(pb1);
  int buttonState2 = digitalRead(pb2);
// On fait une boucle que le programme va executer indéfiniment

    if ( buttonState1 == LOW && buttonState2 == LOW) {
       /*for (int angle_barriere=angle_barriere_fermee; angle_barriere<=angle_barriere_ouverte; angle_barriere+=1){ //OUVERTURE BARRIERES
      servoA.write(angle_barriere);
     servoB.write(angle_barriere);
     delay(vitesse_barriere);
     }*/
      servoA.write(angle_barriere_ouverte);
      servoB.write(angle_barriere_ouverte);
      barriere_ouverte = true;
      digitalWrite(led1,LOW);
      digitalWrite(led2,LOW);
 
    }
 
 else if (( buttonState1 == HIGH )&& (buttonState2 == LOW)) {
   do {   
    Signal () ;
    son() ;
    affichage () ;
    sendAlert();
    /*for (int angle_barriere=angle_barriere_ouverte; angle_barriere>=angle_barriere_fermee; angle_barriere-=1){ //FERMETURE BARRIERES
     servoA.write(angle_barriere);
     servoB.write(angle_barriere);
     delay(vitesse_barriere); // Vitesse angle barrière
    }*/
    servoA.write(angle_barriere_fermee);
    servoB.write(angle_barriere_fermee);
     
   } while (buttonState2 == LOW );  
   delay (10);
 }                                                                                                                                        
  else if (( buttonState1 == LOW )&&( buttonState2 == HIGH ) ) {
  
   servoA.write(angle_barriere_ouverte);
   servoB.write(angle_barriere_ouverte);
   delay (10) ; 
   digitalWrite(led1, LOW); 
   digitalWrite(led2, LOW); 
   delay (10) ;
 }

  else GEOFENCING () ;
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (Serial.available()){
      char c = Serial.read();
      Serial.print(c);
      if (gps.encode(c)) {
        newData = true;   
      getgps(gps); // saisir les données et les afficher sur l'écran LCD
      SendSMS() ; // envoyer un sms au numero saisie qui contient le lien de localisation
       }}} }
