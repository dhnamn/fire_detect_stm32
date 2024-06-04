#define BLYNK_TEMPLATE_ID "TMPL6n7Tk-D5H"
#define BLYNK_TEMPLATE_NAME "Temperature"
#define BLYNK_AUTH_TOKEN "gJZlFJBEb3Rd3e_WyqCbtuSUdn1MIEeq"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <HardwareSerial.h>

HardwareSerial SerialPort(2);

#define HOME

#ifdef UET
  char ssid[] = "UET-Wifi-Free 2.4GHz";
  char pass[] = "";
#endif

#ifdef HOME
  char ssid[] = "Tang";
  char pass[] = "0977177089";
#endif

int connect;
int temperature;
WidgetLED ledConnect(V0);
unsigned long timesBlinkLed = millis();

void setup()
{
  SerialPort.begin(9600, SERIAL_8N1, 16, 17);

  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop()
{
  Blynk.run();

  Blink_LED_Connect();
  UART();
  Blynk.virtualWrite(V1, temperature);

  Set_Color_Temperature();
  delay(1000);
}

void UART()
{
  if (!SerialPort.available())
    {
      return;
    }

    //Read temperature
    String buffer = SerialPort.readString();
    String data = buffer.substring(0, buffer.length()/3);
    temperature = data.toInt();
}

void Blink_LED_Connect()
{
  if(millis() - timesBlinkLed > 1000){
    if(ledConnect.getValue()){
      ledConnect.off();
    } else {
      ledConnect.on();
    }

    unsigned long value = millis() / 1000;
    Blynk.virtualWrite(V2, String(value));
    timesBlinkLed = millis();
  }
}

void Set_Color_Temperature()
{
  if (temperature >= 60)
  { 
    //Red
    Blynk.virtualWrite(V3, "Cảnh báo: Nhiệt độ vượt quá 60 độ!");
    Blynk.setProperty(V1, "color", "#FF0000");
  }
  else if (temperature <= 30) 
  {
    //Blue
    Blynk.virtualWrite(V3, "Thông báo: Nhiệt độ ở mức ổn định!");
    Blynk.setProperty(V1, "color", "#0000FF");
  }
  else 
  {
    //Yellow
    Blynk.virtualWrite(V3, "Cảnh báo: Nhiệt độ cao, có nguy cơ gây cháy!");
    Blynk.setProperty(V1, "color", "#FFFF00");
  }
}