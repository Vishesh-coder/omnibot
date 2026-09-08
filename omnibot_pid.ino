#include <PS4Controller.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno08x(-1);

sh2_SensorValue_t sensorValue;

#define CH_M1 13
#define M1_d 12
#define CH_M2 26
#define M2_d 25
#define CH_M3 33
#define M3_d 32 
#define OFFSET 10
#define OFFSETLR 10
#define LED 2
#define pi 3.141592
int p=1;;
float yaw = 0.0;
float yaw1=0.0;
float yaws = 0.0;
int Wmax=100;
int mode=2; 
bool emergencyStop = false;
float Kp = 9.5;
float Ki = 1.2;
float Kd = 0.75;
float e= 0.0;
float Yi = 0.0;
float E= 0.0;

unsigned long then = 0;

float YPID(float Ty, float Cy)
{
    yaw=yaw*(180.0/pi);
    unsigned long now = millis();

    float dt = (now - then) / 1000.0;

    if (dt <= 0)
        dt = 0.001;

    then = now;

    e = Cy-Ty;

    while (e > pi)
        e -= 2*pi;

    while (e < -pi)
        e += 2*pi;

    Yi += e* dt;

    Yi = constrain(Yi, -100, 100);

    float d = (E- e) / dt;

    E = e;


    float output = (Kp * e) + (Ki * Yi) - (Kd * d);
output=constrain(output, -5, 5);
    float  op=(output/5.0000);
    yaw=yaw*(pi/180.0);
    return (op)*(127);

}

void led()
{
    if(PS4.Square())
    {
        digitalWrite(LED, HIGH);
        delay(500);

        digitalWrite(LED, LOW);
    }
    if(PS4.Triangle())         
   { for(int i = 0; i < 2; i++)
    {
        digitalWrite(LED, HIGH);
        delay(500);

        digitalWrite(LED, LOW);
    }}
}


void failsafe()
{
    motorc(CH_M1,M1_d,0);
    motorc(CH_M2,M2_d,0);
    motorc(CH_M3,M3_d,0);
}


void motorc(int ch, int d, int speed)
{
    digitalWrite(d, speed >= 0 ? HIGH : LOW);
    speed = constrain(abs(speed), 0, Wmax);
    analogWrite(ch, speed);
}


void updateBNO()
{
    if (!bno08x.getSensorEvent(&sensorValue))
        return;

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
    {
        float qr = sensorValue.un.rotationVector.real;
        float qi = sensorValue.un.rotationVector.i;
        float qj = sensorValue.un.rotationVector.j;
        float qk = sensorValue.un.rotationVector.k;

        yaw = atan2(2.0 * (qi * qj + qk * qr), (sq(qi) - sq(qj) - sq(qk) + sq(qr)));
    }
    if(p==1)
   { yaws=yaw;}
    if(p==1)
   { p=2;}
}
 


void setup()
{
  Serial.begin(115200);


    pinMode(LED, OUTPUT);

    pinMode(CH_M1, OUTPUT);
    pinMode(CH_M2, OUTPUT);
    pinMode(CH_M3, OUTPUT);

    pinMode(M1_d, OUTPUT);
    pinMode(M2_d, OUTPUT);
    pinMode(M3_d, OUTPUT);

    analogWrite(CH_M1, 0);
    analogWrite(CH_M2, 0);
    analogWrite(CH_M3, 0);

    digitalWrite(M1_d, LOW);
    digitalWrite(M2_d, LOW);
    digitalWrite(M3_d, LOW);
    Serial.println("Starting BNO08x...");

  if (!bno08x.begin_I2C())
    while (1);

  bno08x.enableReport(SH2_ROTATION_VECTOR, 5000);

   Serial.println("PS4 Controller connecting,just a minute");

  updateBNO();
  yaws=yaw;
    PS4.begin();
}

void loop()
 {   if(PS4.L1())
 {Wmax-=1;}
     if(PS4.R1())
     {Wmax+=1;}    
    
     updateBNO();
    
     if(PS4.Triangle()&&mode==1)
     {yaw1=yaw;}
     if(PS4.Options())
     {yaw1=yaw;}

     
 
    if(PS4.Circle())
    {yaw1=0;}


       float Yaw= (yaw-yaw1);
    
    if(!PS4.isConnected())
    {
        emergencyStop = true;
    }
    else
    {
        emergencyStop = false;
    }

    if(PS4.PSButton())
    {
        emergencyStop = true;
    }

    if(emergencyStop)
    {
        failsafe();
        return;
    }

    if(PS4.Share())
    {
        emergencyStop = false;
    }

    
       

    float x, y,m,n;
    int X,Y;
    


    X=PS4.LStickX();
    Y=(-1)*(PS4.LStickY());
     if(abs(X)<OFFSET)
      { m=0;}
     else
      { m=X;}
    if(abs(Y)<OFFSET)
      {n=0;}
     else
      {n=Y;} 

    if(PS4.Up()||PS4.Down()||PS4.Left()||PS4.Right())
{     if(PS4.Up())
        {
            m = 0;
            n = 127;
        }

        if(PS4.Down())
        {
            m = 0;
            n = -127;
        }

        if(PS4.Left())
        {
            m = -127;
            n = 0;
        }

        if(PS4.Right())
        {
            m = 127;
            n = 0;
        }
        
        
        }

   

    if(PS4.Square()||mode==1)
    {
        x=m;
        y=n;
        mode=1;
    }
   
    if(PS4.Triangle()||mode==2)
{
       x = m*cos(Yaw) - n*sin(Yaw);
       y = n*cos(Yaw) +m*sin(Yaw);
       mode=2;
       
   }
   
      led();                      
  
  
    float r;
    int p=PS4.L2Value();
    int q=PS4.R2Value();
     if (abs(q) < OFFSETLR)
        {q = 0;}
    
      if (abs(p) < OFFSETLR)
        {p = 0;}

      if (abs(p - q) > OFFSETLR)
{

    r = (q - p) / 2.0;

    yaws = yaw;
}
else
{
    r = YPID(yaws, yaw);
}


float W1 = (0.33*r + 0.67*x) / 127.0;
float W2 = (0.33*r - 0.33*x - 0.578*y) / 127.0;
float W3 = (0.33*r - 0.33*x + 0.578*y) / 127.0;


    motorc(CH_M1,M1_d,W1*Wmax);
    motorc(CH_M2,M2_d,W2*Wmax);
    motorc(CH_M3,M3_d,W3*Wmax);
    
   
   /*  Serial.print("  Kp ");
       Serial.print(Kp);
        Serial.print("  Ki ");
       Serial.print(Ki);
        Serial.print("  Kd ");
       Serial.print(Kd);
         Serial.print("  target yaw  ");
        Serial.print(yaws);
        Serial.print("  yaw  ");
        Serial.print(yaw);
         //Serial.print("  time  ");
        //Serial.print(millis());
         Serial.print("  error  ");
        Serial.println(e);
        Serial.print("  rotation value  ");
        Serial.println(r);*/
  }
