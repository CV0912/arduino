#include <ps5Controller.h>
#include <HardwareSerial.h>

#define RXp2 16
#define TXp2 17

int lx,ly,rx,ry,cro,squ,tri,cir,up,down,left,right,ll1,rr1,ll2,rr2,start;

void send(int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,int k,int l,int m,int n,int o,int p)
{

//
    Serial2.write(a);

    Serial2.write(b);
   
    Serial2.write(c);
   
    Serial2.write(d);
   
    Serial2.write(e);
   
    Serial2.write(f);
   
    Serial2.write(g);
   
    Serial2.write(h);
   
    Serial2.write(i);
   
    Serial2.write(j);
   
    Serial2.write(k);
   
    Serial2.write(l);
   
    Serial2.write(m);
   
    Serial2.write(n);
   
    Serial2.write(o);
   
    Serial2.write(p);
   
   

}
void setup()
{   Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  ps5.begin("e8:47:3a:5a:a3:66");
//ps5.begin("7C:66:EF:78:76:F0");    
      Serial.println("Ready.");
}


void loop()
{
 

  if (ps5.isConnected())
  {
//    
      lx =ps5.LStickX();
      ly =ps5.LStickY();
      rx =ps5.RStickX();
      ry =ps5.RStickY();
      cro=ps5.Cross();
      squ=ps5.Square();
      tri=ps5.Triangle();
      cir=ps5.Circle();
      up=ps5.Up();
      down=ps5.Down();
      left=ps5.Left();
      right=ps5.Right();
      
      ll1=ps5.L1();
      rr1=ps5.R1();


      ll2=ps5.L2();
      rr2=ps5.R2();
      //start=ps5.Touchpad();
      //options=ps5.Options();
      //share=ps5.Share();

      Serial.print("LX = ");
      Serial.print(lx);
      Serial.print(" ");
      Serial.print("LY = ");
      Serial.print(ly);
      Serial.print(" ");

      Serial.print("RX = ");
      Serial.print(rx);
      Serial.print(" ");

      Serial.print("RY = ");
      Serial.print(ry);
      Serial.println(" ");

      Serial.print("CROSS = ");
      Serial.print(cro);
      Serial.println(" ");

      Serial.print("SQUARE = ");
      Serial.print(squ);
      Serial.println(" ");
  }

  delay(50);
//  else
//  {
////     lx =0;
//     ly =0;
//    rx =0;
//    ry =0;
//    cro=0;
//   squ=0;
//   tri=0;
//   cir=0;
//   up=0;
//down=0;
// left=0;
// right=0;
// ll1=0;
//rr1=0;
//ll2=0;
//rr2=0;
//start=0;
//options=0;
//share=0;
//Serial.print("not connected");
 // }
  send(lx,ly,rx,ry,cro,squ,tri,cir,up,down,left,right,ll1,ll2,rr1,rr2);

}