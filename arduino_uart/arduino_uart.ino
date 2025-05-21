int lx,ly,rx,ry,up,down,left,right,tri,squ,circle,cross,l1,l2,r1,r2;

int indexA,indexB,indexC,indexD,indexE,indexF,indexG,indexH,indexI,indexJ,indexK,indexL,indexM,indexN,indexO,indexP;

String data1,data2,data3,data4,data5,data6,data7,data8,data9,data10,data11,data12,data13,data14,data15,data16;

String datain;

void fetch_data()
{ 
 if(Serial2.available() > 0)
 {
  datain= Serial2.readStringUntil('!');
 }
}

void parse_data(String datain)
{
indexA=datain.indexOf("A");
indexB=datain.indexOf("B");
indexC=datain.indexOf("C");
indexD=datain.indexOf("D");
indexE=datain.indexOf("E");
indexF=datain.indexOf("F");
indexG=datain.indexOf("G");
indexH=datain.indexOf("H");
indexI=datain.indexOf("I");
indexJ=datain.indexOf("J");
indexK=datain.indexOf("K");
indexL=datain.indexOf("L");
indexM=datain.indexOf("M");
indexN=datain.indexOf("N");
indexO=datain.indexOf("O");
indexP=datain.indexOf("P");

data1=datain.substring(0,indexA);
data2=datain.substring(indexA+1,indexB);
data3=datain.substring(indexB+1,indexC);
data4=datain.substring(indexC+1,indexD);
data5=datain.substring(indexD+1,indexE);
data6=datain.substring(indexE+1,indexF);
data7=datain.substring(indexF+1,indexG);
data8=datain.substring(indexG+1,indexH);
data9=datain.substring(indexH+1,indexI);
data10=datain.substring(indexI+1,indexJ);
data11=datain.substring(indexJ+1,indexK);
data12=datain.substring(indexK+1,indexL);
data13=datain.substring(indexL+1,indexM);
data14=datain.substring(indexM+1,indexN);
data15=datain.substring(indexN+1,indexO);
data16=datain.substring(indexO+1,indexP);

lx=data1.toInt();
ly=data2.toInt();
rx=data3.toInt();
ry=data4.toInt();
up=data5.toInt();
down=data6.toInt();
left=data7.toInt();
right=data8.toInt();
tri=data9.toInt();
squ=data10.toInt();
circle=data11.toInt();
cross=data12.toInt();
l1=data13.toInt();
l2=data14.toInt();
r1=data15.toInt();
r2=data16.toInt();
}
void ps_print()
{
 // Serial.print("lx=");
  Serial.print(lx);
  Serial.print(", ");
//  Serial.print("ly=");
  Serial.print(ly);
  Serial.print(", ");
//  Serial.print("rx=");
  Serial.print(rx);
  Serial.print(", ");
 // Serial.print("ry=");
  Serial.print(ry);
  Serial.print(" ");

 // Serial.print("up=");
  Serial.print(up);
  Serial.print(", ");
 // Serial.print("dowm=");
  Serial.print(down);
  Serial.print(", ");
//  Serial.print("left=");
  Serial.print(left);
  Serial.print(", ");
//  Serial.print("right=");
  Serial.print(right);
  Serial.print(",");
  
//  Serial.print("tri=");
  Serial.print(tri);
  Serial.print(", ");
 // Serial.print("squ=");
  Serial.print(squ);
  Serial.print(", ");
 // Serial.print("circle=");
  Serial.print(circle);
  Serial.print(", ");
//  Serial.print("cross=");
  Serial.print(cross);
  Serial.print(", ");

 // Serial.print("l1=");
  Serial.print(l1);
  Serial.print(", ");
 // Serial.print("l2=");
  Serial.print(l2);
  Serial.print(", ");
//  Serial.print("r1=");
  Serial.print(r1);
  Serial.print(", ");
 // Serial.print("r2=");
  Serial.println(r2);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop()
{
  fetch_data();
  parse_data(datain);
  
  ps_print();
}
