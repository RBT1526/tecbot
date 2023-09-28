
char errorStr[4];
void setup()
{
  Serial.begin(115200);
}

void loop()
{

 Serial.readBytes(errorStr,4); //Read the serial data and store in var
        int error=0;
        int j=0;
        for(int i=1000;i>0;i/10){
            error+=i*int(errorStr[j]);
            j++;
        }
        Serial.println(error);
        
 

}
