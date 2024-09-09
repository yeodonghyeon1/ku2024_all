void rc(){
  //Serial.begin(9600);
  ch3 = pulseIn(ip3, HIGH); //THRUSTER
  ch1 = pulseIn(ip1, HIGH); //THRUSTER

  float R;
  float L;
  float TL;
  float TR;

  float Tx;
  float Tz;
  float Tport;
  float Tstbd;  
  
  if ((ch3 > 1450 & ch3 < 1550) & (ch1 > 1450 & ch1 < 1550)){     //Control stable
    R=1500;
    L=1500;
    thruster1.writeMicroseconds(R);
    thruster2.writeMicroseconds(L);      //thrusters at zero
  }
  else if ((ch3 < 1450 || ch3 > 1550) || (ch1 < 1450 || ch1 > 1550)) {    //Control for moving
//    Tx = map(ch3, 975, 2025, -70, 70); //ch1-> 1450//  turn->975
//    Tz = map(ch1, 975, 2025, -5, 5);   //ch3-> 1100~1900 go/back 
//    

    Tx = map(ch3, 975, 2025, -200, 200); //go/back
    Tz = map(ch1, 975, 2025, -200, 200);  // left/light
      TL=Tx+100;
      TR=Tx+100;

    if(Tz< -50){ //left
        TL=Tx+Tz;
        TR=Tx-Tz;
      }
       else if (Tz>50){ //light
        TL=Tx+Tz;
        TR=Tx-Tz;
      }


      if(TL> 1900){ //left
        TL=1800;
      }
      
      if(TR>   1900){ //left
        TL=1800;
      }
      

    L = round(TL+1500);
    R = round(TR+1500);

    
//    Tstbd = (Tx / 2) - (Tz / 0.41); // 

//    if (Tstbd > 35){
//      Tstbd = 35;
//    }
//    else if (Tstbd < -35){
//      Tstbd = -35;
//    }
//    
//    Tport = (Tx / 2) - (Tz / 0.41);
//    if (Tport > 35){
//      Tport = 35;
//    }
//    else if (Tstbd < -35){
//      Tport = -35;
//    }



//    Tport = (Tx / (2 * 1.27)) + (Tz / (0.41 * 1.27));

//    if (Tport > 27){
//      Tport = 27;
//    }
//    else if (Tstbd < -27){
//      Tstbd = -27;
//    }


//    R = round((Tstbd / 35 * 400)+1500);
//    L = round((Tport / 35 * 400)+1500);
    thruster1.writeMicroseconds(R);
    thruster2.writeMicroseconds(L);
  }  

  ch6 = pulseIn(ip6,HIGH); //RELAY
  //Serial.println(ch6);
    if (ch6 < 1450)
    {
      digitalWrite(relaypin,LOW);
      delay(100);
    }
    else if (ch6 > 1450)
    {
      digitalWrite(relaypin,HIGH);
      delay(100);
    }
} //end RC fuction
