/**********************************************/
/*  Smart LSD Oled control (LSD 1.0)          */
/*  R. Hormigo AIC Columbia University 2021   */
/**********************************************/


#include "LSDoled.h"

  void LSDOled::splash(){
        clearBuffer();
		drawXBMP( 0, 0, ZMBBILogoI_width, ZMBBILogoI_height, ZMBBILogoI_bits);
        sendBuffer();
  }  

  void LSDOled::form(char* header, unsigned int current[8]) {
      clearBuffer();
      drawFrame(0, 0, 256, 64);
      drawLine(0, 22, 255, 22);
      for (int n = 31; n < 256; n += 32) {
          drawLine(n, 23, n, 62);
          setFont(u8g2_font_profont22_tf);
          drawGlyph(n - 21, 39, 48 + ((n + 1) >> 5));
          setFont(u8g2_font_profont15_tf);
          drawStr(n - 29, 61, " mA ");
          setFont(u8g2_font_profont15_tf);
          drawStr(n - 29, 50, u8g2_u16toa(current[(n - 31) / 32], 4)); //Current Value
      }
      setFont(u8g2_font_profont22_tf);
      drawStr(4, 18, header);         
      sendBuffer();

  }
         
  void LSDOled::form(char* mode, char *headerL, char *headerR, unsigned int current[8], int autoDim){
    clearBuffer();
    drawFrame(0,0,256,64);
    drawLine(0,22,255,22);
    drawLine(31, 22, 31, 0); 
    drawLine(159, 22, 159, 0); 
    for(int n=31; n<256 ; n+=32){
      drawLine(n,23,n,62);
      setFont(u8g2_font_profont22_tf);
      drawGlyph(n-21,39,48+((n+1)>>5));
      setFont(u8g2_font_profont15_tf);
      drawStr(n-29,61," mA "); 
      setFont(u8g2_font_profont15_tf);
      drawStr(n-29,50,u8g2_u16toa(current[(n-31)/32], 4)); //Current Value
    }
	setFont(u8g2_font_profont22_tf);
	drawStr(5,18,mode);  
    drawStr(36, 18, headerL);
    drawStr(166,18,headerR);
    sendBuffer();
    if (autoDim) {
        delay(autoDim);
        for(int bright=120;bright>=0;bright-=10) {
            delay(20);
            setContrast(bright); //This set the bright to make a fade out to black
            sendBuffer();   
        }
        clearDisplay();
        setContrast(0x7F); //Restore default old bright/current (Do not increase above 0x7F, or you will have random trouble)
    }
  }
