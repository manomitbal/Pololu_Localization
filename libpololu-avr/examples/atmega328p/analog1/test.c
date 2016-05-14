#include <pololu/orangutan.h>

/*
 * analog1: for the Orangutan LV, SV, SVP, X2, Baby-O and 3pi robot.
 *
 * This example uses the OrangutanAnalog functions to read the voltage
 * output of the trimpot in the background while the rest of the main
 * loop executes.  The LED is flashed so that its brightness appears
 * proportional to the trimpot position.
 *
 * http://www.pololu.com/docs/0J20
 * http://www.pololu.com
 * http://forum.pololu.com
 */

float result;
float avg;

float main()
{
  set_analog_mode(MODE_8_BIT);    // 8-bit analog-to-digital conversions
  
  start_analog_conversion(TRIMPOT);  // start initial conversion
  if (!analog_is_converting())     // if conversion is done...
  {
    result = (float)(25.0 * analog_conversion_result() / 5.12 / 10.0);  // get result
    return result;
  }
  else
    return 0;
}  
