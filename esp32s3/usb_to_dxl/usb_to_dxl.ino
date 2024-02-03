#include <HardwareSerial.h>


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void update_dxl()
{
  int length;
  int i;


  //-- USB -> DXL
  length = CMD_PORT.available();
  if( length > 0 )
  {
    drv_dxl_tx_enable(TRUE);
    for(i=0; i<length; i++ )
    {
      DXL_PORT.write(CMD_PORT.read());
      DXL_PORT.flush();
    }
    drv_dxl_tx_enable(FALSE);

    tx_led_count = 3;

    tx_data_cnt += length;
  }

  //-- DXL -> USB
  length = DXL_PORT.available();
  if( length > 0 )
  {
    if( length > DXL_TX_BUFFER_LENGTH )
    {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for(i=0; i<length; i++ )
    {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);

    rx_led_count = 3;
    rx_data_cnt += length;
  }
}
