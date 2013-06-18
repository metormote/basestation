Reference Firmware Implementation for Basestation
=================================================

http://iop.io
-------------

Basestation Interface
---------------------

<table>
  <tr> <th>Pin</th><th>Name</th><th>Comment</th></tr>
  <tr><td>1</td><td>PDI_DATA</td><td>Programming and debug interface</td></tr>
  <tr><td>2</td><td>3.3V MCU</td><td>Programming and debug interface</td></tr>
  <tr><td>3</td><td></td><td></tr>
  <tr><td>4</td><td></td><td></td></tr>
  <tr><td>5</td><td>RESET/PDI_CLK</td><td>Programming and debug interface</td></tr>
  <tr><td>6</td><td>GND</td><td>Ground</td></tr>
</table>



MCU Pinout Basestation
----------

<table>
  <tr> <th>Port Pin</th><th>MCU Function</th><th>Name</th><th>Comment</th></tr>
  <tr><td>PORTA0</td><td>ADC</td><td>POWER_VBAT_SENSE</td><td>Measures the battery voltage</td></tr>
  <tr><td>PORTA1</td><td>ADC</td><td>POWER_VUSB_SENSE</td><td>Measures the USB power connector voltage</td></tr>
  <tr><td>PORTA2</td><td>INPUT</td><td>CHARGER_FLT</td><td>Charger fault</td></tr>
  <tr><td>PORTA3</td><td>INPUT</td><td>CHARGER_CHG</td><td>Charging</td></tr>
  <tr><td>PORTA4</td><td>ADC</td><td>SENSOR_TEMP</td><td>Internal temperature sensor</td></tr>
  <tr><td>PORTA5</td><td>INPUT</td><td>ETH_nINT</td><td></td></tr>
  <tr><td>PORTA6</td><td>OUTPUT</td><td>ETH_nRST</td><td></td></tr>
  <tr><td>PORTA7</td><td>OUTPUT</td><td>ETH_PWDN</td><td></td></tr>
  <tr><td>PORTB0</td><td>ADC</td><td>POWER_DCM</td><td>Select standby mode</td></tr>
  <tr><td>PORTB1</td><td>ADC</td><td>CHARGER_DOK</td><td>Power OK</td></tr>
  <tr><td>PORTB2</td><td>ADC</td><td>ETH_nLINKLED</td><td></td></tr>
  <tr><td>PORTB3</td><td>INPUT</td><td>UI_SWITCH</td><td>Board switch</td></tr>
  <tr><td>PORTB4</td><td>OUTPUT</td><td>UI_LED_GREEN</td><td></td></tr>
  <tr><td>PORTB5</td><td>OUTPUT</td><td>UI_LED_RED</td><td></td></tr>
  <tr><td>PORTB6</td><td>OUTPUT</td><td>UI_LED_YELLOW</td><td></td></tr>
  <tr><td>PORTB7</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTC0</td><td>SDA</td><td>ETH_SDA</td><td>Ethernet TWI/I2C data</td></tr>
  <tr><td>PORTC1</td><td>SCL</td><td>ETH_SCL</td><td>Ethernet TWI/I2C clock</td></tr>
  <tr><td>PORTC2</td><td>RXD</td><td>GPRS_RXD</td><td></td></tr>
  <tr><td>PORTC3</td><td>TXD</td><td>GPRS_TXD</td><td></td></tr>
  <tr><td>PORTC4</td><td>OUTPUT</td><td>DATAFLASH_RESET</td><td></td></tr>
  <tr><td>PORTC5</td><td>MOSI</td><td>BLE_MOSI</td><td></td></tr>
  <tr><td>PORTC6</td><td>MISO</td><td>BLE_MISO</td><td></td></tr>
  <tr><td>PORTC7</td><td>SCK</td><td>BLE_SCK</td><td></td></tr>
  <tr><td>PORTD0</td><td>GPIO</td><td>BLE_ACTIVE</td><td></td></tr>
  <tr><td>PORTD1</td><td>GPIO</td><td>BLE_RESET</td><td></td></tr>
  <tr><td>PORTD2</td><td>RXD</td><td>ANT_UART_TX</td><td></td></tr>
  <tr><td>PORTD3</td><td>TXD</td><td>ANT_UART_RX</td><td></td></tr>
  <tr><td>PORTD4</td><td>OUTPUT</td><td>DATAFLASH_CS</td><td></td></tr>
  <tr><td>PORTD5</td><td>MOSI</td><td>DATAFLASH_MOSI</td><td></td></tr>
  <tr><td>PORTD6</td><td>MISO</td><td>DATAFLASH_MISO</td><td></td></tr>
  <tr><td>PORTD7</td><td>SCK</td><td>DATAFLASH_SCK</td><td></td></tr>
  <tr><td>PORTE0</td><td>INPUT</td><td>BLE_RDYN</td><td></td></tr>
  <tr><td>PORTE1</td><td>OUTPUT</td><td>BLE_REQN</td><td></td></tr>
  <tr><td>PORTE2</td><td>RXD</td><td>BLE_TXD</td><td>BLE Test Interface</td></tr>
  <tr><td>PORTE3</td><td>TXD</td><td>BLE_RXD</td><td>BLE Test Interface</td></tr>
  <tr><td>PORTE4</td><td>OUTPUT</td><td>ETH_nCSC</td><td>SPI Chip Select</td></tr>
  <tr><td>PORTE5</td><td>MOSI</td><td>ETH_MOSI</td><td></td></tr>
  <tr><td>PORTE6</td><td>MISO</td><td>ETH_MISO</td><td></td></tr>
  <tr><td>PORTE7</td><td>SCK</td><td>ETH_SCK</td><td></td></tr>
  <tr><td>PORTF0</td><td>INPUT</td><td>GPRS_STATUS</td><td></td></tr>
  <tr><td>PORTF1</td><td>INPUT</td><td>GPRS_POWER_MON</td><td></td></tr>
  <tr><td>PORTF2</td><td>OUTPUT</td><td>GPRS_RESET</td><td></td></tr>
  <tr><td>PORTF3</td><td>OUTPUT</td><td>GPRS_ON/OFF</td><td></td></tr>
  <tr><td>PORTF4</td><td>OUTPUT</td><td>ANT_SUSPEND</td><td></td></tr>
  <tr><td>PORTF5</td><td>OUTPUT</td><td>ANT_SLEEP</td><td></td></tr>
  <tr><td>PORTF6</td><td>OUTPUT</td><td>ANT_RTS</td><td></td></tr>
  <tr><td>PORTF7</td><td>OUTPUT</td><td>ANT_RESET</td><td></td></tr>
</table>