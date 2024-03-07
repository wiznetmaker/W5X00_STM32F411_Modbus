# W5X00_STM32F411_Modbus
This project is a Modbus RTU/ASCII TCP/IP example running on the hardware environment of the STM32F411 and W5500.

## Hardware
NucleoF411RE : https://www.st.com/en/evaluation-tools/nucleo-f411re.html

W5500 Ethernet Shield : https://docs.wiznet.io/Product/Open-Source-Hardware/w5500_ethernet_shield

## Set network infomation
On line 66 of main.c, set up your network for your environment.
```cpp
/* USER CODE BEGIN PV */
static wiz_NetInfo g_net_info =
{
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
    .ip = {192, 168, 2, 102},                     // IP address
    .sn = {255, 255, 255, 0},                    // Subnet Mask
    .gw = {192, 168, 2, 1},                     // Gateway
    .dns = {8, 8, 8, 8},                         // DNS server
    .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};
```
## Set Timer
If you have a different MCU or a different clock, you will need to set the Timer. This example is set to a timer of 20khz.

100Mhz / (99 + 1) / (49 + 1) = 20,000hz

```cpp
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
```

## Set Modbus Serial
In this example, the USART1 TX : PA15, RX : PB7 Pins are converted to Modbus Serial. 

```cpp
    /**USART1 GPIO Configuration
    PA15     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

## Test Result
I tested using modbus poll/slave tools.

modbus poll : https://www.modbustools.com/modbus_poll.html

modbus slave : https://www.modbustools.com/modbus_slave.html

Modbus Pool Connect :

Modbus Slave Connect :

Test Result :

