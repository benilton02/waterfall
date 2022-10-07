#include "ds18b20.h"

uint8_t DS_GPIO;
uint8_t init=0;
uint8_t bitResolution=12;
uint8_t devices=0;

DeviceAddress ROM_NO;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
bool LastDeviceFlag;

void ds18b20_get_address(DeviceAddress *temp_sensor_address) {
	unsigned int number_found = 0;
	reset_search();

	while (search(temp_sensor_address[number_found], true)) {
		number_found++;
        ESP_LOGI(TAG, "%d", number_found);

		if (number_found > MAX_DEVICE_BUS) break;
	}
}

void ds18b20_write(char bit){
	if (bit & 1) {
		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
		noInterrupts();
		gpio_set_level(DS_GPIO,0);
		ets_delay_us(6);
		gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
		ets_delay_us(64);
		interrupts();
	} else {
		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
		noInterrupts();
		gpio_set_level(DS_GPIO,0);
		ets_delay_us(60);
		gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
		ets_delay_us(10);
		interrupts();
	}
}

unsigned char ds18b20_read(void){
	unsigned char value = 0;
	gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
	noInterrupts();
	gpio_set_level(DS_GPIO, 0);
	ets_delay_us(6);
	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
	ets_delay_us(9);
	value = gpio_get_level(DS_GPIO);
	ets_delay_us(55);
	interrupts();
	return (value);
}

void ds18b20_write_byte(char data){
  unsigned char i;
  unsigned char x;
  for(i=0;i<8;i++){
    x = data>>i;
    x &= 0x01;
    ds18b20_write(x);
  }
  ets_delay_us(100);
}

unsigned char ds18b20_read_byte(void){
  unsigned char i;
  unsigned char data = 0;
  for (i=0;i<8;i++)
  {
    if(ds18b20_read()) data|=0x01<<i;
    ets_delay_us(15);
  }
  return(data);
}

unsigned char ds18b20_reset(void){
	unsigned char presence;
	gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
	noInterrupts();
	gpio_set_level(DS_GPIO, 0);
	ets_delay_us(480);
	gpio_set_level(DS_GPIO, 1);
	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
	ets_delay_us(70);
	presence = (gpio_get_level(DS_GPIO) == 0);
	ets_delay_us(410);
	interrupts();
	return presence;
}

bool ds18b20_setResolution(const DeviceAddress tempSensorAddresses[], int numAddresses, uint8_t newResolution) {
	bool success = false;
	newResolution = constrain(newResolution, 9, 12);
    
	uint8_t newValue = 0;
	ScratchPad scratchPad;

	for (int i = 0; i < numAddresses; i++){

		if (ds18b20_isConnected((DeviceAddress*) tempSensorAddresses[i], scratchPad)) {
			switch (newResolution) {
			case 12:
				newValue = TEMP_12_BIT;
				break;
			case 11:
				newValue = TEMP_11_BIT;
				break;
			case 10:
				newValue = TEMP_10_BIT;
				break;
			case 9:
			default:
				newValue = TEMP_9_BIT;
				break;
			}

			if (scratchPad[CONFIGURATION] != newValue) {
				scratchPad[CONFIGURATION] = newValue;
				ds18b20_writeScratchPad((DeviceAddress*) tempSensorAddresses[i], scratchPad);
			}

			success = true;
		}
	}
	return success;
}

void ds18b20_writeScratchPad(const DeviceAddress *deviceAddress, const uint8_t *scratchPad) {
	ds18b20_reset();
	ds18b20_select(deviceAddress);
	ds18b20_write_byte(WRITESCRATCH);
	ds18b20_write_byte(scratchPad[HIGH_ALARM_TEMP]);
	ds18b20_write_byte(scratchPad[LOW_ALARM_TEMP]);
	ds18b20_write_byte(scratchPad[CONFIGURATION]);
	ds18b20_reset();
}

bool ds18b20_readScratchPad(const DeviceAddress *deviceAddress, uint8_t* scratchPad) {
	int b = ds18b20_reset();
	if (b == 0) return false;
	ds18b20_select(deviceAddress);
	ds18b20_write_byte(READSCRATCH);

	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = ds18b20_read_byte();
	}
	b = ds18b20_reset();
	return (b == 1);
}

void ds18b20_select(const DeviceAddress *address){
    uint8_t i;
    ds18b20_write_byte(SELECTDEVICE);           // Choose ROM
    for (i = 0; i < 8; i++) ds18b20_write_byte(((uint8_t *)address)[i]);
}

void ds18b20_requestTemperatures(){
	ds18b20_reset();
	ds18b20_write_byte(SKIPROM);
	ds18b20_write_byte(GETTEMP);
    unsigned long start = esp_timer_get_time() / 1000ULL;
    while (!isConversionComplete() && ((esp_timer_get_time() / 1000ULL) - start < millisToWaitForConversion())) vPortYield();
}

bool isConversionComplete() {
	uint8_t b = ds18b20_read();
	return (b == 1);
}

uint16_t millisToWaitForConversion() {
	switch (bitResolution) {
	case 9:
		return 94;
	case 10:
		return 188;
	case 11:
		return 375;
	default:
		return 750;
	}
}

bool ds18b20_isConnected(const DeviceAddress *deviceAddress, uint8_t *scratchPad) {
	bool b = ds18b20_readScratchPad(deviceAddress, scratchPad);
	return b && !ds18b20_isAllZeros(scratchPad) && (ds18b20_crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

uint8_t ds18b20_crc8(const uint8_t *addr, uint8_t len){
	uint8_t crc = 0;
	while (len--) {
		crc = *addr++ ^ crc; 
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}
	return crc;
}

bool ds18b20_isAllZeros(const uint8_t * const scratchPad) {
	for (size_t i = 0; i < 9; i++) {
		if (scratchPad[i] != 0) {
			return false;
		}
	}
	return true;
}

float ds18b20_getTempF(const DeviceAddress *deviceAddress) {
	ScratchPad scratchPad;
	if (ds18b20_isConnected(deviceAddress, scratchPad)){
		int16_t rawTemp = calculateTemperature(deviceAddress, scratchPad);
		if (rawTemp <= DEVICE_DISCONNECTED_RAW)
			return DEVICE_DISCONNECTED_F;
		// C = RAW/128
		// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
		return ((float) rawTemp * 0.0140625f) + 32.0f;
	}
	return DEVICE_DISCONNECTED_F;
}

float ds18b20_getTempC(const DeviceAddress *deviceAddress) {
	ScratchPad scratchPad;
	if (ds18b20_isConnected(deviceAddress, scratchPad)){
		int16_t rawTemp = calculateTemperature(deviceAddress, scratchPad);
		if (rawTemp <= DEVICE_DISCONNECTED_RAW)
			return DEVICE_DISCONNECTED_F;
		// C = RAW/128
		// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
		return (float) rawTemp/128.0f;
	}
	return DEVICE_DISCONNECTED_F;
}

int16_t calculateTemperature(const DeviceAddress *deviceAddress, uint8_t* scratchPad) {
	int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11) | (((int16_t) scratchPad[TEMP_LSB]) << 3);
	return fpTemperature;
}

float ds18b20_get_temp(void) {
  if(init==1){
    unsigned char check;
    char temp1=0, temp2=0;
      check=ds18b20_RST_PULSE();
      if(check==1)
      {
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0x44);
        vTaskDelay(750 / portTICK_RATE_MS);
        check=ds18b20_RST_PULSE();
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0xBE);
        temp1=ds18b20_read_byte();
        temp2=ds18b20_read_byte();
        check=ds18b20_RST_PULSE();
        float temp=0;
        temp=(float)(temp1+(temp2*256))/16;
        return temp;
      }
      else {return 0;}

  }
  else{return 0;}
}

void ds18b20_init(int GPIO) {
	DS_GPIO = GPIO;
	gpio_pad_select_gpio(DS_GPIO);
	init = 1;
}

void reset_search() {
	devices=0;
	LastDiscrepancy = 0;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0;

	for (int i = 7; i >= 0; i--) {
		ROM_NO[i] = 0;
	}
}

bool search(uint8_t *newAddr, bool search_mode) {
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	bool search_result;
	uint8_t id_bit, cmp_id_bit;

	unsigned char rom_byte_mask, search_direction;

	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

	if (!LastDeviceFlag) {
		if (!ds18b20_reset()) {
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			return false;
		}

		if (search_mode == true) {
			ds18b20_write_byte(0xF0);
		} else {
			ds18b20_write_byte(0xEC);
		}

		do {
			id_bit = ds18b20_read();
			cmp_id_bit = ds18b20_read();

			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
			} 
            else {

				if (id_bit != cmp_id_bit) {
					search_direction = id_bit;
				} 
                else {
					if (id_bit_number < LastDiscrepancy) {
						search_direction = ((ROM_NO[rom_byte_number]
								& rom_byte_mask) > 0);
					} 
                    else {

						search_direction = (id_bit_number == LastDiscrepancy);
					}
					if (search_direction == 0) {
						last_zero = id_bit_number;

						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				ds18b20_write(search_direction);

				id_bit_number++;
				rom_byte_mask <<= 1;

				if (rom_byte_mask == 0) {
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);

		if (!(id_bit_number < 65)) {

			LastDiscrepancy = last_zero;

			if (LastDiscrepancy == 0) {
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	if (!search_result || !ROM_NO[0]) {
		devices=0;
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;
	} else {
		for (int i = 0; i < 8; i++){
			newAddr[i] = ROM_NO[i];
		}
		devices++;
	}
	return search_result;
}
