void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus  writePointerINA219(uint8_t deviceRegister);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payloadTwoBytes);
WarpStatus 	configureSensorINA219();
void		printSensorDataINA219(bool hexModeFlag);
