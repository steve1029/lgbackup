import time
import sys
import tof_calib.efuse as efuse
import numpy as np
from tof_calib.efuse import reg_compare_modify_write

def read_spi_temp_sensor(CalibAPI, ts_offset = 0, ts_slope = 5.45, num_averages = 1, print_result = True):
	"""
	Description:
		Sample internal tempature sensor (TS) via spi. Returns temperature as a float.

	Parameters:
		CalibAPI: CalibAPI instance.
		ts_offset: TS offset correction. This value is unique per part and can be calculated via the efuse 0xB and 0xC addresses.
					If > 0 use value entered.
					If 0 attempt to calculate value via efuse.
		ts_slope: TS slope correction. Characterized by MS to be 5.45 LSB/C for Yeat/Thoreau
		num_averages: Number of temperature sensor samples to average. The end result will be the sum of all samples / num_averages.

	Example Code From MS:
		F3P3 ATE temp code:
			thehdw.Digital.Patterns.Pat(".\..\patterns\ATE_108p02.PAT.gz").Run 
			Call WriteRegister("0150", "0105", 0, 15) 'latch shift chain 1
			thehdw.Wait 0.0001                        'wait for shift chain
			Call WriteRegister("0140", "1033", 0, 15) 'power up SYS PLL
			thehdw.Wait 0.0001                        'delay for PLL to lock
			Call WriteRegister("0028", "0008", 0, 15) 'LX5/temp sensor clock = SYS PLL
			Call WriteRegister("0148", "FFFF", 0, 15) 'Power down all columns (not actually needed if col_comparator_pd == 1)
			Call WriteRegister("0146", "007B", 0, 15) 'Power up bandgap, power down all other analog
			Call WriteRegister("0164", "0003", 0, 15) 'power up and initialize temperature sensor
			Call WriteRegister("0164", "0000", 0, 15) 'start conversion
			thehdw.Wait 0.0002                        '~160us required for conversion to complete
			Call ReadRegister("0166", lngRegisterValues, 0, 15) 'read temp sensor ADC value
	
		USEQ code:
			sub TempSense {
					// SYS PLL must already be running to generate the temp sensor ADC clock "ts_clk_dsadc"
					ld_i16(GPR_TEMP1, REG_POWER_DOWN_READOUT);                  // = 0x0148
					poke(REG_POWER_DOWN_READOUT, 0xFFFF);						// Ensure analog read chain is powered down (comp_bias10u is shared between temp sensor and column comparator so these should not be powered up simultaneously)

					poke(REG_TS_CTRL, 0x0003);									// power up
					pad(2592);													// 30 us wait @ 86.4 MHz sysclk
					poke(REG_TS_CTRL, 0x0000);									// start conversion
					
					bit_while_loop(REG_TS_CTRL, GPR_TEMP0, 15, wait_for_tsense_1); // Wait for TS_CTRL to go low
					bit_while_not_loop(REG_TS_CTRL, GPR_TEMP0, 15, wait_for_tsense_2); // Wait for TS_CTRL to go high
					
					// Now we have a measurement
					poke(REG_TS_CTRL, 0x0004);									// power down
					st_i16(GPR_TEMP1, REG_POWER_DOWN_READOUT); 					// restore state
			};
	"""

	# Setup dictionary to save original settings
	origSettings = dict()
	origSettings['reg'] = dict()
	origSettings['reg']['addr'] = []
	origSettings['reg']['val'] = []

	# Setup part for temp sensor measurement
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0146, val=0x0000, mask=0x0080, origSettings=origSettings) # power_down_adc_others - Power up bandgap
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0140, val=0x0000, mask=0x0300, origSettings=origSettings) # pll_ctrl - Power up SYS PLL and take out of reset
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x018A, val=0x0010, mask=0x0010, origSettings=origSettings) # ANA_SPARE_0 - disable XAGC (higher power, lower jitter)
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x018E, val=0x0200, mask=0x0200, origSettings=origSettings) # ANA_SERIAL_SPARE_0 - xtal OSCREG_ENB=0. Enable xtal regulator for improved power supply rejection.
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0014, val=0x0000, mask=0xFFFF, origSettings=origSettings) # digPwrDown - blindly unclock gate all digital
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x015E, val=0x85A4, mask=0xFFFF, origSettings=origSettings) # syspll_ctrl2_s1 - Set uSeq clock to 86.4MHz and LX5 clock to 216MHz
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0162, val=0x0000, mask=0xFFFF, origSettings=origSettings) # ts_ctrl_s2 - Set for 228 MHz ts clock config (ts_freq_select = 0)
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0028, val=0x0008, mask=0x0008, origSettings=origSettings) # systemClockControl - Set LX5/temp sensor clock to SYS PLL
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0148, val=0xFFFF, mask=0xFFFF, origSettings=origSettings) # power_down_readout - Ensure analog read chain is powered down (comp_bias10u is shared between temp sensor and column comparator so these should not be powered up simultaneously). Note that this isn't needed if col_comparator_pd == 1

	# Calculate TS offset via efuse if it's not provided
	if ts_offset == 0:
		try:
			# Read back from efuse 0xB and 0xC
			efuse_data = efuse.read_spi_efuse(CalibAPI, 0xB, 2)
			ate_forced_tj = efuse_data[0] >> 8
			ate_ts_output = efuse_data[1]

			# Calculate TS offset
			ts_offset = ate_ts_output - (ts_slope * ate_forced_tj)

			if(ts_offset <= 1000 or ts_offset >= 2000):
				print(''.join(['read_spi_temp_sensor() Warning: Expected: 1000 < temp sensor offset < 2000. Measured : ', str(ts_offset)]))

		except AssertionError as error:
			print('read_spi_temp_sensor() Error: Unable to read temp sensor offset from efuse.' + error)
			return None
	
	# Perform measurement
	temp_sensor_data = [None]*num_averages
	for cnt in range(num_averages):
		CalibAPI.dev.writeSPISingle(0x0164, 0x0003)  # power up and initialize TS
		# 30 us wait @ 86.4 MHz sysclk - covered by ~1ms wait between spi writes
		CalibAPI.dev.writeSPISingle(0x0164, 0x0000)  # start conversion

		# Check that TS conversion is complete. ts_ctrl[15] = ts_EOC_reg = 1
		ts_complete = (CalibAPI.dev.readSPISingle(0x0164, 1)[0] & 0x8000) == 0x8000
		if ts_complete:
			temp_sensor_data[cnt] = CalibAPI.dev.readSPISingle(0x0166, 1)[0]
		else:
			print('read_spi_temp_sensor(): Warning: Temperature senor conversion did not complete (ts_EOC_reg !=1) so temp sensor sample ' + cnt + ' will be set to None.')
			temp_sensor_data[cnt] = None

	# Calculate average value
	temp_sensor_avg = sum(temp_sensor_data) / num_averages

	# Calulate temperature
	temp_sensor_result = round((temp_sensor_avg - ts_offset) / ts_slope, 2)

	# Restore settings
	origSettings_Addr = np.array(origSettings['reg']['addr'], dtype=np.uint16)
	origSettings_Val = np.array(origSettings['reg']['val'], dtype=np.uint16)
	success = CalibAPI.regwrite(origSettings_Addr, origSettings_Val)
	CalibAPI.error_assert(success, 'temp_sensor(): Unable restore original settings')

	# PD temp sensor
	CalibAPI.dev.writeSPISingle(0x0164, 0x0004)

	if(print_result):
		print(''.join(["The temperature is: ", str(temp_sensor_result), '.']))

	return temp_sensor_result

def read_spi_temp_sensor_ni(io_master, ts_offset = 0, ts_slope = 5.45, num_averages = 1, verify_conditions = True):
	"""
	Description:
		Sample internal tempature sensor (TS) via spi. Returns temperature as a float.
		SYS PLL must already be running to generate the required temp sensor ADC clock "ts_clk_dsadc"

	Parameters:
		io_master: SPI master controller handle.
		ts_offset: TS offset correction. This value is unique per part and can be calculated via the efuse 0xB and 0xC addresses.
					If > 0 use value entered.
					If 0 attempt to calculate value via efuse.
		ts_slope: TS slope correction. Characterized by MS to be 5.45 LSB/C if Yeat/Thoreau
		num_averages: Number of temperature sensor samples to average. The end result will be the sum of all samples / num_averages.
		verify_conditions:	True = Validate SYS PLL and TS_FREQ_SEL settings
							False = Don't validate SYS PLL and TS_FREQ_SEL settings

	MS example code from F3P3 ATE test case:
		# thehdw.Digital.Patterns.Pat(".\..\patterns\ATE_108p02.PAT.gz").Run 
		# Call WriteRegister("0150", "0105", 0, 15) 'latch shift chain 1
		# thehdw.Wait 0.0001                        'wait for shift chain
		# Call WriteRegister("0140", "1033", 0, 15) 'power up SYS PLL
		# thehdw.Wait 0.0001                        'delay for PLL to lock
		# Call WriteRegister("0028", "0008", 0, 15) 'LX5/temp sensor clock = SYS PLL
		# Call WriteRegister("0148", "FFFF", 0, 15) 'Power down all columns (not actually needed if col_comparator_pd == 1)
		# Call WriteRegister("0146", "007B", 0, 15) 'Power up bandgap, power down all other analog
		# Call WriteRegister("0164", "0003", 0, 15) 'power up and initialize temperature sensor
		# Call WriteRegister("0164", "0000", 0, 15) 'start conversion
		# thehdw.Wait 0.0002                        '~160us required for conversion to complete
		# Call ReadRegister("0166", lngRegisterValues, 0, 15) 'read temp sensor ADC value
	"""
   
	# Brute force registers to match what is in the shift chain. 
	#   WARNING - this could lead to problems if the register content doesn't match what's already been shifted in.
	# io_master.write_spi(0x0150, 0x0105)  # latch shift chain 1
	# time.sleep(0.0001)                   # wait for shift chain

	# Verify SYS PLL and TS_FREQ_SEL are valid
	if verify_conditions:
		sys_pll_running = io_master.read_validate_spi(0x0140, 0x0000, 0x0100) # Check that sys_pll is running
		sys_pll_valid = io_master.read_validate_spi(0x015E, 0x85A4) # uSeq 86.4MHz and LX5 216MHz
		ts_freq_sel_valid = io_master.read_validate_spi(0x0162, 0x0000) #  Check for 228 MHz clock config (ts_freq_select = 0)

		if not (sys_pll_valid & ts_freq_sel_valid & sys_pll_running):
			print('read_spi_temp_sensor() Error: syspll_pd: ' + str(sys_pll_running) + '. syspll_ctrl2_s1 valid: ' + str(sys_pll_valid) + '. ts_freq_sel valid: ' + str(ts_freq_sel_valid) + '. Expected 0x015E = 0x85A4, 0x0162 = 0x0000, and 0x0140[8] = 0')
			return None
	
	# Calculate TS offset via efuse if it's not provided
	if ts_offset == 0:
		try:
			# Read back from efuse 0xB and 0xC
			efuse_data = efuse.read_spi_efuse(io_master, 0xB, 2)
			ate_forced_tj = efuse_data[0] >> 8
			ate_ts_output = efuse_data[1]

			# Calculate TS offset
			ts_offset = ate_ts_output - (ts_slope * ate_forced_tj)
		
		except AssertionError as error:
			print('read_spi_temp_sensor(): Error: Unable to read temp sensor offset from efuse.' + error)
			return None
	
	# Capture the state of the power down bits so that they can be restored
	POWER_DOWN_READOUT_TEMP = io_master.read_spi(0x0148)

	# Ensure analog read chain is powered down (comp_bias10u is shared between temp sensor and column comparator so these should not be powered up simultaneously)
	# Note that this isn't needed if col_comparator_pd == 1
	io_master.write_spi(0x0148, 0xFFFF) 
	
	# Perform measurent
	temp_sensor_data = [None]*num_averages
	for cnt in range(num_averages):
		io_master.write_spi(0x0164, 0x0003)  # power up and initialize TS
		time.sleep(.00003) #30 us wait @ 86.4 MHz sysclk - covered by ~1ms wait between spi writes
		io_master.write_spi(0x0164, 0x0000)  # start conversion

		# Check that TS conversion is complete. ts_ctrl[15] = ts_EOC_reg = 1
		ts_complete = io_master.read_validate_spi(0x0164, expected_data = 0x8000, mask = 0x8000)
		if ts_complete:
			temp_sensor_data[cnt] = io_master.read_spi(0x0166)
		else:
			print('read_spi_temp_sensor(): Warning: Temperature senor conversion did not complete (ts_EOC_reg !=1) so temp sensor sample ' + cnt + ' will be set to None.')
			temp_sensor_data[cnt] = None

	# Calculate average value
	temp_sensor_avg = sum(temp_sensor_data) / num_averages

	# Calulate temperature
	temp_sensor_result = (temp_sensor_avg - ts_offset) / ts_slope

	# Restore settings
	io_master.write_spi(0x0148, POWER_DOWN_READOUT_TEMP)
	io_master.write_spi(0x0164, 0x0004) # TS PD

	return round(temp_sensor_result, 2)

