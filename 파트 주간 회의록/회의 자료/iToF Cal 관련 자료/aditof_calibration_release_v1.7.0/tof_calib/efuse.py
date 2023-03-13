import time
import sys
import numpy as np
from tof_calib.interface import reg_compare_modify_write

def read_spi_efuse(CalibAPI, start_addr = 0, num_addr_to_read = 1, print_debug = False):
	"""
	Description:
		Read back from efuse via SPI master. Returns a list of data starting at start_addr and ending at start_addr + num_addr_to_read.
		Set VDDQ to 0V.

	Parameters:
		CalibAPI: CalibAPI instance.
		start_addr: Starting efuse address location to read back from.
		num_addr_to_read: Number of addresses to read back from = start_addr + num_addr_to_read
		print_debug:	True = Prints debug information to stdout
						False = Doesn't print debug information to stdout
	"""
	# Setup dictionary to save original settings
	origSettings = dict()
	origSettings['reg'] = dict()
	origSettings['reg']['addr'] = []
	origSettings['reg']['val'] = []

	# Efuse setup writes
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0014, val=0x0000, mask=0xFFFF, origSettings=origSettings) # digPwrDown - blindly unclock gate all digital
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0602, val=0x00C8, mask=0xFFFF, origSettings=origSettings) # SCLK pulse width for read mode in terms of clk_sys
	ret, origSettings = reg_compare_modify_write(CalibAPI, addr=0x0604, val=0x0333, mask=0xFFFF, origSettings=origSettings) # Default efuse signal timing controls

	efuse_data = [0]*num_addr_to_read
	for cnt_addr in range(num_addr_to_read):
		# Specify address and enable read mode
		efuse_addr = start_addr + cnt_addr
		CalibAPI.dev.writeSPISingle(0x060A, efuse_addr) # Efuse offset [3:0], instance [5:4]
		CalibAPI.dev.writeSPISingle(0x060E, 0x0001) # Enable efuse read mode
		
		# Wait for efuse instance read mode status bit [15] to be ready 
		for cnt_status in range(10):
			efuse_read_status = (CalibAPI.dev.readSPISingle(0x060E, 1)[0] & 0x8000) == 0x8000
			if efuse_read_status == True:
				break
			elif cnt_status == 9:
				sys.exit('Timed out waiting for efuse instance read mode status (bit 15 of 0x060E) to be ready (1).')
			else:
				time.sleep(.5)
				continue

		# Readback efuse data
		efuse_data[cnt_addr] = CalibAPI.dev.readSPISingle(0x0608, 1)[0]

		# Print data to log
		if print_debug:
			print('read_spi_efuse(): Address was: ' + hex(efuse_addr) + ' Data was: ' + hex(efuse_data[cnt_addr]))

		# Disable efuse read mode
		CalibAPI.dev.writeSPISingle(0x060E, 0x0000) 

	# Restore settings
	origSettings_Addr = np.array(origSettings['reg']['addr'], dtype=np.uint16)
	origSettings_Val = np.array(origSettings['reg']['val'], dtype=np.uint16)
	success = CalibAPI.regwrite(origSettings_Addr, origSettings_Val)
	CalibAPI.error_assert(success, 'read_spi_efuse(): Unable restore original settings')

	return efuse_data

def read_spi_efuse_ni(io_master, start_addr = 0, num_addr_to_read = 1, print_debug = False):
	"""
	Description:
		Read back from efuse via SPI master. Returns a list of data starting at start_addr and ending at start_addr + num_addr_to_read.
		Set VDDQ to 0V - seems to work with VDDQ floating

	Parameters:
		io_master: SPI master controller handle.
		start_addr: Starting efuse address location to read back from.
		num_addr_to_read: Number of addresses to read back from = start_addr + num_addr_to_read
		print_debug:	True = Prints debug information to stdout
						False = Doesn't print debug information to stdout
	"""
	# 
	# Setup for write with PLL
	# io_master.write_spi(0x0600", "01F4")
	# io_master.write_spi(0x0602", "01F4")
	# io_master.write_spi(0x0604", "0333")

	# Setup for write without PLL
	#io_master.write_spi(0x0600, 0x00C8, print_writes) # SCLK pulse width for PGM mode in terms of clk_sys
	io_master.write_spi(0x0602, 0x00C8) # SCLK pulse width for read mode in terms of clk_sys
	io_master.write_spi(0x0604, 0x0333) # Default efuse signal timing controls

	efuse_data = [0]*num_addr_to_read
	for cnt_addr in range(num_addr_to_read):
		# Specify address and enable read mode
		efuse_addr = start_addr + cnt_addr
		io_master.write_spi(0x060A, efuse_addr) # Efuse offset [3:0], instance [5:4]
		io_master.write_spi(0x060E, 0x0001) # Enable efuse read mode
		
		# Wait for efuse instance read mode status bit [15] to be ready 
		for cnt_status in range(10):
			efuse_read_status = io_master.read_validate_spi(0x060E, 0x8000, 0x8000)
			if efuse_read_status == True:
				break
			elif cnt_status == 9:
				sys.exit('Timed out waiting for efuse instance read mode status (bit 15 of 0x060E) to be ready (1).')
			else:
				time.sleep(.5)
				continue

		# Readback efuse data
		efuse_data[cnt_addr] = io_master.read_spi(0x0608)

		# Print data to log
		if print_debug:
			print('read_spi_efuse(): Address was: ' + hex(efuse_addr) + ' Data was: ' + hex(efuse_data[cnt_addr]))

		# Disable efuse read mode
		io_master.write_spi(0x060E, 0x0000) 

	return efuse_data

