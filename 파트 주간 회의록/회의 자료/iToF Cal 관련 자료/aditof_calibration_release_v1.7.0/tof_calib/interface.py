
import re
import numpy as np

def load_file_spi(fg_obj,file_path):
    
    """
    Description:
    Write all contents of a load file line by line via SPI. 
    Lines that don't contain a 4 byte address and a 1-4 byte data or that are commented using '//' are ignored.

    Parameters:
    SPIData : An array of 16b address and data pairs
    """
    try:
        line = ""

        # Open load file .. add close?
        load_file = open(file_path,'r').read()

        # Split lines into array elements
        load_file_lines = load_file.split('\n')
        Address = []
        Data = []
        # For each line in the file
        for line_number, line in enumerate(load_file_lines):
            # Check for hex register
            
            if re.search(r'^[0-9A-Fa-f]{1,4}[\s]*[0-9A-Fa-f]{1,4}', line, re.I):
                # Split row elements into an array
                row = line.split()
                # Parse data and address with the following expectations:
				# 	1. They're the 1st and 2nd elements
				#   2. The address must always be 4 bytes but the data can be 1-4

                address = re.search(r'[0-9A-Fa-f]{1,4}', row[0], re.I).group(0)
                data = re.search(r'[0-9A-Fa-f]{1,4}', row[1], re.I).group(0)

                #address_list[line_number] = int(address,16)
                #data_list[line_number] = int(data,16)

                Address.append(int(address,16))
                Data.append(int(data,16))

                #print('write_spi(): Address was: ' + '{:04x}'.format(address,'x') + ' Data was: ' + '{:04x}'.format(data,'x'))
                
                
        # Write SPI data
        fg_obj.dev.writeSPISingle(Address,Data)


        print("load_file_spi(): Loaded file \'" + file_path + "\'")

    except AssertionError as error:

        print(error)
        print('load_file_spi() Error')
        return False

    return True

def reg_data_list_generator(start, end, step_size, bit_start_loc, init_reg_data = 0x0000):
    data_settings = [None] * int((end - start) / step_size)
    reg_data_settings = [None] * int((end - start) / step_size)
    #data_settings = [None] * int((end - start) / step_size)
    for data_setting_cnt, data_setting in enumerate(range(start, end, step_size)):
        reg_data_settings[data_setting_cnt] = "{:04x}".format((data_setting << bit_start_loc | init_reg_data),'x')
        data_settings[data_setting_cnt] = data_setting
    return data_settings, reg_data_settings

def reg_compare_modify_write(fg_obj, addr, val, mask=0x0000, origSettings = []):
    """
	Description:
		Checks if val(ue)&mask is set in addr(ess).
            1. If it is then do nothing and return
            2. If it's not:
                a. Modify and write the new val(ue) to the addr(ess)
                a. Save original addr(ess) value into origSettings if it was provided to the function. 
                And if origSettings dictionary is provided save the previous data into the dictionary
    """
    # Read current value
    prev_val = fg_obj.dev.readSPISingle(addr, 1)[0]

    # If bits are already set correctly do nothing
    if((prev_val & mask) == (val & mask)):
        return 0, origSettings
    # Write new value
    else:
        new_val = (prev_val & ~mask) | (val & mask)
        success = fg_obj.regwrite(addr, new_val)
        fg_obj.error_assert(success, ''.join(['reg_compare_modify_write(): Failed write. Address: ', str(addr), '. Value: ', str(new_val), '.']))

        # Save off previous value
        if "reg" in origSettings:
            origSettings['reg']['addr'] = np.append(origSettings['reg']['addr'], addr)
            origSettings['reg']['val'] = np.append(origSettings['reg']['val'], prev_val)
        
        return 1, origSettings

    return -1, origSettings