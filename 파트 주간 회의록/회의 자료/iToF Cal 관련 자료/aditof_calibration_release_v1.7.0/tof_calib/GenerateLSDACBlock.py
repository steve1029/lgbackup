#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np 


def primesfrom2to(n):
	'''
		Generates prime number array from 2 to n

		Inputs:
		  n                     - End point
		  
		Outputs:
		  Prime number array
		
	'''
	""" Input n>=6, Returns a array of primes, 2 <= p < n
	https://stackoverflow.com/questions/2068372/fastest-way-to-list-all-primes-below-n/3035188#3035188 """
	sieve = np.ones(n//3 + (n%6==2), dtype=np.bool)
	for i in range(1,int(n**0.5)//3+1):
		if sieve[i]:
			k=3*i+1|1
			sieve[       k*k//3     ::2*k] = False
			sieve[k*(k-2*(i&1)+4)//3::2*k] = False
	return np.r_[2,3,((3*np.nonzero(sieve)[0][1:]+1)|1)]


def CalculateChecksum(value):
	'''
		Calculates checksum

		Inputs:
		  value                - input value
		  
		Outputs:
		  checksum             - checksum output
		
	'''
	checksum = 0xDEAF
	for i in range(len(value)):
		checksum = int(np.floor(checksum/2) + (2**15)*(checksum % 2)) ^ int(value[i])

	return checksum


def GenerateLSDACStructuralChecksum(_LSDACs):
	'''
		Calculates checksum based on LSDAC values

		Inputs:
		  _LSDACs              - input LSDAC value
		  
		Outputs:
		  checksum             - checksum output
		
	'''
	multipliers = primesfrom2to(2000)
	LSDACs = np.transpose(_LSDACs).flatten()
	checksum = (np.sum(multipliers[0:len(LSDACs)] * (LSDACs == -1))) % 2039
	return checksum


def GenerateLSDACBlock(LSDACVal, n_modes_cfg=10):
	'''
		Generates LSDAC block (indirect register writes) based on LSDAC values

		Inputs:
		  LSDACs               - input LSDAC value
		  
		Outputs:
		  address              - register address array
		  value                - register data array
		
	'''
	LSDACHeader = [5,1,5,0,1,7,3,7,0,5,7] 

	if n_modes_cfg==11:
		LSDACValDefault = np.array([[120, 114, -1],
									[116, 122, 123],
									[120, -1, -1],
									[-1, -1, -1],
									[104, 105, 107],
									[105, 105, 113],
									[0, 0, 0],
									[106, 106, 114],
									[-1, -1, -1],
									[114, -1, -1],
									[105, 105, 113]])
	elif n_modes_cfg==10:
		LSDACValDefault = np.array([[120, 114, -1],
									[116, 122, 123],
									[120, -1, -1],
									[-1, -1, -1],
									[104, 105, 107],
									[105, 105, 113],
									[0, 0, 0],
									[106, 106, 114],
									[-1, -1, -1],
									[114, -1, -1]])

	try:
		LSDACVal = int(LSDACVal)
		LSDACValDefault[LSDACValDefault != -1] = LSDACVal
		LSDACVal = LSDACValDefault
	except:

		pass

	s = np.shape(LSDACVal)
	position = 0x0746 * 4+1

	address = np.array([])
	value = np.array([])
	address_hex = np.array([])
	value_hex = np.array([])
	comment = np.array(['LSDAC Checksum MSB', 'LSDAC Checksum LSB', 'LSDAC Block Size', '[4:0] Number of LSDAC Modes; [15:5] LSDAC Structural Checksum'])

	indicies = np.array(list(np.arange(s[1])))

	for i in range(s[0]):
		mask = LSDACVal[i,:] != -1
		nFrequencies = len(indicies[mask])
		if nFrequencies == 0:
			header = 0
		else:
			header = (LSDACHeader[i] % 256) + 256 * nFrequencies

		address = np.append(address, position)
		position = position - 4
		value = np.append(value, header)
		comment = np.append(comment, 'Mode ' + str(i) + ' LSDAC Header (' + str(nFrequencies) + ' frequencies)')

		for j in range(nFrequencies):
			address = np.append(address, position)
			position = position - 4
			value = np.append(value, LSDACVal[i,j])
			comment = np.append(comment, 'Frequency '+ str(j) + ' DACs')

	checksum = CalculateChecksum(value)
	structuralChecksum = GenerateLSDACStructuralChecksum(LSDACVal) * (2**5)

	address = np.append(np.array([0x74A, 0x749, 0x748, 0x747])*4 + 1 , address).astype(np.int)
	value = np.append(np.array([checksum, 0, len(value), s[0] + structuralChecksum]), value).astype(np.int)


	return address, value
