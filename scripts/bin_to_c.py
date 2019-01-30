#!/usr/bin/env python
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2018, Linaro Limited
#

import argparse
import array
import os
import re

def get_args():

	parser = argparse.ArgumentParser(description='Converts a binary '
		'file into C source file defining binary data as a'
		'constant byte array.')

	parser.add_argument('--dtb', required=True,
		help='Path to the input binary file')

	parser.add_argument('--label', required=True,
		help='Label for the generated table in the C source file.')

	parser.add_argument('--out', required=True,
		help='Path of the output C file')

	return parser.parse_args()

def main():

	args = get_args();

	with open(args.dtb, 'rb') as indata:
		bytes = indata.read()
		size = len(bytes)

	f = open(args.out, 'w')
	f.write('/* Generated from ' + args.dtb + ' by ' +
		os.path.basename(__file__) + ' */\n\n')
	f.write('#include <compiler.h>\n');
	f.write('#include <stdint.h>\n');
	f.write('__extension__ const uint8_t ' + args.label + '[] ' +
		' __aligned(__alignof__(uint64_t)) = {\n')
	i = 0
	while i < size:
		if i % 8 == 0:
			f.write('\t\t');
		f.write('0x' + '{:02x}'.format(ord(bytes[i])) + ',')
		i = i + 1
		if i % 8 == 0 or i == size:
			f.write('\n')
		else:
			f.write(' ')
	f.write('};\n');
	f.close()

if __name__ == "__main__":
	main()
