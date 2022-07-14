#!/usr/bin/python

import sys

def count_lines_with(symbol, file):
	file.seek(0)
	count = 0

	for line in file.readlines():
		if line.find(symbol) > -1:
			count += 1

	return count


def count_total_lines(file):
	file.seek(0)

	return len(file.readlines())


def count_blank_lines(file):
	file.seek(0)
	count = 0

	for line in file.readlines():
		if line in ['\r\n', '\n']:
			count += 1

	return count


def main(argv):
	print('Reading line:', argv[0])

	with open(argv[0], 'r') as fp:
		print('Total Number of lines:', count_total_lines(fp))

		print('Total Number of empty lines:', count_blank_lines(fp))

		print('Total Number of single commented lines:', count_lines_with('//', fp))
		print('Total Number of start long comment symbol:', count_lines_with('/*', fp))
		print('Total Number of stop long comment symbol:', count_lines_with('*/', fp))


if __name__ == "__main__":
	main(sys.argv[1:])

