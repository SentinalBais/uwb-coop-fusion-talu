#!/usr/bin/env python
import numpy as np

def np_2Darray_to_file(twoD_map, fname="tmap.txt"):
    '''
        Input: numpy 2D array dtype = int

        writes 2D array into a text file with header information and the contents of the 2D array
        using Row Major Ordering. The data in the array is seperated by spaces 
        
        Example content of text file  

        2 # number of rows
        2 # number of collumns 
        1 2 3 4 

    '''
    [num_rows, num_cols] = twoD_map.shape
    # file io 
    with open(fname, 'w') as f:
        f.write(str(num_rows) + '\n')
        f.write(str(num_cols) + '\n')
        # load data into textfile in row major order 
        for row in twoD_map:
            row_str = str(row)
            # removes trailing or leading braces
            row_str = row_str.strip('[]')
            # remove multiple white space
            row_str = ' '.join(row_str.split())
            # add trailing white space 
            row_str = row_str + ' '
            f.write(row_str)



def row_major_extractor(data, row_length):
    '''
        given data, which is a list of ints in row major format 
        return the equivalent 2D array 
    '''
    div = len(data) % row_length
    if div != 0:
        print 'invalid data'
        return 
    num_cols = row_length
    num_rows = len(data)/row_length
    np_map = np.ones(shape=(num_rows, num_cols), dtype=int)
    row_i = 0
    col_i = 0
    for i, value in enumerate(data):
        if i % row_length == 0 and i > 0:
            row_i = row_i + 1
            col_i = 0
        np_map[row_i][col_i] = value
        col_i = col_i + 1 

    return np_map


def row_major_extractor_float(data, row_length):
    '''
        given data, which is a list of ints in row major format 
        return the equivalent 2D array 
    '''
    div = len(data) % row_length
    if div != 0:
        print 'invalid data'
        return 
    num_cols = row_length
    num_rows = len(data)/row_length
    np_map = np.ones(shape=(num_rows, num_cols), dtype=np.float32)
    row_i = 0
    col_i = 0
    for i, value in enumerate(data):
        if i % row_length == 0 and i > 0:
            row_i = row_i + 1
            col_i = 0
        np_map[row_i][col_i] = value
        col_i = col_i + 1 

    return np_map

def load_np_2Darray_from_file(fname):
    '''
        reads a file writen using np_2Darray_to_file and writes the data into a 2D np array 
    '''
    num_rows = 0
    num_cols = 0
    data_str = ''
    with open(fname, 'r') as f:
        for i, line in enumerate(f):
            if i == 0:
                num_rows = int(line)
            elif i == 1:
                num_cols = int(line)
            elif i == 2:
                data_str = line

    if len(data_str.split()) != num_rows*num_cols:
        print 'invalid data in text file'
        return
    tmap = row_major_extractor(data_str.split(), num_cols)
    return tmap


