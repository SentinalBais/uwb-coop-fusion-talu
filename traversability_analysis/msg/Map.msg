Header header
#
# map resolution in meters 
float32 cell_size 
#
# origin_row_index
uint32 origin_row_index
#
# origin_col_index
uint32 origin_col_index 
# 
# value for uncertain data in 2D map 
int16 error_code
#
# data of elevation map/ cost map/ ect in row major format 
int16[] data
#
# size of each row in original 2D map 
uint16 row_length 
