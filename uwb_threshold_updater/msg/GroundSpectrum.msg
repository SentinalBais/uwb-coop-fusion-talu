Header header
#
# the range_bin_index = int(m.floor(range/uwb_range_res))
#
# a list of range bin index corresponding to the gamma values 
uint32[] range_bin_index
#
# a list of gamma average if value is unknwon gamma is -1 
float32[] gamma_avg
#
# a list of gamma standard deviation if value is unknwon gamma is -1 
float32[] gamma_std
#
# the number of measurements 
uint32[] count


