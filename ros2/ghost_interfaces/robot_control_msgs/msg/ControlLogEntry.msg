
# Constants to identify which field is being used
uint8 TYPE_STRING=1
uint8 TYPE_INT=2
uint8 TYPE_FLOAT=3
uint8 TYPE_INT_ARRAY=4
uint8 TYPE_FLOAT_ARRAY=5

# Type of control log entry
uint8 type

# Name of control log entry key
string key

# All possible storage fields (only one is used at a time)
string string_value
int32 int_value
float32 float_value
int32[] int_array_value
float32[] float_array_value